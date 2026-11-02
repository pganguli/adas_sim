/**
 * @file lane_geometry_net.c
 * @brief LaneGeometryNet implementation.
 */
#include "estimation/lane_geometry_net.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "common/camera_geometry.h"
#include "nn/dynamic.h"
#include "nn/nn.h"
#include "params.h"
#include "types/estimation_types.h"
#include "types/perception_types.h"

/** Inverse of a 3x3 matrix (cofactor method). Returns false if singular. */
static bool invert_3x3(float m[3][3], float inv[3][3]) {
  const float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  if (fabsf(det) < 1e-9f) {
    return false;
  }
  const float s = 1.0f / det;
  inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * s;
  inv[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * s;
  inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * s;
  inv[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * s;
  inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * s;
  inv[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * s;
  inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * s;
  inv[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * s;
  inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * s;
  return true;
}

/** Anchors in the order they are kept when pruning (indices into rows). */
static const int k_priority[LANE_N_ANCHORS] = {5, 0, 3, 2, 4, 1};
/** Fewest anchors that still determine the quadratic. */
#define LANE_MIN_ANCHORS 3

/**
 * Least-squares head over the anchors flagged in `use` (k of them, in index
 * order): w[3][k] = (A^T A)^-1 A^T. Returns false if degenerate.
 */
static bool fit_head(const float *range_m, const bool *use, int k, float *w) {
  float A[LANE_N_ANCHORS][3];
  float AtA[3][3] = {{0}};
  float AtA_inv[3][3];
  int n = 0;

  /* Design matrix A: one row [1, X, X^2] per used anchor. */
  for (int i = 0; i < LANE_N_ANCHORS; ++i) {
    if (use[i]) {
      const float X = range_m[i];
      A[n][0] = 1.0f;
      A[n][1] = X;
      A[n][2] = X * X;
      n++;
    }
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int r = 0; r < k; ++r) {
        AtA[i][j] += A[r][i] * A[r][j];
      }
    }
  }
  if (!invert_3x3(AtA, AtA_inv)) {
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    for (int r = 0; r < k; ++r) {
      float acc = 0.0f;
      for (int j = 0; j < 3; ++j) {
        acc += AtA_inv[i][j] * A[r][j];
      }
      w[i * k + r] = acc;
    }
  }
  return true;
}

void lane_geometry_net_init(LaneGeometryNet *net) {
  static const int rows[LANE_N_ANCHORS] = LANE_ANCHOR_ROWS;
  bool all[LANE_N_ANCHORS];

  memset(net, 0, sizeof(*net));
  const DenseLayer head = {LANE_N_ANCHORS, 3, net->head_w, NULL, ACT_NONE};
  net->head = head;

  for (int k = 0; k < LANE_N_ANCHORS; ++k) {
    net->anchor_row[k] = rows[k];
    net->anchor_range_m[k] = cam_row_to_range((float)rows[k] + 0.5f);
    all[k] = true;
  }
  /* A degenerate anchor layout leaves its head unfitted (head_ok false). */
  net->head_ok[LANE_N_ANCHORS] =
      fit_head(net->anchor_range_m, all, LANE_N_ANCHORS, net->head_w);
  for (int k = LANE_MIN_ANCHORS; k < LANE_N_ANCHORS; ++k) {
    bool use[LANE_N_ANCHORS] = {false};
    for (int i = 0; i < k; ++i) {
      use[k_priority[i]] = true;
    }
    net->head_ok[k] = fit_head(net->anchor_range_m, use, k, net->head_k_w[k]);
  }
}

/** Soft-argmax of (prob - 0.5)+ over columns [u0, u1). Returns false if empty. */
static bool soft_argmax_row(const LaneMask *m, int v, int u0, int u1,
                            float *u_c) {
  float mass = 0.0f;
  float moment = 0.0f;
  for (int u = u0; u < u1; ++u) {
    const float w = m->prob[v][u] - 0.5f;
    if (w > 0.0f) {
      mass += w;
      moment += w * ((float)u + 0.5f);
    }
  }
  if (mass < LANE_MIN_ROW_MASS) {
    return false;
  }
  *u_c = moment / mass;
  return true;
}

/** Anchors kept for a given count: the first n_use of k_priority. */
static void select_anchors(int n_use, bool *use) {
  for (int i = 0; i < LANE_N_ANCHORS; ++i) {
    use[i] = false;
  }
  for (int i = 0; i < n_use; ++i) {
    use[k_priority[i]] = true;
  }
}

/** Estimate from the `n_use` highest-priority anchors (all = static net). */
static void infer_anchors(LaneGeometryNet *net, int n_use, const LaneMask *in,
                          LaneEstimate *out) {
  const float half_lane = 0.5f * LANE_WIDTH_NOMINAL_M;
  const int max_imputed = (n_use - LANE_MIN_ANCHORS < LANE_MAX_IMPUTED_ROWS)
                              ? n_use - LANE_MIN_ANCHORS
                              : LANE_MAX_IMPUTED_ROWS;
  bool use[LANE_N_ANCHORS];
  float center[LANE_N_ANCHORS];
  int n_missing = 0;
  int n = 0;

  memset(out, 0, sizeof(*out));
  out->hdr = in->hdr;
  if (!net->head_ok[n_use]) {
    net->prev.valid = false;
    return; /* no fitted head for this anchor set: out->valid stays false */
  }
  select_anchors(n_use, use);

  /* ---- Row-anchor features: lane-center lateral position per anchor ---- */
  for (int k = 0; k < LANE_N_ANCHORS; ++k) {
    if (!use[k]) {
      continue;
    }
    const int v = net->anchor_row[k];
    const float X = net->anchor_range_m[k];
    float u_l = 0.0f;
    float u_r = 0.0f;
    const bool has_l = soft_argmax_row(in, v, 0, CAM_W / 2, &u_l);
    const bool has_r = soft_argmax_row(in, v, CAM_W / 2, CAM_W, &u_r);

    if (has_l && has_r) {
      center[n] =
          0.5f * (cam_col_to_lateral(u_l, X) + cam_col_to_lateral(u_r, X));
    } else if (has_l) {
      center[n] = cam_col_to_lateral(u_l, X) - half_lane;
    } else if (has_r) {
      center[n] = cam_col_to_lateral(u_r, X) + half_lane;
    } else if (net->prev.valid) {
      center[n] = lane_center_y(&net->prev, X); /* impute from prior */
      n_missing++;
    } else {
      n_missing = LANE_N_ANCHORS; /* no prior: give up */
      break;
    }
    n++;
  }
  if (n_missing > max_imputed) {
    net->prev.valid = false;
    return; /* out->valid stays false */
  }

  /* ---- Regression head ---- */
  DenseLayer head = net->head;
  if (n_use < LANE_N_ANCHORS) {
    head.in_dim = n_use;
    head.weight = net->head_k_w[n_use];
  }
  float coeff[3];
  nn_dense(&head, center, coeff);

  out->valid = true;
  out->c0 = coeff[0];
  out->c1 = coeff[1];
  out->c2 = coeff[2];
  out->e_y_m = -coeff[0];
  out->e_psi_rad = -atanf(coeff[1]);
  out->kappa_1pm = 2.0f * coeff[2];
  net->prev = *out;
}

void lane_geometry_net_infer(LaneGeometryNet *net, const LaneMask *in,
                             LaneEstimate *out) {
  infer_anchors(net, LANE_N_ANCHORS, in, out);
}

float lane_geometry_net_infer_dynamic(LaneGeometryNet *net, const LaneMask *in,
                                      LaneEstimate *out, float quality) {
  int n_use = dyn_keep_count(quality, LANE_N_ANCHORS);
  n_use = (n_use < LANE_MIN_ANCHORS) ? LANE_MIN_ANCHORS : n_use;
  /* Degenerate subset: fall back to the next larger fitted one. */
  while (n_use < LANE_N_ANCHORS && !net->head_ok[n_use]) {
    n_use++;
  }
  infer_anchors(net, n_use, in, out);

  /* Per anchor: two half-row soft-argmaxes (CAM_W taps) + 3 head MACs. */
  const long per_anchor = CAM_W + 3;
  return dyn_cost(n_use * per_anchor, LANE_N_ANCHORS * per_anchor);
}
