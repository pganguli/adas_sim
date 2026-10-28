/**
 * @file lane_geometry_net.c
 * @brief LaneGeometryNet implementation.
 */
#include "estimation/lane_geometry_net.h"

#include <math.h>
#include <string.h>

#include "common/camera_geometry.h"
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

void lane_geometry_net_init(LaneGeometryNet *net) {
  static const int rows[LANE_N_ANCHORS] = LANE_ANCHOR_ROWS;
  float A[LANE_N_ANCHORS][3];
  float AtA[3][3] = {{0}};
  float AtA_inv[3][3];

  memset(net, 0, sizeof(*net));
  const DenseLayer head = {LANE_N_ANCHORS, 3, net->head_w, net->head_b,
                           ACT_NONE};
  net->head = head;

  /* Design matrix A: one row [1, X, X^2] per anchor. */
  for (int k = 0; k < LANE_N_ANCHORS; ++k) {
    const float X = cam_row_to_range((float)rows[k] + 0.5f);
    net->anchor_row[k] = rows[k];
    net->anchor_range_m[k] = X;
    A[k][0] = 1.0f;
    A[k][1] = X;
    A[k][2] = X * X;
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < LANE_N_ANCHORS; ++k) {
        AtA[i][j] += A[k][i] * A[k][j];
      }
    }
  }
  if (!invert_3x3(AtA, AtA_inv)) {
    return; /* degenerate anchor layout: head stays zero */
  }

  /* Head weights = pseudo-inverse (A^T A)^-1 A^T, shape 3 x K. */
  for (int i = 0; i < 3; ++i) {
    for (int k = 0; k < LANE_N_ANCHORS; ++k) {
      float w = 0.0f;
      for (int j = 0; j < 3; ++j) {
        w += AtA_inv[i][j] * A[k][j];
      }
      net->head_w[i * LANE_N_ANCHORS + k] = w;
    }
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

void lane_geometry_net_infer(LaneGeometryNet *net, const LaneMask *in,
                             LaneEstimate *out) {
  const float half_lane = 0.5f * LANE_WIDTH_NOMINAL_M;
  float center[LANE_N_ANCHORS];
  int n_missing = 0;

  memset(out, 0, sizeof(*out));
  out->hdr = in->hdr;

  /* ---- Row-anchor features: lane-center lateral position per anchor ---- */
  for (int k = 0; k < LANE_N_ANCHORS; ++k) {
    const int v = net->anchor_row[k];
    const float X = net->anchor_range_m[k];
    float u_l = 0.0f;
    float u_r = 0.0f;
    const bool has_l = soft_argmax_row(in, v, 0, CAM_W / 2, &u_l);
    const bool has_r = soft_argmax_row(in, v, CAM_W / 2, CAM_W, &u_r);

    if (has_l && has_r) {
      center[k] =
          0.5f * (cam_col_to_lateral(u_l, X) + cam_col_to_lateral(u_r, X));
    } else if (has_l) {
      center[k] = cam_col_to_lateral(u_l, X) - half_lane;
    } else if (has_r) {
      center[k] = cam_col_to_lateral(u_r, X) + half_lane;
    } else if (net->prev.valid) {
      const LaneEstimate *p = &net->prev;
      center[k] = p->c0 + p->c1 * X + p->c2 * X * X; /* impute from prior */
      n_missing++;
    } else {
      n_missing = LANE_N_ANCHORS; /* no prior: give up */
      break;
    }
  }
  if (n_missing > LANE_MAX_IMPUTED_ROWS) {
    net->prev.valid = false;
    return; /* out->valid stays false */
  }

  /* ---- Regression head ---- */
  float coeff[3];
  nn_dense(&net->head, center, coeff);

  out->valid = true;
  out->c0 = coeff[0];
  out->c1 = coeff[1];
  out->c2 = coeff[2];
  out->e_y_m = -coeff[0];
  out->e_psi_rad = -atanf(coeff[1]);
  out->kappa_1pm = 2.0f * coeff[2];
  net->prev = *out;
}
