/**
 * @file mono_depth_net.c
 * @brief MonoDepthNet implementation.
 */
#include "estimation/mono_depth_net.h"

#include <stddef.h>
#include <string.h>

#include "common/camera_geometry.h"
#include "nn/dynamic.h"
#include "nn/nn.h"
#include "params.h"
#include "types/estimation_types.h"
#include "types/perception_types.h"

/*
 * fc1 (diagonal) turns each inverse-size feature into a range hypothesis
 * using pinhole geometry and the vehicle-size prior:
 *   X_ground = fx * cam_height / (v_bottom - v_horizon)
 *   X_height = fx * vehicle_height / height_px
 *   X_width  = fx * vehicle_width  / width_px
 * fc2 blends them, trusting the ground-contact cue most.
 */
static const float k_fc1_w[3 * 3] = {
    /* clang-format off */
    CAM_FX * CAM_HEIGHT_M,  0.0f,                      0.0f,
    0.0f,                   CAM_FX * VEHICLE_HEIGHT_M, 0.0f,
    0.0f,                   0.0f,                      CAM_FX * VEHICLE_WIDTH_M,
    /* clang-format on */
};
static const float k_fc2_w[1 * 3] = {0.6f, 0.2f, 0.2f};

void mono_depth_net_init(MonoDepthNet *net) {
  const DenseLayer fc1 = {3, 3, k_fc1_w, NULL, ACT_RELU};
  const DenseLayer fc2 = {3, 1, k_fc2_w, NULL, ACT_NONE};
  net->fc1 = fc1;
  net->fc2 = fc2;

  memset(net->fc2_k_w, 0, sizeof(net->fc2_k_w));
  float total = 0.0f;
  for (int i = 0; i < MONO_N_HYP; ++i) {
    total += k_fc2_w[i];
  }
  for (int k = 1; k <= MONO_N_HYP; ++k) {
    float kept = 0.0f;
    for (int i = 0; i < k; ++i) {
      kept += k_fc2_w[i];
    }
    for (int i = 0; i < k; ++i) {
      net->fc2_k_w[k - 1][i] = k_fc2_w[i] * total / kept;
    }
  }
}

/**
 * Range every box using the first `n_hyp` hypotheses. n_hyp equal to the full
 * width of fc1 is the static network.
 */
static void infer_hypotheses(const MonoDepthNet *net, int n_hyp,
                             const Detections2D *in, DepthEstimates *out) {
  /* Hypotheses are fc1 weight rows; fusion is a Dense(n_hyp -> 1). */
  DenseLayer fc1 = net->fc1;
  fc1.out_dim = n_hyp;
  DenseLayer fc2 = net->fc2;
  fc2.in_dim = n_hyp;
  if (n_hyp < net->fc1.out_dim) {
    fc2.weight = net->fc2_k_w[n_hyp - 1];
  }

  out->hdr = in->hdr;
  out->n = 0;

  for (int i = 0; i < in->n; ++i) {
    const BBox2D *b = &in->boxes[i];
    /* Bottom edge of the bottom pixel row ~ ground-contact line. */
    const float v_bottom = (float)b->v_max + 1.0f;
    const float below_horizon_px = v_bottom - CAM_CV;
    if (below_horizon_px < 1.0f) {
      continue; /* box does not touch the road: cannot range it */
    }

    const float feat[3] = {
        1.0f / below_horizon_px,
        1.0f / (float)(b->v_max - b->v_min + 1),
        1.0f / (float)(b->u_max - b->u_min + 1),
    };
    float hyp[3];
    float range;
    nn_dense(&fc1, feat, hyp);
    nn_dense(&fc2, hyp, &range);

    const float u_center = 0.5f * (float)(b->u_min + b->u_max + 1);
    out->objs[out->n].range_m = range;
    out->objs[out->n].lateral_m = cam_col_to_lateral(u_center, range);
    out->n++;
  }
}

void mono_depth_net_infer(MonoDepthNet *net, const Detections2D *in,
                          DepthEstimates *out) {
  infer_hypotheses(net, net->fc1.out_dim, in, out);
}

float mono_depth_net_infer_dynamic(MonoDepthNet *net, const Detections2D *in,
                                   DepthEstimates *out, float quality) {
  const int n_hyp = dyn_keep_count(quality, net->fc1.out_dim);
  infer_hypotheses(net, n_hyp, in, out);

  DenseLayer fc1 = net->fc1;
  fc1.out_dim = n_hyp;
  DenseLayer fc2 = net->fc2;
  fc2.in_dim = n_hyp;
  return dyn_cost(nn_dense_macs(&fc1) + nn_dense_macs(&fc2),
                  nn_dense_macs(&net->fc1) + nn_dense_macs(&net->fc2));
}
