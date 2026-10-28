/**
 * @file mono_depth_net.c
 * @brief MonoDepthNet implementation.
 */
#include "estimation/mono_depth_net.h"

#include "common/camera_geometry.h"
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
static const float k_fc1_b[3] = {0.0f, 0.0f, 0.0f};
static const float k_fc2_w[1 * 3] = {0.6f, 0.2f, 0.2f};
static const float k_fc2_b[1] = {0.0f};

void mono_depth_net_init(MonoDepthNet *net) {
  const DenseLayer fc1 = {3, 3, k_fc1_w, k_fc1_b, ACT_RELU};
  const DenseLayer fc2 = {3, 1, k_fc2_w, k_fc2_b, ACT_NONE};
  net->fc1 = fc1;
  net->fc2 = fc2;
}

void mono_depth_net_infer(MonoDepthNet *net, const Detections2D *in,
                          DepthEstimates *out) {
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
    nn_dense(&net->fc1, feat, hyp);
    nn_dense(&net->fc2, hyp, &range);

    const float u_center = 0.5f * (float)(b->u_min + b->u_max + 1);
    out->objs[out->n].range_m = range;
    out->objs[out->n].lateral_m = cam_col_to_lateral(u_center, range);
    out->n++;
  }
}
