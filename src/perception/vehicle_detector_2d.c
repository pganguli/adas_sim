/**
 * @file vehicle_detector_2d.c
 * @brief VehicleDetector2D implementation.
 */
#include "perception/vehicle_detector_2d.h"

#include <string.h>

#include "common/math_utils.h"
#include "nn/nn.h"
#include "params.h"
#include "types/perception_types.h"

/* conv1: darkness = ReLU(GAIN * (DARK_LEVEL - I)) */
static const float k_conv1_w[1] = {-DET2D_DARK_GAIN};
static const float k_conv1_b[1] = {DET2D_DARK_GAIN * DET2D_DARK_LEVEL};

/* conv2: heat = sigmoid(G * (mean_3x3(darkness) - FILL_THRESH)) */
#define G9 (DET2D_GAIN / 9.0f)
static const float k_conv2_w[1 * 1 * 3 * 3] = {G9, G9, G9, G9, G9,
                                               G9, G9, G9, G9};
static const float k_conv2_b[1] = {-DET2D_GAIN * DET2D_FILL_THRESH};
#undef G9

void vehicle_detector_2d_init(VehicleDetector2D *net) {
  const Conv2dLayer conv1 = {1, 1, 1, k_conv1_w, k_conv1_b, ACT_RELU, PAD_ZERO};
  const Conv2dLayer conv2 = {1,         1,           3,       k_conv2_w,
                             k_conv2_b, ACT_SIGMOID, PAD_ZERO};
  net->conv1 = conv1;
  net->conv2 = conv2;
}

/** Grow a box from the peak along its row and column (heat > threshold). */
static BBox2D grow_box(const VehicleDetector2D *net, int pu, int pv,
                       float score) {
  const float thr = DET2D_SCORE_THRESH;
  BBox2D b = {pu, pv, pu, pv, score};

  while (b.u_min > 0 && net->heat[pv][b.u_min - 1] > thr) {
    b.u_min--;
  }
  while (b.u_max < CAM_W - 1 && net->heat[pv][b.u_max + 1] > thr) {
    b.u_max++;
  }
  while (b.v_min > 0 && net->heat[b.v_min - 1][pu] > thr) {
    b.v_min--;
  }
  while (b.v_max < CAM_H - 1 && net->heat[b.v_max + 1][pu] > thr) {
    b.v_max++;
  }
  return b;
}

/** NMS: mask the box plus a one-pixel margin. */
static void suppress_box(VehicleDetector2D *net, const BBox2D *b) {
  const int u0 = clampi(b->u_min - 1, 0, CAM_W - 1);
  const int u1 = clampi(b->u_max + 1, 0, CAM_W - 1);
  const int v0 = clampi(b->v_min - 1, 0, CAM_H - 1);
  const int v1 = clampi(b->v_max + 1, 0, CAM_H - 1);
  for (int v = v0; v <= v1; ++v) {
    for (int u = u0; u <= u1; ++u) {
      net->suppressed[v][u] = 1u;
    }
  }
}

void vehicle_detector_2d_infer(VehicleDetector2D *net, const DenoisedImage *in,
                               Detections2D *out) {
  /* ---- Backbone + head ---- */
  const ConstTensor x = const_tensor_make(1, CAM_H, CAM_W, &in->px[0][0]);
  Tensor f = tensor_make(1, CAM_H, CAM_W, &net->feat[0][0]);
  Tensor h = tensor_make(1, CAM_H, CAM_W, &net->heat[0][0]);
  nn_conv2d(&net->conv1, &x, &f);
  const ConstTensor fc = tensor_as_const(&f);
  nn_conv2d(&net->conv2, &fc, &h);

  /* ---- Decode: peak -> box -> suppress, repeat ---- */
  out->hdr = in->hdr;
  out->n = 0;
  memset(net->suppressed, 0, sizeof(net->suppressed));

  for (int iter = 0; iter < 4 * DET2D_MAX_BOXES && out->n < DET2D_MAX_BOXES;
       ++iter) {
    const int peak = nn_argmax_unmasked(&net->heat[0][0],
                                        &net->suppressed[0][0], CAM_H * CAM_W);
    if (peak < 0) {
      break;
    }
    const int pu = peak % CAM_W;
    const int pv = peak / CAM_W;
    const float score = net->heat[pv][pu];
    if (score < DET2D_SCORE_THRESH) {
      break;
    }

    const BBox2D box = grow_box(net, pu, pv, score);
    suppress_box(net, &box);

    const int w = box.u_max - box.u_min + 1;
    const int hgt = box.v_max - box.v_min + 1;
    if (w >= DET2D_MIN_SIZE_PX && hgt >= DET2D_MIN_SIZE_PX) {
      out->boxes[out->n++] = box;
    }
  }
}
