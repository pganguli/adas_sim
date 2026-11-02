/**
 * @file lane_seg_net.c
 * @brief LaneSegNet implementation.
 */
#include "perception/lane_seg_net.h"

#include <string.h>

#include "nn/dynamic.h"
#include "nn/nn.h"
#include "params.h"
#include "types/perception_types.h"

/* conv1 weights: [out_c=2][in_c=1][3][3] */
#define M8 (-1.0f / 8.0f)
static const float k_conv1_w[2 * 1 * 3 * 3] = {
    /* clang-format off */
    /* ch0: brightness (center tap only) */
    0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    /* ch1: ridge / negative Laplacian (thin bright lines, any orientation) */
    M8,   M8,   M8,
    M8,   1.0f, M8,
    M8,   M8,   M8,
    /* clang-format on */
};
#undef M8
static const float k_conv1_b[2] = {-LANESEG_BRIGHT_LEVEL, 0.0f};

/* conv2 weights: [out_c=1][in_c=2][1][1] */
static const float k_conv2_w[1 * 2] = {LANESEG_W_BRIGHT, LANESEG_W_RIDGE};
static const float k_conv2_b[1] = {LANESEG_BIAS};

void lane_seg_net_init(LaneSegNet *net) {
  const Conv2dLayer conv1 = {
      1, 2, 3, k_conv1_w, k_conv1_b, ACT_RELU, PAD_REPLICATE};
  const Conv2dLayer conv2 = {2,         1,           1,       k_conv2_w,
                             k_conv2_b, ACT_SIGMOID, PAD_ZERO};
  net->conv1 = conv1;
  net->conv2 = conv2;
}

void lane_seg_net_infer(LaneSegNet *net, const DenoisedImage *in,
                        LaneMask *out) {
  const ConstTensor x = const_tensor_make(1, CAM_H, CAM_W, &in->px[0][0]);
  Tensor f = tensor_make(2, CAM_H, CAM_W, &net->feat[0][0][0]);
  Tensor y = tensor_make(1, CAM_H, CAM_W, &out->prob[0][0]);

  out->hdr = in->hdr;
  nn_conv2d(&net->conv1, &x, &f);
  const ConstTensor fc = tensor_as_const(&f);
  nn_conv2d(&net->conv2, &fc, &y);
}

float lane_seg_net_infer_dynamic(LaneSegNet *net, const DenoisedImage *in,
                                 LaneMask *out, float quality) {
  const ConstTensor x = const_tensor_make(1, CAM_H, CAM_W, &in->px[0][0]);
  Tensor y = tensor_make(1, CAM_H, CAM_W, &out->prob[0][0]);
  /* conv1 split per output channel: [out_c][1][3][3] rows are contiguous. */
  const Conv2dLayer ch0 = {
      1, 1, 3, net->conv1.weight, net->conv1.bias, ACT_RELU, net->conv1.pad};
  const Conv2dLayer ch1 = {1,
                           1,
                           3,
                           net->conv1.weight + 9,
                           net->conv1.bias + 1,
                           ACT_RELU,
                           net->conv1.pad};
  Tensor f0 = tensor_make(1, CAM_H, CAM_W, &net->feat[0][0][0]);
  Tensor f1 = tensor_make(1, CAM_H, CAM_W, &net->feat[1][0][0]);
  /* Exit gate: ReLU(I - level), a real 1x1 conv; 0 means "exit". */
  const float gate_w[1] = {1.0f};
  const float gate_b[1] = {-(1.0f - dyn_clamp_quality(quality)) *
                           LANESEG_EXIT_LEVEL_MAX};
  const Conv2dLayer gate = {1, 1, 1, gate_w, gate_b, ACT_RELU, PAD_ZERO};
  Tensor g = tensor_make(1, CAM_H, CAM_W, &net->gate[0][0]);

  out->hdr = in->hdr;
  long macs = nn_conv2d_roi(&gate, &x, &g, 0, CAM_H, NULL);
  const ConstTensor gin = tensor_as_const(&g);
  nn_exit_mask(&gin, &net->exited[0][0]);
  memset(net->feat[1], 0, sizeof(net->feat[1]));

  macs += nn_conv2d_roi(&ch0, &x, &f0, 0, CAM_H, NULL);
  macs += nn_conv2d_roi(&ch1, &x, &f1, 0, CAM_H, &net->exited[0][0]);

  const Tensor f = tensor_make(2, CAM_H, CAM_W, &net->feat[0][0][0]);
  const ConstTensor fc = tensor_as_const(&f);
  macs += nn_conv2d_roi(&net->conv2, &fc, &y, 0, CAM_H, NULL);
  return dyn_cost(macs, nn_conv2d_macs(&net->conv1, CAM_H, CAM_W) +
                            nn_conv2d_macs(&net->conv2, CAM_H, CAM_W));
}
