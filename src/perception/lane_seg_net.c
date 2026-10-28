/**
 * @file lane_seg_net.c
 * @brief LaneSegNet implementation.
 */
#include "perception/lane_seg_net.h"

#include "nn/nn.h"
#include "params.h"
#include "types/perception_types.h"

/* conv1 weights: [out_c=2][in_c=1][3][3] */
static const float k_conv1_w[2 * 1 * 3 * 3] = {
    /* ch0: brightness (center tap only) */
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    1.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    /* ch1: ridge / negative Laplacian (thin bright lines, any orientation) */
    -1.0f / 8.0f,
    -1.0f / 8.0f,
    -1.0f / 8.0f,
    -1.0f / 8.0f,
    1.0f, /* = 8/8 */
    -1.0f / 8.0f,
    -1.0f / 8.0f,
    -1.0f / 8.0f,
    -1.0f / 8.0f,
};
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
