/**
 * @file image_denoise_net.c
 * @brief ImageDenoiseNet implementation.
 */
#include "perception/image_denoise_net.h"

#include "nn/nn.h"
#include "params.h"
#include "types/perception_types.h"
#include "types/sensor_types.h"

/* 3x3 Gaussian (sigma ~ 0.85 px): reduces pixel noise std by ~2.7x. */
static const float k_conv1_w[1 * 1 * 3 * 3] = {
    1.0f / 16.0f, 2.0f / 16.0f, 1.0f / 16.0f, 2.0f / 16.0f, 4.0f / 16.0f,
    2.0f / 16.0f, 1.0f / 16.0f, 2.0f / 16.0f, 1.0f / 16.0f,
};
static const float k_conv1_b[1] = {0.0f};

void image_denoise_net_init(ImageDenoiseNet *net) {
  const Conv2dLayer conv1 = {
      1, 1, 3, k_conv1_w, k_conv1_b, ACT_RELU, PAD_REPLICATE};
  net->conv1 = conv1;
}

void image_denoise_net_infer(ImageDenoiseNet *net, const CameraFrame *in,
                             DenoisedImage *out) {
  const ConstTensor x = const_tensor_make(1, CAM_H, CAM_W, &in->px[0][0]);
  Tensor y = tensor_make(1, CAM_H, CAM_W, &out->px[0][0]);

  out->hdr = in->hdr;
  nn_conv2d(&net->conv1, &x, &y);
}
