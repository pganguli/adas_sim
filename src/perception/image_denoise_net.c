/**
 * @file image_denoise_net.c
 * @brief ImageDenoiseNet implementation.
 */
#include "perception/image_denoise_net.h"

#include <stddef.h>

#include "nn/dynamic.h"
#include "nn/nn.h"
#include "params.h"
#include "types/perception_types.h"
#include "types/sensor_types.h"

/* 3x3 Gaussian (sigma ~ 0.85 px): reduces pixel noise std by ~2.7x. */
static const float k_conv1_w[1 * 1 * 3 * 3] = {
    1.0f / 16.0f, 2.0f / 16.0f, 1.0f / 16.0f, 2.0f / 16.0f, 4.0f / 16.0f,
    2.0f / 16.0f, 1.0f / 16.0f, 2.0f / 16.0f, 1.0f / 16.0f,
};

static const float k_bypass_w[1] = {1.0f};

void image_denoise_net_init(ImageDenoiseNet *net) {
  const Conv2dLayer conv1 = {1, 1, 3, k_conv1_w, NULL, ACT_RELU, PAD_REPLICATE};
  const Conv2dLayer bypass = {1, 1, 1, k_bypass_w, NULL, ACT_RELU, PAD_ZERO};
  net->conv1 = conv1;
  net->bypass = bypass;
}

void image_denoise_net_infer(ImageDenoiseNet *net, const CameraFrame *in,
                             DenoisedImage *out) {
  const ConstTensor x = const_tensor_make(1, CAM_H, CAM_W, &in->px[0][0]);
  Tensor y = tensor_make(1, CAM_H, CAM_W, &out->px[0][0]);

  out->hdr = in->hdr;
  nn_conv2d(&net->conv1, &x, &y);
}

float image_denoise_net_infer_dynamic(ImageDenoiseNet *net,
                                      const CameraFrame *in, DenoisedImage *out,
                                      float quality) {
  const ConstTensor x = const_tensor_make(1, CAM_H, CAM_W, &in->px[0][0]);
  Tensor y = tensor_make(1, CAM_H, CAM_W, &out->px[0][0]);
  const int rows = dyn_keep_count(quality, CAM_H);
  const int y0 = CAM_H - rows;

  out->hdr = in->hdr;
  /* Rows outside the region of interest take the identity bypass. */
  long macs = nn_conv2d_roi(&net->bypass, &x, &y, 0, y0, NULL);
  macs += nn_conv2d_roi(&net->conv1, &x, &y, y0, CAM_H, NULL);
  return dyn_cost(macs, nn_conv2d_macs(&net->conv1, CAM_H, CAM_W));
}
