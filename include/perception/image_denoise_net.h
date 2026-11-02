/**
 * @file image_denoise_net.h
 * @brief Camera pre-processing DNN: 1-layer conv denoiser.
 *
 *   input : CameraFrame   (1 x H x W)
 *   layer : Conv3x3(1->1), Gaussian kernel, ReLU, replicate padding
 *   output: DenoisedImage (1 x H x W)
 *
 * The dynamic variant is documented at image_denoise_net_infer_dynamic().
 */
#ifndef ADAS_IMAGE_DENOISE_NET_H
#define ADAS_IMAGE_DENOISE_NET_H

#include "nn/nn.h"
#include "types/perception_types.h"
#include "types/sensor_types.h"

typedef struct {
  Conv2dLayer conv1;  /**< 3x3 Gaussian smoothing                     */
  Conv2dLayer bypass; /**< 1x1 identity (skip path of dynamic rows)  */
} ImageDenoiseNet;

/** Bind the layer to its constant weights. */
void image_denoise_net_init(ImageDenoiseNet *net);

/** Denoise one camera frame. */
void image_denoise_net_infer(ImageDenoiseNet *net, const CameraFrame *in,
                             DenoisedImage *out);

/**
 * Dynamic variant: only the bottom ceil(quality * H) image rows (road and
 * lower vehicle body; the sky above is least useful) are convolved, the
 * rows above take a 1x1 identity bypass layer instead. Returns the executed MACs relative to the
 * static net.
 */
float image_denoise_net_infer_dynamic(ImageDenoiseNet *net,
                                      const CameraFrame *in, DenoisedImage *out,
                                      float quality);

#endif /* ADAS_IMAGE_DENOISE_NET_H */
