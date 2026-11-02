/**
 * @file lane_seg_net.h
 * @brief Lane-marking semantic segmentation DNN (2 layers, 2 feature channels).
 *
 *   input : DenoisedImage (1 x H x W)
 *   conv1 : Conv3x3(1->2), ReLU
 *             ch0 "brightness": I - level      (wide, near-range markings)
 *             ch1 "ridge"     : Laplacian (LoG) (thin, far-range markings)
 *   conv2 : Conv1x1(2->1), sigmoid -> P(lane marking)
 *   output: LaneMask (1 x H x W)
 *
 * Dynamic variant (early exit): the cheap brightness channel is always
 * computed; an exit-gate layer Conv1x1(1->1) ReLU(I - level), with
 * level = (1 - quality) * LANESEG_EXIT_LEVEL_MAX, decides per pixel whether
 * the ridge channel runs. Pixels the gate zeroes (road, vehicles) exit with a
 * ridge response of 0. quality = 1 never changes the result.
 */
#ifndef ADAS_LANE_SEG_NET_H
#define ADAS_LANE_SEG_NET_H

#include <stdint.h>

#include "nn/nn.h"
#include "types/perception_types.h"

typedef struct {
  Conv2dLayer conv1;            /**< 3x3, 1 -> 2 feature channels */
  Conv2dLayer conv2;            /**< 1x1, 2 -> 1 logit + sigmoid  */
  float feat[2][CAM_H][CAM_W];  /**< scratch: conv1 output        */
  float gate[CAM_H][CAM_W];     /**< scratch: exit-gate output    */
  uint8_t exited[CAM_H][CAM_W]; /**< scratch: 1 = ridge skipped   */
} LaneSegNet;

/** Bind the layers to their constant weights. */
void lane_seg_net_init(LaneSegNet *net);

/** Segment lane markings in one denoised image. */
void lane_seg_net_infer(LaneSegNet *net, const DenoisedImage *in,
                        LaneMask *out);

/** Early-exit variant; returns the executed MACs relative to the static net. */
float lane_seg_net_infer_dynamic(LaneSegNet *net, const DenoisedImage *in,
                                 LaneMask *out, float quality);

#endif /* ADAS_LANE_SEG_NET_H */
