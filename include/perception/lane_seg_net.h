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
 */
#ifndef ADAS_LANE_SEG_NET_H
#define ADAS_LANE_SEG_NET_H

#include "nn/nn.h"
#include "types/perception_types.h"

typedef struct {
  Conv2dLayer conv1;           /**< 3x3, 1 -> 2 feature channels */
  Conv2dLayer conv2;           /**< 1x1, 2 -> 1 logit + sigmoid  */
  float feat[2][CAM_H][CAM_W]; /**< scratch: conv1 output        */
} LaneSegNet;

/** Bind the layers to their constant weights. */
void lane_seg_net_init(LaneSegNet *net);

/** Segment lane markings in one denoised image. */
void lane_seg_net_infer(LaneSegNet *net, const DenoisedImage *in,
                        LaneMask *out);

#endif /* ADAS_LANE_SEG_NET_H */
