/**
 * @file mono_depth_net.h
 * @brief ACC state estimation from the camera: range to each 2D detection.
 *
 *   input   : Detections2D
 *   features: per box [1/(v_bottom - v_horizon), 1/height_px, 1/width_px]
 *   fc1     : Dense(3->3), ReLU  -> three range hypotheses
 *             (ground-contact, known height, known width)
 *   fc2     : Dense(3->1)        -> fused range
 *   post    : lateral offset from the box center column and the range
 *   output  : DepthEstimates
 */
#ifndef ADAS_MONO_DEPTH_NET_H
#define ADAS_MONO_DEPTH_NET_H

#include "nn/nn.h"
#include "types/estimation_types.h"
#include "types/perception_types.h"

typedef struct {
  DenseLayer fc1; /**< 3 -> 3 range hypotheses */
  DenseLayer fc2; /**< 3 -> 1 fused range      */
} MonoDepthNet;

/** Bind the layers to their constant weights. */
void mono_depth_net_init(MonoDepthNet *net);

/** Range every detection that touches the road; others are dropped. */
void mono_depth_net_infer(MonoDepthNet *net, const Detections2D *in,
                          DepthEstimates *out);

#endif /* ADAS_MONO_DEPTH_NET_H */
