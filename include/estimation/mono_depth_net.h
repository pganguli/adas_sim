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
 *
 * Dynamic variant: only the first ceil(quality * 3) range hypotheses are
 * evaluated (ground-contact, then height, then width: most to least
 * trusted) and fused by a Dense(k->1) head whose weights are fc2's first k,
 * renormalized (real layers, so each k is a standalone exportable network).
 */
#ifndef ADAS_MONO_DEPTH_NET_H
#define ADAS_MONO_DEPTH_NET_H

#include "nn/nn.h"
#include "types/estimation_types.h"
#include "types/perception_types.h"

#define MONO_N_HYP 3 /**< range hypotheses (fc1 width) */

typedef struct {
  DenseLayer fc1; /**< 3 -> 3 range hypotheses */
  DenseLayer fc2; /**< 3 -> 1 fused range      */
  /** Dynamic fusion heads: row k-1 is the Dense(k->1) that blends the first
   *  k hypotheses (fc2 weights renormalized to sum to the same total). */
  float fc2_k_w[MONO_N_HYP][MONO_N_HYP];
} MonoDepthNet;

/** Bind the layers to their constant weights. */
void mono_depth_net_init(MonoDepthNet *net);

/** Range every detection that touches the road; others are dropped. */
void mono_depth_net_infer(MonoDepthNet *net, const Detections2D *in,
                          DepthEstimates *out);

/** Hypothesis-pruned variant; returns the executed MACs relative to the static net. */
float mono_depth_net_infer_dynamic(MonoDepthNet *net, const Detections2D *in,
                                   DepthEstimates *out, float quality);

#endif /* ADAS_MONO_DEPTH_NET_H */
