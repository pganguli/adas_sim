/**
 * @file point_filter_net.h
 * @brief LiDAR pre-processing DNN: per-point MLP (PointNet-style shared MLP).
 *
 *   input : LidarScan (N points, features [x, y, z])
 *   fc1   : Dense(3->2), ReLU
 *   fc2   : Dense(2->1), sigmoid -> P(keep)
 *   output: FilteredPointCloud (points with P(keep) > threshold)
 *
 * The weights encode "keep points between PFILTER_Z_MIN and PFILTER_Z_MAX
 * above the road", i.e. ground removal + height band filtering.
 *
 * Dynamic variant: hidden units are pruned to ceil(quality * 2) (the ground
 * test h0 is kept first, the height-band test h1 is the one dropped), and
 * every point exits right after h0 if it is below the ground band, which
 * cannot change the result.
 */
#ifndef ADAS_POINT_FILTER_NET_H
#define ADAS_POINT_FILTER_NET_H

#include "nn/nn.h"
#include "types/perception_types.h"
#include "types/sensor_types.h"

typedef struct {
  DenseLayer fc1; /**< 3 -> 2, ReLU    */
  DenseLayer fc2; /**< 2 -> 1, sigmoid */
} PointFilterNet;

/** Bind the layers to their constant weights. */
void point_filter_net_init(PointFilterNet *net);

/** Keep only the points the MLP classifies as in-band. */
void point_filter_net_infer(PointFilterNet *net, const LidarScan *in,
                            FilteredPointCloud *out);

/** Pruned + early-exit variant; returns the executed MACs relative to the static net. */
float point_filter_net_infer_dynamic(PointFilterNet *net, const LidarScan *in,
                                     FilteredPointCloud *out, float quality);

#endif /* ADAS_POINT_FILTER_NET_H */
