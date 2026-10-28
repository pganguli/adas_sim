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

#endif /* ADAS_POINT_FILTER_NET_H */
