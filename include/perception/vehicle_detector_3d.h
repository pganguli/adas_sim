/**
 * @file vehicle_detector_3d.h
 * @brief LiDAR vehicle detector on a bird's-eye-view (BEV) grid.
 *
 *   input : FilteredPointCloud
 *   voxel : scatter points into a 1 x NX x NY BEV occupancy-count grid
 *   conv1 : Conv3x3(1->1) all-ones (neighborhood point count) - (MIN-0.5), ReLU
 *   decode: peak -> gather points of the 3x3 neighborhood -> 3D box, NMS,
 *           repeat
 *   output: Detections3D
 */
#ifndef ADAS_VEHICLE_DETECTOR_3D_H
#define ADAS_VEHICLE_DETECTOR_3D_H

#include <stdint.h>

#include "nn/nn.h"
#include "types/perception_types.h"

typedef struct {
  Conv2dLayer conv1;                  /**< 3x3 neighborhood point count */
  float bev[BEV_NX][BEV_NY];          /**< occupancy counts             */
  float heat[BEV_NX][BEV_NY];         /**< conv1 output                 */
  uint8_t suppressed[BEV_NX][BEV_NY]; /**< NMS mask (1 = masked)        */
} VehicleDetector3D;

/** Bind the layer to its constant weights. */
void vehicle_detector_3d_init(VehicleDetector3D *net);

/** Detect up to DET3D_MAX_BOXES vehicles in a filtered point cloud. */
void vehicle_detector_3d_infer(VehicleDetector3D *net,
                               const FilteredPointCloud *in, Detections3D *out);

#endif /* ADAS_VEHICLE_DETECTOR_3D_H */
