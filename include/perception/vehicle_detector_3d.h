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
 *
 * Dynamic variant: a gate layer Conv1x1 ReLU(count - (m - 0.5)), with
 * m = ceil((1 - quality) * DET3D_MIN_POINTS), lets the conv run only on cells
 * holding at least m points themselves (sparse BEV); quality = 1 runs on
 * every cell.
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
  float gate[BEV_NX][BEV_NY];         /**< dynamic: exit-gate output    */
  uint8_t exited[BEV_NX][BEV_NY];     /**< dynamic: 1 = conv skipped    */
} VehicleDetector3D;

/** Bind the layer to its constant weights. */
void vehicle_detector_3d_init(VehicleDetector3D *net);

/** Detect up to DET3D_MAX_BOXES vehicles in a filtered point cloud. */
void vehicle_detector_3d_infer(VehicleDetector3D *net,
                               const FilteredPointCloud *in, Detections3D *out);

/** Sparse variant; returns the executed MACs relative to the static net. */
float vehicle_detector_3d_infer_dynamic(VehicleDetector3D *net,
                                        const FilteredPointCloud *in,
                                        Detections3D *out, float quality);

#endif /* ADAS_VEHICLE_DETECTOR_3D_H */
