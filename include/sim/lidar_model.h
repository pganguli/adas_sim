/**
 * @file lidar_model.h
 * @brief Synthetic LiDAR: ground returns, lead-vehicle returns, clutter.
 */
#ifndef ADAS_LIDAR_MODEL_H
#define ADAS_LIDAR_MODEL_H

#include "common/rng.h"
#include "types/sensor_types.h"
#include "types/world_types.h"

/**
 * Generate one LiDAR scan of the current world state in the ego frame.
 *
 * @param w    ground-truth world
 * @param rng  noise/clutter source (advanced)
 * @param seq  pipeline frame counter stored in the header
 * @param out  resulting point cloud
 */
void lidar_model_scan(const WorldState *w, Rng *rng, uint32_t seq,
                      LidarScan *out);

#endif /* ADAS_LIDAR_MODEL_H */
