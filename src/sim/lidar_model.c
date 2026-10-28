/**
 * @file lidar_model.c
 * @brief Generates a point cloud in the ego frame.
 *
 *  1. Ground returns on a polar grid (z ~ 0 + noise)  -> must be filtered out
 *  2. Lead-vehicle rear face; point density falls with range like a real
 *     scanner (fixed angular resolution)
 *  3. Random clutter (spray, debris, multipath)        -> false positives
 */
#include "sim/lidar_model.h"

#include <math.h>
#include <stdint.h>

#include "common/math_utils.h"
#include "common/rng.h"
#include "params.h"
#include "sim/world.h"
#include "types/common_types.h"
#include "types/sensor_types.h"
#include "types/world_types.h"

_Static_assert((LIDAR_GROUND_N_RINGS * LIDAR_GROUND_N_AZ) +
                       (LIDAR_MAX_COLS_PER_OBJ * LIDAR_MAX_ROWS_PER_OBJ) +
                       LIDAR_N_CLUTTER <=
                   LIDAR_MAX_POINTS,
               "LIDAR_MAX_POINTS too small for the configured scan pattern");

/** Append a point if the scan has room. */
static void push_point(LidarScan *scan, float x, float y, float z) {
  if (scan->n_points < LIDAR_MAX_POINTS) {
    LidarPoint p = {x, y, z};
    scan->points[scan->n_points++] = p;
  }
}

/** Ground returns on a polar grid. */
static void add_ground(LidarScan *scan, Rng *rng) {
  for (int i = 0; i < LIDAR_GROUND_N_RINGS; ++i) {
    for (int j = 0; j < LIDAR_GROUND_N_AZ; ++j) {
      const float r = LIDAR_GROUND_R_MIN_M + (float)i * LIDAR_GROUND_R_STEP_M +
                      rng_gaussian(rng, LIDAR_RANGE_NOISE_STD_M);
      const float az =
          LIDAR_GROUND_AZ_MIN_RAD + (float)j * LIDAR_GROUND_AZ_STEP_RAD;
      push_point(scan, r * cosf(az), r * sinf(az),
                 rng_gaussian(rng, LIDAR_GROUND_NOISE_STD_M));
    }
  }
}

/** Lead-vehicle rear-face returns; density falls with range. */
static void add_lead(LidarScan *scan, Rng *rng, const WorldState *w) {
  if (!w->lead.present) {
    return;
  }
  const Vec2 lead = world_lead_in_ego(w);
  if (lead.x < 1.0f || lead.x > LIDAR_MAX_RANGE_M) {
    return;
  }

  const float body_h = VEHICLE_HEIGHT_M - VEHICLE_BODY_Z_MIN_M;
  const int n_cols =
      clampi((int)(VEHICLE_WIDTH_M / (lead.x * LIDAR_AZ_RES_RAD)), 2,
             LIDAR_MAX_COLS_PER_OBJ);
  const int n_rows = clampi((int)(body_h / (lead.x * LIDAR_EL_RES_RAD)), 1,
                            LIDAR_MAX_ROWS_PER_OBJ);

  for (int i = 0; i < n_rows; ++i) {
    const float z =
        VEHICLE_BODY_Z_MIN_M + body_h * ((float)i + 0.5f) / (float)n_rows;
    for (int j = 0; j < n_cols; ++j) {
      const float y = lead.y - 0.5f * VEHICLE_WIDTH_M +
                      VEHICLE_WIDTH_M * ((float)j + 0.5f) / (float)n_cols;
      push_point(scan, lead.x + rng_gaussian(rng, LIDAR_RANGE_NOISE_STD_M), y,
                 z);
    }
  }
}

/** Random spurious returns. */
static void add_clutter(LidarScan *scan, Rng *rng) {
  for (int k = 0; k < LIDAR_N_CLUTTER; ++k) {
    push_point(scan, rng_uniform_range(rng, 2.0f, LIDAR_MAX_RANGE_M),
               rng_uniform_range(rng, -10.0f, 10.0f),
               rng_uniform_range(rng, 0.0f, 2.0f));
  }
}

void lidar_model_scan(const WorldState *w, Rng *rng, uint32_t seq,
                      LidarScan *out) {
  out->hdr.seq = seq;
  out->hdr.stamp_s = w->t_s;
  out->n_points = 0;

  add_ground(out, rng);
  add_lead(out, rng, w);
  add_clutter(out, rng);
}
