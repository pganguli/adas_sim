/**
 * @file camera_geometry.h
 * @brief Pinhole model of the front camera over a flat road (the "calibration").
 *
 * Image convention: column u grows to the right, row v grows downward, pixel
 * (u, v) has its center at (u + 0.5, v + 0.5). The camera has zero pitch, so
 * the horizon is the row CAM_CV. A ground point at forward range X [m] and
 * lateral offset Y [m] (left positive) projects to
 *
 *     v = CAM_CV + CAM_FX * CAM_HEIGHT_M / X
 *     u = CAM_CU - CAM_FX * Y / X
 *
 * The simulator renders with these functions and perception inverts them,
 * exactly like a real stack shares one calibration file.
 */
#ifndef ADAS_CAMERA_GEOMETRY_H
#define ADAS_CAMERA_GEOMETRY_H

#include "params.h"

/** Ground range [m] seen at image row center v_c (valid only for v_c > CAM_CV). */
static inline float cam_row_to_range(float v_c) {
  return CAM_FX * CAM_HEIGHT_M / (v_c - CAM_CV);
}

/** Lateral offset [m] (left +) of image column center u_c at a given range. */
static inline float cam_col_to_lateral(float u_c, float range_m) {
  return (CAM_CU - u_c) * range_m / CAM_FX;
}

/** Image row of a point at forward range X and height z above ground. */
static inline float cam_point_to_row(float range_m, float z_m) {
  return CAM_CV + CAM_FX * (CAM_HEIGHT_M - z_m) / range_m;
}

/** Image column of a point at forward range X and lateral offset Y. */
static inline float cam_lateral_to_col(float lateral_m, float range_m) {
  return CAM_CU - CAM_FX * lateral_m / range_m;
}

#endif /* ADAS_CAMERA_GEOMETRY_H */
