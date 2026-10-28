/**
 * @file vehicle_types.h
 * @brief State of the ego vehicle plant (kinematic bicycle + actuator states).
 */
#ifndef ADAS_VEHICLE_TYPES_H
#define ADAS_VEHICLE_TYPES_H

#include "types/common_types.h"

/** Ego plant state. */
typedef struct {
  Pose2D pose;     /**< rear-axle center, world frame                  */
  float v_mps;     /**< longitudinal speed (>= 0)                      */
  float a_mps2;    /**< net longitudinal acceleration                  */
  float a_pt_mps2; /**< powertrain/brake accel (lagged actuator state) */
  float delta_rad; /**< front-wheel angle (lagged actuator state)      */
} VehicleState;

#endif /* ADAS_VEHICLE_TYPES_H */
