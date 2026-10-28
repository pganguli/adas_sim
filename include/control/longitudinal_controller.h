/**
 * @file longitudinal_controller.h
 * @brief Tracks the planner's acceleration reference with throttle / brake.
 *
 *   u = a_des (feed-forward) + PID(a_des - a_meas)
 *   u >= 0 -> throttle = u / A_drive_max,   brake = 0
 *   u <  0 -> brake    = -u / A_brake_max,  throttle = 0
 */
#ifndef ADAS_LONGITUDINAL_CONTROLLER_H
#define ADAS_LONGITUDINAL_CONTROLLER_H

#include "control/pid.h"
#include "types/control_types.h"
#include "types/sensor_types.h"

/** Longitudinal controller state. */
typedef struct {
  Pid pid;
} LongitudinalController;

/** Configure the PID from the LON_* parameters. */
void longitudinal_controller_init(LongitudinalController *c);

/** Writes throttle and brake of `cmd` (steering is left untouched). */
void longitudinal_controller_step(LongitudinalController *c, float a_des_mps2,
                                  const EgoMotion *ego, float dt,
                                  ActuatorCmd *cmd);

#endif /* ADAS_LONGITUDINAL_CONTROLLER_H */
