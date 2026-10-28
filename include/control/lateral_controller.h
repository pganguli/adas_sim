/**
 * @file lateral_controller.h
 * @brief Stanley path-tracking controller (Hoffmann et al., 2007).
 *
 *   delta = theta_e + atan(k * e_fa / (v + k_soft))
 * with theta_e = lane heading - vehicle heading = -e_psi_front and
 * e_fa = cross-track error of the front axle, positive when the lane center
 * is to the left = -e_y_front. Output is clipped to the steering limit.
 */
#ifndef ADAS_LATERAL_CONTROLLER_H
#define ADAS_LATERAL_CONTROLLER_H

#include "types/control_types.h"
#include "types/planning_types.h"

/** Writes steer_rad of `cmd` (0 when LKA is inactive). */
void lateral_controller_step(const LateralRef *ref, float v_mps,
                             ActuatorCmd *cmd);

#endif /* ADAS_LATERAL_CONTROLLER_H */
