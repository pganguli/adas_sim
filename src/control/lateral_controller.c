/**
 * @file lateral_controller.c
 * @brief Stanley controller implementation.
 */
#include "control/lateral_controller.h"

#include <math.h>

#include "common/math_utils.h"
#include "params.h"
#include "types/control_types.h"
#include "types/planning_types.h"

void lateral_controller_step(const LateralRef *ref, float v_mps,
                             ActuatorCmd *cmd) {
  if (!ref->active) {
    cmd->steer_rad = 0.0f; /* hands back to the driver */
    return;
  }

  const float theta_e = -ref->e_psi_front_rad;
  const float e_fa = -ref->e_y_front_m;
  const float delta =
      theta_e + atanf(STANLEY_K * e_fa / (v_mps + STANLEY_K_SOFT_MPS));

  cmd->steer_rad = clampf(delta, -VEH_STEER_MAX_RAD, VEH_STEER_MAX_RAD);
}
