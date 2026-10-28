/**
 * @file longitudinal_controller.c
 * @brief Longitudinal controller implementation.
 */
#include "control/longitudinal_controller.h"

#include "common/math_utils.h"
#include "control/pid.h"
#include "params.h"
#include "types/control_types.h"
#include "types/sensor_types.h"

void longitudinal_controller_init(LongitudinalController *c) {
  pid_init(&c->pid, LON_KP, LON_KI, LON_KD, LON_I_LIMIT);
}

void longitudinal_controller_step(LongitudinalController *c, float a_des_mps2,
                                  const EgoMotion *ego, float dt,
                                  ActuatorCmd *cmd) {
  /* Standstill hold: don't let the integrator wind up against the brakes. */
  if (ego->v_mps < 0.1f && a_des_mps2 <= 0.0f) {
    pid_reset(&c->pid);
    cmd->throttle = 0.0f;
    cmd->brake = 0.3f;
    return;
  }

  const float u = a_des_mps2 + pid_step(&c->pid, a_des_mps2 - ego->a_mps2, dt);
  if (u >= 0.0f) {
    cmd->throttle = clampf(u / VEH_A_DRIVE_MAX_MPS2, 0.0f, 1.0f);
    cmd->brake = 0.0f;
  } else {
    cmd->throttle = 0.0f;
    cmd->brake = clampf(-u / VEH_A_BRAKE_MAX_MPS2, 0.0f, 1.0f);
  }
}
