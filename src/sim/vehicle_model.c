/**
 * @file vehicle_model.c
 * @brief Kinematic bicycle plant (see header for equations).
 */
#include "sim/vehicle_model.h"

#include <math.h>
#include <string.h>

#include "common/math_utils.h"
#include "common/rng.h"
#include "params.h"
#include "types/common_types.h"
#include "types/control_types.h"
#include "types/sensor_types.h"
#include "types/vehicle_types.h"

/** Rolling + aerodynamic deceleration at speed v. */
static float driving_resistance(float v_mps) {
  return VEH_C_ROLL_MPS2 + VEH_C_DRAG_1PM * v_mps * v_mps;
}

/** Kinematic-bicycle yaw rate v / L * tan(delta). */
static float yaw_rate(const VehicleState *s) {
  return s->v_mps / VEH_WHEELBASE_M * tanf(s->delta_rad);
}

void vehicle_model_init(VehicleState *s, Pose2D pose, float v_mps) {
  memset(s, 0, sizeof(*s));
  s->pose = pose;
  s->v_mps = v_mps;
  /* Start in equilibrium: powertrain already cancels driving resistance. */
  s->a_pt_mps2 = driving_resistance(v_mps);
}

void vehicle_model_step(VehicleState *s, const ActuatorCmd *cmd, float dt) {
  /* --- Actuators (first-order lags) --- */
  const float throttle = clampf(cmd->throttle, 0.0f, 1.0f);
  const float brake = clampf(cmd->brake, 0.0f, 1.0f);
  const float a_pt_cmd =
      throttle * VEH_A_DRIVE_MAX_MPS2 - brake * VEH_A_BRAKE_MAX_MPS2;
  const float delta_cmd =
      clampf(cmd->steer_rad, -VEH_STEER_MAX_RAD, VEH_STEER_MAX_RAD);

  s->a_pt_mps2 += (a_pt_cmd - s->a_pt_mps2) * dt / VEH_TAU_ACCEL_S;
  s->delta_rad += (delta_cmd - s->delta_rad) * dt / VEH_TAU_STEER_S;

  /* --- Longitudinal dynamics --- */
  const float resistance =
      (s->v_mps > 0.01f) ? driving_resistance(s->v_mps) : 0.0f;
  s->a_mps2 = s->a_pt_mps2 - resistance;
  if (s->v_mps <= 0.0f && s->a_mps2 < 0.0f) {
    s->a_mps2 = 0.0f; /* brakes hold the car, no reversing */
  }

  /* --- Kinematic bicycle (rear-axle reference) --- */
  s->pose.x += s->v_mps * cosf(s->pose.psi) * dt;
  s->pose.y += s->v_mps * sinf(s->pose.psi) * dt;
  s->pose.psi = wrap_angle(s->pose.psi + yaw_rate(s) * dt);
  s->v_mps += s->a_mps2 * dt;
  if (s->v_mps < 0.0f) {
    s->v_mps = 0.0f;
  }
}

void vehicle_model_measure(const VehicleState *s, Rng *rng, double t_s,
                           EgoMotion *out) {
  out->hdr.seq = 0u;
  out->hdr.stamp_s = t_s;
  out->v_mps = s->v_mps + rng_gaussian(rng, EGO_V_NOISE_STD_MPS);
  out->a_mps2 = s->a_mps2 + rng_gaussian(rng, EGO_A_NOISE_STD_MPS2);
  out->yaw_rate_rps = yaw_rate(s);
  if (out->v_mps < 0.0f) {
    out->v_mps = 0.0f;
  }
}
