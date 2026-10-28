/**
 * @file vehicle_model.h
 * @brief Ego plant: kinematic bicycle model with first-order actuators.
 *
 *   x'     = v cos(psi)
 *   y'     = v sin(psi)
 *   psi'   = v / L * tan(delta)
 *   v'     = a = a_pt - (c_roll + c_drag v^2)
 *   a_pt'  = (a_pt_cmd - a_pt) / tau_a
 *            with a_pt_cmd = throttle * A_drive - brake * A_brake
 *   delta' = (delta_cmd - delta) / tau_delta
 */
#ifndef ADAS_VEHICLE_MODEL_H
#define ADAS_VEHICLE_MODEL_H

#include "common/rng.h"
#include "types/control_types.h"
#include "types/sensor_types.h"
#include "types/vehicle_types.h"

/** Initialize the plant at `pose` and speed `v_mps`, in drive equilibrium. */
void vehicle_model_init(VehicleState *s, Pose2D pose, float v_mps);

/** Advance the plant by dt [s] (explicit Euler). */
void vehicle_model_step(VehicleState *s, const ActuatorCmd *cmd, float dt);

/** Simulated wheel-speed + IMU measurement of the current state. */
void vehicle_model_measure(const VehicleState *s, Rng *rng, double t_s,
                           EgoMotion *out);

#endif /* ADAS_VEHICLE_MODEL_H */
