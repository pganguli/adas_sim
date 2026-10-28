/**
 * @file pid.h
 * @brief Generic discrete PID controller with integrator clamping
 * (anti-windup).
 *
 *   u = Kp e + Ki * sum(e dt) + Kd * de/dt
 */
#ifndef ADAS_PID_H
#define ADAS_PID_H

#include <stdbool.h>

/** PID gains and state. */
typedef struct {
  float kp, ki, kd; /**< proportional / integral / derivative gains */
  float i_limit;    /**< |integral term| <= i_limit */
  float integral;   /**< integral term (already multiplied by ki)   */
  float prev_error; /**< error of the previous step                 */
  bool has_prev;    /**< prev_error is valid                        */
} Pid;

/** Set the gains and clear the state. */
void pid_init(Pid *pid, float kp, float ki, float kd, float i_limit);

/** Clear the integrator and derivative history (gains are kept). */
void pid_reset(Pid *pid);

/** Advance one step of length `dt` [s] and return the control output. */
float pid_step(Pid *pid, float error, float dt);

#endif /* ADAS_PID_H */
