/**
 * @file pid.c
 * @brief PID implementation.
 */
#include "control/pid.h"

#include "common/math_utils.h"

void pid_init(Pid *pid, float kp, float ki, float kd, float i_limit) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->i_limit = i_limit;
  pid_reset(pid);
}

void pid_reset(Pid *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->has_prev = false;
}

float pid_step(Pid *pid, float error, float dt) {
  pid->integral += pid->ki * error * dt;
  pid->integral = clampf(pid->integral, -pid->i_limit, pid->i_limit);

  const float derivative =
      pid->has_prev ? (error - pid->prev_error) / dt : 0.0f;
  pid->prev_error = error;
  pid->has_prev = true;

  return pid->kp * error + pid->integral + pid->kd * derivative;
}
