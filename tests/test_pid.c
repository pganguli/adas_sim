/**
 * @file test_pid.c
 * @brief Tests for the PID controller.
 */
#include "control/pid.h"
#include "test_util.h"

void test_pid(void) {
  Pid p;

  /* Proportional only. */
  pid_init(&p, 2.0f, 0.0f, 0.0f, 1.0f);
  CHECK_NEAR(pid_step(&p, 1.5f, 0.1f), 3.0f, 1e-6);

  /* Integral accumulates and is clamped to i_limit. */
  pid_init(&p, 0.0f, 1.0f, 0.0f, 0.5f);
  CHECK_NEAR(pid_step(&p, 0.2f, 1.0f), 0.2f, 1e-6);
  CHECK_NEAR(pid_step(&p, 10.0f, 1.0f), 0.5f, 1e-6); /* clamped */
  CHECK_NEAR(pid_step(&p, -10.0f, 1.0f), -0.5f, 1e-6);

  /* Derivative is skipped on the first step, then uses the error difference. */
  pid_init(&p, 0.0f, 0.0f, 1.0f, 1.0f);
  CHECK_NEAR(pid_step(&p, 1.0f, 0.5f), 0.0f, 1e-6);
  CHECK_NEAR(pid_step(&p, 2.0f, 0.5f), 2.0f, 1e-6);

  /* Reset clears the state but keeps the gains. */
  pid_reset(&p);
  CHECK(!p.has_prev);
  CHECK_NEAR(p.integral, 0.0f, 0.0);
  CHECK_NEAR(p.kd, 1.0f, 0.0);
}
