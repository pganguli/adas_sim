/**
 * @file test_math.c
 * @brief Tests for math_utils.h.
 */
#include <math.h>

#include "common/math_utils.h"
#include "test_util.h"

/** True if a is in (-pi, pi]. */
static int in_wrapped_range(float a) { return a > -ADAS_PI && a <= ADAS_PI; }

void test_math(void) {
  CHECK_NEAR(clampf(5.0f, 0.0f, 1.0f), 1.0f, 0.0);
  CHECK_NEAR(clampf(-5.0f, 0.0f, 1.0f), 0.0f, 0.0);
  CHECK_NEAR(clampf(0.5f, 0.0f, 1.0f), 0.5f, 0.0);
  CHECK(clampi(7, 0, 3) == 3);
  CHECK(clampi(-7, 0, 3) == 0);
  CHECK_NEAR(sqf(-3.0f), 9.0f, 0.0);
  CHECK_NEAR(sigmoidf(0.0f), 0.5f, 1e-6);

  CHECK_NEAR(wrap_angle(0.0f), 0.0f, 1e-6);
  CHECK_NEAR(wrap_angle(ADAS_PI), ADAS_PI, 1e-5);
  CHECK_NEAR(wrap_angle(-ADAS_PI), ADAS_PI, 1e-5); /* range is (-pi, pi] */
  CHECK_NEAR(wrap_angle(3.0f * ADAS_PI), ADAS_PI, 1e-4);
  CHECK_NEAR(wrap_angle(2.0f * ADAS_PI + 0.25f), 0.25f, 1e-5);
  CHECK_NEAR(wrap_angle(-2.0f * ADAS_PI - 0.25f), -0.25f, 1e-5);
  CHECK(in_wrapped_range(wrap_angle(1.0e6f)));
  CHECK(in_wrapped_range(wrap_angle(-1.0e6f)));

  /* Non-finite input must neither hang nor produce a finite angle. */
  CHECK(isnan(wrap_angle(INFINITY)));
  CHECK(isnan(wrap_angle(NAN)));
}
