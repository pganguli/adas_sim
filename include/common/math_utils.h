/**
 * @file math_utils.h
 * @brief Small scalar helpers used everywhere.
 */
#ifndef ADAS_MATH_UTILS_H
#define ADAS_MATH_UTILS_H

#include <math.h>

/** Pi as a single-precision constant. */
#define ADAS_PI 3.14159265358979f

/** Clamp x to [lo, hi]. */
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) {
    return lo;
  }
  return (x > hi) ? hi : x;
}

/** Clamp x to [lo, hi]. */
static inline int clampi(int x, int lo, int hi) {
  if (x < lo) {
    return lo;
  }
  return (x > hi) ? hi : x;
}

/** x squared. */
static inline float sqf(float x) { return x * x; }

/** Logistic function 1 / (1 + e^-x). */
static inline float sigmoidf(float x) { return 1.0f / (1.0f + expf(-x)); }

/**
 * Wrap an angle to (-pi, pi]. Constant time for any finite input; NaN and
 * infinity give NaN.
 */
static inline float wrap_angle(float a) {
  float r = fmodf(a + ADAS_PI, 2.0f * ADAS_PI); /* (-2pi, 2pi) */
  if (r <= 0.0f) {
    r += 2.0f * ADAS_PI;
  }
  return r - ADAS_PI;
}

#endif /* ADAS_MATH_UTILS_H */
