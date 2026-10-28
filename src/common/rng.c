/**
 * @file rng.c
 * @brief xorshift64* generator + Box-Muller Gaussian.
 */
#include "common/rng.h"

#include <math.h>
#include <stdint.h>

#include "common/math_utils.h"

/** Advance the xorshift64* state and return 64 pseudo-random bits. */
static uint64_t rng_next(Rng *rng) {
  uint64_t x = rng->state;
  x ^= x >> 12;
  x ^= x << 25;
  x ^= x >> 27;
  rng->state = x;
  return x * 2685821657736338717ULL;
}

void rng_seed(Rng *rng, uint64_t seed) {
  /* Scramble the seed so small seeds (1, 2, 3...) give unrelated streams. */
  rng->state = (seed + 1u) * 0x9E3779B97F4A7C15ULL;
  if (rng->state == 0u) {
    rng->state = 1u;
  }
}

float rng_uniform(Rng *rng) {
  /* Top 24 bits -> exactly representable float in [0, 1). */
  return (float)(rng_next(rng) >> 40) * (1.0f / 16777216.0f);
}

float rng_uniform_range(Rng *rng, float lo, float hi) {
  return lo + (hi - lo) * rng_uniform(rng);
}

float rng_gaussian(Rng *rng, float std) {
  float u1 = rng_uniform(rng);
  const float u2 = rng_uniform(rng);
  if (u1 < 1e-7f) {
    u1 = 1e-7f;
  }
  return std * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * ADAS_PI * u2);
}
