/**
 * @file rng.h
 * @brief Deterministic pseudo-random generator (xorshift64*).
 *
 * Every noise source owns its own Rng so runs are reproducible and adding a
 * new noise source does not change the noise seen by existing ones.
 */
#ifndef ADAS_RNG_H
#define ADAS_RNG_H

#include <stdint.h>

/** Generator state; never all-zero once seeded. */
typedef struct {
  uint64_t state;
} Rng;

/** Seed the generator (nearby seeds give unrelated streams). */
void rng_seed(Rng *rng, uint64_t seed);

/** Uniform sample in [0, 1). */
float rng_uniform(Rng *rng);

/** Uniform sample in [lo, hi). */
float rng_uniform_range(Rng *rng, float lo, float hi);

/** Gaussian sample N(0, std^2). */
float rng_gaussian(Rng *rng, float std);

#endif /* ADAS_RNG_H */
