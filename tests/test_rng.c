/**
 * @file test_rng.c
 * @brief Tests for the deterministic random generator.
 */
#include <math.h>

#include "common/rng.h"
#include "test_util.h"

void test_rng(void) {
  Rng a;
  Rng b;
  Rng c;
  rng_seed(&a, 7u);
  rng_seed(&b, 7u);
  rng_seed(&c, 8u);

  int same_ab = 1;
  int same_ac = 1;
  for (int i = 0; i < 100; ++i) {
    const float x = rng_uniform(&a);
    same_ab = same_ab && (x == rng_uniform(&b));
    same_ac = same_ac && (x == rng_uniform(&c));
  }
  CHECK(same_ab);  /* same seed -> same stream */
  CHECK(!same_ac); /* nearby seeds -> different streams */

  Rng r;
  rng_seed(&r, 1u);
  double sum = 0.0;
  double sum_sq = 0.0;
  const int n = 20000;
  int in_range = 1;
  for (int i = 0; i < n; ++i) {
    const float u = rng_uniform(&r);
    in_range = in_range && (u >= 0.0f && u < 1.0f);
    const float g = rng_gaussian(&r, 2.0f);
    sum += (double)g;
    sum_sq += (double)g * (double)g;
  }
  CHECK(in_range);
  CHECK_NEAR(sum / n, 0.0, 0.1);                       /* mean 0   */
  CHECK_NEAR(sqrt(sum_sq / n - (sum / n) * (sum / n)), /* std 2    */
             2.0, 0.1);

  const float v = rng_uniform_range(&r, 5.0f, 6.0f);
  CHECK(v >= 5.0f && v < 6.0f);
}
