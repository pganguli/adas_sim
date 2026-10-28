/**
 * @file test_main.c
 * @brief Test runner: calls every test group and reports the totals.
 */
#include <stdio.h>

#include "test_util.h"

int g_test_failures = 0;
int g_test_checks = 0;

/** Run one named test group and print its result. */
static void run_group(const char *name, void (*fn)(void)) {
  const int before = g_test_failures;
  fn();
  printf("[%s] %s\n", (g_test_failures == before) ? " ok " : "FAIL", name);
}

int main(void) {
  run_group("math_utils", test_math);
  run_group("rng", test_rng);
  run_group("pid", test_pid);
  run_group("nn", test_nn);
  run_group("scheduler", test_scheduler);
  run_group("world", test_world);
  run_group("metrics", test_metrics);
  run_group("scenario (integration)", test_scenario);

  printf("%d checks, %d failed\n", g_test_checks, g_test_failures);
  return (g_test_failures == 0) ? 0 : 1;
}
