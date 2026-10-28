/**
 * @file test_util.h
 * @brief Minimal assertion helpers for the test runner (no dependencies).
 */
#ifndef ADAS_TEST_UTIL_H
#define ADAS_TEST_UTIL_H

#include <math.h>
#include <stdio.h>

/** Number of failed checks so far (defined in test_main.c). */
extern int g_test_failures;
/** Number of checks executed so far (defined in test_main.c). */
extern int g_test_checks;

/** Record a failed check with its location. */
#define TEST_FAIL(msg)                                                         \
  do {                                                                         \
    g_test_failures++;                                                         \
    fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, (msg));          \
  } while (0)

/** Check that a condition holds. */
#define CHECK(cond)                                                            \
  do {                                                                         \
    g_test_checks++;                                                           \
    if (!(cond)) {                                                             \
      TEST_FAIL(#cond);                                                        \
    }                                                                          \
  } while (0)

/** Check that |a - b| <= tol. */
#define CHECK_NEAR(a, b, tol)                                                  \
  do {                                                                         \
    g_test_checks++;                                                           \
    if (!(fabs((double)(a) - (double)(b)) <= (double)(tol))) {                 \
      TEST_FAIL(#a " ~= " #b);                                                 \
    }                                                                          \
  } while (0)

void test_math(void);
void test_rng(void);
void test_pid(void);
void test_nn(void);
void test_scheduler(void);
void test_plan_delay(void);
void test_world(void);
void test_metrics(void);
void test_scenario(void);

#endif /* ADAS_TEST_UTIL_H */
