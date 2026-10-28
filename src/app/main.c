/**
 * @file main.c
 * @brief Closed-loop simulation entry point (see sim_runner.c for the loop).
 */
#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>

#include "app/metrics.h"
#include "app/sim_runner.h"
#include "sim/scenario.h"

#define LOG_DIR "out"
#define LOG_PATH LOG_DIR "/log.csv"

/** Run the default scenario; exit code 0 = ok, 1 = setup failure, 2 = collision. */
int main(void) {
  const SimOptions opt = {LOG_PATH, stdout};
  RunMetrics metrics;

  /* NOLINTNEXTLINE(misc-include-cleaner): EEXIST comes from <errno.h> */
  if (mkdir(LOG_DIR, 0755) != 0 && errno != EEXIST) {
    perror(LOG_DIR);
    return 1;
  }
  if (sim_run(scenario_get(), &opt, &metrics) != 0) {
    return 1;
  }

  metrics_print(&metrics, stdout);
  printf("Log written to %s\n", LOG_PATH);
  return metrics.collision ? 2 : 0;
}
