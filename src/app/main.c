/**
 * @file main.c
 * @brief Closed-loop simulation entry point (see sim_runner.c for the loop).
 */
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/stat.h>

#include "app/metrics.h"
#include "app/sim_runner.h"
#include "params.h"
#include "sim/scenario.h"

/** Run the selected scenario; exit code 0 = ok, 1 = setup failure, 2 = collision. */
int main(void) {
  const ScenarioConfig *sc = scenario_get();
  char log_path[LOG_PATH_MAX];
  RunMetrics metrics;

  const int len =
      snprintf(log_path, sizeof(log_path), LOG_PATH_FMT, sc->name, RNG_SEED);
  if (len < 0 || (size_t)len >= sizeof(log_path)) {
    fprintf(stderr, "log path too long for LOG_PATH_MAX\n");
    return 1;
  }
  /* NOLINTNEXTLINE(misc-include-cleaner): EEXIST comes from <errno.h> */
  if (mkdir(LOG_DIR, 0755) != 0 && errno != EEXIST) {
    perror(LOG_DIR);
    return 1;
  }

  const SimOptions opt = {log_path, stdout};
  if (sim_run(sc, &opt, &metrics) != 0) {
    return 1;
  }

  metrics_print(&metrics, stdout);
  printf("Log written to %s\n", log_path);
  return metrics.collision ? 2 : 0;
}
