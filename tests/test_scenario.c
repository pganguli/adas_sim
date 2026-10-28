/**
 * @file test_scenario.c
 * @brief Integration test: runs the default scenario and checks the KPIs.
 */
#include <math.h>

#include "app/metrics.h"
#include "app/sim_runner.h"
#include "params.h"
#include "sim/scenario.h"
#include "test_util.h"
#include "types/planning_types.h"

void test_scenario(void) {
  const ScenarioConfig *sc = scenario_get();
  const SimOptions quiet = {NULL, NULL};
  RunMetrics a;
  RunMetrics b;

  CHECK(sim_run(sc, &quiet, &a) == 0);

  /* Safety and completion. */
  CHECK(!a.collision);
  CHECK(a.min_bumper_gap_m > 2.0f);
  CHECK(a.n_ticks == (int)(sc->duration_s / PIPELINE_DT_S + 0.5f));

  /* Estimation quality. */
  CHECK((double)a.n_lane_valid / a.n_ticks > 0.99);
  CHECK((double)a.n_lead_valid / a.n_ticks > 0.95);
  CHECK(sqrt(a.sq_err_e_y / a.n_lane_valid) < 0.1);
  CHECK(sqrt(a.sq_err_gap / a.n_lead_valid) < 0.25);
  CHECK(sqrt(a.sq_err_e_y_true / a.n_ticks) < 0.15);

  /* The scenario exercises all three active modes. */
  CHECK(a.mode_ticks[LON_MODE_CRUISE] > 0);
  CHECK(a.mode_ticks[LON_MODE_FOLLOW] > 0);
  CHECK(a.mode_ticks[LON_MODE_EMERGENCY_BRAKE] > 0);

  /* Pipeline parallelism pays off. */
  CHECK(a.makespan_ms < a.serial_ms);

  /* Determinism: a second run reproduces the first bit for bit. */
  CHECK(sim_run(sc, &quiet, &b) == 0);
  CHECK(a.n_ticks == b.n_ticks);
  CHECK(a.sq_err_gap == b.sq_err_gap);
  CHECK(a.sq_err_e_y == b.sq_err_e_y);
  CHECK(a.min_bumper_gap_m == b.min_bumper_gap_m);
}
