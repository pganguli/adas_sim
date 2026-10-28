/**
 * @file test_scenario.c
 * @brief Integration tests: run the scenarios closed loop and check the KPIs.
 */
#include <math.h>

#include "app/metrics.h"
#include "app/sim_runner.h"
#include "params.h"
#include "sim/scenario.h"
#include "test_util.h"
#include "types/planning_types.h"

/** Detailed checks on the default (reference) scenario. */
static void test_highway_stop_and_go(void) {
  const ScenarioConfig *sc = scenario_get_by_id(SCENARIO_HIGHWAY_STOP_AND_GO);
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

  /* The injected delay is the latency rounded up to whole sim steps. */
  const float step_ms = SIM_DT_MS;
  const float latency_ms =
      LATENCY_INJECTION ? SENSOR_LATENCY_MS + a.makespan_ms : 0.0f;
  CHECK(a.plan_delay_ms >= latency_ms - 1e-3f);
  CHECK(a.plan_delay_ms < latency_ms + step_ms);
  CHECK_NEAR(fmodf(a.plan_delay_ms, step_ms), 0.0f, 1e-3);

  /* Determinism: a second run reproduces the first bit for bit. */
  CHECK(sim_run(sc, &quiet, &b) == 0);
  CHECK(a.n_ticks == b.n_ticks);
  CHECK(a.sq_err_gap == b.sq_err_gap);
  CHECK(a.sq_err_e_y == b.sq_err_e_y);
  CHECK(a.min_bumper_gap_m == b.min_bumper_gap_m);
}

/** Every scenario runs to completion safely and hits its intended mode. */
static void test_all_scenarios(void) {
  const SimOptions quiet = {NULL, NULL};
  RunMetrics m[SCENARIO_COUNT];

  CHECK(scenario_get_by_id(SCENARIO_COUNT) == NULL);
  CHECK(scenario_get() == scenario_get_by_id(SCENARIO_SELECTED));

  for (int i = 0; i < SCENARIO_COUNT; ++i) {
    const ScenarioConfig *sc = scenario_get_by_id((ScenarioId)i);

    CHECK(sim_run(sc, &quiet, &m[i]) == 0);
    if (m[i].collision) {
      TEST_FAIL(sc->name);
    }
    CHECK(m[i].n_ticks == (int)(sc->duration_s / PIPELINE_DT_S + 0.5f));
    /* A lead stopped close ahead hides the far lane markings (the end of
     * lead_hard_brake), which invalidates the lane fit at standstill. */
    CHECK((double)m[i].n_lane_valid / m[i].n_ticks > 0.7);
    CHECK(sqrt(m[i].sq_err_e_y_true / m[i].n_ticks) < 0.25);
  }

  /* Free road: no lead, so no (ghost) track and pure cruise. */
  const RunMetrics *free_road = &m[SCENARIO_FREE_ROAD_CURVE];
  CHECK(free_road->n_lead_valid == 0);
  CHECK(free_road->mode_ticks[LON_MODE_CRUISE] == free_road->n_ticks);
  CHECK(isinf(free_road->min_bumper_gap_m));

  /* Cut-in: the lead is ignored in the next lane, then followed. */
  const RunMetrics *cut_in = &m[SCENARIO_CUT_IN];
  CHECK(cut_in->mode_ticks[LON_MODE_CRUISE] > 0);
  CHECK(cut_in->mode_ticks[LON_MODE_FOLLOW] > 0);
  CHECK(cut_in->min_bumper_gap_m > 2.0f);

  /* Hard brake: emergency braking is needed and suffices. */
  const RunMetrics *hard_brake = &m[SCENARIO_LEAD_HARD_BRAKE];
  CHECK(hard_brake->mode_ticks[LON_MODE_EMERGENCY_BRAKE] > 0);
  CHECK(hard_brake->min_bumper_gap_m > 0.0f);
}

void test_scenario(void) {
  test_highway_stop_and_go();
  test_all_scenarios();
}
