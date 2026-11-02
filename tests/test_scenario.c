/**
 * @file test_scenario.c
 * @brief Integration tests: run the scenarios closed loop and check the KPIs.
 */
#include <math.h>

#include "app/metrics.h"
#include "app/sim_runner.h"
#include "nn/dynamic.h"
#include "params.h"
#include "pipeline/adas_pipeline.h"
#include "sim/scenario.h"
#include "test_util.h"
#include "types/planning_types.h"

/** Detailed checks on the default (reference) scenario. */
static void test_highway_stop_and_go(void) {
  const ScenarioConfig *sc = scenario_get_by_id(SCENARIO_HIGHWAY_STOP_AND_GO);
  const SimOptions quiet = {NULL, NULL, NULL};
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
  CHECK(tick_stat_mean(&a.makespan_ms, a.n_ticks) <
        tick_stat_mean(&a.serial_ms, a.n_ticks));
  CHECK(a.makespan_ms.max < a.serial_ms.min);

  /* The injected delay is the latency itself, not rounded to sim steps. */
  if (LATENCY_INJECTION) {
    CHECK_NEAR(a.plan_delay_ms.min, SENSOR_LATENCY_MS + a.makespan_ms.min,
               1e-4);
    CHECK_NEAR(a.plan_delay_ms.max, SENSOR_LATENCY_MS + a.makespan_ms.max,
               1e-4);
  } else {
    CHECK(a.plan_delay_ms.max == 0.0f);
  }

  /* Determinism: a second run reproduces the first bit for bit. */
  CHECK(sim_run(sc, &quiet, &b) == 0);
  CHECK(a.n_ticks == b.n_ticks);
  CHECK(a.sq_err_gap == b.sq_err_gap);
  CHECK(a.sq_err_e_y == b.sq_err_e_y);
  CHECK(a.min_bumper_gap_m == b.min_bumper_gap_m);
}

/** Every scenario runs to completion safely and hits its intended mode. */
static void test_all_scenarios(void) {
  const SimOptions quiet = {NULL, NULL, NULL};
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

/** Run the reference scenario with every DNN set to `variant` at `quality`. */
static void run_variant(DnnVariant variant, float quality, RunMetrics *m) {
  DnnConfig dnn[NODE_COUNT];
  for (int i = 0; i < NODE_COUNT; ++i) {
    dnn[i].variant = variant;
    dnn[i].quality = quality;
  }
  const SimOptions opt = {NULL, NULL, dnn};
  CHECK(sim_run(scenario_get_by_id(SCENARIO_HIGHWAY_STOP_AND_GO), &opt, m) ==
        0);
}

/** Static and dynamic DNNs both drive the loop; dynamic ones cut the delay. */
static void test_static_vs_dynamic(void) {
  RunMetrics st;
  RunMetrics dy;
  run_variant(DNN_STATIC, 1.0f, &st);
  run_variant(DNN_DYNAMIC, 0.75f, &dy);

  const RunMetrics *runs[2] = {&st, &dy};
  for (int i = 0; i < 2; ++i) {
    const RunMetrics *m = runs[i];
    CHECK(!m->collision);
    CHECK(m->min_bumper_gap_m > 2.0f);
    CHECK((double)m->n_lane_valid / m->n_ticks > 0.99);
    CHECK(sqrt(m->sq_err_e_y / m->n_lane_valid) < 0.1);
    CHECK(sqrt(m->sq_err_gap / m->n_lead_valid) < 0.25);
  }
  /* The shorter pipeline latency reaches the controllers (it is not
   * swallowed by rounding to sim steps), so the closed loop differs. */
  CHECK(dy.makespan_ms.max < st.makespan_ms.min);
  CHECK(dy.plan_delay_ms.max < st.plan_delay_ms.min);
  for (int i = 0; i < NODE_COUNT; ++i) {
    if (!adas_pipeline_is_dnn(i)) {
      continue;
    }
    /* Static nets always run in full; the dynamic run records its quality. */
    CHECK(st.dynamic_ticks[i] == 0 && dy.dynamic_ticks[i] == dy.n_ticks);
    CHECK(st.cost[i].min == 1.0f && st.cost[i].max == 1.0f);
    CHECK_NEAR(tick_stat_mean(&dy.quality[i], dy.n_ticks), 0.75f, 1e-6);
  }
  CHECK(dy.sq_err_e_y_true != st.sq_err_e_y_true);
}

void test_scenario(void) {
  test_highway_stop_and_go();
  test_static_vs_dynamic();
  test_all_scenarios();
}
