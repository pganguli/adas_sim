/**
 * @file sim_runner.c
 * @brief Closed-loop simulation loop.
 *
 * Multi-rate loop (base step SIM_DT_S = 10 ms):
 *
 *   every step (100 Hz)          every PIPELINE_DECIMATION steps (20 Hz)
 *   -------------------          ---------------------------------------
 *   ego-motion sensors           camera + LiDAR sensor models
 *   longitudinal control (PID)   ADAS pipeline DAG (perception -> planning)
 *   lateral control (Stanley)    logging / metrics
 *   plant + world update
 *
 * Each plan enters a delay line and reaches the controllers exactly
 * SENSOR_LATENCY_MS + the pipeline's critical-path latency after its sensor
 * capture. Plan arrival is an event: when a plan arrives inside a step, the
 * plant is integrated up to the arrival time with the old command, and the
 * controllers recompute the command with the new plan for the rest of the
 * step. So latency changes smaller than SIM_DT_S still reach the plant. The
 * controllers hold the newest released plan (zero-order hold).
 */
#include "app/sim_runner.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/logger.h"
#include "app/metrics.h"
#include "common/rng.h"
#include "control/lateral_controller.h"
#include "control/longitudinal_controller.h"
#include "params.h"
#include "pipeline/adas_pipeline.h"
#include "pipeline/plan_delay.h"
#include "pipeline/scheduler.h"
#include "planning/behavior_planner.h"
#include "sim/camera_model.h"
#include "sim/lidar_model.h"
#include "sim/scenario.h"
#include "sim/vehicle_model.h"
#include "sim/world.h"
#include "types/control_types.h"
#include "types/planning_types.h"
#include "types/sensor_types.h"
#include "types/world_types.h"

/* Large objects are static so they don't live on the stack. */
static AdasPipeline g_pipeline;
static SensorFrame g_sensors;

/** Plan arrivals closer than this to a segment start are applied there
 *  (avoids sub-microsecond plant segments). */
#define PLAN_ARRIVAL_EPS_S 1e-6

/** Start time of sim step `step` (world.t_s also accumulates float steps). */
static double step_time_s(int step) { return (double)step * (double)SIM_DT_S; }

/**
 * Low-level control + plant over one segment [t, t + dt) of a sim step:
 * the controllers compute a command from `plan` and the plant integrates it.
 */
static void control_and_step(WorldState *world, const ScenarioConfig *sc,
                             LongitudinalController *lon_ctrl,
                             const PlanOutput *plan, const EgoMotion *ego_meas,
                             float dt, ActuatorCmd *cmd) {
  longitudinal_controller_step(lon_ctrl, plan->a_des_mps2, ego_meas, dt, cmd);
  lateral_controller_step(&plan->lat, ego_meas->v_mps, cmd);
  world_step(world, sc, cmd, dt);
}

/** Print one console status line (ground truth next to estimates). */
static void print_status(FILE *fp, const WorldState *w, const GroundTruth *gt,
                         const AdasPipeline *p) {
  const PipelineData *d = &p->data;
  fprintf(fp,
          "t=%5.1fs | %-6s | v=%5.1f m/s | gap true %6.1f est %6.1f m%s | "
          "e_y true %+5.2f est %+5.2f m | steer %+6.3f rad\n",
          w->t_s, lon_mode_name(d->plan.mode), (double)w->ego.v_mps,
          (double)gt->gap_m, (double)d->lead.d_m,
          d->lead.valid ? "" : " (no lead)", (double)gt->e_y_m,
          (double)d->lane.e_y_m, (double)w->ego.delta_rad);
}

int sim_run(const ScenarioConfig *sc, const SimOptions *opt,
            RunMetrics *metrics) {
  WorldState world;
  Rng rng_cam;
  Rng rng_lidar;
  Rng rng_ego;
  LongitudinalController lon_ctrl;
  PlanDelay plan_delay;
  ActuatorCmd cmd = {0.0f, 0.0f, 0.0f};
  Logger logger = {NULL};
  uint32_t seq = 0u;

  /* ---- Initialization ---- */
  world_init(&world, sc);
  rng_seed(&rng_cam, RNG_SEED + 1u);
  rng_seed(&rng_lidar, RNG_SEED + 2u);
  rng_seed(&rng_ego, RNG_SEED + 3u);
  longitudinal_controller_init(&lon_ctrl);
  plan_delay_init(&plan_delay);
  metrics_init(metrics);

  if (adas_pipeline_init(&g_pipeline) != 0) {
    fprintf(stderr, "pipeline init failed\n");
    return 1;
  }
  if (opt->dnn != NULL) {
    memcpy(g_pipeline.data.dnn, opt->dnn, sizeof(g_pipeline.data.dnn));
  }
  if (opt->log_path != NULL && logger_open(&logger, opt->log_path) != 0) {
    return 1;
  }

  if (opt->console != NULL) {
    fprintf(opt->console, "Scenario: %s (%.0f s)\n", sc->name,
            (double)sc->duration_s);
    scheduler_print(&g_pipeline.sched, opt->console);
    adas_pipeline_print_dnn(&g_pipeline, opt->console);
    fprintf(opt->console, "\n");
  }

  /* ---- Main loop ---- */
  const int n_steps = (int)(sc->duration_s / SIM_DT_S + 0.5f);
  const int print_every = (int)(CONSOLE_PRINT_PERIOD_S / SIM_DT_S + 0.5f);

  for (int step = 0; step < n_steps; ++step) {
    EgoMotion ego_meas;
    vehicle_model_measure(&world.ego, &rng_ego, world.t_s, &ego_meas);

    /* 20 Hz: sense -> perceive -> estimate -> plan */
    if (step % PIPELINE_DECIMATION == 0) {
      seq++;
      camera_model_render(&world, &rng_cam, seq, &g_sensors.camera);
      lidar_model_scan(&world, &rng_lidar, seq, &g_sensors.lidar);
      g_sensors.ego = ego_meas;
      g_sensors.ego.hdr.seq = seq;

      adas_pipeline_run(&g_pipeline, &g_sensors, &sc->driver);

      const float latency_ms =
          LATENCY_INJECTION ? SENSOR_LATENCY_MS + g_pipeline.stats.makespan_ms
                            : 0.0f;
      const float delay_ms = latency_ms;
      plan_delay_push(&plan_delay, &g_pipeline.data.plan,
                      step_time_s(step) + 1e-3 * (double)delay_ms);

      const GroundTruth gt = world_ground_truth(&world);
      logger_write(&logger, &world, &gt, &g_pipeline, &cmd, delay_ms);
      metrics_update(metrics, &gt, &g_pipeline, delay_ms);
      if (opt->console != NULL && step % print_every == 0) {
        print_status(opt->console, &world, &gt, &g_pipeline);
      }
      if (metrics->collision) {
        if (opt->console != NULL) {
          fprintf(opt->console,
                  "!!! COLLISION at t=%.2f s, stopping simulation\n",
                  world.t_s);
        }
        break;
      }
    }

    /* 100 Hz: low-level control -> actuators -> plant, split at every plan
     * arrival inside the step. */
    const double t_end = step_time_s(step + 1);
    double t = step_time_s(step);
    const PlanOutput *plan = plan_delay_poll(&plan_delay, t);
    double due = 0.0;
    while (plan_delay_next_due(&plan_delay, &due) &&
           due < t_end - PLAN_ARRIVAL_EPS_S) {
      if (due > t + PLAN_ARRIVAL_EPS_S) {
        control_and_step(&world, sc, &lon_ctrl, plan, &ego_meas,
                         (float)(due - t), &cmd);
        t = due;
      }
      plan = plan_delay_poll(&plan_delay, due);
    }
    control_and_step(&world, sc, &lon_ctrl, plan, &ego_meas, (float)(t_end - t),
                     &cmd);
  }

  return (logger_close(&logger) != 0) ? 1 : 0;
}
