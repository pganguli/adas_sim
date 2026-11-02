/**
 * @file metrics.c
 * @brief KPI accumulation and report.
 */
#include "app/metrics.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "nn/dynamic.h"
#include "params.h"
#include "pipeline/adas_pipeline.h"
#include "planning/behavior_planner.h"
#include "types/planning_types.h"
#include "types/world_types.h"

/** Empty statistic (min/max are set by the first sample). */
static void tick_stat_init(TickStat *s) {
  s->sum = 0.0;
  s->min = INFINITY;
  s->max = -INFINITY;
}

static void tick_stat_add(TickStat *s, float x) {
  s->sum += (double)x;
  s->min = fminf(s->min, x);
  s->max = fmaxf(s->max, x);
}

void metrics_init(RunMetrics *m) {
  memset(m, 0, sizeof(*m));
  m->min_bumper_gap_m = INFINITY;
  tick_stat_init(&m->makespan_ms);
  tick_stat_init(&m->serial_ms);
  tick_stat_init(&m->plan_delay_ms);
  for (int i = 0; i < NODE_COUNT; ++i) {
    tick_stat_init(&m->quality[i]);
    tick_stat_init(&m->cost[i]);
  }
}

void metrics_update(RunMetrics *m, const GroundTruth *gt, const AdasPipeline *p,
                    float delay_ms) {
  const PipelineData *d = &p->data;
  const float bumper_gap = gt->gap_m - VEH_FRONT_OVERHANG_M;

  m->n_ticks++;
  m->mode_ticks[d->plan.mode]++;
  tick_stat_add(&m->makespan_ms, p->stats.makespan_ms);
  tick_stat_add(&m->serial_ms, p->stats.serial_ms);
  tick_stat_add(&m->plan_delay_ms, delay_ms);
  for (int i = 0; i < NODE_COUNT; ++i) {
    tick_stat_add(&m->quality[i], adas_pipeline_quality(p, i));
    tick_stat_add(&m->cost[i], d->cost[i]);
    m->dynamic_ticks[i] += (d->dnn[i].variant == DNN_DYNAMIC) ? 1 : 0;
  }
  m->sq_err_e_y_true += (double)gt->e_y_m * (double)gt->e_y_m;

  if (d->lane.valid) {
    const double e = (double)(d->lane.e_y_m - gt->e_y_m);
    m->sq_err_e_y += e * e;
    m->n_lane_valid++;
  }
  if (d->lead.valid) {
    const double e = (double)(d->lead.d_m - gt->gap_m);
    m->sq_err_gap += e * e;
    m->n_lead_valid++;
  }
  /* Only a lead ahead that overlaps the ego laterally can be hit. */
  if (gt->gap_m > 0.0f && fabsf(gt->lead_y_m) < VEHICLE_WIDTH_M) {
    if (bumper_gap < m->min_bumper_gap_m) {
      m->min_bumper_gap_m = bumper_gap;
    }
    if (bumper_gap < 0.0f) {
      m->collision = true;
    }
  }
}

/** Root-mean-square from a sum of squares (0 when n == 0). */
static double rms(double sum_sq, int n) {
  return (n > 0) ? sqrt(sum_sq / n) : 0.0;
}

/** Min / max of a statistic, 0 when it has no samples (no inf in output). */
static float stat_min(const TickStat *s, int n) {
  return (n > 0) ? s->min : 0.0f;
}
static float stat_max(const TickStat *s, int n) {
  return (n > 0) ? s->max : 0.0f;
}

/** One "label : mean unit (min..max)" summary line. */
static void print_stat(FILE *fp, const char *label, const TickStat *s, int n,
                       const char *unit) {
  fprintf(fp, "%-25s: %.1f %s mean (%.1f..%.1f)\n", label,
          (double)tick_stat_mean(s, n), unit, (double)stat_min(s, n),
          (double)stat_max(s, n));
}

void metrics_print(const RunMetrics *m, FILE *fp) {
  const double n = (m->n_ticks > 0) ? (double)m->n_ticks : 1.0;

  fprintf(fp, "\n==================== Run summary ====================\n");
  fprintf(fp, "Lane estimate valid      : %5.1f %% of ticks\n",
          100.0 * m->n_lane_valid / n);
  fprintf(fp, "e_y estimation RMS error : %.3f m\n",
          rms(m->sq_err_e_y, m->n_lane_valid));
  fprintf(fp,
          "Lane-keeping RMS |e_y|   : %.3f m (true offset from lane center)\n",
          rms(m->sq_err_e_y_true, m->n_ticks));
  fprintf(fp, "Lead track valid         : %5.1f %% of ticks\n",
          100.0 * m->n_lead_valid / n);
  fprintf(fp, "Gap estimation RMS error : %.3f m\n",
          rms(m->sq_err_gap, m->n_lead_valid));
  if (isfinite(m->min_bumper_gap_m)) {
    fprintf(fp, "Min bumper-to-bumper gap : %.2f m\n",
            (double)m->min_bumper_gap_m);
  } else {
    fprintf(fp, "Min bumper-to-bumper gap : n/a (no lead in path)\n");
  }
  fprintf(fp, "Collision                : %s\n", m->collision ? "YES" : "no");
  fprintf(fp, "Time per mode            :");
  for (int i = 0; i < LON_MODE_COUNT; ++i) {
    fprintf(fp, " %s %.1fs", lon_mode_name((LongitudinalMode)i),
            (double)m->mode_ticks[i] * (double)PIPELINE_DT_S);
  }
  fprintf(fp, "\n");
  print_stat(fp, "Pipeline latency (par.)", &m->makespan_ms, m->n_ticks, "ms");
  print_stat(fp, "Pipeline latency (serial)", &m->serial_ms, m->n_ticks, "ms");
  print_stat(fp, "Injected plan delay", &m->plan_delay_ms, m->n_ticks, "ms");
  fprintf(fp,
          "  (capture -> control: %.0f ms sensor + pipeline, applied at the "
          "exact arrival time)\n",
          LATENCY_INJECTION ? (double)SENSOR_LATENCY_MS : 0.0);
  fprintf(fp, "DNNs (cost = executed MACs / static net MACs; mean, "
              "min..max):\n");
  for (int i = 0; i < NODE_COUNT; ++i) {
    if (!adas_pipeline_is_dnn(i)) {
      continue;
    }
    const char *variant = "mixed";
    if (m->dynamic_ticks[i] == 0) {
      variant = "static";
    } else if (m->dynamic_ticks[i] == m->n_ticks) {
      variant = "dynamic";
    }
    fprintf(fp, "  %-18s %-7s quality %.2f  cost %.2f (%.2f..%.2f)\n",
            adas_pipeline_node_name(i), variant,
            (double)tick_stat_mean(&m->quality[i], m->n_ticks),
            (double)tick_stat_mean(&m->cost[i], m->n_ticks),
            (double)stat_min(&m->cost[i], m->n_ticks),
            (double)stat_max(&m->cost[i], m->n_ticks));
  }
  fprintf(fp, "=====================================================\n");
}
