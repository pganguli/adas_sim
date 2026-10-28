/**
 * @file metrics.c
 * @brief KPI accumulation and report.
 */
#include "app/metrics.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "params.h"
#include "pipeline/adas_pipeline.h"
#include "planning/behavior_planner.h"
#include "types/planning_types.h"
#include "types/world_types.h"

void metrics_init(RunMetrics *m) {
  memset(m, 0, sizeof(*m));
  m->min_bumper_gap_m = 1e9f;
}

void metrics_update(RunMetrics *m, const GroundTruth *gt,
                    const AdasPipeline *p) {
  const PipelineData *d = &p->data;
  const float bumper_gap = gt->gap_m - VEH_FRONT_OVERHANG_M;

  m->n_ticks++;
  m->mode_ticks[d->plan.mode]++;
  m->makespan_ms = p->stats.makespan_ms;
  m->serial_ms = p->stats.serial_ms;
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
  if (gt->gap_m > 0.0f && bumper_gap < m->min_bumper_gap_m) {
    m->min_bumper_gap_m = bumper_gap;
  }
  if (gt->gap_m > 0.0f && bumper_gap < 0.0f &&
      fabsf(gt->lead_y_m) < VEHICLE_WIDTH_M) {
    m->collision = true;
  }
}

/** Root-mean-square from a sum of squares (0 when n == 0). */
static double rms(double sum_sq, int n) {
  return (n > 0) ? sqrt(sum_sq / n) : 0.0;
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
  fprintf(fp, "Min bumper-to-bumper gap : %.2f m\n",
          (double)m->min_bumper_gap_m);
  fprintf(fp, "Collision                : %s\n", m->collision ? "YES" : "no");
  fprintf(fp, "Time per mode            :");
  for (int i = 0; i < LON_MODE_COUNT; ++i) {
    fprintf(fp, " %s %.1fs", lon_mode_name((LongitudinalMode)i),
            (double)m->mode_ticks[i] * (double)PIPELINE_DT_S);
  }
  fprintf(fp,
          "\nPipeline latency         : %.1f ms with parallel stages vs %.1f "
          "ms serial\n",
          (double)m->makespan_ms, (double)m->serial_ms);
  fprintf(fp, "=====================================================\n");
}
