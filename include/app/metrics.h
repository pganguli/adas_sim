/**
 * @file metrics.h
 * @brief End-of-run KPIs: estimation accuracy, safety, mode usage, latency.
 */
#ifndef ADAS_METRICS_H
#define ADAS_METRICS_H

#include <stdbool.h>
#include <stdio.h>

#include "pipeline/adas_pipeline.h"
#include "types/world_types.h"

/** Accumulated statistics over one simulation run. */
typedef struct {
  int n_ticks;                    /**< pipeline ticks processed           */
  int n_lane_valid;               /**< ticks with a valid lane estimate   */
  double sq_err_e_y;              /**< sum of (e_y_est - e_y_true)^2      */
  double sq_err_e_y_true;         /**< sum of e_y_true^2 (lane keeping)   */
  int n_lead_valid;               /**< ticks with a confirmed lead track  */
  double sq_err_gap;              /**< sum of (gap_est - gap_true)^2      */
  float min_bumper_gap_m;         /**< smallest bumper-to-bumper gap      */
  bool collision;                 /**< ego reached the lead's rear        */
  int mode_ticks[LON_MODE_COUNT]; /**< ticks spent in each mode           */
  float makespan_ms;              /**< pipeline latency, parallel stages  */
  float serial_ms;                /**< pipeline latency, serial execution */
} RunMetrics;

/** Reset all counters. */
void metrics_init(RunMetrics *m);

/** Fold one pipeline tick into the statistics. */
void metrics_update(RunMetrics *m, const GroundTruth *gt,
                    const AdasPipeline *p);
/** Print the end-of-run summary table to `fp`. */
void metrics_print(const RunMetrics *m, FILE *fp);

#endif /* ADAS_METRICS_H */
