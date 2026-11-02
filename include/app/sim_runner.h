/**
 * @file sim_runner.h
 * @brief Runs one closed-loop scenario (the multi-rate simulation loop).
 *
 * Kept separate from main() so tests can run scenarios and inspect metrics.
 * The pipeline and sensor buffers are large and live in static storage, so
 * sim_run() is not reentrant.
 */
#ifndef ADAS_SIM_RUNNER_H
#define ADAS_SIM_RUNNER_H

#include <stdio.h>

#include "app/metrics.h"
#include "nn/dynamic.h"
#include "sim/scenario.h"

/** Options of a run. */
typedef struct {
  const char *log_path; /**< CSV log file, or NULL for no log      */
  FILE *console;        /**< status lines go here, or NULL (quiet) */
  /** Per-node DNN selection (NODE_COUNT entries, indexed by
   *  PipelineNodeId), or NULL for the params.h defaults. */
  const DnnConfig *dnn;
} SimOptions;

/**
 * Run `sc` to completion (or until a collision) and fill `metrics`.
 * Returns 0 on success, 1 if setup or writing the log failed.
 */
int sim_run(const ScenarioConfig *sc, const SimOptions *opt,
            RunMetrics *metrics);

#endif /* ADAS_SIM_RUNNER_H */
