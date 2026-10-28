/**
 * @file logger.h
 * @brief CSV logger: one row per pipeline tick (ground truth vs. estimates).
 */
#ifndef ADAS_LOGGER_H
#define ADAS_LOGGER_H

#include <stdio.h>

#include "pipeline/adas_pipeline.h"
#include "types/control_types.h"
#include "types/world_types.h"

/** Open CSV file handle (NULL when closed). */
typedef struct {
  FILE *fp;
} Logger;

/** Open `path` for writing and emit the CSV header. Returns 0 on success. */
int logger_open(Logger *log, const char *path);

/** Append one row for the current pipeline tick (no-op if not open). */
void logger_write(Logger *log, const WorldState *w, const GroundTruth *gt,
                  const AdasPipeline *p, const ActuatorCmd *cmd);
/**
 * Flush and close the file; safe to call more than once.
 * Returns 0 on success, -1 if any write to the file failed.
 */
int logger_close(Logger *log);

#endif /* ADAS_LOGGER_H */
