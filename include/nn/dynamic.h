/**
 * @file dynamic.h
 * @brief Shared definitions for dynamic (accuracy/latency-scalable) DNNs.
 *
 * Every DNN of the pipeline has a static variant (`xxx_infer`, always the
 * full network) and a dynamic one (`xxx_infer_dynamic`) that takes an extra
 * `quality` input in (0, 1]:
 *
 *   1.0  full accuracy: the network does everything the static one does
 *   -> 0 as little work as possible (fewer rows, channels, anchors, ...)
 *
 * A dynamic variant returns its executed multiply-accumulates (including
 * its own gate and bypass layers) relative to the full static network. Some
 * variants are input dependent (per-pixel / per-point early exits), so the
 * ratio differs from frame to frame. It can exceed 1 when the gates cost
 * more than they save (e.g. near quality 1), so a dynamic net is never
 * reported as cheaper than it is. The pipeline scales the node's nominal
 * latency by it.
 */
#ifndef ADAS_DYNAMIC_H
#define ADAS_DYNAMIC_H

#include <math.h>

/** Lowest quality a dynamic network accepts (0 itself is not a valid input). */
#define DYN_QUALITY_MIN 0.01f

/** Which implementation of a DNN the pipeline runs. */
typedef enum {
  DNN_STATIC = 0, /**< fixed full network           */
  DNN_DYNAMIC     /**< quality-scalable variant     */
} DnnVariant;

/** Runtime selection for one DNN node. */
typedef struct {
  DnnVariant variant; /**< implementation to run                 */
  float quality;      /**< (0, 1], only used by DNN_DYNAMIC      */
} DnnConfig;

/** Clamp a quality request into [DYN_QUALITY_MIN, 1]. */
static inline float dyn_clamp_quality(float quality) {
  if (!(quality > DYN_QUALITY_MIN)) { /* also catches NaN */
    return DYN_QUALITY_MIN;
  }
  return (quality > 1.0f) ? 1.0f : quality;
}

/** Number of the `n` units (channels, anchors, ...) to keep: ceil(q * n) in [1, n]. */
static inline int dyn_keep_count(float quality, int n) {
  const int k = (int)ceilf(dyn_clamp_quality(quality) * (float)n - 1e-4f);
  if (k < 1) {
    return 1;
  }
  return (k > n) ? n : k;
}

/** Executed / full multiply-accumulates (>= 0, may exceed 1). */
static inline float dyn_cost(long macs, long full_macs) {
  return (full_macs > 0) ? (float)macs / (float)full_macs : 1.0f;
}

#endif /* ADAS_DYNAMIC_H */
