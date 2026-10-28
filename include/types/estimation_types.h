/**
 * @file estimation_types.h
 * @brief State-estimation outputs consumed by planning (LKA and ACC states).
 */
#ifndef ADAS_ESTIMATION_TYPES_H
#define ADAS_ESTIMATION_TYPES_H

#include "params.h"
#include "types/common_types.h"

/**
 * Ego-lane geometry. The lane center line in the ego frame is
 *     y_c(x) = c0 + c1 * x + c2 * x^2
 * and the classic LKA state is derived from it at the reference point:
 *     e_y   = -c0         lateral offset of the vehicle from lane center (left +)
 *     e_psi = -atan(c1)   heading of the vehicle relative to the lane    (CCW +)
 *     kappa =  2 * c2     lane curvature                                 (left +)
 */
typedef struct {
  MsgHeader hdr;
  bool valid;       /**< polynomial fit is usable                 */
  float c0, c1, c2; /**< center-line polynomial coefficients      */
  float e_y_m;      /**< lateral offset from lane center (left +) */
  float e_psi_rad;  /**< heading relative to the lane (CCW +)     */
  float kappa_1pm;  /**< lane curvature (left +)                  */
} LaneEstimate;

/** Lateral position y_c(x) of the lane center line at forward distance x. */
static inline float lane_center_y(const LaneEstimate *lane, float x) {
  return lane->c0 + lane->c1 * x + lane->c2 * x * x;
}

/** Camera-only range estimate for one 2D detection (output of MonoDepthNet). */
typedef struct {
  float range_m;   /**< forward distance to the object's rear face */
  float lateral_m; /**< lateral offset of the box center (left +)  */
} ObjectDepth;

/** Output of MonoDepthNet: one range estimate per rangeable 2D detection. */
typedef struct {
  MsgHeader hdr;
  int n; /**< number of valid entries in objs */
  ObjectDepth objs[DET2D_MAX_BOXES];
} DepthEstimates;

/** Fused in-path lead vehicle (output of LeadFusion; the ACC state). */
typedef struct {
  MsgHeader hdr;
  bool valid;       /**< confirmed track exists                  */
  float d_m;        /**< gap: ref point -> lead rear face        */
  float v_rel_mps;  /**< relative speed, v_lead - v_ego          */
  float y_m;        /**< lateral position of the lead (left +)   */
  float d_std_m;    /**< 1-sigma uncertainty of d                */
  bool lidar_used;  /**< a LiDAR measurement updated this cycle  */
  bool camera_used; /**< a camera measurement updated this cycle */
} LeadTrack;

#endif /* ADAS_ESTIMATION_TYPES_H */
