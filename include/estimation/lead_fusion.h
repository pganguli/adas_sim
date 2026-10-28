/**
 * @file lead_fusion.h
 * @brief Camera + LiDAR fusion of the in-path lead vehicle (ACC state).
 *
 * State x = [d, v_rel]^T, constant-velocity Kalman filter:
 *   predict : x = F x,  P = F P F^T + Q,   F = [1 dt; 0 1]
 *   update  : z = d (LiDAR box rear face, then camera mono-depth), H = [1 0]
 * Measurement selection ("in-path target selection"): the closest detection
 * whose lateral position lies inside the ego lane, using the lane polynomial
 * from LaneGeometryNet (falls back to a straight lane if it is invalid).
 * Divergence guard: if LiDAR is gated out several cycles in a row, the state
 * is re-seeded from LiDAR (the track and its confirmation are kept).
 */
#ifndef ADAS_LEAD_FUSION_H
#define ADAS_LEAD_FUSION_H

#include "types/estimation_types.h"
#include "types/perception_types.h"

/** Kalman filter state and track bookkeeping. */
typedef struct {
  bool has_track;       /**< a (possibly unconfirmed) track exists    */
  float x[2];           /**< [d, v_rel]                               */
  float P[2][2];        /**< state covariance                         */
  float y_m;            /**< last associated lateral position         */
  int hits;             /**< cycles with at least one measurement     */
  int misses;           /**< consecutive cycles without measurement   */
  int lidar_rejects;    /**< consecutive gated-out LiDAR measurements */
  bool has_last_lidar;  /**< LiDAR measurement present last cycle     */
  float last_lidar_d_m; /**< previous cycle's LiDAR range             */
} LeadFusion;

/** Drop any track and clear the state. */
void lead_fusion_init(LeadFusion *f);

/**
 * Run one predict / select / update cycle and publish the lead track.
 *
 * @param f           filter state (updated)
 * @param lidar_dets  3D detections (preferred, more accurate)
 * @param camera_dets camera mono-depth estimates
 * @param lane        ego-lane estimate used for in-path selection
 * @param dt          time since the previous cycle [s]
 * @param out         fused lead track
 */
void lead_fusion_update(LeadFusion *f, const Detections3D *lidar_dets,
                        const DepthEstimates *camera_dets,
                        const LaneEstimate *lane, float dt, LeadTrack *out);

#endif /* ADAS_LEAD_FUSION_H */
