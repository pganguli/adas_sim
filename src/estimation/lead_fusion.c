/**
 * @file lead_fusion.c
 * @brief LeadFusion implementation (2-state linear Kalman filter).
 */
#include "estimation/lead_fusion.h"

#include <math.h>
#include <string.h>

#include "common/math_utils.h"
#include "params.h"
#include "types/estimation_types.h"
#include "types/perception_types.h"

void lead_fusion_init(LeadFusion *f) { memset(f, 0, sizeof(*f)); }

/* ---------------------------------------------------------------------- */
/* Measurement selection                                                  */
/* ---------------------------------------------------------------------- */

/** True if (x, y) lies inside the ego lane (straight lane if no valid fit). */
static bool is_in_path(const LaneEstimate *lane, float x, float y) {
  const float y_center =
      lane->valid ? lane->c0 + lane->c1 * x + lane->c2 * x * x : 0.0f;
  return fabsf(y - y_center) < 0.5f * LANE_WIDTH_NOMINAL_M;
}

/** Closest in-path LiDAR box. Returns false if none. */
static bool select_lidar(const Detections3D *dets, const LaneEstimate *lane,
                         float *d, float *y) {
  bool found = false;
  for (int i = 0; i < dets->n; ++i) {
    const BBox3D *b = &dets->boxes[i];
    const float rear_x = b->center.x - 0.5f * b->size.x;
    if (is_in_path(lane, rear_x, b->center.y) && (!found || rear_x < *d)) {
      *d = rear_x;
      *y = b->center.y;
      found = true;
    }
  }
  return found;
}

/** Closest in-path camera object. Returns false if none. */
static bool select_camera(const DepthEstimates *dets, const LaneEstimate *lane,
                          float *d, float *y) {
  bool found = false;
  for (int i = 0; i < dets->n; ++i) {
    const ObjectDepth *o = &dets->objs[i];
    if (is_in_path(lane, o->range_m, o->lateral_m) &&
        (!found || o->range_m < *d)) {
      *d = o->range_m;
      *y = o->lateral_m;
      found = true;
    }
  }
  return found;
}

/* ---------------------------------------------------------------------- */
/* Kalman filter                                                          */
/* ---------------------------------------------------------------------- */

/** Time update with a constant-velocity model. */
static void kf_predict(LeadFusion *f, float dt) {
  const float q = sqf(FUSION_ACCEL_NOISE_STD);
  float (*P)[2] = f->P;

  f->x[0] += dt * f->x[1];

  /* P = F P F^T + Q, with Q for white-noise acceleration */
  const float p00 = P[0][0] + dt * (P[0][1] + P[1][0]) + dt * dt * P[1][1];
  const float p01 = P[0][1] + dt * P[1][1];
  const float p10 = P[1][0] + dt * P[1][1];
  P[0][0] = p00 + q * dt * dt * dt * dt / 4.0f;
  P[0][1] = p01 + q * dt * dt * dt / 2.0f;
  P[1][0] = p10 + q * dt * dt * dt / 2.0f;
  P[1][1] = P[1][1] + q * dt * dt;
}

/** Scalar range update with chi-square gating. Returns true if applied. */
static bool kf_update_range(LeadFusion *f, float z, float r_std) {
  float (*P)[2] = f->P;
  const float S = P[0][0] + r_std * r_std;
  const float innov = z - f->x[0];
  if (innov * innov / S > FUSION_GATE_CHI2) {
    return false; /* outlier */
  }

  const float K0 = P[0][0] / S;
  const float K1 = P[1][0] / S;
  f->x[0] += K0 * innov;
  f->x[1] += K1 * innov;

  /* P = (I - K H) P */
  const float p00 = P[0][0];
  const float p01 = P[0][1];
  P[0][0] = (1.0f - K0) * p00;
  P[0][1] = (1.0f - K0) * p01;
  P[1][0] -= K1 * p00;
  P[1][1] -= K1 * p01;
  return true;
}

/** Start a new track from a first range measurement `d` at lateral `y`. */
static void kf_start(LeadFusion *f, float d, float r_std, float y) {
  f->has_track = true;
  f->x[0] = d;
  f->x[1] = 0.0f;
  f->P[0][0] = r_std * r_std;
  f->P[0][1] = 0.0f;
  f->P[1][0] = 0.0f;
  f->P[1][1] = sqf(FUSION_INIT_VREL_STD);
  f->y_m = y;
  f->hits = 1;
  f->misses = 0;
}

/**
 * Re-seed the state from LiDAR while keeping the track (and its confirmation).
 * Relative speed is re-estimated by differencing consecutive LiDAR ranges.
 */
static void kf_restart_from_lidar(LeadFusion *f, float d_lidar, float dt) {
  if (f->has_last_lidar) {
    f->x[1] = (d_lidar - f->last_lidar_d_m) / dt;
  }
  f->x[0] = d_lidar;
  f->P[0][0] = sqf(FUSION_LIDAR_STD_M);
  f->P[0][1] = 0.0f;
  f->P[1][0] = 0.0f;
  f->P[1][1] = sqf(FUSION_INIT_VREL_STD);
  f->lidar_rejects = 0;
}

/** Camera range standard deviation, growing linearly with range. */
static float camera_std(float range_m) {
  return FUSION_CAM_STD_BASE_M + FUSION_CAM_STD_PER_M * range_m;
}

/* ---------------------------------------------------------------------- */
/* Public API                                                             */
/* ---------------------------------------------------------------------- */

void lead_fusion_update(LeadFusion *f, const Detections3D *lidar_dets,
                        const DepthEstimates *camera_dets,
                        const LaneEstimate *lane, float dt, LeadTrack *out) {
  float d_lidar = 0.0f;
  float y_lidar = 0.0f;
  float d_cam = 0.0f;
  float y_cam = 0.0f;
  const bool has_lidar = select_lidar(lidar_dets, lane, &d_lidar, &y_lidar);
  const bool has_cam = select_camera(camera_dets, lane, &d_cam, &y_cam);
  bool used_lidar = false;
  bool used_cam = false;

  if (!f->has_track) {
    /* Track initiation: prefer the more accurate sensor. */
    if (has_lidar) {
      kf_start(f, d_lidar, FUSION_LIDAR_STD_M, y_lidar);
      used_lidar = true;
    } else if (has_cam) {
      kf_start(f, d_cam, camera_std(d_cam), y_cam);
      used_cam = true;
    }
  } else {
    kf_predict(f, dt);
    if (has_lidar) {
      if (kf_update_range(f, d_lidar, FUSION_LIDAR_STD_M)) {
        used_lidar = true;
        f->lidar_rejects = 0;
      } else if (++f->lidar_rejects >= FUSION_LIDAR_REJECTS_REINIT) {
        /* The accurate sensor keeps disagreeing: the filter has
         * diverged (e.g. lead braked harder than the motion model
         * expects). Restart the state from LiDAR. */
        kf_restart_from_lidar(f, d_lidar, dt);
        used_lidar = true;
      }
      if (used_lidar) {
        f->y_m = y_lidar;
      }
    }
    if (has_cam && kf_update_range(f, d_cam, camera_std(d_cam))) {
      used_cam = true;
      if (!used_lidar) {
        f->y_m = y_cam;
      }
    }

    /* Track maintenance */
    if (used_lidar || used_cam) {
      f->hits++;
      f->misses = 0;
    } else if (++f->misses > FUSION_MAX_MISSES) {
      f->has_track = false;
    }
  }

  f->has_last_lidar = has_lidar;
  f->last_lidar_d_m = d_lidar;

  /* ---- Output message ---- */
  out->hdr = lidar_dets->hdr;
  out->valid = f->has_track && f->hits >= FUSION_CONFIRM_HITS;
  out->d_m = f->has_track ? f->x[0] : 0.0f;
  out->v_rel_mps = f->has_track ? f->x[1] : 0.0f;
  out->y_m = f->y_m;
  out->d_std_m = f->has_track ? sqrtf(f->P[0][0]) : 0.0f;
  out->lidar_used = used_lidar;
  out->camera_used = used_cam;
}
