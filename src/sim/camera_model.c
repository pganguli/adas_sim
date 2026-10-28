/**
 * @file camera_model.c
 * @brief Renders a grayscale image by ray-casting each pixel onto the road.
 *
 * Scene: bright sky above the horizon, gray road below, bright lane markings
 * (anti-aliased by pixel coverage), dark lead-vehicle rear face, plus noise.
 */
#include "sim/camera_model.h"

#include <math.h>
#include <stdint.h>

#include "common/camera_geometry.h"
#include "common/math_utils.h"
#include "common/rng.h"
#include "params.h"
#include "sim/world.h"
#include "types/common_types.h"
#include "types/sensor_types.h"
#include "types/world_types.h"

/** Length of the overlap of intervals [a0, a1] and [b0, b1]. */
static float overlap_1d(float a0, float a1, float b0, float b1) {
  const float lo = fmaxf(a0, b0);
  const float hi = fminf(a1, b1);
  return (hi > lo) ? (hi - lo) : 0.0f;
}

/** Render sky, road and anti-aliased lane markings by ray-casting each row. */
static void render_road(const WorldState *w, CameraFrame *out) {
  const RoadGeometry *road = &w->road;
  const Pose2D ego = w->ego.pose;
  const float c = cosf(ego.psi);
  const float s = sinf(ego.psi);
  const float half_lane = 0.5f * road->lane_width_m;
  const float half_mark = 0.5f * road->marking_width_m;

  for (int v = 0; v < CAM_H; ++v) {
    const float v_c = (float)v + 0.5f;
    if (v_c <= CAM_CV) {
      for (int u = 0; u < CAM_W; ++u) {
        out->px[v][u] = CAM_I_SKY;
      }
      continue;
    }

    const float range = cam_row_to_range(v_c);
    const float px_m = range / CAM_FX; /* lateral footprint of one pixel */

    for (int u = 0; u < CAM_W; ++u) {
      const float lat = cam_col_to_lateral((float)u + 0.5f, range);
      const Vec2 p_world = {ego.x + c * range - s * lat,
                            ego.y + s * range + c * lat};
      const float off = road_project(road, p_world).offset_m;

      /* Fraction of the pixel covered by the left or right marking. */
      const float a0 = off - 0.5f * px_m;
      const float a1 = off + 0.5f * px_m;
      float cov =
          overlap_1d(a0, a1, half_lane - half_mark, half_lane + half_mark) +
          overlap_1d(a0, a1, -half_lane - half_mark, -half_lane + half_mark);
      cov = clampf(cov / px_m, 0.0f, 1.0f);

      out->px[v][u] = CAM_I_ROAD + (CAM_I_MARKING - CAM_I_ROAD) * cov;
    }
  }
}

/** Paint the lead vehicle's rear face as a dark rectangle. */
static void render_lead(const WorldState *w, CameraFrame *out) {
  const Vec2 lead = world_lead_in_ego(w);
  if (lead.x < 1.0f) {
    return; /* behind or inside the camera */
  }

  /* Rear face rectangle (treated as facing the camera). */
  const float u_left =
      cam_lateral_to_col(lead.y + 0.5f * VEHICLE_WIDTH_M, lead.x);
  const float u_right =
      cam_lateral_to_col(lead.y - 0.5f * VEHICLE_WIDTH_M, lead.x);
  const float v_top = cam_point_to_row(lead.x, VEHICLE_HEIGHT_M);
  const float v_bot = cam_point_to_row(lead.x, 0.0f);

  /* Fill every pixel whose center lies inside the rectangle. */
  const int u0 = clampi((int)ceilf(u_left - 0.5f), 0, CAM_W);
  const int u1 = clampi((int)floorf(u_right - 0.5f), -1, CAM_W - 1);
  const int v0 = clampi((int)ceilf(v_top - 0.5f), 0, CAM_H);
  const int v1 = clampi((int)floorf(v_bot - 0.5f), -1, CAM_H - 1);

  for (int v = v0; v <= v1; ++v) {
    for (int u = u0; u <= u1; ++u) {
      out->px[v][u] = CAM_I_VEHICLE;
    }
  }
}

void camera_model_render(const WorldState *w, Rng *rng, uint32_t seq,
                         CameraFrame *out) {
  out->hdr.seq = seq;
  out->hdr.stamp_s = w->t_s;

  render_road(w, out);
  render_lead(w, out);

  for (int v = 0; v < CAM_H; ++v) {
    for (int u = 0; u < CAM_W; ++u) {
      out->px[v][u] =
          clampf(out->px[v][u] + rng_gaussian(rng, CAM_NOISE_STD), 0.0f, 1.0f);
    }
  }
}
