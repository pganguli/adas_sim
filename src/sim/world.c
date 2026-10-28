/**
 * @file world.c
 * @brief Road geometry, lead-vehicle motion and ground truth.
 */
#include "sim/world.h"

#include <math.h>
#include <string.h>

#include "common/math_utils.h"
#include "sim/scenario.h"
#include "sim/vehicle_model.h"
#include "types/common_types.h"
#include "types/control_types.h"
#include "types/world_types.h"

/* ---------------------------------------------------------------------- */
/* Road geometry: straight along +x, then a left arc of radius R whose    */
/* center is at (straight_len, R).                                        */
/* ---------------------------------------------------------------------- */

RoadProjection road_project(const RoadGeometry *road, Vec2 p) {
  RoadProjection r;
  const float L = road->straight_len_m;
  const float R = road->radius_m;

  if (p.x <= L) {
    r.s_m = p.x;
    r.offset_m = p.y;
    r.heading_rad = 0.0f;
    r.curvature_1pm = 0.0f;
    return r;
  }

  /* Angle swept along the arc: 0 at the arc start, increasing CCW. */
  const float dx = p.x - L;
  const float dy = p.y - R;
  const float theta = atan2f(dx, -dy);

  r.s_m = L + R * theta;
  r.offset_m = R - sqrtf(dx * dx + dy * dy); /* closer to center = left */
  r.heading_rad = theta;
  r.curvature_1pm = 1.0f / R;
  return r;
}

Pose2D road_pose_at(const RoadGeometry *road, float s_m, float offset_m) {
  Pose2D p;
  const float L = road->straight_len_m;
  const float R = road->radius_m;

  if (s_m <= L) {
    p.x = s_m;
    p.y = offset_m;
    p.psi = 0.0f;
    return p;
  }

  const float theta = (s_m - L) / R;
  const float r = R - offset_m;
  p.x = L + r * sinf(theta);
  p.y = R - r * cosf(theta);
  p.psi = theta;
  return p;
}

Vec2 world_to_ego(Pose2D ego, Vec2 p) {
  const float dx = p.x - ego.x;
  const float dy = p.y - ego.y;
  const float c = cosf(ego.psi);
  const float s = sinf(ego.psi);
  const Vec2 q = {c * dx + s * dy, -s * dx + c * dy};
  return q;
}

Vec2 world_lead_in_ego(const WorldState *w) {
  const Pose2D lead = road_pose_at(&w->road, w->lead.s_m, w->lead.offset_m);
  const Vec2 p = {lead.x, lead.y};
  return world_to_ego(w->ego.pose, p);
}

/* ---------------------------------------------------------------------- */
/* World lifecycle                                                        */
/* ---------------------------------------------------------------------- */

void world_init(WorldState *w, const ScenarioConfig *sc) {
  memset(w, 0, sizeof(*w));
  w->road = sc->road;

  Pose2D ego_pose = road_pose_at(&w->road, 0.0f, sc->ego_init.offset_m);
  ego_pose.psi += sc->ego_init.heading_rad;
  vehicle_model_init(&w->ego, ego_pose, sc->ego_init.speed_mps);

  w->lead.present = sc->lead_init.present;
  w->lead.s_m = sc->lead_init.gap_m; /* ego starts at s = 0 */
  w->lead.v_mps = sc->lead_init.speed_mps;
  w->lead.offset_m = sc->lead_init.offset_m;
}

/** Ramp *x towards the latest event of `prof` that has started by t_s. */
static void apply_profile(const LeadProfile *prof, double t_s, float dt,
                          float *x) {
  const LeadRamp *ev = NULL;
  for (int i = 0; i < prof->n; ++i) {
    if ((double)prof->events[i].t_s <= t_s) {
      ev = &prof->events[i];
    }
  }
  if (ev != NULL) {
    const float max_dx = ev->rate * dt;
    *x += clampf(ev->target - *x, -max_dx, max_dx);
  }
}

void world_step(WorldState *w, const ScenarioConfig *sc, const ActuatorCmd *cmd,
                float dt) {
  vehicle_model_step(&w->ego, cmd, dt);

  apply_profile(&sc->lead_speed, w->t_s, dt, &w->lead.v_mps);
  apply_profile(&sc->lead_offset, w->t_s, dt, &w->lead.offset_m);
  w->lead.s_m += w->lead.v_mps * dt;
  w->t_s += (double)dt;
}

GroundTruth world_ground_truth(const WorldState *w) {
  GroundTruth gt;
  const Vec2 ego_xy = {w->ego.pose.x, w->ego.pose.y};
  const RoadProjection proj = road_project(&w->road, ego_xy);

  gt.e_y_m = proj.offset_m;
  gt.e_psi_rad = wrap_angle(w->ego.pose.psi - proj.heading_rad);
  gt.kappa_1pm = proj.curvature_1pm;
  if (w->lead.present) {
    const Vec2 lead = world_lead_in_ego(w);
    gt.gap_m = lead.x;
    gt.lead_y_m = lead.y;
    gt.v_lead_mps = w->lead.v_mps;
  } else {
    gt.gap_m = INFINITY;
    gt.lead_y_m = 0.0f;
    gt.v_lead_mps = 0.0f;
  }
  return gt;
}
