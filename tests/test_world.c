/**
 * @file test_world.c
 * @brief Tests for road geometry and frame transforms.
 */

#include <math.h>

#include "common/math_utils.h"
#include "sim/scenario.h"
#include "sim/world.h"
#include "test_util.h"
#include "types/common_types.h"
#include "types/control_types.h"
#include "types/world_types.h"

void test_world(void) {
  const RoadGeometry road = {200.0f, 400.0f, 3.5f, 0.3f};

  /* Straight segment: pose and projection are inverse of each other. */
  Pose2D p = road_pose_at(&road, 100.0f, 1.0f);
  CHECK_NEAR(p.x, 100.0f, 1e-4);
  CHECK_NEAR(p.y, 1.0f, 1e-4);
  CHECK_NEAR(p.psi, 0.0f, 1e-6);
  Vec2 xy = {p.x, p.y};
  RoadProjection pr = road_project(&road, xy);
  CHECK_NEAR(pr.s_m, 100.0f, 1e-3);
  CHECK_NEAR(pr.offset_m, 1.0f, 1e-4);
  CHECK_NEAR(pr.curvature_1pm, 0.0f, 0.0);

  /* Left arc: round trip, heading and curvature. */
  p = road_pose_at(&road, 400.0f, -0.5f);
  xy.x = p.x;
  xy.y = p.y;
  pr = road_project(&road, xy);
  CHECK_NEAR(pr.s_m, 400.0f, 0.05);
  CHECK_NEAR(pr.offset_m, -0.5f, 1e-3);
  CHECK_NEAR(pr.heading_rad, 0.5f, 1e-3);
  CHECK_NEAR(p.psi, 0.5f, 1e-3);
  CHECK_NEAR(pr.curvature_1pm, 1.0f / 400.0f, 1e-6);

  /* Ego frame: a point 3 m to the ego's front when the ego faces +y. */
  const Pose2D ego = {10.0f, 5.0f, 0.5f * ADAS_PI};
  const Vec2 target = {10.0f, 8.0f};
  const Vec2 q = world_to_ego(ego, target);
  CHECK_NEAR(q.x, 3.0f, 1e-4);
  CHECK_NEAR(q.y, 0.0f, 1e-4);

  /* Lead offset profile ramps at its rate; an absent lead has no gap. */
  ScenarioConfig sc = *scenario_get_by_id(SCENARIO_CUT_IN);
  WorldState w;
  const ActuatorCmd coast = {0.0f, 0.0f, 0.0f};
  world_init(&w, &sc);
  CHECK_NEAR(world_ground_truth(&w).lead_y_m, -3.5f, 1e-4);
  for (int i = 0; i < 300; ++i) { /* 3 s: offset starts ramping at 2 s */
    world_step(&w, &sc, &coast, 0.01f);
  }
  CHECK_NEAR(w.lead.offset_m, -3.5f + 1.2f, 0.02);
  sc.lead_init.present = false;
  world_init(&w, &sc);
  CHECK(isinf(world_ground_truth(&w).gap_m));
}
