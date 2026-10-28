/**
 * @file test_world.c
 * @brief Tests for road geometry and frame transforms.
 */

#include "common/math_utils.h"
#include "sim/world.h"
#include "test_util.h"
#include "types/common_types.h"
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
}
