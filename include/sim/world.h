/**
 * @file world.h
 * @brief Ground-truth world simulation: road geometry, lead vehicle, ego plant.
 */
#ifndef ADAS_WORLD_H
#define ADAS_WORLD_H

#include "sim/scenario.h"
#include "types/control_types.h"
#include "types/world_types.h"

/** Place the ego and lead vehicles at the scenario's initial conditions. */
void world_init(WorldState *w, const ScenarioConfig *sc);

/** Advance ego plant, lead vehicle and time by dt [s]. */
void world_step(WorldState *w, const ScenarioConfig *sc, const ActuatorCmd *cmd,
                float dt);

/** Project a world point onto the road center line. */
RoadProjection road_project(const RoadGeometry *road, Vec2 p_world);

/** World pose of the point at arc length s and lateral offset (left +). */
Pose2D road_pose_at(const RoadGeometry *road, float s_m, float offset_m);

/** Transform a world point into the ego frame of `ego`. */
Vec2 world_to_ego(Pose2D ego, Vec2 p_world);

/** Lead vehicle rear-bumper center in the ego frame (only if present). */
Vec2 world_lead_in_ego(const WorldState *w);

/** Ground-truth lane errors and lead state for logging / metrics. */
GroundTruth world_ground_truth(const WorldState *w);

#endif /* ADAS_WORLD_H */
