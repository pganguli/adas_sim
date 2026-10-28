/**
 * @file scenario.c
 * @brief The scenario catalog (select one with SCENARIO_SELECTED in params.h).
 *
 * Every road is a straight followed by a left curve, single lane, 3.5 m wide.
 * Lead events ramp the lead's speed (or lateral offset) towards a target from
 * the given time on; see the timeline comment above each scenario.
 */
#include "sim/scenario.h"

#include <stddef.h>

#include "params.h"

/** Road with the standard lane and marking widths. */
#define ROAD(straight_m, radius_m) {(straight_m), (radius_m), 3.5f, 0.3f}

static const ScenarioConfig k_scenarios[SCENARIO_COUNT] = {
    /*
     *  0 s  ego at 22 m/s, 0.5 m left of lane center      -> LKA recenters
     *       lead 60 m ahead at 24 m/s                      -> ACC FOLLOW
     *  8 s  lead slows to 16 m/s                          -> ACC follows down
     * 18 s  lead brakes hard to standstill (8 m/s^2)      -> EMERGENCY_BRAKE
     * 26 s  lead accelerates to 30 m/s and drives away    -> back to CRUISE
     * The road turns into a 400 m radius left curve after 200 m.
     */
    [SCENARIO_HIGHWAY_STOP_AND_GO] =
        {
            .name = "highway_follow_stop_and_go",
            .duration_s = 40.0f,
            .road = ROAD(200.0f, 400.0f),
            .ego_init = {.offset_m = 0.5f, .speed_mps = 22.0f},
            .lead_init = {.present = true, .gap_m = 60.0f, .speed_mps = 24.0f},
            .driver = {27.0f, true, true},
            .lead_speed = {4,
                           {{0.0f, 24.0f, 1.0f},
                            {8.0f, 16.0f, 2.0f},
                            {18.0f, 0.0f, 8.0f},
                            {26.0f, 30.0f, 2.5f}}},
        },
    /*
     *  0 s  no lead; ego at 15 m/s, 1.0 m left of center, heading 0.03 rad
     *       further left                                   -> LKA recovers
     *       set speed 28 m/s                               -> CRUISE speeds up
     *  The road turns into a 300 m radius left curve after 100 m (about
     *  2.6 m/s^2 lateral at the set speed)                 -> LKA holds the curve
     */
    [SCENARIO_FREE_ROAD_CURVE] =
        {
            .name = "free_road_lane_recovery_curve",
            .duration_s = 30.0f,
            .road = ROAD(100.0f, 300.0f),
            .ego_init = {.offset_m = 1.0f,
                         .heading_rad = 0.03f,
                         .speed_mps = 15.0f},
            .lead_init = {.present = false},
            .driver = {28.0f, true, true},
        },
    /*
     *  0 s  ego cruising at 25 m/s; lead in the right-hand lane, 30 m ahead
     *       at 20 m/s                                      -> not in path: CRUISE
     *  2 s  lead cuts in at 1.2 m/s lateral, lane center reached at ~4.9 s;
     *       it enters the path ~11 m ahead                 -> FOLLOW, hard braking
     * 10 s  lead speeds up to 27 m/s                       -> back to CRUISE
     */
    [SCENARIO_CUT_IN] =
        {
            .name = "cut_in",
            .duration_s = 25.0f,
            .road = ROAD(400.0f, 600.0f),
            .ego_init = {.speed_mps = 25.0f},
            .lead_init = {.present = true,
                          .gap_m = 30.0f,
                          .speed_mps = 20.0f,
                          .offset_m = -3.5f},
            .driver = {25.0f, true, true},
            .lead_speed = {2, {{0.0f, 20.0f, 1.0f}, {10.0f, 27.0f, 1.5f}}},
            .lead_offset = {1, {{2.0f, 0.0f, 1.2f}}},
        },
    /*
     *  0 s  ego at 30 m/s (set speed); a slow truck 78 m ahead at 15 m/s,
     *       just inside LiDAR range                        -> FOLLOW, braking at
     *                                                         the comfort limit,
     *                                                         then settles at
     *                                                         15 m/s on the time gap
     */
    [SCENARIO_SLOW_LEAD_APPROACH] =
        {
            .name = "slow_lead_approach",
            .duration_s = 30.0f,
            .road = ROAD(300.0f, 500.0f),
            .ego_init = {.speed_mps = 30.0f},
            .lead_init = {.present = true, .gap_m = 78.0f, .speed_mps = 15.0f},
            .driver = {30.0f, true, true},
        },
    /*
     *  0 s  ego and lead at 25 m/s, 55 m apart (just over the desired gap)
     *                                                      -> CRUISE
     *  5 s  lead brakes at 7 m/s^2 to standstill           -> FOLLOW, then
     *                                                         EMERGENCY_BRAKE;
     *                                                         ego stops < 1 m behind
     * The AEB margin is thin on purpose: the injected latency costs about half
     * of it, and a lead braking at 7.5 m/s^2 or more is hit.
     */
    [SCENARIO_LEAD_HARD_BRAKE] =
        {
            .name = "lead_hard_brake",
            .duration_s = 15.0f,
            .road = ROAD(400.0f, 600.0f),
            .ego_init = {.speed_mps = 25.0f},
            .lead_init = {.present = true, .gap_m = 55.0f, .speed_mps = 25.0f},
            .driver = {25.0f, true, true},
            .lead_speed = {1, {{5.0f, 0.0f, 7.0f}}},
        },
    /*
     * Low-speed queue: the lead stops and restarts three times
     *  0 s  ego and lead at 8 m/s, 25 m apart              -> FOLLOW
     *  3 s  lead stops (2.5 m/s^2)                         -> ego creeps up behind
     * 10 s  lead to 10 m/s, 18 s stop (3 m/s^2), 24 s to 6 m/s,
     * 30 s  stop (2 m/s^2), 36 s off to 12 m/s             -> stop & go
     */
    [SCENARIO_STOP_AND_GO_TRAFFIC] =
        {
            .name = "stop_and_go_traffic",
            .duration_s = 45.0f,
            .road = ROAD(150.0f, 250.0f),
            .ego_init = {.speed_mps = 8.0f},
            .lead_init = {.present = true, .gap_m = 25.0f, .speed_mps = 8.0f},
            .driver = {15.0f, true, true},
            .lead_speed = {6,
                           {{3.0f, 0.0f, 2.5f},
                            {10.0f, 10.0f, 1.5f},
                            {18.0f, 0.0f, 3.0f},
                            {24.0f, 6.0f, 1.5f},
                            {30.0f, 0.0f, 2.0f},
                            {36.0f, 12.0f, 2.0f}}},
        },
};

_Static_assert(SCENARIO_SELECTED >= 0 && SCENARIO_SELECTED < SCENARIO_COUNT,
               "SCENARIO_SELECTED in params.h is not a valid ScenarioId");

const ScenarioConfig *scenario_get(void) {
  return &k_scenarios[SCENARIO_SELECTED];
}

const ScenarioConfig *scenario_get_by_id(ScenarioId id) {
  return ((int)id >= 0 && id < SCENARIO_COUNT) ? &k_scenarios[id] : NULL;
}
