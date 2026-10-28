/**
 * @file scenario.c
 * @brief The default scenario: highway follow, stop-and-go, then free road.
 *
 * Timeline:
 *   0 s  ego at 22 m/s, 0.5 m left of lane center      -> LKA recenters
 *        lead 60 m ahead at 24 m/s                      -> ACC FOLLOW
 *   8 s  lead slows to 16 m/s                          -> ACC follows down
 *  18 s  lead brakes hard to standstill (8 m/s^2)      -> EMERGENCY_BRAKE
 *  26 s  lead accelerates to 30 m/s and drives away    -> back to CRUISE
 *  The road turns into a 400 m radius left curve after 200 m.
 */
#include "sim/scenario.h"

static const ScenarioConfig k_scenario = {
    .name = "highway_follow_stop_and_go",
    .duration_s = 40.0f,
    .road =
        {
            .straight_len_m = 200.0f,
            .radius_m = 400.0f,
            .lane_width_m = 3.5f,
            .marking_width_m = 0.3f,
        },
    .ego_init = {.offset_m = 0.5f, .heading_rad = 0.0f, .speed_mps = 22.0f},
    .lead_init = {.gap_m = 60.0f, .speed_mps = 24.0f},
    .driver = {.v_set_mps = 27.0f, .acc_enabled = true, .lka_enabled = true},
    .n_lead_events = 4,
    .lead_events =
        {
            {0.0f, 24.0f, 1.0f},
            {8.0f, 16.0f, 2.0f},
            {18.0f, 0.0f, 8.0f},
            {26.0f, 30.0f, 2.5f},
        },
};

const ScenarioConfig *scenario_get(void) { return &k_scenario; }
