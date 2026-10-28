/**
 * @file behavior_planner.h
 * @brief Longitudinal mode state machine + final plan assembly.
 *
 *          lead valid & a_follow < a_cruise
 *   CRUISE  ------------------------------->  FOLLOW
 *      ^   <-------------------------------     |
 *      |                                        | TTC < TTC_enter
 *      +---- TTC > TTC_exit or no lead ----  EMERGENCY_BRAKE
 */
#ifndef ADAS_BEHAVIOR_PLANNER_H
#define ADAS_BEHAVIOR_PLANNER_H

#include "types/planning_types.h"

/** State machine memory (the current mode gives the AEB hysteresis). */
typedef struct {
  LongitudinalMode mode;
} BehaviorPlanner;

/** Start in CRUISE. */
void behavior_planner_init(BehaviorPlanner *bp);

/** Update the mode and assemble the plan from the ACC and LKA outputs. */
void behavior_planner_step(BehaviorPlanner *bp, const DriverInput *driver,
                           const AccOutput *acc, const LateralRef *lat,
                           PlanOutput *out);

/** Short display name of a mode ("?" if out of range). */
const char *lon_mode_name(LongitudinalMode mode);

#endif /* ADAS_BEHAVIOR_PLANNER_H */
