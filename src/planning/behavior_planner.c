/**
 * @file behavior_planner.c
 * @brief Behavior planner implementation.
 */
#include "planning/behavior_planner.h"

#include "params.h"
#include "types/planning_types.h"

void behavior_planner_init(BehaviorPlanner *bp) { bp->mode = LON_MODE_CRUISE; }

/** Mode transition function (see the diagram in the header). */
static LongitudinalMode next_mode(LongitudinalMode mode,
                                  const DriverInput *driver,
                                  const AccOutput *acc) {
  if (!driver->acc_enabled) {
    return LON_MODE_OFF;
  }

  /* Emergency braking has priority and uses hysteresis to avoid chattering. */
  if (mode == LON_MODE_EMERGENCY_BRAKE) {
    if (acc->lead_valid && acc->ttc_s < AEB_TTC_EXIT_S) {
      return LON_MODE_EMERGENCY_BRAKE;
    }
  } else if (acc->lead_valid && acc->ttc_s < AEB_TTC_ENTER_S) {
    return LON_MODE_EMERGENCY_BRAKE;
  }

  if (acc->lead_valid && acc->a_follow_mps2 < acc->a_cruise_mps2) {
    return LON_MODE_FOLLOW;
  }
  return LON_MODE_CRUISE;
}

void behavior_planner_step(BehaviorPlanner *bp, const DriverInput *driver,
                           const AccOutput *acc, const LateralRef *lat,
                           PlanOutput *out) {
  bp->mode = next_mode(bp->mode, driver, acc);

  out->hdr = acc->hdr;
  out->mode = bp->mode;
  switch (bp->mode) {
  case LON_MODE_CRUISE:
    out->a_des_mps2 = acc->a_cruise_mps2;
    break;
  case LON_MODE_FOLLOW:
    out->a_des_mps2 = acc->a_follow_mps2;
    break;
  case LON_MODE_EMERGENCY_BRAKE:
    out->a_des_mps2 = AEB_DECEL_MPS2;
    break;
  case LON_MODE_OFF:
  default:
    out->a_des_mps2 = 0.0f;
    break;
  }
  out->lat = *lat;
}

const char *lon_mode_name(LongitudinalMode mode) {
  switch (mode) {
  case LON_MODE_OFF:
    return "OFF";
  case LON_MODE_CRUISE:
    return "CRUISE";
  case LON_MODE_FOLLOW:
    return "FOLLOW";
  case LON_MODE_EMERGENCY_BRAKE:
    return "EBRAKE";
  default:
    return "?";
  }
}
