/**
 * @file scenario.h
 * @brief Test scenario definition (road, initial conditions, lead behavior).
 */
#ifndef ADAS_SCENARIO_H
#define ADAS_SCENARIO_H

#include "types/planning_types.h"
#include "types/world_types.h"

/** Capacity of ScenarioConfig::lead_events. */
#define SCENARIO_MAX_LEAD_EVENTS 8

/** From t_s on, the lead ramps its speed to v_target at |accel| = accel_mag. */
typedef struct {
  float t_s;            /**< event start time [s]         */
  float v_target_mps;   /**< speed the lead ramps towards */
  float accel_mag_mps2; /**< magnitude of the ramp rate   */
} LeadSpeedEvent;

/** Initial state of the ego vehicle. */
typedef struct {
  float offset_m;    /**< initial lateral offset from lane center (left +) */
  float heading_rad; /**< initial heading relative to the road             */
  float speed_mps;   /**< initial speed                                    */
} EgoInit;

/** Initial state of the lead vehicle. */
typedef struct {
  float gap_m;     /**< initial distance ahead of the ego reference point */
  float speed_mps; /**< initial speed                                     */
} LeadInit;

/** Everything needed to define and run one simulation. */
typedef struct {
  const char *name; /**< scenario identifier              */
  float duration_s; /**< simulated duration               */
  RoadGeometry road;
  EgoInit ego_init;
  LeadInit lead_init;
  DriverInput driver; /**< HMI state held for the whole run */
  int n_lead_events;  /**< number of valid lead_events      */
  /** Sorted by t_s. */
  LeadSpeedEvent lead_events[SCENARIO_MAX_LEAD_EVENTS];
} ScenarioConfig;

/** Returns the scenario to run (edit src/sim/scenario.c to change it). */
const ScenarioConfig *scenario_get(void);

#endif /* ADAS_SCENARIO_H */
