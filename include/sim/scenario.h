/**
 * @file scenario.h
 * @brief Test scenario definition (road, initial conditions, lead behavior).
 */
#ifndef ADAS_SCENARIO_H
#define ADAS_SCENARIO_H

#include "types/planning_types.h"
#include "types/world_types.h"

/** The scenarios in src/sim/scenario.c (pick one with SCENARIO_SELECTED). */
typedef enum {
  SCENARIO_HIGHWAY_STOP_AND_GO = 0,
  SCENARIO_FREE_ROAD_CURVE,
  SCENARIO_CUT_IN,
  SCENARIO_SLOW_LEAD_APPROACH,
  SCENARIO_LEAD_HARD_BRAKE,
  SCENARIO_STOP_AND_GO_TRAFFIC,
  SCENARIO_COUNT
} ScenarioId;

/** Capacity of LeadProfile::events. */
#define SCENARIO_MAX_LEAD_EVENTS 8

/**
 * From t_s on, the lead ramps one quantity towards `target` at |rate| per
 * second (speed: m/s and m/s^2; lateral offset: m and m/s).
 */
typedef struct {
  float t_s;    /**< event start time [s]                */
  float target; /**< value the quantity ramps towards    */
  float rate;   /**< magnitude of the ramp rate [unit/s] */
} LeadRamp;

/** Timeline of one lead quantity; the latest started event is active. */
typedef struct {
  int n;                                     /**< number of valid events */
  LeadRamp events[SCENARIO_MAX_LEAD_EVENTS]; /**< sorted by t_s          */
} LeadProfile;

/** Initial state of the ego vehicle. */
typedef struct {
  float offset_m;    /**< initial lateral offset from lane center (left +) */
  float heading_rad; /**< initial heading relative to the road             */
  float speed_mps;   /**< initial speed                                    */
} EgoInit;

/** Initial state of the lead vehicle. */
typedef struct {
  bool present;    /**< false: free road, no lead vehicle in the world     */
  float gap_m;     /**< initial distance ahead of the ego reference point */
  float speed_mps; /**< initial speed                                     */
  float offset_m;  /**< initial lateral offset from lane center (left +)  */
} LeadInit;

/** Everything needed to define and run one simulation. */
typedef struct {
  const char *name; /**< scenario identifier              */
  float duration_s; /**< simulated duration               */
  RoadGeometry road;
  EgoInit ego_init;
  LeadInit lead_init;
  DriverInput driver;      /**< HMI state held for the whole run */
  LeadProfile lead_speed;  /**< lead speed [m/s], ramp [m/s^2]   */
  LeadProfile lead_offset; /**< lead lateral offset [m], ramp [m/s] */
} ScenarioConfig;

/** The scenario selected by SCENARIO_SELECTED in params.h. */
const ScenarioConfig *scenario_get(void);

/** Any scenario by id (NULL if out of range). */
const ScenarioConfig *scenario_get_by_id(ScenarioId id);

#endif /* ADAS_SCENARIO_H */
