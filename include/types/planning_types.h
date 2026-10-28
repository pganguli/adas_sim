/**
 * @file planning_types.h
 * @brief Driver inputs and planner outputs (references for the controllers).
 */
#ifndef ADAS_PLANNING_TYPES_H
#define ADAS_PLANNING_TYPES_H

#include "types/common_types.h"

/** HMI: what the driver has switched on / set. */
typedef struct {
  float v_set_mps;  /**< ACC set speed */
  bool acc_enabled; /**< ACC switched on */
  bool lka_enabled; /**< LKA switched on */
} DriverInput;

/** Longitudinal behavior selected by the behavior planner. */
typedef enum {
  LON_MODE_OFF = 0,
  LON_MODE_CRUISE,          /**< no relevant lead: track set speed    */
  LON_MODE_FOLLOW,          /**< keep time gap to lead                */
  LON_MODE_EMERGENCY_BRAKE, /**< time-to-collision below threshold    */
  LON_MODE_COUNT            /**< number of modes (array sizing)       */
} LongitudinalMode;

/** Output of the ACC function: candidate accelerations, the planner decides. */
typedef struct {
  MsgHeader hdr;
  bool lead_valid;     /**< a confirmed lead vehicle exists        */
  float a_cruise_mps2; /**< speed-control candidate                */
  float a_follow_mps2; /**< gap-control candidate                  */
  float d_des_m;       /**< desired gap D0 + T_gap * v             */
  float ttc_s;         /**< time to collision (INFINITY if opening) */
} AccOutput;

/** Output of the LKA function: lane errors expressed at the front axle. */
typedef struct {
  MsgHeader hdr;
  bool active;           /**< LKA is steering                            */
  float e_y_front_m;     /**< lateral offset of the front axle (left +)  */
  float e_psi_front_rad; /**< vehicle heading rel. to lane at front axle */
  float kappa_1pm;       /**< low-pass filtered lane curvature           */
} LateralRef;

/** Final plan handed to the low-level controllers. */
typedef struct {
  MsgHeader hdr;
  LongitudinalMode mode;
  float a_des_mps2; /**< longitudinal reference  */
  LateralRef lat;   /**< lateral reference (LKA) */
} PlanOutput;

#endif /* ADAS_PLANNING_TYPES_H */
