/**
 * @file acc.h
 * @brief Adaptive Cruise Control: constant time-gap (CTG) policy.
 *
 *   a_cruise = k_v  * (v_set - v)
 *   d_des    = D0 + T_gap * v
 *   a_follow = k_d  * (d - d_des) + k_vr * v_rel
 *   TTC      = d / (-v_rel)      (only when closing in)
 * Both candidates are clipped to comfort limits; the behavior planner
 * arbitrates.
 */
#ifndef ADAS_ACC_H
#define ADAS_ACC_H

#include "types/estimation_types.h"
#include "types/planning_types.h"

/**
 * Compute the ACC acceleration candidates, desired gap and TTC.
 *
 * @param v_ego_mps ego speed
 * @param v_set_mps driver set speed
 * @param lead      fused lead track
 * @param out       candidates for the behavior planner
 */
void acc_compute(float v_ego_mps, float v_set_mps, const LeadTrack *lead,
                 AccOutput *out);

#endif /* ADAS_ACC_H */
