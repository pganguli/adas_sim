/**
 * @file acc.c
 * @brief ACC implementation.
 */
#include "planning/acc.h"

#include <math.h>

#include "common/math_utils.h"
#include "params.h"
#include "types/estimation_types.h"
#include "types/planning_types.h"

void acc_compute(float v_ego_mps, float v_set_mps, const LeadTrack *lead,
                 AccOutput *out) {
  out->hdr = lead->hdr;
  out->lead_valid = lead->valid;
  out->a_cruise_mps2 = clampf(ACC_K_SPEED * (v_set_mps - v_ego_mps),
                              ACC_A_MIN_MPS2, ACC_A_MAX_MPS2);
  out->d_des_m = ACC_D0_M + ACC_TIME_GAP_S * v_ego_mps;

  if (!lead->valid) {
    out->a_follow_mps2 = ACC_A_MAX_MPS2;
    out->ttc_s = INFINITY;
    return;
  }

  const float a_follow =
      ACC_K_GAP * (lead->d_m - out->d_des_m) + ACC_K_VREL * lead->v_rel_mps;
  out->a_follow_mps2 = clampf(a_follow, ACC_A_MIN_MPS2, ACC_A_MAX_MPS2);
  out->ttc_s =
      (lead->v_rel_mps < -0.1f) ? lead->d_m / -lead->v_rel_mps : INFINITY;
}
