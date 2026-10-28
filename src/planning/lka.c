/**
 * @file lka.c
 * @brief LKA implementation.
 */
#include "planning/lka.h"

#include <math.h>
#include <string.h>

#include "params.h"
#include "types/estimation_types.h"
#include "types/planning_types.h"

void lka_init(Lka *lka) { memset(lka, 0, sizeof(*lka)); }

void lka_compute(Lka *lka, const LaneEstimate *lane, float v_ego_mps,
                 bool enabled, LateralRef *out) {
  const float L = VEH_WHEELBASE_M;

  memset(out, 0, sizeof(*out));
  out->hdr = lane->hdr;
  out->active = enabled && lane->valid && (v_ego_mps > LKA_MIN_SPEED_MPS);
  if (!lane->valid) {
    return;
  }

  lka->kappa_filt_1pm +=
      LKA_KAPPA_LPF_ALPHA * (lane->kappa_1pm - lka->kappa_filt_1pm);

  out->e_y_front_m = -(lane->c0 + lane->c1 * L + lane->c2 * L * L);
  out->e_psi_front_rad = -atanf(lane->c1 + 2.0f * lane->c2 * L);
  out->kappa_1pm = lka->kappa_filt_1pm;
}
