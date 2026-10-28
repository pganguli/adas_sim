/**
 * @file lka.h
 * @brief Lane Keeping Assist: activation logic + lateral reference.
 *
 * Re-expresses the lane errors at the front axle (x = L), which is where the
 * Stanley controller measures them:
 *   e_y_front   = -(c0 + c1 L + c2 L^2)
 *   e_psi_front = -atan(c1 + 2 c2 L)
 */
#ifndef ADAS_LKA_H
#define ADAS_LKA_H

#include "types/estimation_types.h"
#include "types/planning_types.h"

/** LKA filter state. */
typedef struct {
  float kappa_filt_1pm; /**< low-pass filtered lane curvature */
} Lka;

/** Clear the filter state. */
void lka_init(Lka *lka);

/** Compute the lateral reference; `out->active` follows enable/validity/speed. */
void lka_compute(Lka *lka, const LaneEstimate *lane, float v_ego_mps,
                 bool enabled, LateralRef *out);

#endif /* ADAS_LKA_H */
