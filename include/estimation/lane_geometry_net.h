/**
 * @file lane_geometry_net.h
 * @brief LKA state estimation head: lane mask -> (e_y, e_psi, kappa).
 *
 *   input  : LaneMask
 *   anchors: for each of K row anchors (fixed image rows = fixed ranges X_k),
 *            soft-argmax of the mask over the left / right image half gives the
 *            left / right marking column -> lateral position -> lane center y_k
 *            (row-anchor formulation, as in "Ultra Fast Lane Detection")
 *   head   : Dense(K->3) regression to the polynomial
 *            y_c(x) = c0 + c1 x + c2 x^2
 *            (weights = least-squares pseudo-inverse, computed at init)
 *   output : LaneEstimate
 *
 * Rows where no marking is visible (e.g. occluded by the lead car) are filled
 * from the previous estimate; too many missing rows -> estimate invalid.
 */
#ifndef ADAS_LANE_GEOMETRY_NET_H
#define ADAS_LANE_GEOMETRY_NET_H

#include "nn/nn.h"
#include "types/estimation_types.h"
#include "types/perception_types.h"

typedef struct {
  DenseLayer head;                      /**< K -> 3 regression head      */
  float head_w[3 * LANE_N_ANCHORS];     /**< head weights (set at init)  */
  int anchor_row[LANE_N_ANCHORS];       /**< image row of each anchor    */
  float anchor_range_m[LANE_N_ANCHORS]; /**< ground range of each anchor */
  LaneEstimate prev;                    /**< last valid estimate (prior) */
} LaneGeometryNet;

/** Compute anchor ranges and the least-squares head weights. */
void lane_geometry_net_init(LaneGeometryNet *net);

/** Estimate the lane polynomial from a lane mask (out->valid may be false). */
void lane_geometry_net_infer(LaneGeometryNet *net, const LaneMask *in,
                             LaneEstimate *out);

#endif /* ADAS_LANE_GEOMETRY_NET_H */
