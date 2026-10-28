/**
 * @file world_types.h
 * @brief Ground-truth world: road, lead vehicle, ego plant.
 *
 * Perception never reads these types directly; it only sees sensor messages.
 * They are used by the sensor models and for logging ground truth.
 */
#ifndef ADAS_WORLD_TYPES_H
#define ADAS_WORLD_TYPES_H

#include "types/common_types.h"
#include "types/vehicle_types.h"

/**
 * Road = straight segment along world +x starting at the origin, followed by
 * a constant-radius left curve. Single lane, solid markings on both sides.
 */
typedef struct {
  float straight_len_m;  /**< length of the straight segment      */
  float radius_m;        /**< radius of the following left curve  */
  float lane_width_m;    /**< distance between marking centers    */
  float marking_width_m; /**< painted marking width               */
} RoadGeometry;

/** Projection of a world point onto the road center line (Frenet coords). */
typedef struct {
  float s_m;           /**< arc length along the center line   */
  float offset_m;      /**< signed lateral offset (left +)     */
  float heading_rad;   /**< center-line heading at s           */
  float curvature_1pm; /**< center-line curvature at s         */
} RoadProjection;

/** Lead vehicle drives on the lane center; s is its rear bumper. */
typedef struct {
  float s_m;   /**< arc length of the rear bumper along the road */
  float v_mps; /**< speed along the road                         */
} LeadVehicleState;

/** Complete ground-truth state of the simulated world. */
typedef struct {
  double t_s; /**< simulation time [s] */
  RoadGeometry road;
  VehicleState ego;
  LeadVehicleState lead;
} WorldState;

/** Ground-truth versions of the estimated quantities (for logging/metrics). */
typedef struct {
  float e_y_m;      /**< lateral offset from lane center (left +) */
  float e_psi_rad;  /**< heading relative to the lane (CCW +)     */
  float kappa_1pm;  /**< lane curvature at the ego (left +)       */
  float gap_m;      /**< lead rear face x in the ego frame        */
  float lead_y_m;   /**< lead lateral position in the ego frame   */
  float v_lead_mps; /**< lead speed                               */
} GroundTruth;

#endif /* ADAS_WORLD_TYPES_H */
