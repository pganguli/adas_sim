/**
 * @file common_types.h
 * @brief Geometry primitives and the message header shared by all messages.
 *
 * Frames (ISO 8855):
 *   world : fixed, x east / y north, yaw CCW+ from world x-axis
 *   ego   : attached to the vehicle reference point (rear-axle center),
 *           x forward, y left, z up. All sensors are mounted at this point.
 */
#ifndef ADAS_COMMON_TYPES_H
#define ADAS_COMMON_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/** 2D vector / point [m]. */
typedef struct {
  float x, y;
} Vec2;

/** 3D vector / point [m]. */
typedef struct {
  float x, y, z;
} Vec3;

/** Planar pose: position [m] and yaw. */
typedef struct {
  float x, y;
  float psi; /**< yaw [rad], CCW+ */
} Pose2D;

/**
 * Every message produced in the pipeline starts with a header so data can be
 * traced back to the sensor frame it came from (seq) and when it was taken.
 */
typedef struct {
  uint32_t seq;   /**< pipeline frame counter             */
  double stamp_s; /**< sensor capture time [s] (sim time) */
} MsgHeader;

#endif /* ADAS_COMMON_TYPES_H */
