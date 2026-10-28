/**
 * @file sensor_types.h
 * @brief Raw sensor messages (output of the sensor models, input of perception).
 */
#ifndef ADAS_SENSOR_TYPES_H
#define ADAS_SENSOR_TYPES_H

#include "params.h"
#include "types/common_types.h"

/** Single-channel image, row-major px[v][u], intensity in [0, 1]. */
typedef struct {
  MsgHeader hdr;
  float px[CAM_H][CAM_W];
} GrayImage;

/** Raw front-camera frame. */
typedef GrayImage CameraFrame;

/** One LiDAR return in the ego frame [m]. */
typedef Vec3 LidarPoint;

/** Unordered set of LiDAR returns. */
typedef struct {
  MsgHeader hdr;
  int n_points; /**< number of valid entries in points */
  LidarPoint points[LIDAR_MAX_POINTS];
} PointCloud;

/** One full raw LiDAR scan. */
typedef PointCloud LidarScan;

/** Ego-motion sensors: wheel speed + IMU. */
typedef struct {
  MsgHeader hdr;
  float v_mps;        /**< longitudinal speed        */
  float a_mps2;       /**< longitudinal acceleration */
  float yaw_rate_rps; /**< yaw rate [rad/s]          */
} EgoMotion;

/** All sensor data captured at one pipeline tick. */
typedef struct {
  CameraFrame camera;
  LidarScan lidar;
  EgoMotion ego;
} SensorFrame;

#endif /* ADAS_SENSOR_TYPES_H */
