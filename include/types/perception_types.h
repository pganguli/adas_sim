/**
 * @file perception_types.h
 * @brief Outputs of the perception DNNs.
 */
#ifndef ADAS_PERCEPTION_TYPES_H
#define ADAS_PERCEPTION_TYPES_H

#include "params.h"
#include "types/common_types.h"
#include "types/sensor_types.h"

/* ---- Camera branch ---------------------------------------------------- */

/** Output of ImageDenoiseNet: same layout as the raw frame. */
typedef GrayImage DenoisedImage;

/** Output of LaneSegNet: per-pixel probability of "lane marking". */
typedef struct {
  MsgHeader hdr;
  float prob[CAM_H][CAM_W]; /**< prob[v][u] in [0, 1] */
} LaneMask;

/** Axis-aligned image box, inclusive pixel bounds. */
typedef struct {
  int u_min, v_min;
  int u_max, v_max;
  float score; /**< detector confidence in [0, 1] */
} BBox2D;

/** Output of VehicleDetector2D. */
typedef struct {
  MsgHeader hdr;
  int n; /**< number of valid entries in boxes */
  BBox2D boxes[DET2D_MAX_BOXES];
} Detections2D;

/* ---- LiDAR branch ----------------------------------------------------- */

/** Output of PointFilterNet: the points that survived filtering. */
typedef PointCloud FilteredPointCloud;

/** Axis-aligned 3D box in the ego frame. */
typedef struct {
  Vec3 center;  /**< [m]                      */
  Vec3 size;    /**< extent along x, y, z [m] */
  float score;  /**< confidence in [0, 1]     */
  int n_points; /**< supporting LiDAR points  */
} BBox3D;

/** Output of VehicleDetector3D. */
typedef struct {
  MsgHeader hdr;
  int n; /**< number of valid entries in boxes */
  BBox3D boxes[DET3D_MAX_BOXES];
} Detections3D;

#endif /* ADAS_PERCEPTION_TYPES_H */
