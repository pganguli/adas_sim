/**
 * @file adas_pipeline.h
 * @brief The ADAS perception -> estimation -> planning pipeline as a DAG.
 *
 *  camera ─► image_denoise ─┬─► lane_seg ──────► lane_geometry ─┬──────────► lka ─────┐
 *                           │                                   │                      │
 *                           └─► vehicle_det_2d ─► mono_depth ──┐│                      ├─► behavior_planner
 *                                                              ▼▼                      │
 *  lidar  ─► point_filter ────► vehicle_det_3d ──────────► lead_fusion ─────► acc ────┘
 *
 * All intermediate results live in PipelineData (a "blackboard"); each node
 * reads its inputs from it and writes its typed output back.
 */
#ifndef ADAS_PIPELINE_H
#define ADAS_PIPELINE_H

#include <stdbool.h>
#include <stdio.h>

#include "estimation/lane_geometry_net.h"
#include "estimation/lead_fusion.h"
#include "estimation/mono_depth_net.h"
#include "nn/dynamic.h"
#include "perception/image_denoise_net.h"
#include "perception/lane_seg_net.h"
#include "perception/point_filter_net.h"
#include "perception/vehicle_detector_2d.h"
#include "perception/vehicle_detector_3d.h"
#include "pipeline/scheduler.h"
#include "planning/behavior_planner.h"
#include "planning/lka.h"
#include "types/planning_types.h"
#include "types/sensor_types.h"

/** Node identifiers = indices into the node table. */
typedef enum {
  NODE_IMAGE_DENOISE = 0,
  NODE_LANE_SEG,
  NODE_VEHICLE_DET_2D,
  NODE_POINT_FILTER,
  NODE_VEHICLE_DET_3D,
  NODE_LANE_GEOMETRY,
  NODE_MONO_DEPTH,
  NODE_LEAD_FUSION,
  NODE_ACC,
  NODE_LKA,
  NODE_BEHAVIOR_PLANNER,
  NODE_COUNT
} PipelineNodeId;

/** Blackboard: every message flowing through the pipeline. */
typedef struct {
  /* inputs */
  SensorFrame sensors;
  DriverInput driver;
  /* perception */
  DenoisedImage denoised;
  LaneMask lane_mask;
  Detections2D det2d;
  FilteredPointCloud filtered;
  Detections3D det3d;
  /* estimation */
  LaneEstimate lane;
  DepthEstimates depth;
  LeadTrack lead;
  /* planning */
  AccOutput acc;
  LateralRef lat_ref;
  PlanOutput plan;
  /* dynamic DNNs: per-node selection (written by whoever controls quality,
   * e.g. a controller; initialized from params.h) and the MACs the node
   * executed in its last run relative to the static net (1 for static
   * nodes and non-DNNs; may exceed 1 when gate overhead dominates) */
  DnnConfig dnn[NODE_COUNT];
  float cost[NODE_COUNT];
} PipelineData;

/** Stateful algorithm instances (networks, filters, state machines). */
typedef struct {
  ImageDenoiseNet denoise;
  LaneSegNet lane_seg;
  VehicleDetector2D det2d;
  PointFilterNet point_filter;
  VehicleDetector3D det3d;
  LaneGeometryNet lane_geometry;
  MonoDepthNet mono_depth;
  LeadFusion fusion;
  Lka lka;
  BehaviorPlanner behavior;
} PipelineModules;

/** The complete pipeline: data, algorithm instances and the scheduler. */
typedef struct {
  PipelineData data;
  PipelineModules mod;
  Scheduler sched;
  SchedulerStats stats; /**< timing of the last run */
} AdasPipeline;

/** Initialize all modules and build the DAG. Returns 0 on success. */
int adas_pipeline_init(AdasPipeline *p);

/** Display name of node `node` (a PipelineNodeId). */
const char *adas_pipeline_node_name(int node);

/** True for the DNN nodes (those with a static and a dynamic variant). */
bool adas_pipeline_is_dnn(int node);

/** Quality node `node` runs with: its quality if dynamic, 1 if static. */
float adas_pipeline_quality(const AdasPipeline *p, int node);

/** Print the variant (and quality) each DNN node runs with to `fp`. */
void adas_pipeline_print_dnn(const AdasPipeline *p, FILE *fp);

/** Run one pipeline tick on a new sensor frame. Result in p->data.plan. */
void adas_pipeline_run(AdasPipeline *p, const SensorFrame *sensors,
                       const DriverInput *driver);

#endif /* ADAS_PIPELINE_H */
