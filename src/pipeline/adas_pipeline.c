/**
 * @file adas_pipeline.c
 * @brief Node adapters + the node table (the pipeline's dependency graph).
 *
 * To add a stage: write the module, add a field to PipelineData /
 * PipelineModules, write a run_xxx() adapter, add a NODE_ id and a row below.
 */
#include "pipeline/adas_pipeline.h"

#include <string.h>

#include "estimation/lane_geometry_net.h"
#include "estimation/lead_fusion.h"
#include "estimation/mono_depth_net.h"
#include "params.h"
#include "perception/image_denoise_net.h"
#include "perception/lane_seg_net.h"
#include "perception/point_filter_net.h"
#include "perception/vehicle_detector_2d.h"
#include "perception/vehicle_detector_3d.h"
#include "pipeline/scheduler.h"
#include "planning/acc.h"
#include "planning/behavior_planner.h"
#include "planning/lka.h"

/* ---------------------------------------------------------------------- */
/* Node adapters: unpack the blackboard and call the typed module API.    */
/* Each adapter is a NodeFn; `ctx` is the owning AdasPipeline.            */
/* ---------------------------------------------------------------------- */

static void run_image_denoise(void *ctx) {
  AdasPipeline *p = ctx;
  image_denoise_net_infer(&p->mod.denoise, &p->data.sensors.camera,
                          &p->data.denoised);
}

static void run_lane_seg(void *ctx) {
  AdasPipeline *p = ctx;
  lane_seg_net_infer(&p->mod.lane_seg, &p->data.denoised, &p->data.lane_mask);
}

static void run_vehicle_det_2d(void *ctx) {
  AdasPipeline *p = ctx;
  vehicle_detector_2d_infer(&p->mod.det2d, &p->data.denoised, &p->data.det2d);
}

static void run_point_filter(void *ctx) {
  AdasPipeline *p = ctx;
  point_filter_net_infer(&p->mod.point_filter, &p->data.sensors.lidar,
                         &p->data.filtered);
}

static void run_vehicle_det_3d(void *ctx) {
  AdasPipeline *p = ctx;
  vehicle_detector_3d_infer(&p->mod.det3d, &p->data.filtered, &p->data.det3d);
}

static void run_lane_geometry(void *ctx) {
  AdasPipeline *p = ctx;
  lane_geometry_net_infer(&p->mod.lane_geometry, &p->data.lane_mask,
                          &p->data.lane);
}

static void run_mono_depth(void *ctx) {
  AdasPipeline *p = ctx;
  mono_depth_net_infer(&p->mod.mono_depth, &p->data.det2d, &p->data.depth);
}

static void run_lead_fusion(void *ctx) {
  AdasPipeline *p = ctx;
  lead_fusion_update(&p->mod.fusion, &p->data.det3d, &p->data.depth,
                     &p->data.lane, PIPELINE_DT_S, &p->data.lead);
}

static void run_acc(void *ctx) {
  AdasPipeline *p = ctx;
  acc_compute(p->data.sensors.ego.v_mps, p->data.driver.v_set_mps,
              &p->data.lead, &p->data.acc);
}

static void run_lka(void *ctx) {
  AdasPipeline *p = ctx;
  lka_compute(&p->mod.lka, &p->data.lane, p->data.sensors.ego.v_mps,
              p->data.driver.lka_enabled, &p->data.lat_ref);
}

static void run_behavior_planner(void *ctx) {
  AdasPipeline *p = ctx;
  behavior_planner_step(&p->mod.behavior, &p->data.driver, &p->data.acc,
                        &p->data.lat_ref, &p->data.plan);
}

/* ---------------------------------------------------------------------- */
/* The graph. latency_ms = nominal inference time on the target ECU.      */
/* ---------------------------------------------------------------------- */

/* Row layout: {name, adapter, #deps, {deps...}, latency_ms} */
static const TaskNode k_nodes[NODE_COUNT] = {
    [NODE_IMAGE_DENOISE] =
        {"image_denoise_net", run_image_denoise, 0, {0}, 3.0f},
    [NODE_LANE_SEG] =
        {"lane_seg_net", run_lane_seg, 1, {NODE_IMAGE_DENOISE}, 8.0f},
    [NODE_VEHICLE_DET_2D] =
        {"vehicle_det_2d", run_vehicle_det_2d, 1, {NODE_IMAGE_DENOISE}, 12.0f},
    [NODE_POINT_FILTER] = {"point_filter_net", run_point_filter, 0, {0}, 2.0f},
    [NODE_VEHICLE_DET_3D] =
        {"vehicle_det_3d", run_vehicle_det_3d, 1, {NODE_POINT_FILTER}, 10.0f},
    [NODE_LANE_GEOMETRY] =
        {"lane_geometry_net", run_lane_geometry, 1, {NODE_LANE_SEG}, 2.0f},
    [NODE_MONO_DEPTH] =
        {"mono_depth_net", run_mono_depth, 1, {NODE_VEHICLE_DET_2D}, 1.0f},
    [NODE_LEAD_FUSION] = {"lead_fusion",
                          run_lead_fusion,
                          3,
                          {NODE_VEHICLE_DET_3D, NODE_MONO_DEPTH,
                           NODE_LANE_GEOMETRY},
                          0.5f},
    [NODE_ACC] = {"acc", run_acc, 1, {NODE_LEAD_FUSION}, 0.2f},
    [NODE_LKA] = {"lka", run_lka, 1, {NODE_LANE_GEOMETRY}, 0.2f},
    [NODE_BEHAVIOR_PLANNER] = {"behavior_planner",
                               run_behavior_planner,
                               2,
                               {NODE_ACC, NODE_LKA},
                               0.3f},
};

/* ---------------------------------------------------------------------- */
/* Public API                                                             */
/* ---------------------------------------------------------------------- */

int adas_pipeline_init(AdasPipeline *p) {
  memset(p, 0, sizeof(*p));

  image_denoise_net_init(&p->mod.denoise);
  lane_seg_net_init(&p->mod.lane_seg);
  vehicle_detector_2d_init(&p->mod.det2d);
  point_filter_net_init(&p->mod.point_filter);
  vehicle_detector_3d_init(&p->mod.det3d);
  lane_geometry_net_init(&p->mod.lane_geometry);
  mono_depth_net_init(&p->mod.mono_depth);
  lead_fusion_init(&p->mod.fusion);
  lka_init(&p->mod.lka);
  behavior_planner_init(&p->mod.behavior);

  return scheduler_init(&p->sched, k_nodes, NODE_COUNT);
}

void adas_pipeline_run(AdasPipeline *p, const SensorFrame *sensors,
                       const DriverInput *driver) {
  p->data.sensors = *sensors;
  p->data.driver = *driver;
  scheduler_run(&p->sched, p, &p->stats);
}
