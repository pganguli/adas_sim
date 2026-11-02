/**
 * @file test_dynamic.c
 * @brief Dynamic DNN variants: exact at quality 1, cheaper at lower quality.
 */
#include <stdbool.h>

#include "common/rng.h"
#include "estimation/lane_geometry_net.h"
#include "estimation/mono_depth_net.h"
#include "nn/dynamic.h"
#include "params.h"
#include "perception/image_denoise_net.h"
#include "perception/lane_seg_net.h"
#include "perception/point_filter_net.h"
#include "perception/vehicle_detector_2d.h"
#include "perception/vehicle_detector_3d.h"
#include "sim/camera_model.h"
#include "sim/lidar_model.h"
#include "sim/scenario.h"
#include "sim/world.h"
#include "test_util.h"
#include "types/estimation_types.h"
#include "types/perception_types.h"
#include "types/sensor_types.h"
#include "types/world_types.h"

/* Large objects are static so they don't live on the stack. */
static CameraFrame g_cam;
static LidarScan g_lidar;
static DenoisedImage g_den_s, g_den_d;
static LaneMask g_mask_s, g_mask_d;
static Detections2D g_d2_s, g_d2_d;
static FilteredPointCloud g_pc_s, g_pc_d;
static Detections3D g_d3_s, g_d3_d;
static LaneSegNet g_lane_seg;
static VehicleDetector2D g_det2d;
static VehicleDetector3D g_det3d;
static ImageDenoiseNet g_denoise;
static PointFilterNet g_pfilter;
static LaneGeometryNet g_geom_s, g_geom_d;
static MonoDepthNet g_depth;

static bool same_px(const float *a, const float *b, int n) {
  for (int i = 0; i < n; ++i) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

static bool same_box2d(const Detections2D *a, const Detections2D *b) {
  if (a->n != b->n) {
    return false;
  }
  for (int i = 0; i < a->n; ++i) {
    const BBox2D *p = &a->boxes[i];
    const BBox2D *q = &b->boxes[i];
    if (p->u_min != q->u_min || p->u_max != q->u_max || p->v_min != q->v_min ||
        p->v_max != q->v_max) {
      return false;
    }
  }
  return true;
}

static bool same_box3d(const Detections3D *a, const Detections3D *b) {
  if (a->n != b->n) {
    return false;
  }
  for (int i = 0; i < a->n; ++i) {
    const BBox3D *p = &a->boxes[i];
    const BBox3D *q = &b->boxes[i];
    if (p->n_points != q->n_points || p->center.x != q->center.x ||
        p->center.y != q->center.y || p->size.x != q->size.x) {
      return false;
    }
  }
  return true;
}

void test_dynamic(void) {
  const ScenarioConfig *sc = scenario_get_by_id(SCENARIO_SLOW_LEAD_APPROACH);
  WorldState w;
  Rng rc;
  Rng rl;
  world_init(&w, sc);
  rng_seed(&rc, 1u);
  rng_seed(&rl, 2u);
  camera_model_render(&w, &rc, 1u, &g_cam);
  lidar_model_scan(&w, &rl, 1u, &g_lidar);

  /* ---- clamp / keep-count helpers ---- */
  CHECK(dyn_keep_count(1.0f, 6) == 6);
  CHECK(dyn_keep_count(0.75f, 6) == 5);
  CHECK(dyn_keep_count(0.0f, 6) == 1);
  CHECK_NEAR(dyn_clamp_quality(2.0f), 1.0f, 0.0);
  CHECK_NEAR(dyn_clamp_quality(-1.0f), DYN_QUALITY_MIN, 0.0);

  /* ---- ImageDenoiseNet ---- */
  image_denoise_net_init(&g_denoise);
  image_denoise_net_infer(&g_denoise, &g_cam, &g_den_s);
  float c = image_denoise_net_infer_dynamic(&g_denoise, &g_cam, &g_den_d, 1.0f);
  CHECK(same_px(&g_den_s.px[0][0], &g_den_d.px[0][0], CAM_H * CAM_W));
  CHECK_NEAR(c, 1.0f, 1e-6);
  c = image_denoise_net_infer_dynamic(&g_denoise, &g_cam, &g_den_d, 0.5f);
  CHECK(c > 0.5f && c < 0.6f); /* half the rows + the 1x1 bypass */

  /* ---- LaneSegNet: quality 1 never exits early ---- */
  lane_seg_net_init(&g_lane_seg);
  lane_seg_net_infer(&g_lane_seg, &g_den_s, &g_mask_s);
  c = lane_seg_net_infer_dynamic(&g_lane_seg, &g_den_s, &g_mask_d, 1.0f);
  CHECK(same_px(&g_mask_s.prob[0][0], &g_mask_d.prob[0][0], CAM_H * CAM_W));
  CHECK_NEAR(c, 21.0f / 20.0f, 1e-6); /* + the 1x1 gate, nothing exits */
  const float c_lane75 =
      lane_seg_net_infer_dynamic(&g_lane_seg, &g_den_s, &g_mask_d, 0.75f);
  const float c_lane25 =
      lane_seg_net_infer_dynamic(&g_lane_seg, &g_den_s, &g_mask_d, 0.25f);
  CHECK(c_lane75 < 1.0f && c_lane25 <= c_lane75);

  /* ---- VehicleDetector2D: quality 1 early exit is exact ---- */
  vehicle_detector_2d_init(&g_det2d);
  vehicle_detector_2d_infer(&g_det2d, &g_den_s, &g_d2_s);
  c = vehicle_detector_2d_infer_dynamic(&g_det2d, &g_den_s, &g_d2_d, 1.0f);
  CHECK(same_box2d(&g_d2_s, &g_d2_d));
  CHECK(c < 1.0f);
  const float c_det75 =
      vehicle_detector_2d_infer_dynamic(&g_det2d, &g_den_s, &g_d2_d, 0.75f);
  CHECK(c_det75 <= c);

  /* ---- PointFilterNet: early exit is exact, pruning only below 0.5 ---- */
  point_filter_net_init(&g_pfilter);
  point_filter_net_infer(&g_pfilter, &g_lidar, &g_pc_s);
  c = point_filter_net_infer_dynamic(&g_pfilter, &g_lidar, &g_pc_d, 1.0f);
  CHECK(g_pc_s.n_points == g_pc_d.n_points);
  CHECK(c < 1.0f);
  point_filter_net_infer_dynamic(&g_pfilter, &g_lidar, &g_pc_d, 0.75f);
  CHECK(g_pc_s.n_points == g_pc_d.n_points); /* both hidden units kept */
  const float c_pf =
      point_filter_net_infer_dynamic(&g_pfilter, &g_lidar, &g_pc_d, 0.5f);
  CHECK(c_pf < c);
  CHECK(g_pc_d.n_points >= g_pc_s.n_points); /* height band dropped */

  /* ---- VehicleDetector3D ---- */
  vehicle_detector_3d_init(&g_det3d);
  vehicle_detector_3d_infer(&g_det3d, &g_pc_s, &g_d3_s);
  c = vehicle_detector_3d_infer_dynamic(&g_det3d, &g_pc_s, &g_d3_d, 1.0f);
  CHECK_NEAR(c, 10.0f / 9.0f, 1e-6); /* + the 1x1 gate, every cell runs */
  CHECK(same_box3d(&g_d3_s, &g_d3_d));
  c = vehicle_detector_3d_infer_dynamic(&g_det3d, &g_pc_s, &g_d3_d, 0.75f);
  CHECK(c < 1.0f);

  /* ---- LaneGeometryNet: all anchors at 1, fewer below ---- */
  lane_geometry_net_init(&g_geom_s);
  lane_geometry_net_init(&g_geom_d);
  LaneEstimate es;
  LaneEstimate ed;
  lane_geometry_net_infer(&g_geom_s, &g_mask_s, &es);
  c = lane_geometry_net_infer_dynamic(&g_geom_d, &g_mask_s, &ed, 1.0f);
  CHECK_NEAR(c, 1.0f, 1e-6);
  CHECK(es.valid && ed.valid);
  CHECK_NEAR(es.c0, ed.c0, 1e-6);
  CHECK_NEAR(es.c2, ed.c2, 1e-6);
  c = lane_geometry_net_infer_dynamic(&g_geom_d, &g_mask_s, &ed, 0.5f);
  CHECK_NEAR(c, 0.5f, 1e-6);
  CHECK(ed.valid);
  CHECK_NEAR(es.e_y_m, ed.e_y_m, 0.15); /* 3-anchor fit stays close */

  /* Every head of the default anchor layout fits. */
  for (int k = 3; k <= LANE_N_ANCHORS; ++k) {
    CHECK(g_geom_s.head_ok[k]);
  }
  /* A degenerate subset falls back to the next larger fitted one. */
  g_geom_d.head_ok[3] = false;
  c = lane_geometry_net_infer_dynamic(&g_geom_d, &g_mask_s, &ed, 0.5f);
  CHECK_NEAR(c, 4.0f / 6.0f, 1e-6);
  CHECK(ed.valid);
  /* No fitted full head: static and dynamic estimates are invalid. */
  g_geom_s.head_ok[LANE_N_ANCHORS] = false;
  lane_geometry_net_infer(&g_geom_s, &g_mask_s, &es);
  CHECK(!es.valid && !g_geom_s.prev.valid);
  g_geom_d.head_ok[4] = false;
  g_geom_d.head_ok[5] = false;
  g_geom_d.head_ok[LANE_N_ANCHORS] = false;
  c = lane_geometry_net_infer_dynamic(&g_geom_d, &g_mask_s, &ed, 0.5f);
  CHECK(!ed.valid);
  CHECK_NEAR(c, 1.0f, 1e-6); /* fell back all the way to the full set */

  /* ---- MonoDepthNet ---- */
  mono_depth_net_init(&g_depth);
  Detections2D boxes = {{0, 0}, 1, {{70, 70, 90, 84, 0.9f}}};
  DepthEstimates ds;
  DepthEstimates dd;
  mono_depth_net_infer(&g_depth, &boxes, &ds);
  c = mono_depth_net_infer_dynamic(&g_depth, &boxes, &dd, 1.0f);
  CHECK_NEAR(c, 1.0f, 1e-6);
  CHECK(ds.n == 1 && dd.n == 1);
  CHECK_NEAR(ds.objs[0].range_m, dd.objs[0].range_m, 1e-5);
  c = mono_depth_net_infer_dynamic(&g_depth, &boxes, &dd, 0.3f);
  CHECK(c < 0.5f);
  CHECK(dd.n == 1 && dd.objs[0].range_m > 0.0f);
  /* One hypothesis only: the ground-contact range, renormalized weight 1. */
  CHECK_NEAR(dd.objs[0].range_m, CAM_FX * CAM_HEIGHT_M / (85.0f - CAM_CV),
             1e-3);
}
