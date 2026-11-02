/**
 * @file vehicle_detector_3d.c
 * @brief VehicleDetector3D implementation.
 */
#include "perception/vehicle_detector_3d.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "nn/dynamic.h"
#include "nn/nn.h"
#include "params.h"
#include "types/common_types.h"
#include "types/perception_types.h"
#include "types/sensor_types.h"

/* conv1: all-ones 3x3 count of neighborhood points, minus (MIN - 0.5). */
static const float k_conv1_w[1 * 1 * 3 * 3] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
static const float k_conv1_b[1] = {-((float)DET3D_MIN_POINTS - 0.5f)};

void vehicle_detector_3d_init(VehicleDetector3D *net) {
  const Conv2dLayer conv1 = {1, 1, 3, k_conv1_w, k_conv1_b, ACT_RELU, PAD_ZERO};
  net->conv1 = conv1;
}

/** Map an ego-frame point to its BEV cell. Returns false if outside the grid. */
static bool point_to_cell(const LidarPoint *p, int *ix, int *iy) {
  *ix = (int)floorf((p->x - BEV_X_MIN_M) / BEV_RES_M);
  *iy = (int)floorf((p->y - BEV_Y_MIN_M) / BEV_RES_M);
  return (*ix >= 0 && *ix < BEV_NX && *iy >= 0 && *iy < BEV_NY);
}

/** Scatter the points into the BEV occupancy-count grid. */
static void voxelize(VehicleDetector3D *net, const FilteredPointCloud *in) {
  memset(net->bev, 0, sizeof(net->bev));
  for (int i = 0; i < in->n_points; ++i) {
    int ix;
    int iy;
    if (point_to_cell(&in->points[i], &ix, &iy)) {
      net->bev[ix][iy] += 1.0f;
    }
  }
}

/** Fit an axis-aligned box to all points in the 3x3 cells around the peak. */
static BBox3D fit_box(const FilteredPointCloud *in, int pix, int piy) {
  Vec3 lo = {1e9f, 1e9f, 1e9f};
  Vec3 hi = {-1e9f, -1e9f, -1e9f};
  int n = 0;

  for (int i = 0; i < in->n_points; ++i) {
    const LidarPoint *p = &in->points[i];
    int ix;
    int iy;
    if (!point_to_cell(p, &ix, &iy) || abs(ix - pix) > 1 || abs(iy - piy) > 1) {
      continue;
    }
    lo.x = fminf(lo.x, p->x);
    hi.x = fmaxf(hi.x, p->x);
    lo.y = fminf(lo.y, p->y);
    hi.y = fmaxf(hi.y, p->y);
    lo.z = fminf(lo.z, p->z);
    hi.z = fmaxf(hi.z, p->z);
    n++;
  }

  BBox3D b;
  b.center.x = 0.5f * (lo.x + hi.x);
  b.center.y = 0.5f * (lo.y + hi.y);
  b.center.z = 0.5f * (lo.z + hi.z);
  b.size.x = hi.x - lo.x;
  b.size.y = hi.y - lo.y;
  b.size.z = hi.z - lo.z;
  b.n_points = n;
  b.score = 1.0f - expf(-(float)n / 10.0f);
  return b;
}

/** NMS: mask the 5x5 cells around a peak. */
static void suppress_cells(VehicleDetector3D *net, int pix, int piy) {
  for (int ix = pix - 2; ix <= pix + 2; ++ix) {
    for (int iy = piy - 2; iy <= piy + 2; ++iy) {
      if (ix >= 0 && ix < BEV_NX && iy >= 0 && iy < BEV_NY) {
        net->suppressed[ix][iy] = 1u;
      }
    }
  }
}

/** Peak -> box -> suppress decode of net->heat into `out`. */
static void decode(VehicleDetector3D *net, const FilteredPointCloud *in,
                   Detections3D *out) {
  out->hdr = in->hdr;
  out->n = 0;
  memset(net->suppressed, 0, sizeof(net->suppressed));

  while (out->n < DET3D_MAX_BOXES) {
    const int peak = nn_argmax_unmasked(
        &net->heat[0][0], &net->suppressed[0][0], BEV_NX * BEV_NY);
    if (peak < 0) {
      break;
    }
    const int pix = peak / BEV_NY;
    const int piy = peak % BEV_NY;
    out->boxes[out->n++] = fit_box(in, pix, piy);
    suppress_cells(net, pix, piy);
  }
}

void vehicle_detector_3d_infer(VehicleDetector3D *net,
                               const FilteredPointCloud *in,
                               Detections3D *out) {
  /* ---- Voxelization + conv head ---- */
  voxelize(net, in);
  const ConstTensor x = const_tensor_make(1, BEV_NX, BEV_NY, &net->bev[0][0]);
  Tensor h = tensor_make(1, BEV_NX, BEV_NY, &net->heat[0][0]);
  nn_conv2d(&net->conv1, &x, &h);

  decode(net, in, out);
}

float vehicle_detector_3d_infer_dynamic(VehicleDetector3D *net,
                                        const FilteredPointCloud *in,
                                        Detections3D *out, float quality) {
  const float q = dyn_clamp_quality(quality);
  const float min_center = ceilf((1.0f - q) * (float)DET3D_MIN_POINTS - 1e-4f);

  voxelize(net, in);
  const ConstTensor x = const_tensor_make(1, BEV_NX, BEV_NY, &net->bev[0][0]);
  Tensor h = tensor_make(1, BEV_NX, BEV_NY, &net->heat[0][0]);
  /* Exit gate: ReLU(count - (m - 0.5)); counts are integers, so it is
   * positive exactly for cells with at least m points. */
  const float gate_w[1] = {1.0f};
  const float gate_b[1] = {0.5f - min_center};
  const Conv2dLayer gate = {1, 1, 1, gate_w, gate_b, ACT_RELU, PAD_ZERO};
  Tensor g = tensor_make(1, BEV_NX, BEV_NY, &net->gate[0][0]);
  long macs = nn_conv2d_roi(&gate, &x, &g, 0, BEV_NX, NULL);
  const ConstTensor gin = tensor_as_const(&g);
  nn_exit_mask(&gin, &net->exited[0][0]);

  memset(net->heat, 0, sizeof(net->heat)); /* ReLU response of a skipped cell */
  macs += nn_conv2d_roi(&net->conv1, &x, &h, 0, BEV_NX, &net->exited[0][0]);

  decode(net, in, out);
  return dyn_cost(macs, nn_conv2d_macs(&net->conv1, BEV_NX, BEV_NY));
}
