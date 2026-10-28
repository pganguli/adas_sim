/**
 * @file point_filter_net.c
 * @brief PointFilterNet implementation.
 */
#include "perception/point_filter_net.h"

#include "nn/nn.h"
#include "params.h"
#include "types/perception_types.h"
#include "types/sensor_types.h"

/*
 * fc1: h0 = ReLU(10 z - 10 Z_MIN)   > 0 once the point is above the ground band
 *      h1 = ReLU(10 z - 10 Z_MAX)   > 0 once the point is above the height band
 * fc2: logit = 4 h0 - 30 h1 - 2     high inside the band, low outside
 * x and y weights are zero (kept as inputs so the net can be extended).
 */
static const float k_fc1_w[2 * 3] = {
    0.0f, 0.0f, 10.0f, 0.0f, 0.0f, 10.0f,
};
static const float k_fc1_b[2] = {-10.0f * PFILTER_Z_MIN_M,
                                 -10.0f * PFILTER_Z_MAX_M};
static const float k_fc2_w[1 * 2] = {4.0f, -30.0f};
static const float k_fc2_b[1] = {-2.0f};

void point_filter_net_init(PointFilterNet *net) {
  const DenseLayer fc1 = {3, 2, k_fc1_w, k_fc1_b, ACT_RELU};
  const DenseLayer fc2 = {2, 1, k_fc2_w, k_fc2_b, ACT_SIGMOID};
  net->fc1 = fc1;
  net->fc2 = fc2;
}

void point_filter_net_infer(PointFilterNet *net, const LidarScan *in,
                            FilteredPointCloud *out) {
  out->hdr = in->hdr;
  out->n_points = 0;

  for (int i = 0; i < in->n_points; ++i) {
    const LidarPoint *p = &in->points[i];
    const float feat[3] = {p->x, p->y, p->z};
    float hidden[2];
    float p_keep;

    nn_dense(&net->fc1, feat, hidden);
    nn_dense(&net->fc2, hidden, &p_keep);

    if (p_keep > PFILTER_KEEP_THRESH) {
      out->points[out->n_points++] = *p;
    }
  }
}
