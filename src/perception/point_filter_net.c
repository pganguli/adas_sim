/**
 * @file point_filter_net.c
 * @brief PointFilterNet implementation.
 */
#include "perception/point_filter_net.h"

#include "nn/dynamic.h"
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

float point_filter_net_infer_dynamic(PointFilterNet *net, const LidarScan *in,
                                     FilteredPointCloud *out, float quality) {
  const int keep = dyn_keep_count(quality, net->fc1.out_dim);
  /* Hidden units are weight rows: unit 0 alone, the rest as a second layer. */
  DenseLayer first = net->fc1;
  first.out_dim = 1;
  DenseLayer rest = net->fc1;
  rest.out_dim = keep - 1;
  rest.weight += net->fc1.in_dim;
  rest.bias += 1;
  DenseLayer head = net->fc2;
  head.in_dim = keep; /* single output row: the first `keep` weights */
  long macs = 0;

  out->hdr = in->hdr;
  out->n_points = 0;

  for (int i = 0; i < in->n_points; ++i) {
    const LidarPoint *p = &in->points[i];
    const float feat[3] = {p->x, p->y, p->z};
    float hidden[2] = {0.0f, 0.0f};
    float p_keep;

    nn_dense(&first, feat, &hidden[0]);
    macs += nn_dense_macs(&first);
    if (hidden[0] <= 0.0f) {
      continue; /* early exit: below the ground band, logit <= bias */
    }
    if (keep > 1) {
      nn_dense(&rest, feat, &hidden[1]);
      macs += nn_dense_macs(&rest);
    }
    nn_dense(&head, hidden, &p_keep);
    macs += nn_dense_macs(&head);

    if (p_keep > PFILTER_KEEP_THRESH) {
      out->points[out->n_points++] = *p;
    }
  }
  return dyn_cost(macs, (nn_dense_macs(&net->fc1) + nn_dense_macs(&net->fc2)) *
                            in->n_points);
}
