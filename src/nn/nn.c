/**
 * @file nn.c
 * @brief Reference (naive, readable) implementations of the NN layers.
 */
#include "nn/nn.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include "common/math_utils.h"

float nn_activate(Activation act, float x) {
  switch (act) {
  case ACT_RELU:
    return (x > 0.0f) ? x : 0.0f;
  case ACT_SIGMOID:
    return sigmoidf(x);
  case ACT_NONE:
  default:
    return x;
  }
}

void nn_conv2d(const Conv2dLayer *layer, const ConstTensor *in, Tensor *out) {
  assert(in != NULL);
  nn_conv2d_roi(layer, in, out, 0, in->h, NULL);
}

long nn_conv2d_macs(const Conv2dLayer *layer, int h, int w) {
  return (long)layer->out_c * layer->in_c * layer->k * layer->k * h * w;
}

long nn_conv2d_roi(const Conv2dLayer *layer, const ConstTensor *in, Tensor *out,
                   int y_begin, int y_end, const uint8_t *skip) {
  assert(layer != NULL && in != NULL && out != NULL);
  assert(in->c == layer->in_c && out->c == layer->out_c);
  assert(in->h == out->h && in->w == out->w);
  assert((layer->k % 2) == 1);
  assert(y_begin >= 0 && y_begin <= y_end && y_end <= in->h);

  const int r = layer->k / 2;
  const int h = in->h;
  const int w = in->w;
  const int k = layer->k;
  long macs = 0;

  for (int oc = 0; oc < layer->out_c; ++oc) {
    for (int y = y_begin; y < y_end; ++y) {
      for (int x = 0; x < w; ++x) {
        if (skip != NULL && skip[y * w + x] != 0u) {
          continue;
        }
        float acc = (layer->bias != NULL) ? layer->bias[oc] : 0.0f;
        macs += (long)layer->in_c * k * k;

        for (int ic = 0; ic < layer->in_c; ++ic) {
          for (int ky = 0; ky < k; ++ky) {
            for (int kx = 0; kx < k; ++kx) {
              int yy = y + ky - r;
              int xx = x + kx - r;
              if (yy < 0 || yy >= h || xx < 0 || xx >= w) {
                if (layer->pad == PAD_ZERO) {
                  continue;
                }
                yy = clampi(yy, 0, h - 1);
                xx = clampi(xx, 0, w - 1);
              }
              acc +=
                  layer->weight[((oc * layer->in_c + ic) * k + ky) * k + kx] *
                  in->data[(ic * h + yy) * w + xx];
            }
          }
        }
        out->data[(oc * h + y) * w + x] = nn_activate(layer->act, acc);
      }
    }
  }
  return macs;
}

void nn_dense(const DenseLayer *layer, const float *in, float *out) {
  assert(layer != NULL && in != NULL && out != NULL);

  for (int o = 0; o < layer->out_dim; ++o) {
    float acc = (layer->bias != NULL) ? layer->bias[o] : 0.0f;
    for (int i = 0; i < layer->in_dim; ++i) {
      acc += layer->weight[o * layer->in_dim + i] * in[i];
    }
    out[o] = nn_activate(layer->act, acc);
  }
}

int nn_argmax_unmasked(const float *data, const uint8_t *mask, int n) {
  int best = -1;
  float best_val = 0.0f;
  for (int i = 0; i < n; ++i) {
    if (!mask[i] && data[i] > best_val) {
      best_val = data[i];
      best = i;
    }
  }
  return best;
}

void nn_maxpool2d(const ConstTensor *in, Tensor *out, int k) {
  assert(in != NULL && out != NULL);
  assert(in->c == out->c && in->h == out->h && in->w == out->w);
  assert((k % 2) == 1);

  const int r = k / 2;
  for (int c = 0; c < in->c; ++c) {
    for (int y = 0; y < in->h; ++y) {
      for (int x = 0; x < in->w; ++x) {
        float best = in->data[(c * in->h + y) * in->w + x];
        for (int dy = -r; dy <= r; ++dy) {
          for (int dx = -r; dx <= r; ++dx) {
            const int yy = y + dy;
            const int xx = x + dx;
            if (yy >= 0 && yy < in->h && xx >= 0 && xx < in->w) {
              const float v = in->data[(c * in->h + yy) * in->w + xx];
              best = (v > best) ? v : best;
            }
          }
        }
        out->data[(c * in->h + y) * in->w + x] = best;
      }
    }
  }
}

void nn_exit_mask(const ConstTensor *gate, uint8_t *skip) {
  assert(gate != NULL && skip != NULL);
  const int n = gate->c * gate->h * gate->w;
  for (int i = 0; i < n; ++i) {
    skip[i] = (uint8_t)((gate->data[i] > 0.0f) ? 0u : 1u);
  }
}
