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
  assert(layer != NULL && in != NULL && out != NULL);
  assert(in->c == layer->in_c && out->c == layer->out_c);
  assert(in->h == out->h && in->w == out->w);
  assert((layer->k % 2) == 1);

  const int r = layer->k / 2;
  const int h = in->h;
  const int w = in->w;
  const int k = layer->k;

  for (int oc = 0; oc < layer->out_c; ++oc) {
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        float acc = (layer->bias != NULL) ? layer->bias[oc] : 0.0f;

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
