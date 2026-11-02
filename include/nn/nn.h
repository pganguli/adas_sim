/**
 * @file nn.h
 * @brief Minimal neural-network inference library (no heap, no dependencies).
 *
 * Provides the two layer types every DNN in this project is built from:
 *   - Conv2dLayer : stride-1 "same" 2D convolution, CHW layout
 *   - DenseLayer  : fully connected layer
 * Each layer applies its own activation. Weights are plain arrays owned by
 * the network that uses the layer (here: hand-set constants; in a real
 * system they would be loaded from a trained checkpoint).
 */
#ifndef ADAS_NN_H
#define ADAS_NN_H

#include <stdint.h>

/** Activation function applied to a layer's output. */
typedef enum {
  ACT_NONE = 0, /**< identity      */
  ACT_RELU,     /**< max(0, x)     */
  ACT_SIGMOID   /**< 1 / (1 + e^-x) */
} Activation;

/** How a convolution treats pixels outside the input. */
typedef enum {
  PAD_ZERO = 0, /**< outside pixels are 0                 */
  PAD_REPLICATE /**< outside pixels copy the nearest edge */
} Padding;

/** Non-owning view of a C x H x W float tensor stored contiguously (CHW). */
typedef struct {
  int c;       /**< channels */
  int h;       /**< height   */
  int w;       /**< width    */
  float *data; /**< c * h * w floats, not owned */
} Tensor;

/** Read-only counterpart of Tensor, for layer inputs. */
typedef struct {
  int c;             /**< channels */
  int h;             /**< height   */
  int w;             /**< width    */
  const float *data; /**< c * h * w floats, not owned */
} ConstTensor;

/**
 * Conv2d weights layout: weight[out_c][in_c][k][k], bias[out_c].
 * k must be odd ("same" output size).
 */
typedef struct {
  int in_c;            /**< input channels                 */
  int out_c;           /**< output channels                */
  int k;               /**< kernel size (odd)              */
  const float *weight; /**< [out_c][in_c][k][k], not owned */
  const float *bias;   /**< [out_c] or NULL, not owned     */
  Activation act;      /**< applied to each output         */
  Padding pad;         /**< border handling                */
} Conv2dLayer;

/** Dense weights layout: weight[out_dim][in_dim] (row-major), bias[out_dim]. */
typedef struct {
  int in_dim;          /**< input length                 */
  int out_dim;         /**< output length                */
  const float *weight; /**< [out_dim][in_dim], not owned */
  const float *bias;   /**< [out_dim] or NULL, not owned */
  Activation act;      /**< applied to each output       */
} DenseLayer;

/** Wrap an existing buffer as a c x h x w tensor (no copy). */
static inline Tensor tensor_make(int c, int h, int w, float *data) {
  Tensor t = {c, h, w, data};
  return t;
}

/** Wrap an existing read-only buffer as a c x h x w tensor (no copy). */
static inline ConstTensor const_tensor_make(int c, int h, int w,
                                            const float *data) {
  ConstTensor t = {c, h, w, data};
  return t;
}

/** Read-only view of a writable tensor. */
static inline ConstTensor tensor_as_const(const Tensor *t) {
  return const_tensor_make(t->c, t->h, t->w, t->data);
}

/** Apply the activation function `act` to `x`. */
float nn_activate(Activation act, float x);

/**
 * out = act(conv(in)). `in` is read-only; `out` must already have the right
 * shape.
 */
void nn_conv2d(const Conv2dLayer *layer, const ConstTensor *in, Tensor *out);

/**
 * Partial convolution for dynamic networks: only output rows
 * [y_begin, y_end) are computed, and within them every pixel whose `skip`
 * entry (h * w bytes, one per pixel, shared by all channels) is nonzero is
 * left untouched. `skip` may be NULL. Pixels that are not computed keep
 * whatever the caller pre-filled them with. Returns the number of
 * multiply-accumulates executed (padding taps count, so the full layer costs
 * nn_conv2d_macs()).
 */
long nn_conv2d_roi(const Conv2dLayer *layer, const ConstTensor *in, Tensor *out,
                   int y_begin, int y_end, const uint8_t *skip);

/**
 * k x k max-pool, stride 1, "same" size (k odd, outside pixels ignored),
 * applied per channel. `in` and `out` must have the same shape.
 */
void nn_maxpool2d(const ConstTensor *in, Tensor *out, int k);

/**
 * Early-exit decision from a gate network's output (a ReLU-ed map):
 * skip[i] = 1 where the gate is 0, i.e. the pixel exits.
 */
void nn_exit_mask(const ConstTensor *gate, uint8_t *skip);

/** Multiply-accumulates of one full convolution over an h x w input. */
long nn_conv2d_macs(const Conv2dLayer *layer, int h, int w);

/** out[out_dim] = act(W * in[in_dim] + b). */
void nn_dense(const DenseLayer *layer, const float *in, float *out);

/** Multiply-accumulates of one dense layer evaluation. */
static inline long nn_dense_macs(const DenseLayer *layer) {
  return (long)layer->in_dim * layer->out_dim;
}

/**
 * Index of the first maximum of data[0..n) among entries with mask[i] == 0
 * and data[i] > 0; -1 if there is none. This is the peak step of the
 * detectors' peak-decode + NMS loop over a heat map.
 */
int nn_argmax_unmasked(const float *data, const uint8_t *mask, int n);

#endif /* ADAS_NN_H */
