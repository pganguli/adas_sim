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

/** out[out_dim] = act(W * in[in_dim] + b). */
void nn_dense(const DenseLayer *layer, const float *in, float *out);

#endif /* ADAS_NN_H */
