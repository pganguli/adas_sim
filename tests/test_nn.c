/**
 * @file test_nn.c
 * @brief Tests for the dense and convolution layers.
 */
#include "nn/nn.h"
#include "test_util.h"

void test_nn(void) {
  CHECK_NEAR(nn_activate(ACT_RELU, -2.0f), 0.0f, 0.0);
  CHECK_NEAR(nn_activate(ACT_RELU, 2.0f), 2.0f, 0.0);
  CHECK_NEAR(nn_activate(ACT_SIGMOID, 0.0f), 0.5f, 1e-6);
  CHECK_NEAR(nn_activate(ACT_NONE, -2.0f), -2.0f, 0.0);

  /* Dense: out = W in + b, row-major weights. */
  const float dw[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  const float db[2] = {0.5f, -10.0f};
  const float din[2] = {1.0f, 1.0f};
  float dout[2];
  DenseLayer dense = {2, 2, dw, db, ACT_NONE};
  nn_dense(&dense, din, dout);
  CHECK_NEAR(dout[0], 3.5f, 1e-6);
  CHECK_NEAR(dout[1], -3.0f, 1e-6);
  dense.act = ACT_RELU;
  nn_dense(&dense, din, dout);
  CHECK_NEAR(dout[1], 0.0f, 0.0);

  /* Conv 3x3 with only the center tap set is the identity. */
  float img[9];
  float out[9];
  for (int i = 0; i < 9; ++i) {
    img[i] = (float)(i + 1);
  }
  const float ident[9] = {0, 0, 0, 0, 1, 0, 0, 0, 0};
  const float zero_bias[1] = {0.0f};
  Conv2dLayer conv = {1, 1, 3, ident, zero_bias, ACT_NONE, PAD_ZERO};
  const ConstTensor in = const_tensor_make(1, 3, 3, img);
  Tensor ot = tensor_make(1, 3, 3, out);
  nn_conv2d(&conv, &in, &ot);
  int identical = 1;
  for (int i = 0; i < 9; ++i) {
    identical = identical && (out[i] == img[i]);
  }
  CHECK(identical);

  /* Box filter on a constant image: border handling differs by padding. */
  const float ones_img[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
  const float box[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
  const ConstTensor ones = const_tensor_make(1, 3, 3, ones_img);
  conv.weight = box;
  conv.pad = PAD_ZERO;
  nn_conv2d(&conv, &ones, &ot);
  CHECK_NEAR(out[4], 9.0f, 1e-6); /* center */
  CHECK_NEAR(out[0], 4.0f, 1e-6); /* corner: 5 taps fall outside */
  conv.pad = PAD_REPLICATE;
  nn_conv2d(&conv, &ones, &ot);
  CHECK_NEAR(out[0], 9.0f, 1e-6);

  /* tensor_as_const keeps shape and data. */
  const ConstTensor view = tensor_as_const(&ot);
  CHECK(view.c == 1 && view.h == 3 && view.w == 3 && view.data == out);
}
