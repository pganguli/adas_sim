/**
 * @file vehicle_detector_2d.h
 * @brief Camera vehicle detector (heat-map head + CenterNet-style decode).
 *
 *   input : DenoisedImage (1 x H x W)
 *   conv1 : Conv1x1(1->1) "darkness" feature, ReLU
 *   conv2 : Conv3x3(1->1) box filter -> objectness heat map, sigmoid
 *   decode: repeatedly take the heat-map peak, grow the box along the peak's
 *           row/column while heat > threshold, suppress it (NMS), repeat.
 *   output: Detections2D
 */
#ifndef ADAS_VEHICLE_DETECTOR_2D_H
#define ADAS_VEHICLE_DETECTOR_2D_H

#include <stdint.h>

#include "nn/nn.h"
#include "types/perception_types.h"

typedef struct {
  Conv2dLayer conv1; /**< 1x1 darkness feature */
  Conv2dLayer conv2; /**< 3x3 objectness heat map */
  /* scratch buffers (kept here so nothing large lives on the stack) */
  float feat[CAM_H][CAM_W];         /**< conv1 output          */
  float heat[CAM_H][CAM_W];         /**< conv2 output          */
  uint8_t suppressed[CAM_H][CAM_W]; /**< NMS mask (1 = masked) */
} VehicleDetector2D;

/** Bind the layers to their constant weights. */
void vehicle_detector_2d_init(VehicleDetector2D *net);

/** Detect up to DET2D_MAX_BOXES vehicles in one denoised image. */
void vehicle_detector_2d_infer(VehicleDetector2D *net, const DenoisedImage *in,
                               Detections2D *out);

#endif /* ADAS_VEHICLE_DETECTOR_2D_H */
