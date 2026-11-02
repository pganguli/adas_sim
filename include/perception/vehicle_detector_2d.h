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
 *
 * Dynamic variant (early exit): the heat-map conv is only evaluated where
 * the exit gate says so: MaxPool3x3(darkness) -> Conv1x1 ReLU(x - level) with
 * level = (1 - quality) * DET2D_EXIT_DARKNESS_MAX. Elsewhere the heat is the
 * conv's bias response. quality = 1 skips only all-zero neighborhoods, which
 * is exact.
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
  float gate[CAM_H][CAM_W];         /**< dynamic: exit-gate output      */
  uint8_t exited[CAM_H][CAM_W];     /**< dynamic: 1 = heat conv skipped */
} VehicleDetector2D;

/** Bind the layers to their constant weights. */
void vehicle_detector_2d_init(VehicleDetector2D *net);

/** Detect up to DET2D_MAX_BOXES vehicles in one denoised image. */
void vehicle_detector_2d_infer(VehicleDetector2D *net, const DenoisedImage *in,
                               Detections2D *out);

/** Early-exit variant; returns the executed MACs relative to the static net. */
float vehicle_detector_2d_infer_dynamic(VehicleDetector2D *net,
                                        const DenoisedImage *in,
                                        Detections2D *out, float quality);

#endif /* ADAS_VEHICLE_DETECTOR_2D_H */
