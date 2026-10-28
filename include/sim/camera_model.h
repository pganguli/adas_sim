/**
 * @file camera_model.h
 * @brief Synthetic front-camera renderer (flat road, lane markings, lead car).
 */
#ifndef ADAS_CAMERA_MODEL_H
#define ADAS_CAMERA_MODEL_H

#include "common/rng.h"
#include "types/sensor_types.h"
#include "types/world_types.h"

/**
 * Render the front-camera image of the current world state, including noise.
 *
 * @param w    ground-truth world
 * @param rng  noise source (advanced)
 * @param seq  pipeline frame counter stored in the header
 * @param out  rendered frame
 */
void camera_model_render(const WorldState *w, Rng *rng, uint32_t seq,
                         CameraFrame *out);

#endif /* ADAS_CAMERA_MODEL_H */
