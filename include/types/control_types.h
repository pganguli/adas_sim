/**
 * @file control_types.h
 * @brief Actuator commands (output of low-level control, input of the plant).
 */
#ifndef ADAS_CONTROL_TYPES_H
#define ADAS_CONTROL_TYPES_H

/** Actuator commands for one control step. */
typedef struct {
  float throttle;  /**< [0, 1]                            */
  float brake;     /**< [0, 1]                            */
  float steer_rad; /**< front-wheel angle command, left + */
} ActuatorCmd;

#endif /* ADAS_CONTROL_TYPES_H */
