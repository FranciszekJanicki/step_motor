#ifndef STEP_MOTOR_STEP_MOTOR_H
#define STEP_MOTOR_STEP_MOTOR_H

#include "step_motor_config.h"
#include <stdbool.h>

typedef struct {
    step_motor_config_t config;
    step_motor_interface_t interface;
    step_motor_state_t state;
} step_motor_t;

step_motor_err_t step_motor_initialize(step_motor_t* motor,
                                       step_motor_config_t const* config,
                                       step_motor_interface_t const* interface,
                                       float32_t start_position);
step_motor_err_t step_motor_deinitialize(step_motor_t* motor);

step_motor_err_t step_motor_reset(step_motor_t* motor);

void step_motor_update_step_count(step_motor_t* motor);

step_motor_err_t step_motor_set_position(step_motor_t* motor,
                                         float32_t position,
                                         float32_t delta_time);
step_motor_err_t step_motor_set_speed(step_motor_t* motor, float32_t speed);
step_motor_err_t step_motor_set_acceleration(step_motor_t* motor,
                                             float32_t acceleration,
                                             float32_t delta_time);

float32_t step_motor_get_position(step_motor_t* motor);
float32_t step_motor_get_speed(step_motor_t* motor, float32_t delta_time);
float32_t step_motor_get_acceleration(step_motor_t* motor, float32_t delta_time);

#endif // STEP_MOTOR_STEP_MOTOR_H