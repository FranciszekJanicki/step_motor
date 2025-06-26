#ifndef STEP_MOTOR_STEP_MOTOR_CONFIG_H
#define STEP_MOTOR_STEP_MOTOR_CONFIG_H

#include <stdint.h>

typedef float float32_t;

typedef enum {
    STEP_MOTOR_ERR_OK = 0,
    STEP_MOTOR_ERR_FAIL = 1 << 0,
    STEP_MOTOR_ERR_NULL = 1 << 1,
} step_motor_err_t;

typedef enum {
    STEP_MOTOR_DIRECTION_FORWARD,
    STEP_MOTOR_DIRECTION_BACKWARD,
    STEP_MOTOR_DIRECTION_STOP,
} step_motor_direction_t;

typedef struct {
    uint32_t frequency;
    int64_t step_count;
    float32_t prev_position;
    float32_t prev_speed;
    step_motor_direction_t direction;
} step_motor_state_t;

typedef struct {
    float32_t min_position;
    float32_t max_position;
    float32_t min_speed;
    float32_t max_speed;
    float32_t min_acceleration;
    float32_t max_acceleration;
    float32_t step_change;
} step_motor_config_t;

typedef struct {
    void* device_user;
    step_motor_err_t (*device_initialize)(void*);
    step_motor_err_t (*device_deinitialize)(void*);
    step_motor_err_t (*device_set_frequency)(void*, uint32_t);
    step_motor_err_t (*device_set_direction)(void*, step_motor_direction_t);
} step_motor_interface_t;

#endif // STEP_MOTOR_STEP_MOTOR_CONFIG_H