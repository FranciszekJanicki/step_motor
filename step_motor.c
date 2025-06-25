#include "step_motor.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <string.h>

static step_motor_err_t step_motor_device_initialize(step_motor_t const* motor)
{
    return motor->interface.device_initialize
               ? motor->interface.device_initialize(motor->interface.device_user)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_device_deinitialize(step_motor_t const* motor)
{
    return motor->interface.device_deinitialize
               ? motor->interface.device_deinitialize(motor->interface.device_user)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_device_set_frequency(step_motor_t const* motor,
                                                        uint32_t frequency)
{
    return motor->interface.device_set_frequency
               ? motor->interface.device_set_frequency(motor->interface.device_user, frequency)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_device_set_direction(step_motor_t const* motor,
                                                        step_motor_direction_t direction)
{
    return motor->interface.device_set_direction
               ? motor->interface.device_set_direction(motor->interface.device_user, direction)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_set_direction(step_motor_t* motor,
                                                 step_motor_direction_t direction)
{
    if (direction == motor->state.direction) {
        return STEP_MOTOR_ERR_OK;
    }

    motor->state.direction = direction;

    return step_motor_device_set_direction(motor, direction);
}

static step_motor_err_t step_motor_set_frequency(step_motor_t* motor, uint32_t frequency)
{
    if (frequency == motor->state.frequency) {
        return STEP_MOTOR_ERR_OK;
    }

    motor->state.frequency = frequency;

    return step_motor_device_set_frequency(motor, frequency);
}

static inline float32_t step_motor_clamp_position(step_motor_t const* motor, float32_t position)
{
    if (position < motor->config.min_position) {
        position = motor->config.min_position;
    } else if (position > motor->config.max_position) {
        position = motor->config.max_position;
    }

    return position;
}

static inline float32_t step_motor_clamp_speed(step_motor_t const* motor, float32_t speed)
{
    if (speed != 0.0F) {
        if (fabsf(speed) < motor->config.min_speed) {
            speed = copysignf(motor->config.min_speed, speed);
        } else if (fabsf(speed) > motor->config.max_speed) {
            speed = copysignf(motor->config.max_speed, speed);
        }
    }

    return speed;
}

static inline float32_t step_motor_clamp_acceleration(step_motor_t const* motor,
                                                      float32_t acceleration)
{
    if (acceleration != 0.0F) {
        if (fabsf(acceleration) < motor->config.min_acceleration) {
            acceleration = copysignf(motor->config.min_acceleration, acceleration);
        } else if (fabsf(acceleration) > motor->config.max_acceleration) {
            acceleration = copysignf(motor->config.max_acceleration, acceleration);
        }
    }

    return acceleration;
}

static inline step_motor_direction_t step_motor_speed_to_direction(step_motor_t const* motor,
                                                                   float32_t speed)
{
    if (fabsf(speed) < motor->config.min_speed /*||
        fabsf(speed) < (motor->config.step_change / delta_time)*/) {
        return STEP_MOTOR_DIRECTION_STOP;
    }

    return speed > 0.0F ? STEP_MOTOR_DIRECTION_FORWARD : STEP_MOTOR_DIRECTION_BACKWARD;
}

static inline uint32_t step_motor_speed_to_frequency(step_motor_t const* motor, float32_t speed)
{
    assert(motor->config.step_change > 0.0F);

    if (fabsf(speed) < motor->config.min_speed /*||
        fabsf(speed) < (motor->config.step_change / delta_time)*/) {
        return 0U;
    }

    return (uint32_t)fabsf(speed / motor->config.step_change);
}

static inline float32_t step_motor_wrap_position(float32_t position)
{
    position = fmodf(position, 360.0F);

    while (position < 0.0F) {
        position += 360.0F;
    }
    if (position >= 360.0F) {
        position -= 360.0F;
    }

    return position;
}

static inline int64_t step_motor_position_to_step_count(step_motor_t const* motor,
                                                        float32_t position)
{
    assert(motor->config.step_change > 0.0F);

    float32_t step_count = step_motor_wrap_position(position) / motor->config.step_change;

    return (int64_t)roundf(step_count);
}

static inline float32_t step_motor_step_count_to_position(step_motor_t const* motor,
                                                          int64_t step_count)
{
    assert(motor->config.step_change > 0.0F);

    float32_t position = (float32_t)step_count * motor->config.step_change;

    return step_motor_wrap_position(position);
}

step_motor_err_t step_motor_initialize(step_motor_t* motor,
                                       step_motor_config_t const* config,
                                       step_motor_interface_t const* interface,
                                       float32_t start_position)
{
    assert(motor && config && interface);

    memset(motor, 0, sizeof(*motor));
    memcpy(&motor->config, config, sizeof(*config));
    memcpy(&motor->interface, interface, sizeof(*interface));

    motor->state.frequency = 0UL;
    motor->state.direction = STEP_MOTOR_DIRECTION_STOP;
    motor->state.step_count = step_motor_position_to_step_count(motor, start_position);

    return step_motor_device_initialize(motor);
}

step_motor_err_t step_motor_deinitialize(step_motor_t* motor)
{
    assert(motor);

    step_motor_err_t err = step_motor_device_deinitialize(motor);

    memset(motor, 0, sizeof(*motor));

    return err;
}

step_motor_err_t step_motor_reset(step_motor_t* motor)
{
    assert(motor);

    step_motor_err_t err = step_motor_set_direction(motor, STEP_MOTOR_DIRECTION_STOP);
    err |= step_motor_set_frequency(motor, 0UL);

    motor->state.step_count = 0L;

    return err;
}

void step_motor_update_step_count(step_motor_t* motor)
{
    assert(motor);

    if (motor->state.direction == STEP_MOTOR_DIRECTION_BACKWARD &&
        motor->state.step_count != LLONG_MIN) {
        motor->state.step_count--;
    } else if (motor->state.direction == STEP_MOTOR_DIRECTION_FORWARD &&
               motor->state.step_count != LLONG_MAX) {
        motor->state.step_count++;
    }
}

step_motor_err_t step_motor_set_position(step_motor_t* motor,
                                         float32_t position,
                                         float32_t delta_time)
{
    assert(motor && delta_time > 0.0F);

    position = step_motor_clamp_position(motor, position);

    float32_t current_position = step_motor_get_position(motor);
    float32_t speed = (position - current_position) / delta_time;

    return step_motor_set_speed(motor, speed);
}

step_motor_err_t step_motor_set_speed(step_motor_t* motor, float32_t speed)
{
    assert(motor);

    step_motor_direction_t direction = step_motor_speed_to_direction(motor, speed);

    step_motor_err_t err = step_motor_set_direction(motor, direction);
    if (err != STEP_MOTOR_ERR_OK || direction == STEP_MOTOR_DIRECTION_STOP) {
        return err;
    }

    speed = step_motor_clamp_speed(motor, speed);
    uint32_t frequency = step_motor_speed_to_frequency(motor, speed);

    return step_motor_set_frequency(motor, frequency);
}

step_motor_err_t step_motor_set_acceleration(step_motor_t* motor,
                                             float32_t acceleration,
                                             float32_t delta_time)
{
    assert(motor && delta_time > 0.0F);

    acceleration = step_motor_clamp_acceleration(motor, acceleration);

    float32_t current_acceleration = step_motor_get_acceleration(motor, delta_time);
    float32_t speed = (acceleration + current_acceleration) * delta_time / 2.0F;

    return step_motor_set_speed(motor, speed);
}

float32_t step_motor_get_position(step_motor_t* motor)
{
    assert(motor);

    float32_t position = step_motor_step_count_to_position(motor, motor->state.step_count);

    return position;
}

float32_t step_motor_get_speed(step_motor_t* motor, float32_t delta_time)
{
    assert(motor && delta_time > 0.0F);

    float32_t position = step_motor_get_position(motor);
    float32_t speed = (position - motor->state.prev_position) / delta_time;

    motor->state.prev_position = position;

    return speed;
}

float32_t step_motor_get_acceleration(step_motor_t* motor, float32_t delta_time)
{
    assert(motor && delta_time > 0.0F);

    float32_t speed = step_motor_get_speed(motor, delta_time);
    float32_t acceleration = (speed - motor->state.prev_speed) / delta_time;

    motor->state.prev_speed = speed;

    return acceleration;
}