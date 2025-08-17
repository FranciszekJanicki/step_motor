#include "step_motor.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <string.h>

static step_motor_err_t step_motor_device_initialize(step_motor_t const* motor)
{
    return (motor->interface.device_initialize != NULL)
               ? motor->interface.device_initialize(
                     motor->interface.device_user)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_device_deinitialize(
    step_motor_t const* motor)
{
    return (motor->interface.device_deinitialize != NULL)
               ? motor->interface.device_deinitialize(
                     motor->interface.device_user)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_device_set_frequency(
    step_motor_t const* motor,
    uint32_t frequency)
{
    return (motor->interface.device_set_frequency != NULL)
               ? motor->interface.device_set_frequency(
                     motor->interface.device_user,
                     frequency)
               : STEP_MOTOR_ERR_NULL;
}

static step_motor_err_t step_motor_device_set_direction(
    step_motor_t const* motor,
    step_motor_direction_t direction)
{
    return (motor->interface.device_set_direction != NULL)
               ? motor->interface.device_set_direction(
                     motor->interface.device_user,
                     direction)
               : STEP_MOTOR_ERR_NULL;
}

static inline step_motor_err_t step_motor_set_direction(
    step_motor_t* motor,
    step_motor_direction_t direction)
{
    if (direction == motor->state.direction) {
        return STEP_MOTOR_ERR_OK;
    }

    motor->state.direction = direction;

    return step_motor_device_set_direction(motor, direction);
}

static inline step_motor_err_t step_motor_set_frequency(step_motor_t* motor,
                                                        uint32_t frequency)
{
    if (frequency == motor->state.frequency) {
        return STEP_MOTOR_ERR_OK;
    }

    motor->state.frequency = frequency;

    return step_motor_device_set_frequency(motor, frequency);
}

static inline float32_t step_motor_clamp_position(step_motor_t const* motor,
                                                  float32_t position)
{
    if (position < motor->config.min_position) {
        return motor->config.min_position;
    } else if (position > motor->config.max_position) {
        return motor->config.max_position;
    }

    return position;
}

static inline float32_t step_motor_clamp_speed(step_motor_t const* motor,
                                               float32_t speed)
{
    if (speed != 0.0F) {
        if (fabsf(speed) < motor->config.min_speed) {
            return copysignf(motor->config.min_speed, speed);
        } else if (fabsf(speed) > motor->config.max_speed) {
            return copysignf(motor->config.max_speed, speed);
        }
    }

    return speed;
}

static inline float32_t step_motor_clamp_acceleration(step_motor_t const* motor,
                                                      float32_t acceleration)
{
    if (acceleration != 0.0F) {
        if (fabsf(acceleration) < motor->config.min_acceleration) {
            return copysignf(motor->config.min_acceleration, acceleration);
        } else if (fabsf(acceleration) > motor->config.max_acceleration) {
            return copysignf(motor->config.max_acceleration, acceleration);
        }
    }

    return acceleration;
}

static inline step_motor_direction_t step_motor_speed_to_direction(
    step_motor_t const* motor,
    float32_t speed)
{
    if (fabsf(speed) < motor->config.min_speed) {
        return STEP_MOTOR_DIRECTION_STOP;
    }

    return (speed > 0.0F) ? STEP_MOTOR_DIRECTION_FORWARD
                          : STEP_MOTOR_DIRECTION_BACKWARD;
}

static inline uint32_t step_motor_speed_to_frequency(step_motor_t const* motor,
                                                     float32_t speed)
{
    if (motor->config.step_change <= 0.0F) {
        return 0U;
    }

    if (fabsf(speed) < motor->config.min_speed) {
        return 0U;
    }

    return (uint32_t)fabsf(speed / motor->config.step_change);
}

static inline float32_t step_motor_wrap_position(step_motor_t const* motor,
                                                 float32_t position)
{
    float32_t position_range =
        motor->config.max_position - motor->config.min_position;

    position = fmodf(position - motor->config.min_position, position_range);
    if (position < 0.0F) {
        position += position_range;
    }

    return position + motor->config.min_position;
}

static inline int64_t step_motor_position_to_step_count(
    step_motor_t const* motor,
    float32_t position)
{
    if (motor->config.step_change <= 0.0F) {
        return 0L;
    }

    if (motor->config.should_wrap_position) {
        position = step_motor_wrap_position(motor, position);
    }

    float32_t step_count = position / motor->config.step_change;

    return (int64_t)roundf(step_count);
}

static inline float32_t step_motor_step_count_to_position(
    step_motor_t const* motor,
    int64_t step_count)
{
    if (motor->config.step_change <= 0.0F) {
        return 0.0F;
    }

    float32_t position = (float32_t)step_count * motor->config.step_change;

    if (motor->config.should_wrap_position) {
        position = step_motor_wrap_position(motor, position);
    }

    return position;
}

step_motor_err_t step_motor_initialize(step_motor_t* motor,
                                       step_motor_config_t const* config,
                                       step_motor_interface_t const* interface,
                                       float32_t start_position)
{
    if (motor == NULL || config == NULL || interface == NULL) {
        return STEP_MOTOR_ERR_NULL;
    }

    memset(motor, 0, sizeof(*motor));
    memcpy(&motor->config, config, sizeof(*config));
    memcpy(&motor->interface, interface, sizeof(*interface));

    motor->state.frequency = 0U;
    motor->state.direction = STEP_MOTOR_DIRECTION_STOP;
    motor->state.step_count =
        step_motor_position_to_step_count(motor, start_position);

    return step_motor_device_initialize(motor);
}

step_motor_err_t step_motor_deinitialize(step_motor_t* motor)
{
    if (motor == NULL) {
        return STEP_MOTOR_ERR_NULL;
    }

    step_motor_err_t err = step_motor_device_deinitialize(motor);
    if (err != STEP_MOTOR_ERR_OK) {
        return err;
    }

    memset(motor, 0, sizeof(*motor));

    return STEP_MOTOR_ERR_OK;
}

step_motor_err_t step_motor_reset(step_motor_t* motor)
{
    if (motor == NULL) {
        return STEP_MOTOR_ERR_NULL;
    }

    step_motor_err_t err =
        step_motor_set_direction(motor, STEP_MOTOR_DIRECTION_STOP);
    if (err != STEP_MOTOR_ERR_OK) {
    }

    err = step_motor_set_frequency(motor, 0U);
    if (err != STEP_MOTOR_ERR_OK) {
    }

    motor->state.step_count = 0L;

    return STEP_MOTOR_ERR_OK;
}

step_motor_err_t step_motor_update_step_count(step_motor_t* motor)
{
    if (motor == NULL) {
        return STEP_MOTOR_ERR_NULL;
    }

    if (motor->state.direction == STEP_MOTOR_DIRECTION_BACKWARD &&
        motor->state.step_count != LLONG_MIN) {
        motor->state.step_count--;
    } else if (motor->state.direction == STEP_MOTOR_DIRECTION_FORWARD &&
               motor->state.step_count != LLONG_MAX) {
        motor->state.step_count++;
    }

    return STEP_MOTOR_ERR_OK;
}

step_motor_err_t step_motor_set_position(step_motor_t* motor,
                                         float32_t position,
                                         float32_t delta_time)
{
    if (motor == NULL) {
        return STEP_MOTOR_ERR_NULL;
    }

    if (delta_time <= 0.0F) {
        return STEP_MOTOR_ERR_FAIL;
    }

    float32_t current_position;
    step_motor_err_t err = step_motor_get_position(motor, &current_position);
    if (err != STEP_MOTOR_ERR_OK) {
        return err;
    }

    position = step_motor_clamp_position(motor, position);
    float32_t speed = (position - current_position) / delta_time;

    return step_motor_set_speed(motor, speed);
}

step_motor_err_t step_motor_set_speed(step_motor_t* motor, float32_t speed)
{
    if (motor == NULL) {
        return STEP_MOTOR_ERR_NULL;
    }

    step_motor_direction_t direction =
        step_motor_speed_to_direction(motor, speed);

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
    if (motor == NULL) {
        return STEP_MOTOR_ERR_NULL;
    }

    if (delta_time <= 0.0F) {
        return STEP_MOTOR_ERR_FAIL;
    }

    float32_t current_acceleration;
    step_motor_err_t err =
        step_motor_get_acceleration(motor, &current_acceleration, delta_time);
    if (err != STEP_MOTOR_ERR_OK) {
        return err;
    }

    acceleration = step_motor_clamp_acceleration(motor, acceleration);
    float32_t speed = (acceleration + current_acceleration) * delta_time / 2.0F;

    return step_motor_set_speed(motor, speed);
}

step_motor_err_t step_motor_get_position(step_motor_t* motor,
                                         float32_t* position)
{
    if (motor == NULL || position == NULL) {
        return STEP_MOTOR_ERR_NULL;
    }

    *position =
        step_motor_step_count_to_position(motor, motor->state.step_count);

    return STEP_MOTOR_ERR_OK;
}

step_motor_err_t step_motor_get_speed(step_motor_t* motor,
                                      float32_t* speed,
                                      float32_t delta_time)
{
    if (motor == NULL || speed == NULL) {
        return STEP_MOTOR_ERR_NULL;
    }

    if (delta_time <= 0.0F) {
        return STEP_MOTOR_ERR_FAIL;
    }

    float32_t position;
    step_motor_err_t err = step_motor_get_position(motor, &position);
    if (err != STEP_MOTOR_ERR_OK) {
        return err;
    }

    *speed = (position - motor->state.prev_position) / delta_time;
    motor->state.prev_position = position;

    return STEP_MOTOR_ERR_OK;
}

step_motor_err_t step_motor_get_acceleration(step_motor_t* motor,
                                             float32_t* acceleration,
                                             float32_t delta_time)
{
    if (motor == NULL || acceleration == NULL) {
        return STEP_MOTOR_ERR_NULL;
    }

    if (delta_time <= 0.0F) {
        return STEP_MOTOR_ERR_FAIL;
    }

    float32_t speed;
    step_motor_err_t err = step_motor_get_speed(motor, &speed, delta_time);
    if (err != STEP_MOTOR_ERR_OK) {
        return err;
    }

    *acceleration = (speed - motor->state.prev_speed) / delta_time;
    motor->state.prev_speed = speed;

    return STEP_MOTOR_ERR_OK;
}