#include "amr_motor.h"

#include <stdbool.h>

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

void AmrMotor_ComputeTargets(
    const AmrMotorConfig *config,
    const AmrSafetyManager *safety,
    const AmrDriveCommand *command,
    AmrMotorTargets *targets_out
)
{
    int32_t linear;
    int32_t turn;
    int32_t left;
    int32_t right;
    int32_t state_limit;
    int32_t command_limit;
    int32_t final_limit;
    int32_t lower_linear;

    if (targets_out == NULL) {
        return;
    }
    targets_out->left_target_mm_s = 0;
    targets_out->right_target_mm_s = 0;

    if ((config == NULL)
        || (safety == NULL)
        || (command == NULL)
        || !AmrSafety_DriveAllowed(safety)) {
        return;
    }

    state_limit = (safety->state == AMR_STATE_SLOW)
        ? (int32_t)config->slow_wheel_speed_mm_s
        : (int32_t)config->maximum_wheel_speed_mm_s;
    command_limit = (int32_t)command->speed_limit_mm_s;
    final_limit = (command_limit < state_limit) ? command_limit : state_limit;
    if (final_limit <= 0) {
        return;
    }

    lower_linear = config->allow_reverse ? -final_limit : 0;
    linear = clamp_i32(
        (int32_t)command->linear_velocity_mm_s,
        lower_linear,
        final_limit
    );

    /*
     * angular[mrad/s] / 1000 * wheel_base[mm] / 2
     * = wheel tangential velocity[mm/s].
     */
    turn = ((int32_t)command->angular_velocity_mrad_s
        * (int32_t)config->wheel_base_mm) / 2000;
    left = clamp_i32(linear - turn, -final_limit, final_limit);
    right = clamp_i32(linear + turn, -final_limit, final_limit);
    targets_out->left_target_mm_s = (int16_t)left;
    targets_out->right_target_mm_s = (int16_t)right;
}

void AmrMotor_ComputeWheelRpmTargets(
    const AmrMotorConfig *config,
    const AmrSafetyManager *safety,
    const AmrWheelCommand *command,
    AmrMotorTargets *targets_out
)
{
    float circumference;
    int32_t left;
    int32_t right;
    int32_t limit;
    if (targets_out == NULL) {
        return;
    }
    targets_out->left_target_mm_s = 0;
    targets_out->right_target_mm_s = 0;
    if ((config == NULL) || (safety == NULL) || (command == NULL)
        || !AmrSafety_DriveAllowed(safety) || (config->wheel_diameter_mm <= 0.0f)) {
        return;
    }
    circumference = config->wheel_diameter_mm * 3.14159265358979323846f;
    left = (int32_t)((float)command->left_target_rpm * circumference / 60.0f);
    right = (int32_t)((float)command->right_target_rpm * circumference / 60.0f);
    limit = (safety->state == AMR_STATE_SLOW)
        ? (int32_t)config->slow_wheel_speed_mm_s
        : (int32_t)config->maximum_wheel_speed_mm_s;
    if (!config->allow_reverse) {
        left = clamp_i32(left, 0, limit);
        right = clamp_i32(right, 0, limit);
    } else {
        left = clamp_i32(left, -limit, limit);
        right = clamp_i32(right, -limit, limit);
    }
    targets_out->left_target_mm_s = (int16_t)left;
    targets_out->right_target_mm_s = (int16_t)right;
}
