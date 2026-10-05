#ifndef AMR_MOTOR_H
#define AMR_MOTOR_H

#include <stdint.h>

#include "amr_protocol.h"
#include "amr_safety.h"

typedef struct {
    uint16_t wheel_base_mm;
    float wheel_diameter_mm;
    uint16_t maximum_wheel_speed_mm_s;
    uint16_t slow_wheel_speed_mm_s;
    bool allow_reverse;
} AmrMotorConfig;

typedef struct {
    int16_t left_target_mm_s;
    int16_t right_target_mm_s;
} AmrMotorTargets;

void AmrMotor_ComputeTargets(
    const AmrMotorConfig *config,
    const AmrSafetyManager *safety,
    const AmrDriveCommand *command,
    AmrMotorTargets *targets_out
);

void AmrMotor_ComputeWheelRpmTargets(
    const AmrMotorConfig *config,
    const AmrSafetyManager *safety,
    const AmrWheelCommand *command,
    AmrMotorTargets *targets_out
);

#endif
