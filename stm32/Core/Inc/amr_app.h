#ifndef AMR_APP_H
#define AMR_APP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "amr_motor.h"
#include "amr_protocol.h"
#include "amr_safety.h"
#include "amr_watchdog.h"

typedef struct {
    bool initialization_complete;
    bool estop_active;
    bool cliff_left;
    bool cliff_right;
    bool user_released;
    bool motor_fault;
    bool critical_sensor_fault;
    bool low_battery_warning;
    int16_t measured_left_velocity_mm_s;
    int16_t measured_right_velocity_mm_s;
    uint16_t battery_voltage_mv;
    uint16_t motor_error_code;
    uint16_t ultrasonic_front_mm;
    uint16_t sharp_left_mm;
    uint16_t sharp_right_mm;
    bool push_switch_pressed;
    int16_t requested_base_rpm;
    int16_t measured_left_velocity_rpm;
    int16_t measured_right_velocity_rpm;
} AmrHardwareInputs;

typedef struct {
    AmrProtocolParser parser;
    AmrSafetyManager safety;
    AmrCommandWatchdog watchdog;
    AmrMotorConfig motor_config;
    AmrDriveCommand last_command;
    AmrWheelCommand last_wheel_command;
    AmrMotorTargets motor_targets;
    AmrRobotStatus status;
    bool has_valid_command;
    bool has_valid_wheel_command;
    uint8_t tx_sequence;
} AmrApp;

void AmrApp_Init(
    AmrApp *app,
    const AmrMotorConfig *motor_config,
    uint32_t command_timeout_ms
);

AmrParseResult AmrApp_ProcessRxByte(
    AmrApp *app,
    uint8_t byte,
    uint32_t now_ms
);

void AmrApp_Tick(
    AmrApp *app,
    uint32_t now_ms,
    const AmrHardwareInputs *hardware
);

size_t AmrApp_EncodeStatus(
    AmrApp *app,
    uint8_t *frame_out,
    size_t frame_capacity
);

#endif
