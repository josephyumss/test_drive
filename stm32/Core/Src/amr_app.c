#include "amr_app.h"

#include <string.h>

void AmrApp_Init(
    AmrApp *app,
    const AmrMotorConfig *motor_config,
    uint32_t command_timeout_ms
)
{
    if ((app == NULL) || (motor_config == NULL)) {
        return;
    }

    memset(app, 0, sizeof(*app));
    AmrProtocolParser_Init(&app->parser);
    AmrSafety_Init(&app->safety);
    AmrWatchdog_Init(&app->watchdog, command_timeout_ms);
    app->motor_config = *motor_config;
    app->status.system_state = AMR_STATE_INIT;
}

AmrParseResult AmrApp_ProcessRxByte(
    AmrApp *app,
    uint8_t byte,
    uint32_t now_ms
)
{
    AmrPacket packet;
    AmrDriveCommand command;
    AmrWheelCommand wheel_command;
    AmrParseResult result;

    if (app == NULL) {
        return AMR_PARSE_FORMAT_ERROR;
    }

    result = AmrProtocolParser_Feed(&app->parser, byte, &packet);
    if (result != AMR_PARSE_PACKET) {
        return result;
    }

    if (packet.message_id == AMR_MSG_WHEEL_COMMAND) {
        if (!AmrProtocol_DecodeWheelCommand(&packet, &wheel_command)) {
            app->parser.format_error_count++;
            return AMR_PARSE_FORMAT_ERROR;
        }
        app->last_wheel_command = wheel_command;
        app->has_valid_wheel_command = true;
        app->has_valid_command = false;
        AmrWatchdog_Kick(&app->watchdog, now_ms);
        return AMR_PARSE_PACKET;
    }
    if (packet.message_id != AMR_MSG_DRIVE_COMMAND) {
        return result;
    }
    if (!AmrProtocol_DecodeDriveCommand(&packet, &command)) {
        app->parser.format_error_count++;
        return AMR_PARSE_FORMAT_ERROR;
    }

    app->last_command = command;
    app->has_valid_command = true;
    app->has_valid_wheel_command = false;
    AmrWatchdog_Kick(&app->watchdog, now_ms);
    return AMR_PARSE_PACKET;
}

void AmrApp_Tick(
    AmrApp *app,
    uint32_t now_ms,
    const AmrHardwareInputs *hardware
)
{
    AmrSafetyInputs safety_inputs;
    uint8_t flags;

    if ((app == NULL) || (hardware == NULL)) {
        return;
    }

    memset(&safety_inputs, 0, sizeof(safety_inputs));
    flags = app->has_valid_wheel_command
        ? app->last_wheel_command.control_flags
        : (app->has_valid_command ? app->last_command.control_flags : 0U);
    safety_inputs.initialization_complete = hardware->initialization_complete;
    safety_inputs.drive_enable =
        (flags & AMR_DRIVE_ENABLE) != 0U;
    safety_inputs.slow_request =
        (flags & AMR_DRIVE_SLOW_MODE) != 0U;
    safety_inputs.controlled_stop_request =
        (flags & AMR_DRIVE_CONTROLLED_STOP) != 0U;
    safety_inputs.reset_request =
        (flags & AMR_DRIVE_RESET_REQUEST) != 0U;
    safety_inputs.estop_active = hardware->estop_active
        || (app->has_valid_wheel_command
            && (app->last_wheel_command.emergency != 0U));
    safety_inputs.cliff_left = hardware->cliff_left;
    safety_inputs.cliff_right = hardware->cliff_right;
    safety_inputs.communication_timeout =
        AmrWatchdog_IsTimedOut(&app->watchdog, now_ms);
    safety_inputs.user_released = hardware->user_released;
    safety_inputs.motor_fault = hardware->motor_fault;
    safety_inputs.critical_sensor_fault = hardware->critical_sensor_fault;
    safety_inputs.low_battery_warning = hardware->low_battery_warning;

    AmrSafety_Update(&app->safety, &safety_inputs);
    if (app->has_valid_wheel_command) {
        AmrMotor_ComputeWheelRpmTargets(
            &app->motor_config,
            &app->safety,
            &app->last_wheel_command,
            &app->motor_targets
        );
    } else {
        AmrMotor_ComputeTargets(
            &app->motor_config,
            &app->safety,
            app->has_valid_command ? &app->last_command : NULL,
            &app->motor_targets
        );
    }

    app->status.system_state = app->safety.state;
    app->status.safety_flags = app->safety.flags;
    if (app->parser.version_error_count > 0U) {
        app->status.safety_flags |= AMR_SAFETY_PROTOCOL_MISMATCH;
    }
    app->status.left_velocity_mm_s =
        hardware->measured_left_velocity_mm_s;
    app->status.right_velocity_mm_s =
        hardware->measured_right_velocity_mm_s;
    app->status.battery_voltage_mv = hardware->battery_voltage_mv;
    app->status.motor_error = hardware->motor_error_code;
    app->status.last_command_id =
        app->has_valid_wheel_command
            ? app->last_wheel_command.command_id
            : (app->has_valid_command ? app->last_command.command_id : 0U);
    app->status.rx_error_count = (uint16_t)(
        app->parser.crc_error_count
        + app->parser.format_error_count
        + app->parser.version_error_count
    );
    app->status.uptime_ms = now_ms;
    app->status.ultrasonic_front_mm = hardware->ultrasonic_front_mm;
    app->status.sharp_left_mm = hardware->sharp_left_mm;
    app->status.sharp_right_mm = hardware->sharp_right_mm;
    app->status.push_switch_pressed = hardware->push_switch_pressed ? 1U : 0U;
    app->status.requested_base_rpm = hardware->requested_base_rpm;
    app->status.left_velocity_rpm = hardware->measured_left_velocity_rpm;
    app->status.right_velocity_rpm = hardware->measured_right_velocity_rpm;
}

size_t AmrApp_EncodeStatus(
    AmrApp *app,
    uint8_t *frame_out,
    size_t frame_capacity
)
{
    size_t size;
    if (app == NULL) {
        return 0U;
    }
    size = AmrProtocol_EncodeRobotStatus(
        &app->status,
        app->tx_sequence,
        frame_out,
        frame_capacity
    );
    if (size > 0U) {
        app->tx_sequence++;
    }
    return size;
}
