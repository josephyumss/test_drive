#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "amr_app.h"
#include "amr_crc16.h"
#include "amr_motor.h"
#include "amr_protocol.h"
#include "amr_safety.h"
#include "amr_watchdog.h"
#include "range_sensors.h"

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

static size_t make_drive_frame(
    uint16_t command_id,
    int16_t linear_mm_s,
    int16_t angular_mrad_s,
    uint16_t speed_limit_mm_s,
    uint8_t flags,
    uint8_t *frame
)
{
    uint8_t payload[AMR_DRIVE_COMMAND_PAYLOAD_SIZE] = {0};
    write_u16_le(&payload[0], command_id);
    write_u16_le(&payload[2], (uint16_t)linear_mm_s);
    write_u16_le(&payload[4], (uint16_t)angular_mrad_s);
    write_u16_le(&payload[6], speed_limit_mm_s);
    payload[8] = flags;
    return AmrProtocol_EncodeFrame(
        AMR_MSG_DRIVE_COMMAND,
        7U,
        payload,
        AMR_DRIVE_COMMAND_PAYLOAD_SIZE,
        frame,
        AMR_MAX_FRAME_SIZE
    );
}

static void feed_frame(AmrApp *app, const uint8_t *frame, size_t size, uint32_t now)
{
    size_t index;
    AmrParseResult result = AMR_PARSE_NONE;
    for (index = 0U; index < size; ++index) {
        result = AmrApp_ProcessRxByte(app, frame[index], now);
    }
    assert(result == AMR_PARSE_PACKET);
}

static void test_crc(void)
{
    const uint8_t check[] = "123456789";
    assert(AmrCrc16_CcittFalse(check, 9U) == 0x29B1U);
    assert(AmrCrc16_CcittFalse(NULL, 0U) == 0xFFFFU);
}

static void test_range_sensors(void)
{
    static const AmrSharpCalibrationPoint calibration[] = {
        {500U, 800U}, {1000U, 400U}, {1500U, 200U}
    };
    AmrRangeMedianFilter filter;
    uint16_t distance;

    distance = RangeSensors_UltrasonicPulseToMm(5831U, 200, 20U, 4000U);
    assert(distance >= 995U);
    assert(distance <= 1005U);
    assert(RangeSensors_UltrasonicPulseToMm(0U, 200, 20U, 4000U)
        == AMR_RANGE_INVALID_MM);
    assert(RangeSensors_SharpAdcToMm(1250U, calibration, 3U) == 300U);

    RangeSensors_FilterInit(&filter);
    (void)RangeSensors_FilterUpdate(&filter, 400U);
    (void)RangeSensors_FilterUpdate(&filter, 100U);
    assert(RangeSensors_FilterUpdate(&filter, 300U) == 300U);
}

static void test_protocol_round_trip(void)
{
    static const uint8_t python_reference[] = {
        0xAAU, 0x55U, 0x02U, 0x10U, 0x07U, 0x0AU,
        0x2AU, 0x00U, 0x5EU, 0x01U, 0x88U, 0xFFU,
        0xF4U, 0x01U, 0x01U, 0x00U, 0xC2U, 0xD1U
    };
    uint8_t frame[AMR_MAX_FRAME_SIZE];
    size_t size = make_drive_frame(
        42U,
        350,
        -120,
        500U,
        AMR_DRIVE_ENABLE,
        frame
    );
    AmrProtocolParser parser;
    AmrPacket packet;
    AmrDriveCommand command;
    AmrParseResult result = AMR_PARSE_NONE;
    size_t index;

    assert(size == AMR_DRIVE_COMMAND_PAYLOAD_SIZE + AMR_FRAME_OVERHEAD_SIZE);
    assert(size == sizeof(python_reference));
    assert(memcmp(frame, python_reference, size) == 0);
    AmrProtocolParser_Init(&parser);
    for (index = 0U; index < size; ++index) {
        result = AmrProtocolParser_Feed(&parser, frame[index], &packet);
    }
    assert(result == AMR_PARSE_PACKET);
    assert(AmrProtocol_DecodeDriveCommand(&packet, &command));
    assert(command.command_id == 42U);
    assert(command.linear_velocity_mm_s == 350);
    assert(command.angular_velocity_mrad_s == -120);
    assert(command.speed_limit_mm_s == 500U);

    frame[size - 1U] ^= 0xFFU;
    AmrProtocolParser_Init(&parser);
    for (index = 0U; index < size; ++index) {
        result = AmrProtocolParser_Feed(&parser, frame[index], &packet);
    }
    assert(result == AMR_PARSE_CRC_ERROR);
    assert(parser.crc_error_count == 1U);
}

static void test_safety_latch(void)
{
    AmrSafetyManager manager;
    AmrSafetyInputs inputs;
    memset(&inputs, 0, sizeof(inputs));
    inputs.initialization_complete = true;
    inputs.drive_enable = true;
    AmrSafety_Init(&manager);
    AmrSafety_Update(&manager, &inputs);
    assert(manager.state == AMR_STATE_RUN);

    inputs.cliff_left = true;
    AmrSafety_Update(&manager, &inputs);
    assert(manager.state == AMR_STATE_EMERGENCY_STOP);
    assert(manager.restart_latched);
    assert((manager.flags & AMR_SAFETY_CLIFF_LEFT) != 0U);

    inputs.cliff_left = false;
    AmrSafety_Update(&manager, &inputs);
    assert(manager.state == AMR_STATE_EMERGENCY_STOP);

    inputs.reset_request = true;
    AmrSafety_Update(&manager, &inputs);
    assert(manager.state == AMR_STATE_READY);
    assert(!manager.restart_latched);
}

static void test_watchdog_wraparound(void)
{
    AmrCommandWatchdog watchdog;
    AmrWatchdog_Init(&watchdog, 300U);
    assert(!AmrWatchdog_IsTimedOut(&watchdog, 1000U));
    AmrWatchdog_Kick(&watchdog, UINT32_MAX - 100U);
    assert(!AmrWatchdog_IsTimedOut(&watchdog, 50U));
    assert(AmrWatchdog_IsTimedOut(&watchdog, 250U));
}

static void test_motor_targets(void)
{
    AmrMotorConfig config = {500U, 200.0f, 500U, 250U, false};
    AmrSafetyManager safety;
    AmrDriveCommand command = {1U, 500, 400, 500U, AMR_DRIVE_ENABLE};
    AmrMotorTargets targets;

    AmrSafety_Init(&safety);
    safety.state = AMR_STATE_RUN;
    AmrMotor_ComputeTargets(&config, &safety, &command, &targets);
    assert(targets.left_target_mm_s == 400);
    assert(targets.right_target_mm_s == 500);

    safety.state = AMR_STATE_SLOW;
    AmrMotor_ComputeTargets(&config, &safety, &command, &targets);
    assert(targets.left_target_mm_s == 150);
    assert(targets.right_target_mm_s == 250);

    safety.state = AMR_STATE_EMERGENCY_STOP;
    AmrMotor_ComputeTargets(&config, &safety, &command, &targets);
    assert(targets.left_target_mm_s == 0);
    assert(targets.right_target_mm_s == 0);
}

static void test_app_end_to_end(void)
{
    AmrMotorConfig config = {500U, 200.0f, 500U, 250U, false};
    AmrHardwareInputs hardware;
    AmrApp app;
    uint8_t frame[AMR_MAX_FRAME_SIZE];
    uint8_t status_frame[AMR_MAX_FRAME_SIZE];
    size_t size;

    memset(&hardware, 0, sizeof(hardware));
    hardware.initialization_complete = true;
    hardware.battery_voltage_mv = 24000U;
    AmrApp_Init(&app, &config, 300U);

    size = make_drive_frame(
        1U, 300, 0, 500U, AMR_DRIVE_ENABLE, frame
    );
    feed_frame(&app, frame, size, 0U);
    AmrApp_Tick(&app, 0U, &hardware);
    assert(app.safety.state == AMR_STATE_RUN);
    assert(app.motor_targets.left_target_mm_s == 300);

    AmrApp_Tick(&app, 301U, &hardware);
    assert(app.safety.state == AMR_STATE_EMERGENCY_STOP);
    assert(app.motor_targets.left_target_mm_s == 0);
    assert((app.status.safety_flags & AMR_SAFETY_COMM_TIMEOUT) != 0U);

    size = make_drive_frame(
        2U, 0, 0, 0U, AMR_DRIVE_RESET_REQUEST, frame
    );
    feed_frame(&app, frame, size, 302U);
    AmrApp_Tick(&app, 302U, &hardware);
    assert(app.safety.state == AMR_STATE_READY);

    size = make_drive_frame(
        3U, 250, 0, 500U, AMR_DRIVE_ENABLE, frame
    );
    feed_frame(&app, frame, size, 303U);
    AmrApp_Tick(&app, 303U, &hardware);
    assert(app.safety.state == AMR_STATE_RUN);
    assert(app.motor_targets.left_target_mm_s == 250);

    hardware.cliff_right = true;
    AmrApp_Tick(&app, 304U, &hardware);
    assert(app.safety.state == AMR_STATE_EMERGENCY_STOP);
    assert(app.motor_targets.right_target_mm_s == 0);

    size = AmrApp_EncodeStatus(&app, status_frame, sizeof(status_frame));
    assert(size == AMR_ROBOT_STATUS_PAYLOAD_SIZE + AMR_FRAME_OVERHEAD_SIZE);
}

int main(void)
{
    test_crc();
    test_range_sensors();
    test_protocol_round_trip();
    test_safety_latch();
    test_watchdog_wraparound();
    test_motor_targets();
    test_app_end_to_end();
    puts("All STM32 safety core tests passed.");
    return 0;
}
