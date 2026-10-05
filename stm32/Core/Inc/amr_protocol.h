#ifndef AMR_PROTOCOL_H
#define AMR_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "protocol_constants.h"

#define AMR_FRAME_OVERHEAD_SIZE 8U
#define AMR_MAX_FRAME_SIZE (AMR_MAX_PAYLOAD_SIZE + AMR_FRAME_OVERHEAD_SIZE)
#define AMR_DRIVE_COMMAND_PAYLOAD_SIZE 10U
#define AMR_WHEEL_COMMAND_PAYLOAD_SIZE 8U
#define AMR_ROBOT_STATUS_PAYLOAD_SIZE 32U

typedef struct {
    uint8_t version;
    uint8_t message_id;
    uint8_t sequence;
    uint8_t payload_length;
    uint8_t payload[AMR_MAX_PAYLOAD_SIZE];
} AmrPacket;

typedef struct {
    uint16_t command_id;
    int16_t linear_velocity_mm_s;
    int16_t angular_velocity_mrad_s;
    uint16_t speed_limit_mm_s;
    uint8_t control_flags;
} AmrDriveCommand;

typedef struct {
    uint16_t command_id;
    int16_t left_target_rpm;
    int16_t right_target_rpm;
    uint8_t control_flags;
    uint8_t emergency;
} AmrWheelCommand;

typedef struct {
    AmrSystemState system_state;
    uint16_t safety_flags;
    int16_t left_velocity_mm_s;
    int16_t right_velocity_mm_s;
    uint16_t battery_voltage_mv;
    uint16_t motor_error;
    uint16_t last_command_id;
    uint16_t rx_error_count;
    uint32_t uptime_ms;
    uint16_t ultrasonic_front_mm;
    uint16_t sharp_left_mm;
    uint16_t sharp_right_mm;
    uint8_t push_switch_pressed;
    int16_t requested_base_rpm;
    int16_t left_velocity_rpm;
    int16_t right_velocity_rpm;
} AmrRobotStatus;

typedef enum {
    AMR_PARSE_NONE = 0,
    AMR_PARSE_PACKET,
    AMR_PARSE_CRC_ERROR,
    AMR_PARSE_FORMAT_ERROR,
    AMR_PARSE_VERSION_ERROR
} AmrParseResult;

typedef struct {
    uint8_t buffer[AMR_MAX_FRAME_SIZE];
    uint8_t length;
    uint16_t crc_error_count;
    uint16_t format_error_count;
    uint16_t version_error_count;
} AmrProtocolParser;

void AmrProtocolParser_Init(AmrProtocolParser *parser);
AmrParseResult AmrProtocolParser_Feed(
    AmrProtocolParser *parser,
    uint8_t byte,
    AmrPacket *packet_out
);

size_t AmrProtocol_EncodeFrame(
    uint8_t message_id,
    uint8_t sequence,
    const uint8_t *payload,
    uint8_t payload_length,
    uint8_t *frame_out,
    size_t frame_capacity
);

bool AmrProtocol_DecodeDriveCommand(
    const AmrPacket *packet,
    AmrDriveCommand *command_out
);

bool AmrProtocol_DecodeWheelCommand(
    const AmrPacket *packet,
    AmrWheelCommand *command_out
);

size_t AmrProtocol_EncodeRobotStatus(
    const AmrRobotStatus *status,
    uint8_t sequence,
    uint8_t *frame_out,
    size_t frame_capacity
);

#endif
