#include "amr_protocol.h"

#include <string.h>

#include "amr_crc16.h"

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void parser_resync(AmrProtocolParser *parser, uint8_t last_byte)
{
    parser->length = 0U;
    if (last_byte == AMR_SOF_BYTE_0) {
        parser->buffer[0] = last_byte;
        parser->length = 1U;
    }
}

void AmrProtocolParser_Init(AmrProtocolParser *parser)
{
    if (parser != NULL) {
        memset(parser, 0, sizeof(*parser));
    }
}

AmrParseResult AmrProtocolParser_Feed(
    AmrProtocolParser *parser,
    uint8_t byte,
    AmrPacket *packet_out
)
{
    uint8_t payload_length;
    uint8_t expected_length;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((parser == NULL) || (packet_out == NULL)) {
        return AMR_PARSE_FORMAT_ERROR;
    }

    if (parser->length == 0U) {
        if (byte == AMR_SOF_BYTE_0) {
            parser->buffer[0] = byte;
            parser->length = 1U;
        }
        return AMR_PARSE_NONE;
    }

    if (parser->length == 1U) {
        if (byte == AMR_SOF_BYTE_1) {
            parser->buffer[1] = byte;
            parser->length = 2U;
        } else {
            parser_resync(parser, byte);
        }
        return AMR_PARSE_NONE;
    }

    if (parser->length >= AMR_MAX_FRAME_SIZE) {
        parser->format_error_count++;
        parser_resync(parser, byte);
        return AMR_PARSE_FORMAT_ERROR;
    }

    parser->buffer[parser->length++] = byte;
    if (parser->length < 6U) {
        return AMR_PARSE_NONE;
    }

    payload_length = parser->buffer[5];
    if (payload_length > AMR_MAX_PAYLOAD_SIZE) {
        parser->format_error_count++;
        parser_resync(parser, byte);
        return AMR_PARSE_FORMAT_ERROR;
    }

    expected_length = (uint8_t)(payload_length + AMR_FRAME_OVERHEAD_SIZE);
    if (parser->length < expected_length) {
        return AMR_PARSE_NONE;
    }
    if (parser->length > expected_length) {
        parser->format_error_count++;
        parser_resync(parser, byte);
        return AMR_PARSE_FORMAT_ERROR;
    }

    if (parser->buffer[2] != AMR_PROTOCOL_VERSION) {
        parser->version_error_count++;
        parser->length = 0U;
        return AMR_PARSE_VERSION_ERROR;
    }

    received_crc = read_u16_le(&parser->buffer[expected_length - 2U]);
    calculated_crc = AmrCrc16_CcittFalse(
        &parser->buffer[2],
        (size_t)payload_length + 4U
    );
    if (received_crc != calculated_crc) {
        parser->crc_error_count++;
        parser->length = 0U;
        return AMR_PARSE_CRC_ERROR;
    }

    packet_out->version = parser->buffer[2];
    packet_out->message_id = parser->buffer[3];
    packet_out->sequence = parser->buffer[4];
    packet_out->payload_length = payload_length;
    if (payload_length > 0U) {
        memcpy(packet_out->payload, &parser->buffer[6], payload_length);
    }
    parser->length = 0U;
    return AMR_PARSE_PACKET;
}

size_t AmrProtocol_EncodeFrame(
    uint8_t message_id,
    uint8_t sequence,
    const uint8_t *payload,
    uint8_t payload_length,
    uint8_t *frame_out,
    size_t frame_capacity
)
{
    size_t frame_size;
    uint16_t crc;

    if ((frame_out == NULL)
        || (payload_length > AMR_MAX_PAYLOAD_SIZE)
        || ((payload == NULL) && (payload_length > 0U))) {
        return 0U;
    }

    frame_size = (size_t)payload_length + AMR_FRAME_OVERHEAD_SIZE;
    if (frame_capacity < frame_size) {
        return 0U;
    }

    frame_out[0] = AMR_SOF_BYTE_0;
    frame_out[1] = AMR_SOF_BYTE_1;
    frame_out[2] = AMR_PROTOCOL_VERSION;
    frame_out[3] = message_id;
    frame_out[4] = sequence;
    frame_out[5] = payload_length;
    if (payload_length > 0U) {
        memcpy(&frame_out[6], payload, payload_length);
    }

    crc = AmrCrc16_CcittFalse(&frame_out[2], (size_t)payload_length + 4U);
    write_u16_le(&frame_out[6U + payload_length], crc);
    return frame_size;
}

bool AmrProtocol_DecodeDriveCommand(
    const AmrPacket *packet,
    AmrDriveCommand *command_out
)
{
    if ((packet == NULL)
        || (command_out == NULL)
        || (packet->message_id != AMR_MSG_DRIVE_COMMAND)
        || (packet->payload_length != AMR_DRIVE_COMMAND_PAYLOAD_SIZE)) {
        return false;
    }

    command_out->command_id = read_u16_le(&packet->payload[0]);
    command_out->linear_velocity_mm_s =
        (int16_t)read_u16_le(&packet->payload[2]);
    command_out->angular_velocity_mrad_s =
        (int16_t)read_u16_le(&packet->payload[4]);
    command_out->speed_limit_mm_s = read_u16_le(&packet->payload[6]);
    command_out->control_flags = packet->payload[8];
    return true;
}

bool AmrProtocol_DecodeWheelCommand(
    const AmrPacket *packet,
    AmrWheelCommand *command_out
)
{
    if ((packet == NULL)
        || (command_out == NULL)
        || (packet->message_id != AMR_MSG_WHEEL_COMMAND)
        || (packet->payload_length != AMR_WHEEL_COMMAND_PAYLOAD_SIZE)) {
        return false;
    }
    command_out->command_id = read_u16_le(&packet->payload[0]);
    command_out->left_target_rpm = (int16_t)read_u16_le(&packet->payload[2]);
    command_out->right_target_rpm = (int16_t)read_u16_le(&packet->payload[4]);
    command_out->control_flags = packet->payload[6];
    command_out->emergency = packet->payload[7];
    return command_out->emergency <= 1U;
}

size_t AmrProtocol_EncodeRobotStatus(
    const AmrRobotStatus *status,
    uint8_t sequence,
    uint8_t *frame_out,
    size_t frame_capacity
)
{
    uint8_t payload[AMR_ROBOT_STATUS_PAYLOAD_SIZE];

    if (status == NULL) {
        return 0U;
    }

    payload[0] = (uint8_t)status->system_state;
    write_u16_le(&payload[1], status->safety_flags);
    write_u16_le(&payload[3], (uint16_t)status->left_velocity_mm_s);
    write_u16_le(&payload[5], (uint16_t)status->right_velocity_mm_s);
    write_u16_le(&payload[7], status->battery_voltage_mv);
    write_u16_le(&payload[9], status->motor_error);
    write_u16_le(&payload[11], status->last_command_id);
    write_u16_le(&payload[13], status->rx_error_count);
    write_u32_le(&payload[15], status->uptime_ms);
    write_u16_le(&payload[19], status->ultrasonic_front_mm);
    write_u16_le(&payload[21], status->sharp_left_mm);
    write_u16_le(&payload[23], status->sharp_right_mm);
    payload[25] = status->push_switch_pressed;
    write_u16_le(&payload[26], (uint16_t)status->requested_base_rpm);
    write_u16_le(&payload[28], (uint16_t)status->left_velocity_rpm);
    write_u16_le(&payload[30], (uint16_t)status->right_velocity_rpm);

    return AmrProtocol_EncodeFrame(
        AMR_MSG_ROBOT_STATUS,
        sequence,
        payload,
        AMR_ROBOT_STATUS_PAYLOAD_SIZE,
        frame_out,
        frame_capacity
    );
}
