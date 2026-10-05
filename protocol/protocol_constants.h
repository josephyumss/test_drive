#ifndef GUIDE_AMR_PROTOCOL_CONSTANTS_H
#define GUIDE_AMR_PROTOCOL_CONSTANTS_H

#include <stdint.h>

#define AMR_PROTOCOL_VERSION 2U
#define AMR_SOF_BYTE_0 0xAAU
#define AMR_SOF_BYTE_1 0x55U
#define AMR_MAX_PAYLOAD_SIZE 32U

typedef enum {
    AMR_MSG_HEARTBEAT       = 0x01,
    AMR_MSG_DRIVE_COMMAND   = 0x10,
    AMR_MSG_HAPTIC_COMMAND  = 0x11,
    AMR_MSG_SYSTEM_COMMAND  = 0x12,
    AMR_MSG_WHEEL_COMMAND   = 0x13,
    AMR_MSG_ROBOT_STATUS    = 0x80,
    AMR_MSG_SAFETY_EVENT    = 0x81,
    AMR_MSG_DIAGNOSTIC      = 0x82
} AmrMessageId;

typedef enum {
    AMR_STATE_INIT = 0,
    AMR_STATE_READY,
    AMR_STATE_RUN,
    AMR_STATE_SLOW,
    AMR_STATE_CONTROLLED_STOP,
    AMR_STATE_EMERGENCY_STOP,
    AMR_STATE_FAULT
} AmrSystemState;

typedef enum {
    AMR_SAFETY_NONE              = 0,
    AMR_SAFETY_ESTOP_ACTIVE      = 1U << 0,
    AMR_SAFETY_CLIFF_LEFT        = 1U << 1,
    AMR_SAFETY_CLIFF_RIGHT       = 1U << 2,
    AMR_SAFETY_TOF_INVALID       = 1U << 3,
    AMR_SAFETY_COMM_TIMEOUT      = 1U << 4,
    AMR_SAFETY_MOTOR_FAULT       = 1U << 5,
    AMR_SAFETY_OVERCURRENT       = 1U << 6,
    AMR_SAFETY_UNDERVOLTAGE      = 1U << 7,
    AMR_SAFETY_IMU_FAULT         = 1U << 8,
    AMR_SAFETY_ENCODER_FAULT     = 1U << 9,
    AMR_SAFETY_USER_RELEASED     = 1U << 10,
    AMR_SAFETY_PROTOCOL_MISMATCH = 1U << 11
} AmrSafetyFlag;

typedef enum {
    AMR_DRIVE_NONE            = 0,
    AMR_DRIVE_ENABLE          = 1U << 0,
    AMR_DRIVE_CONTROLLED_STOP = 1U << 1,
    AMR_DRIVE_NAV_ACTIVE      = 1U << 2,
    AMR_DRIVE_MANUAL_MODE     = 1U << 3,
    AMR_DRIVE_RESET_REQUEST   = 1U << 4,
    AMR_DRIVE_SLOW_MODE       = 1U << 5
} AmrDriveControlFlag;

#endif
