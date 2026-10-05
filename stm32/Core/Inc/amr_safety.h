#ifndef AMR_SAFETY_H
#define AMR_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "protocol_constants.h"

typedef enum {
    AMR_REASON_INITIALIZING = 0,
    AMR_REASON_READY,
    AMR_REASON_RUN,
    AMR_REASON_SLOW_REQUEST,
    AMR_REASON_CONTROLLED_STOP,
    AMR_REASON_ESTOP,
    AMR_REASON_CLIFF,
    AMR_REASON_COMM_TIMEOUT,
    AMR_REASON_USER_RELEASED,
    AMR_REASON_MOTOR_FAULT,
    AMR_REASON_SENSOR_FAULT,
    AMR_REASON_MANUAL_RESET_REQUIRED,
    AMR_REASON_MANUAL_RESET_ACCEPTED
} AmrSafetyReason;

typedef struct {
    bool initialization_complete;
    bool drive_enable;
    bool slow_request;
    bool controlled_stop_request;
    bool reset_request;
    bool estop_active;
    bool cliff_left;
    bool cliff_right;
    bool communication_timeout;
    bool user_released;
    bool motor_fault;
    bool critical_sensor_fault;
    bool low_battery_warning;
} AmrSafetyInputs;

typedef struct {
    AmrSystemState state;
    uint16_t flags;
    AmrSafetyReason reason;
    bool restart_latched;
} AmrSafetyManager;

void AmrSafety_Init(AmrSafetyManager *manager);
void AmrSafety_Update(
    AmrSafetyManager *manager,
    const AmrSafetyInputs *inputs
);
bool AmrSafety_DriveAllowed(const AmrSafetyManager *manager);

#endif
