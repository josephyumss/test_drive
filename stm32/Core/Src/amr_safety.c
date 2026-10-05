#include "amr_safety.h"

#include <string.h>

static uint16_t make_flags(const AmrSafetyInputs *inputs)
{
    uint16_t flags = AMR_SAFETY_NONE;
    if (inputs->estop_active) {
        flags |= AMR_SAFETY_ESTOP_ACTIVE;
    }
    if (inputs->cliff_left) {
        flags |= AMR_SAFETY_CLIFF_LEFT;
    }
    if (inputs->cliff_right) {
        flags |= AMR_SAFETY_CLIFF_RIGHT;
    }
    if (inputs->communication_timeout) {
        flags |= AMR_SAFETY_COMM_TIMEOUT;
    }
    if (inputs->motor_fault) {
        flags |= AMR_SAFETY_MOTOR_FAULT;
    }
    if (inputs->critical_sensor_fault) {
        flags |= AMR_SAFETY_TOF_INVALID;
    }
    if (inputs->low_battery_warning) {
        flags |= AMR_SAFETY_UNDERVOLTAGE;
    }
    if (inputs->user_released) {
        flags |= AMR_SAFETY_USER_RELEASED;
    }
    return flags;
}

static void latch(
    AmrSafetyManager *manager,
    AmrSystemState state,
    AmrSafetyReason reason
)
{
    manager->state = state;
    manager->reason = reason;
    manager->restart_latched = true;
}

void AmrSafety_Init(AmrSafetyManager *manager)
{
    if (manager != NULL) {
        memset(manager, 0, sizeof(*manager));
        manager->state = AMR_STATE_INIT;
        manager->reason = AMR_REASON_INITIALIZING;
    }
}

void AmrSafety_Update(
    AmrSafetyManager *manager,
    const AmrSafetyInputs *inputs
)
{
    if ((manager == NULL) || (inputs == NULL)) {
        return;
    }

    manager->flags = make_flags(inputs);

    if (inputs->motor_fault) {
        latch(manager, AMR_STATE_FAULT, AMR_REASON_MOTOR_FAULT);
        return;
    }
    if (inputs->critical_sensor_fault) {
        latch(manager, AMR_STATE_FAULT, AMR_REASON_SENSOR_FAULT);
        return;
    }
    if (inputs->estop_active) {
        latch(manager, AMR_STATE_EMERGENCY_STOP, AMR_REASON_ESTOP);
        return;
    }
    if (inputs->cliff_left || inputs->cliff_right) {
        latch(manager, AMR_STATE_EMERGENCY_STOP, AMR_REASON_CLIFF);
        return;
    }
    if (inputs->communication_timeout) {
        latch(manager, AMR_STATE_EMERGENCY_STOP, AMR_REASON_COMM_TIMEOUT);
        return;
    }
    if (inputs->user_released) {
        latch(manager, AMR_STATE_EMERGENCY_STOP, AMR_REASON_USER_RELEASED);
        return;
    }

    if (!inputs->initialization_complete) {
        manager->state = AMR_STATE_INIT;
        manager->reason = AMR_REASON_INITIALIZING;
        manager->restart_latched = false;
        return;
    }

    if (manager->restart_latched) {
        if (inputs->reset_request) {
            manager->state = AMR_STATE_READY;
            manager->reason = AMR_REASON_MANUAL_RESET_ACCEPTED;
            manager->restart_latched = false;
        } else {
            manager->reason = AMR_REASON_MANUAL_RESET_REQUIRED;
        }
        return;
    }

    if (inputs->controlled_stop_request) {
        latch(
            manager,
            AMR_STATE_CONTROLLED_STOP,
            AMR_REASON_CONTROLLED_STOP
        );
        return;
    }

    if (!inputs->drive_enable) {
        manager->state = AMR_STATE_READY;
        manager->reason = AMR_REASON_READY;
        return;
    }

    if (inputs->slow_request || inputs->low_battery_warning) {
        manager->state = AMR_STATE_SLOW;
        manager->reason = AMR_REASON_SLOW_REQUEST;
        return;
    }

    manager->state = AMR_STATE_RUN;
    manager->reason = AMR_REASON_RUN;
}

bool AmrSafety_DriveAllowed(const AmrSafetyManager *manager)
{
    if (manager == NULL) {
        return false;
    }
    return (manager->state == AMR_STATE_RUN)
        || (manager->state == AMR_STATE_SLOW);
}

