#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

void MotorDriver_Init(void);
void MotorDriver_UpdateFeedback(void);
void MotorDriver_ControlTick(void);

void MotorDriver_SetTargetMmS(
    int16_t left_target_mm_s,
    int16_t right_target_mm_s
);

void MotorDriver_BrakeOn(void);
void MotorDriver_BrakeOff(void);

int16_t MotorDriver_GetLeftVelocityMmS(void);
int16_t MotorDriver_GetRightVelocityMmS(void);
int16_t MotorDriver_GetLeftVelocityRpm(void);
int16_t MotorDriver_GetRightVelocityRpm(void);
bool MotorDriver_HasFault(void);
uint16_t MotorDriver_GetErrorCode(void);
void MotorDriver_ReportFault(uint16_t error_code);

#endif
