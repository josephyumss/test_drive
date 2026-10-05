#include "app_integration.h"

#include <stdbool.h>

#include "amr_app.h"
#include "board_config.h"
#include "board_io.h"
#include "jetson_uart.h"
#include "main.h"
#include "motor_driver.h"

extern TIM_HandleTypeDef htim6;

static AmrApp amr_app;
static volatile uint8_t control_ticks_pending;
static uint32_t last_status_ms;
static uint8_t status_frame[AMR_MAX_FRAME_SIZE];

static void run_control_tick(uint32_t now_ms)
{
    AmrHardwareInputs hardware;

    MotorDriver_UpdateFeedback();
    BoardIO_ReadHardwareInputs(&hardware);
    AmrApp_Tick(&amr_app, now_ms, &hardware);

    if (AmrSafety_DriveAllowed(&amr_app.safety)) {
        MotorDriver_BrakeOff();
        MotorDriver_SetTargetMmS(
            amr_app.motor_targets.left_target_mm_s,
            amr_app.motor_targets.right_target_mm_s
        );
    } else {
        MotorDriver_SetTargetMmS(0, 0);
        MotorDriver_BrakeOn();
    }

    MotorDriver_ControlTick();
}

void AppIntegration_Init(void)
{
    const AmrMotorConfig motor_config = {
        AMR_WHEEL_BASE_MM,
        AMR_WHEEL_DIAMETER_MM,
        AMR_MAXIMUM_WHEEL_SPEED_MM_S,
        AMR_SLOW_WHEEL_SPEED_MM_S,
        AMR_ALLOW_REVERSE != 0
    };

    control_ticks_pending = 0U;
    last_status_ms = HAL_GetTick();
    MotorDriver_Init();
    BoardIO_Init();
    AmrApp_Init(&amr_app, &motor_config, AMR_COMMAND_TIMEOUT_MS);
    JetsonUart_Init();
    (void)HAL_TIM_Base_Start_IT(&htim6);
}

void AppIntegration_Process(void)
{
    uint32_t now_ms = HAL_GetTick();

    JetsonUart_ProcessRx(&amr_app, now_ms);

    if (control_ticks_pending > 0U) {
        __disable_irq();
        control_ticks_pending--;
        __enable_irq();
        run_control_tick(now_ms);
    }

    if ((uint32_t)(now_ms - last_status_ms) >= AMR_STATUS_PERIOD_MS) {
        size_t frame_size = AmrApp_EncodeStatus(
            &amr_app,
            status_frame,
            sizeof(status_frame)
        );
        if (JetsonUart_Transmit(status_frame, frame_size)) {
            last_status_ms = now_ms;
        }
    }
}

void AppIntegration_OnControlTimerElapsed(void)
{
    if (control_ticks_pending < UINT8_MAX) {
        control_ticks_pending++;
    }
}

void AppIntegration_ForceSafeStop(void)
{
    MotorDriver_SetTargetMmS(0, 0);
    MotorDriver_BrakeOn();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        AppIntegration_OnControlTimerElapsed();
    }
}
