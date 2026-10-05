#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*
 * Values that must be measured or tuned on the real vehicle.
 * Keep all physical units explicit in each macro name.
 */
#define AMR_WHEEL_BASE_MM                    500U
#define AMR_WHEEL_DIAMETER_MM                200.0f
#define AMR_ENCODER_COUNTS_PER_OUTPUT_REV    40000.0f

#define AMR_MAXIMUM_WHEEL_SPEED_MM_S         1000U
#define AMR_SLOW_WHEEL_SPEED_MM_S            300U
#define AMR_ALLOW_REVERSE                    0

#define AMR_CONTROL_PERIOD_MS                10U
#define AMR_COMMAND_TIMEOUT_MS               500U
#define AMR_STATUS_PERIOD_MS                 100U
#define AMR_REQUESTED_BASE_RPM               30

#define AMR_PWM_MAX                          8499.0f
#define AMR_PWM_MIN                          0.0f

/* These gains are intentionally zero until the wheels are lifted and tuned. */
#define AMR_LEFT_KP                          0.0f
#define AMR_LEFT_KI                          0.0f
#define AMR_LEFT_KD                          0.0f
#define AMR_RIGHT_KP                         0.0f
#define AMR_RIGHT_KI                         0.0f
#define AMR_RIGHT_KD                         0.0f

#define AMR_PRECHARGE_TIME_MS                1000U
#define AMR_BATTERY_LOW_THRESHOLD_MV         21000U

/* Set each item to 1 only after its fail-safe input pin/driver is connected. */
#define AMR_HAS_ESTOP_INPUT                   0
#define AMR_HAS_CLIFF_LEFT_INPUT              0
#define AMR_HAS_CLIFF_RIGHT_INPUT             0
#define AMR_HAS_HANDLE_INPUT                  0
#define AMR_HAS_BATTERY_MONITOR               0
#define AMR_HAS_DRIVE_SWITCH_INPUT             0

/* Change these if the real circuit uses active-low logic. */
#define AMR_ESTOP_ACTIVE_LEVEL                GPIO_PIN_SET
#define AMR_CLIFF_ACTIVE_LEVEL                GPIO_PIN_SET
#define AMR_HANDLE_RELEASED_LEVEL              GPIO_PIN_SET
#define AMR_DRIVE_SWITCH_ACTIVE_LEVEL           GPIO_PIN_SET

#define AMR_UART_RX_BUFFER_SIZE              256U

#endif
