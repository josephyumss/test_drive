#include "motor_driver.h"

#include <stddef.h>

#include "board_config.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

typedef struct {
    float integral;
    float previous_error;
} PidState;

static int16_t left_target_mm_s;
static int16_t right_target_mm_s;
static int16_t left_velocity_mm_s;
static int16_t right_velocity_mm_s;
static int16_t left_velocity_rpm;
static int16_t right_velocity_rpm;
static uint32_t left_encoder_previous;
static uint16_t right_encoder_previous;
static PidState left_pid;
static PidState right_pid;
static bool brake_active = true;
static bool motor_fault;
static uint16_t motor_error_code;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float wheel_circumference_mm(void)
{
    return AMR_WHEEL_DIAMETER_MM * 3.14159265358979323846f;
}

static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float mm_s_to_rpm(int16_t velocity_mm_s)
{
    return ((float)velocity_mm_s * 60.0f) / wheel_circumference_mm();
}

static int16_t rpm_to_mm_s(float rpm)
{
    float velocity = rpm * wheel_circumference_mm() / 60.0f;
    velocity = clamp_float(velocity, -32768.0f, 32767.0f);
    return (int16_t)(velocity + ((velocity >= 0.0f) ? 0.5f : -0.5f));
}

static float pid_calculate(
    float target_rpm,
    float actual_rpm,
    float kp,
    float ki,
    float kd,
    PidState *state
)
{
    const float period_sec = (float)AMR_CONTROL_PERIOD_MS / 1000.0f;
    float error = target_rpm - actual_rpm;
    float derivative = (error - state->previous_error) / period_sec;
    float new_integral = state->integral + (error * period_sec);
    float output = (kp * error) + (ki * new_integral) + (kd * derivative);

    if (output > AMR_PWM_MAX) {
        output = AMR_PWM_MAX;
        if (error < 0.0f) {
            state->integral = new_integral;
        }
    } else if (output < AMR_PWM_MIN) {
        output = AMR_PWM_MIN;
        if (error > 0.0f) {
            state->integral = new_integral;
        }
    } else {
        state->integral = new_integral;
    }

    state->previous_error = error;
    return output;
}

static void reset_pid(PidState *state)
{
    state->integral = 0.0f;
    state->previous_error = 0.0f;
}

static void set_direction(bool left, int16_t velocity_mm_s)
{
    GPIO_TypeDef *port = left ? LEFT_DIR_GPIO_Port : RIGHT_DIR_GPIO_Port;
    uint16_t pin = left ? LEFT_DIR_Pin : RIGHT_DIR_Pin;
    HAL_GPIO_WritePin(
        port,
        pin,
        (velocity_mm_s < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

static void set_pwm(uint32_t channel, float pwm)
{
    __HAL_TIM_SET_COMPARE(&htim1, channel, (uint32_t)pwm);
}

void MotorDriver_Init(void)
{
    set_pwm(TIM_CHANNEL_1, 0.0f);
    set_pwm(TIM_CHANNEL_2, 0.0f);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    left_encoder_previous = __HAL_TIM_GET_COUNTER(&htim2);
    right_encoder_previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    MotorDriver_BrakeOn();
}

void MotorDriver_UpdateFeedback(void)
{
    uint32_t left_now = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t right_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t left_delta = (int32_t)(left_now - left_encoder_previous);
    int16_t right_delta = (int16_t)(right_now - right_encoder_previous);
    const float period_sec = (float)AMR_CONTROL_PERIOD_MS / 1000.0f;
    float left_rpm;
    float right_rpm;

    left_encoder_previous = left_now;
    right_encoder_previous = right_now;
    left_rpm = ((float)left_delta * 60.0f)
        / (AMR_ENCODER_COUNTS_PER_OUTPUT_REV * period_sec);
    right_rpm = ((float)right_delta * 60.0f)
        / (AMR_ENCODER_COUNTS_PER_OUTPUT_REV * period_sec);
    left_velocity_mm_s = rpm_to_mm_s(left_rpm);
    right_velocity_mm_s = rpm_to_mm_s(right_rpm);
    left_velocity_rpm = (int16_t)(left_rpm + ((left_rpm >= 0.0f) ? 0.5f : -0.5f));
    right_velocity_rpm = (int16_t)(right_rpm + ((right_rpm >= 0.0f) ? 0.5f : -0.5f));
}

void MotorDriver_SetTargetMmS(
    int16_t left_mm_s,
    int16_t right_mm_s
)
{
    left_target_mm_s = left_mm_s;
    right_target_mm_s = right_mm_s;
}

static void control_wheel(
    bool left,
    int16_t target_mm_s,
    int16_t actual_mm_s,
    PidState *pid
)
{
    uint32_t channel = left ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
    float kp = left ? AMR_LEFT_KP : AMR_RIGHT_KP;
    float ki = left ? AMR_LEFT_KI : AMR_RIGHT_KI;
    float kd = left ? AMR_LEFT_KD : AMR_RIGHT_KD;
    float output;

    if ((target_mm_s == 0) || brake_active || motor_fault) {
        reset_pid(pid);
        set_pwm(channel, 0.0f);
        return;
    }

    set_direction(left, target_mm_s);
    output = pid_calculate(
        absolute_float(mm_s_to_rpm(target_mm_s)),
        absolute_float(mm_s_to_rpm(actual_mm_s)),
        kp,
        ki,
        kd,
        pid
    );
    set_pwm(channel, output);
}

void MotorDriver_ControlTick(void)
{
    control_wheel(
        true,
        left_target_mm_s,
        left_velocity_mm_s,
        &left_pid
    );
    control_wheel(
        false,
        right_target_mm_s,
        right_velocity_mm_s,
        &right_pid
    );
}

void MotorDriver_BrakeOn(void)
{
    brake_active = true;
    left_target_mm_s = 0;
    right_target_mm_s = 0;
    set_pwm(TIM_CHANNEL_1, 0.0f);
    set_pwm(TIM_CHANNEL_2, 0.0f);
    reset_pid(&left_pid);
    reset_pid(&right_pid);
    HAL_GPIO_WritePin(LEFT_BRK_GPIO_Port, LEFT_BRK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RIGHT_BRK_GPIO_Port, RIGHT_BRK_Pin, GPIO_PIN_SET);
}

void MotorDriver_BrakeOff(void)
{
    if (!motor_fault) {
        HAL_GPIO_WritePin(LEFT_BRK_GPIO_Port, LEFT_BRK_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RIGHT_BRK_GPIO_Port, RIGHT_BRK_Pin, GPIO_PIN_RESET);
        brake_active = false;
    }
}

int16_t MotorDriver_GetLeftVelocityMmS(void)
{
    return left_velocity_mm_s;
}

int16_t MotorDriver_GetRightVelocityMmS(void)
{
    return right_velocity_mm_s;
}

int16_t MotorDriver_GetLeftVelocityRpm(void)
{
    return left_velocity_rpm;
}

int16_t MotorDriver_GetRightVelocityRpm(void)
{
    return right_velocity_rpm;
}

bool MotorDriver_HasFault(void)
{
    return motor_fault;
}

uint16_t MotorDriver_GetErrorCode(void)
{
    return motor_error_code;
}

void MotorDriver_ReportFault(uint16_t error_code)
{
    motor_error_code = error_code;
    motor_fault = error_code != 0U;
    if (motor_fault) {
        MotorDriver_BrakeOn();
    }
}
