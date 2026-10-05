/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* =========================================================
 * ENCODER DIRECTION
 *
 * 로봇 전진 방향을 +RPM으로 정의
 *
 * 실제 시험에서 한쪽만 음수면
 * 그쪽만 -1.0f로 변경
 * ========================================================= */

#define LEFT_ENCODER_SIGN                     1.0f
#define RIGHT_ENCODER_SIGN                    1.0f


/* =========================================================
 * BUTTON
 * ========================================================= */

#define BUTTON_DEBOUNCE_MS                    50U
#define RPM_STEP                              5.0f


/* =========================================================
 * CONTROL LOOP
 *
 * TIM6 = 10 ms
 *      = 100 Hz
 * ========================================================= */

#define CONTROL_PERIOD_SEC                    0.01f


/* =========================================================
 * ENCODER
 *
 * 500 PPR
 * x4 Quadrature = 2000 count
 * Gear Ratio 20:1
 *
 * 2000 x 20
 * = 40000 count / output revolution
 * ========================================================= */

#define ENCODER_COUNTS_PER_OUTPUT_REV         40000.0f


/* =========================================================
 * PWM
 *
 * TIM1 ARR = 8499
 *
 * PWM 증가 -> 모터 속도 증가
 * ========================================================= */

#define PWM_MAX                               8499U
#define PWM_MIN                               0U


/* =========================================================
 * USART1 RX
 * ========================================================= */

#define UART1_RX_BUFFER_SIZE                  64U


/* =========================================================
 * COMMAND WATCHDOG
 *
 * Jetson CMD가 500 ms 이상 없으면 정지
 * ========================================================= */

#define COMMAND_TIMEOUT_MS                    800U


/* =========================================================
 * INITIAL PI GAIN
 *
 * 첫 실제 시험용
 * 최종값은 실험 후 튜닝
 * ========================================================= */

#define LEFT_KP                               70.0f
#define LEFT_KI                               20.0f
#define LEFT_KD                               0.0f

#define RIGHT_KP                              70.0f
#define RIGHT_KI                              20.0f
#define RIGHT_KD                              0.0f


/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;


/* USER CODE BEGIN PV */


/* =========================================================
 * DEBUG
 *
 * Live Expressions용
 * ========================================================= */

/* 정상적으로 해석된 CMD 패킷 수 */
volatile uint32_t dbg_cmd_rx_count = 0;

/* 형식이 잘못된 CMD 패킷 수 */
volatile uint32_t dbg_cmd_parse_error_count = 0;

/* 정상적으로 송신된 STATUS 패킷 수 */
volatile uint32_t dbg_status_tx_count = 0;


/* =========================================================
 * USART3 TELEMETRY
 *
 * STM32 -> Jetson
 * ========================================================= */

char telemetry_tx_buf[128];

uint32_t telemetry_last_tick = 0;


/* =========================================================
 * USART1 COMMAND RECEIVE
 *
 * Jetson -> STM32
 *
 * $CMD,<LEFT_RPM>,<RIGHT_RPM>,<EMERGENCY>\r\n
 *
 * Example:
 *
 * $CMD,10,10,0
 * ========================================================= */

uint8_t uart1_rx_byte = 0;

char uart1_rx_buffer[UART1_RX_BUFFER_SIZE];

volatile uint8_t uart1_rx_index = 0;

volatile uint8_t uart1_packet_ready = 0;


/* =========================================================
 * COMMAND WATCHDOG
 * ========================================================= */

uint32_t last_valid_command_tick = 0;

uint8_t command_received = 0;


/* =========================================================
 * DRIVE ENABLE
 *
 * PWM을 먼저 계산한 뒤 Brake를 해제하기 위해 사용
 * ========================================================= */

volatile uint8_t drive_enable_request = 0;


/* =========================================================
 * SHARP SENSOR
 * ========================================================= */

volatile uint32_t sharp_adc_value = 0;

volatile float sharp_voltage = 0.0f;

volatile float sharp_distance_cm = 0.0f;

uint32_t sharp_last_read_tick = 0;


/* =========================================================
 * MOTOR PWM
 * ========================================================= */

volatile uint32_t left_motor_pwm = 0;

volatile uint32_t right_motor_pwm = 0;


/* =========================================================
 * ENCODER / RPM
 * ========================================================= */

volatile int32_t left_encoder_delta = 0;

volatile int16_t right_encoder_delta = 0;

volatile float left_rpm = 0.0f;

volatile float right_rpm = 0.0f;

uint32_t left_encoder_prev = 0;

uint16_t right_encoder_prev = 0;


/* =========================================================
 * BUTTON
 * ========================================================= */

volatile uint8_t rpm_up_flag = 0;

volatile uint8_t rpm_down_flag = 0;

static uint32_t rpm_up_last_tick = 0;

static uint32_t rpm_down_last_tick = 0;


/* =========================================================
 * TARGET RPM
 * ========================================================= */

volatile float base_target_rpm = 0.0f;

volatile float left_target_rpm = 0.0f;

volatile float right_target_rpm = 0.0f;


/* =========================================================
 * PID
 * ========================================================= */

static float left_integral = 0.0f;

static float left_prev_error = 0.0f;

static float right_integral = 0.0f;

static float right_prev_error = 0.0f;

volatile float left_pwm_output = 0.0f;

volatile float right_pwm_output = 0.0f;


/* =========================================================
 * SAFETY
 * ========================================================= */

volatile uint8_t emergency_stop = 0;


/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);


/* USER CODE BEGIN PFP */

float PID_Calculate(
    float target_rpm,
    float actual_rpm,
    float kp,
    float ki,
    float kd,
    float *integral,
    float *prev_error
);

void Motor_Brake_On(void);

void Motor_Brake_Off(void);

void Motor_Set_PWM(
    uint32_t left_pwm,
    uint32_t right_pwm
);

uint32_t Sharp_Read_ADC(void);

void Sharp_Update(void);

/* USER CODE END PFP */


/**
  * @brief  The application entry point.
  */
int main(void)
{
    HAL_Init();

    SystemClock_Config();


    /* =========================================================
     * PERIPHERAL INITIALIZATION
     * ========================================================= */

    MX_GPIO_Init();

    MX_TIM1_Init();

    MX_TIM2_Init();

    MX_TIM3_Init();

    MX_TIM6_Init();

    MX_USART1_UART_Init();

    MX_ADC1_Init();

    MX_I2C1_Init();

    MX_USART3_UART_Init();


    /* USER CODE BEGIN 2 */


    /* =========================================================
     * ADC CALIBRATION
     * ========================================================= */

    if (HAL_ADCEx_Calibration_Start(
            &hadc1,
            ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }


    /* =========================================================
     * USART1 RECEIVE START
     * ========================================================= */

    if (HAL_UART_Receive_IT(
            &huart1,
            &uart1_rx_byte,
            1) != HAL_OK)
    {
        Error_Handler();
    }


    /* =========================================================
     * PRECHARGE
     *
     * Active Low
     * ========================================================= */

    HAL_GPIO_WritePin(
        PRECHARGE_RELAY_GPIO_Port,
        PRECHARGE_RELAY_Pin,
        GPIO_PIN_SET
    );

    HAL_Delay(3000);

    HAL_GPIO_WritePin(
        PRECHARGE_RELAY_GPIO_Port,
        PRECHARGE_RELAY_Pin,
        GPIO_PIN_RESET
    );


    /* =========================================================
     * INITIAL PWM
     *
     * PWM = 0
     * ========================================================= */

    left_motor_pwm = 0;

    right_motor_pwm = 0;


    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        0
    );


    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_2,
        0
    );


    /* =========================================================
     * PWM START
     * ========================================================= */

    if (HAL_TIM_PWM_Start(
            &htim1,
            TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_TIM_PWM_Start(
            &htim1,
            TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }


    /* =========================================================
     * ENCODER START
     *
     * TIM2 = LEFT
     * TIM3 = RIGHT
     * ========================================================= */

    if (HAL_TIM_Encoder_Start(
            &htim2,
            TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_TIM_Encoder_Start(
            &htim3,
            TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }


    left_encoder_prev =
        __HAL_TIM_GET_COUNTER(
            &htim2
        );


    right_encoder_prev =
        (uint16_t)
        __HAL_TIM_GET_COUNTER(
            &htim3
        );


    /* =========================================================
     * TIM6 START
     *
     * 10 ms / 100 Hz
     * ========================================================= */

    if (HAL_TIM_Base_Start_IT(
            &htim6) != HAL_OK)
    {
        Error_Handler();
    }


    /* =========================================================
     * SAFE START
     * ========================================================= */

    Motor_Brake_On();


    /* USER CODE END 2 */


    /* Initialize LED */

    BSP_LED_Init(
        LED_GREEN
    );


    /* Initialize USER push button */

    BSP_PB_Init(
        BUTTON_USER,
        BUTTON_MODE_EXTI
    );


    /* Initialize COM1 */

    BspCOMInit.BaudRate =
        115200;

    BspCOMInit.WordLength =
        COM_WORDLENGTH_8B;

    BspCOMInit.StopBits =
        COM_STOPBITS_1;

    BspCOMInit.Parity =
        COM_PARITY_NONE;

    BspCOMInit.HwFlowCtl =
        COM_HWCONTROL_NONE;


    if (BSP_COM_Init(
            COM1,
            &BspCOMInit) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }


    /* USER CODE BEGIN WHILE */

    while (1)
    {
        /* =====================================================
         * SHARP
         * ===================================================== */

        Sharp_Update();


        /* =====================================================
         * USART1 CMD PROCESS
         *
         * Jetson -> STM32
         *
         * $CMD,L,R,E
         * ===================================================== */

        if (uart1_packet_ready == 1)
        {
            int left_cmd = 0;

            int right_cmd = 0;

            int emergency_cmd = 0;


            int parsed_count =
                sscanf(
                    uart1_rx_buffer,
                    "$CMD,%d,%d,%d",
                    &left_cmd,
                    &right_cmd,
                    &emergency_cmd
                );


            /* =================================================
             * VALID PACKET
             * ================================================= */

            if (parsed_count == 3)
            {
                if ((emergency_cmd == 0) ||
                    (emergency_cmd == 1))
                {
                    dbg_cmd_rx_count++;


                    last_valid_command_tick =
                        HAL_GetTick();


                    command_received =
                        1;


                    /* =========================================
                     * EMERGENCY
                     * ========================================= */

                    if (emergency_cmd == 1)
                    {
                        emergency_stop =
                            1;


                        drive_enable_request =
                            0;


                        left_target_rpm =
                            0.0f;


                        right_target_rpm =
                            0.0f;


                        Motor_Brake_On();
                    }


                    /* =========================================
                     * NORMAL CMD
                     * ========================================= */

                    else
                    {
                        emergency_stop =
                            0;


                        /* RPM LIMIT */

                        if (left_cmd < 0)
                        {
                            left_cmd = 0;
                        }

                        if (left_cmd > 65)
                        {
                            left_cmd = 65;
                        }


                        if (right_cmd < 0)
                        {
                            right_cmd = 0;
                        }

                        if (right_cmd > 65)
                        {
                            right_cmd = 65;
                        }


                        /* =====================================
                         * Jetson 명령 -> Target RPM
                         * ===================================== */

                        left_target_rpm =
                            (float)left_cmd;


                        right_target_rpm =
                            (float)right_cmd;


                        /* =====================================
                         * DRIVE
                         * ===================================== */

                        if ((left_target_rpm > 0.0f) ||
                            (right_target_rpm > 0.0f))
                        {
                            /*
                             * TIM6에서 먼저 PID 계산
                             * → PWM 준비
                             * → Brake 해제
                             */

                            drive_enable_request =
                                1;
                        }


                        /* =====================================
                         * NORMAL STOP
                         * ===================================== */

                        else
                        {
                            drive_enable_request =
                                0;


                            Motor_Brake_On();
                        }
                    }
                }

                else
                {
                    dbg_cmd_parse_error_count++;
                }
            }

            else
            {
                dbg_cmd_parse_error_count++;
            }


            /* =================================================
             * CURRENT PACKET FINISHED
             * ================================================= */

            uart1_packet_ready =
                0;


            uart1_rx_index =
                0;


            /* =================================================
             * RECEIVE NEXT PACKET
             * ================================================= */

            if (HAL_UART_Receive_IT(
                    &huart1,
                    &uart1_rx_byte,
                    1) != HAL_OK)
            {
                Error_Handler();
            }
        }


        /* =====================================================
         * COMMAND WATCHDOG
         *
         * 500 ms 이상 CMD가 없으면 Brake
         * ===================================================== */

        if (command_received == 1)
        {
            if ((HAL_GetTick() -
                 last_valid_command_tick)
                >
                COMMAND_TIMEOUT_MS)
            {
                emergency_stop =
                    1;


                command_received =
                    0;


                drive_enable_request =
                    0;


                left_target_rpm =
                    0.0f;


                right_target_rpm =
                    0.0f;


                Motor_Brake_On();
            }
        }


        /* =====================================================
         * RPM UP BUTTON
         * ===================================================== */

        if (rpm_up_flag == 1)
        {
            rpm_up_flag =
                0;


            base_target_rpm +=
                RPM_STEP;


            if (base_target_rpm >
                65.0f)
            {
                base_target_rpm =
                    65.0f;
            }
        }


        /* =====================================================
         * RPM DOWN BUTTON
         * ===================================================== */

        if (rpm_down_flag == 1)
        {
            rpm_down_flag =
                0;


            if (base_target_rpm >=
                RPM_STEP)
            {
                base_target_rpm -=
                    RPM_STEP;
            }

            else
            {
                base_target_rpm =
                    0.0f;
            }
        }


        /* =====================================================
         * USART3 STATUS
         *
         * STM32 -> Jetson
         *
         * FORMAT:
         *
         * $STATUS,
         * BASE,
         * ADC,
         * SHARP_CM,
         * LEFT_TARGET,
         * RIGHT_TARGET,
         * LEFT_RPM,
         * RIGHT_RPM,
         * LEFT_PWM,
         * RIGHT_PWM,
         * ESTOP
         *
         * ===================================================== */

        if ((HAL_GetTick() -
             telemetry_last_tick)
            >= 100U)
        {
            telemetry_last_tick =
                HAL_GetTick();


            int len =
                snprintf(
                    telemetry_tx_buf,
                    sizeof(
                        telemetry_tx_buf
                    ),

                    "$STATUS,%d,%lu,%d,%d,%d,%d,%d,%lu,%lu,%u\r\n",

                    (int)
                    base_target_rpm,

                    (unsigned long)
                    sharp_adc_value,

                    (int)
                    sharp_distance_cm,

                    (int)
                    left_target_rpm,

                    (int)
                    right_target_rpm,

                    (int)
                    left_rpm,

                    (int)
                    right_rpm,

                    (unsigned long)
                    left_motor_pwm,

                    (unsigned long)
                    right_motor_pwm,

                    emergency_stop
                );


            if ((len > 0) &&
                (len <
                 (int)sizeof(
                     telemetry_tx_buf)))
            {
                if (HAL_UART_Transmit(
                        &huart3,
                        (uint8_t *)
                        telemetry_tx_buf,
                        (uint16_t)len,
                        20) == HAL_OK)
                {
                    dbg_status_tx_count++;
                }
            }
        }
    }

    /* USER CODE END WHILE */
}


/* ========================================================================== */
/* SYSTEM CLOCK                                                               */
/* ========================================================================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef
        RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef
        RCC_ClkInitStruct = {0};


    HAL_PWREx_ControlVoltageScaling(
        PWR_REGULATOR_VOLTAGE_SCALE1_BOOST
    );


    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;


    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;


    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;


    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;


    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSI;


    RCC_OscInitStruct.PLL.PLLM =
        RCC_PLLM_DIV4;


    RCC_OscInitStruct.PLL.PLLN =
        85;


    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV2;


    RCC_OscInitStruct.PLL.PLLQ =
        RCC_PLLQ_DIV2;


    RCC_OscInitStruct.PLL.PLLR =
        RCC_PLLR_DIV2;


    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }


    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;


    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;


    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;


    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;


    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;


    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* ADC1                                                                       */
/* ========================================================================== */

static void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef
        multimode = {0};

    ADC_ChannelConfTypeDef
        sConfig = {0};


    hadc1.Instance =
        ADC1;


    hadc1.Init.ClockPrescaler =
        ADC_CLOCK_SYNC_PCLK_DIV4;


    hadc1.Init.Resolution =
        ADC_RESOLUTION_12B;


    hadc1.Init.DataAlign =
        ADC_DATAALIGN_RIGHT;


    hadc1.Init.GainCompensation =
        0;


    hadc1.Init.ScanConvMode =
        ADC_SCAN_DISABLE;


    hadc1.Init.EOCSelection =
        ADC_EOC_SINGLE_CONV;


    hadc1.Init.LowPowerAutoWait =
        DISABLE;


    hadc1.Init.ContinuousConvMode =
        DISABLE;


    hadc1.Init.NbrOfConversion =
        1;


    hadc1.Init.DiscontinuousConvMode =
        DISABLE;


    hadc1.Init.ExternalTrigConv =
        ADC_SOFTWARE_START;


    hadc1.Init.ExternalTrigConvEdge =
        ADC_EXTERNALTRIGCONVEDGE_NONE;


    hadc1.Init.DMAContinuousRequests =
        DISABLE;


    hadc1.Init.Overrun =
        ADC_OVR_DATA_PRESERVED;


    hadc1.Init.OversamplingMode =
        DISABLE;


    if (HAL_ADC_Init(
            &hadc1) != HAL_OK)
    {
        Error_Handler();
    }


    multimode.Mode =
        ADC_MODE_INDEPENDENT;


    if (HAL_ADCEx_MultiModeConfigChannel(
            &hadc1,
            &multimode) != HAL_OK)
    {
        Error_Handler();
    }


    sConfig.Channel =
        ADC_CHANNEL_1;


    sConfig.Rank =
        ADC_REGULAR_RANK_1;


    sConfig.SamplingTime =
        ADC_SAMPLETIME_47CYCLES_5;


    sConfig.SingleDiff =
        ADC_SINGLE_ENDED;


    sConfig.OffsetNumber =
        ADC_OFFSET_NONE;


    sConfig.Offset =
        0;


    if (HAL_ADC_ConfigChannel(
            &hadc1,
            &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* I2C1                                                                       */
/* ========================================================================== */

static void MX_I2C1_Init(void)
{
    hi2c1.Instance =
        I2C1;


    hi2c1.Init.Timing =
        0x30E1A6F2;


    hi2c1.Init.OwnAddress1 =
        0;


    hi2c1.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;


    hi2c1.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;


    hi2c1.Init.OwnAddress2 =
        0;


    hi2c1.Init.OwnAddress2Masks =
        I2C_OA2_NOMASK;


    hi2c1.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;


    hi2c1.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;


    if (HAL_I2C_Init(
            &hi2c1) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_I2CEx_ConfigAnalogFilter(
            &hi2c1,
            I2C_ANALOGFILTER_ENABLE)
        != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_I2CEx_ConfigDigitalFilter(
            &hi2c1,
            2) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* TIM1                                                                       */
/* ========================================================================== */

static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef
        sClockSourceConfig = {0};

    TIM_MasterConfigTypeDef
        sMasterConfig = {0};

    TIM_OC_InitTypeDef
        sConfigOC = {0};

    TIM_BreakDeadTimeConfigTypeDef
        sBreakDeadTimeConfig = {0};


    htim1.Instance =
        TIM1;


    htim1.Init.Prescaler =
        0;


    htim1.Init.CounterMode =
        TIM_COUNTERMODE_UP;


    htim1.Init.Period =
        8499;


    htim1.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;


    htim1.Init.RepetitionCounter =
        0;


    htim1.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_ENABLE;


    if (HAL_TIM_Base_Init(
            &htim1) != HAL_OK)
    {
        Error_Handler();
    }


    sClockSourceConfig.ClockSource =
        TIM_CLOCKSOURCE_INTERNAL;


    if (HAL_TIM_ConfigClockSource(
            &htim1,
            &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_TIM_PWM_Init(
            &htim1) != HAL_OK)
    {
        Error_Handler();
    }


    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;


    sMasterConfig.MasterOutputTrigger2 =
        TIM_TRGO2_RESET;


    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim1,
            &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }


    sConfigOC.OCMode =
        TIM_OCMODE_PWM1;


    sConfigOC.Pulse =
        0;


    sConfigOC.OCPolarity =
        TIM_OCPOLARITY_HIGH;


    sConfigOC.OCNPolarity =
        TIM_OCNPOLARITY_HIGH;


    sConfigOC.OCFastMode =
        TIM_OCFAST_DISABLE;


    sConfigOC.OCIdleState =
        TIM_OCIDLESTATE_RESET;


    sConfigOC.OCNIdleState =
        TIM_OCNIDLESTATE_RESET;


    if (HAL_TIM_PWM_ConfigChannel(
            &htim1,
            &sConfigOC,
            TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_TIM_PWM_ConfigChannel(
            &htim1,
            &sConfigOC,
            TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }


    sBreakDeadTimeConfig.OffStateRunMode =
        TIM_OSSR_DISABLE;


    sBreakDeadTimeConfig.OffStateIDLEMode =
        TIM_OSSI_DISABLE;


    sBreakDeadTimeConfig.LockLevel =
        TIM_LOCKLEVEL_OFF;


    sBreakDeadTimeConfig.DeadTime =
        0;


    sBreakDeadTimeConfig.BreakState =
        TIM_BREAK_DISABLE;


    sBreakDeadTimeConfig.BreakPolarity =
        TIM_BREAKPOLARITY_HIGH;


    sBreakDeadTimeConfig.BreakFilter =
        0;


    sBreakDeadTimeConfig.BreakAFMode =
        TIM_BREAK_AFMODE_INPUT;


    sBreakDeadTimeConfig.Break2State =
        TIM_BREAK2_DISABLE;


    sBreakDeadTimeConfig.Break2Polarity =
        TIM_BREAK2POLARITY_HIGH;


    sBreakDeadTimeConfig.Break2Filter =
        0;


    sBreakDeadTimeConfig.Break2AFMode =
        TIM_BREAK_AFMODE_INPUT;


    sBreakDeadTimeConfig.AutomaticOutput =
        TIM_AUTOMATICOUTPUT_DISABLE;


    if (HAL_TIMEx_ConfigBreakDeadTime(
            &htim1,
            &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }


    HAL_TIM_MspPostInit(
        &htim1
    );
}


/* ========================================================================== */
/* TIM2                                                                       */
/* ========================================================================== */

static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef
        sConfig = {0};

    TIM_MasterConfigTypeDef
        sMasterConfig = {0};


    htim2.Instance =
        TIM2;


    htim2.Init.Prescaler =
        0;


    htim2.Init.CounterMode =
        TIM_COUNTERMODE_UP;


    htim2.Init.Period =
        4294967295;


    htim2.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;


    htim2.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    sConfig.EncoderMode =
        TIM_ENCODERMODE_TI12;


    sConfig.IC1Polarity =
        TIM_ICPOLARITY_RISING;


    sConfig.IC1Selection =
        TIM_ICSELECTION_DIRECTTI;


    sConfig.IC1Prescaler =
        TIM_ICPSC_DIV1;


    sConfig.IC1Filter =
        0;


    sConfig.IC2Polarity =
        TIM_ICPOLARITY_RISING;


    sConfig.IC2Selection =
        TIM_ICSELECTION_DIRECTTI;


    sConfig.IC2Prescaler =
        TIM_ICPSC_DIV1;


    sConfig.IC2Filter =
        0;


    if (HAL_TIM_Encoder_Init(
            &htim2,
            &sConfig) != HAL_OK)
    {
        Error_Handler();
    }


    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;


    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim2,
            &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* TIM3                                                                       */
/* ========================================================================== */

static void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef
        sConfig = {0};

    TIM_MasterConfigTypeDef
        sMasterConfig = {0};


    htim3.Instance =
        TIM3;


    htim3.Init.Prescaler =
        0;


    htim3.Init.CounterMode =
        TIM_COUNTERMODE_UP;


    htim3.Init.Period =
        65535;


    htim3.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;


    htim3.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    sConfig.EncoderMode =
        TIM_ENCODERMODE_TI12;


    sConfig.IC1Polarity =
        TIM_ICPOLARITY_RISING;


    sConfig.IC1Selection =
        TIM_ICSELECTION_DIRECTTI;


    sConfig.IC1Prescaler =
        TIM_ICPSC_DIV1;


    sConfig.IC1Filter =
        0;


    sConfig.IC2Polarity =
        TIM_ICPOLARITY_RISING;


    sConfig.IC2Selection =
        TIM_ICSELECTION_DIRECTTI;


    sConfig.IC2Prescaler =
        TIM_ICPSC_DIV1;


    sConfig.IC2Filter =
        0;


    if (HAL_TIM_Encoder_Init(
            &htim3,
            &sConfig) != HAL_OK)
    {
        Error_Handler();
    }


    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;


    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim3,
            &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* TIM6                                                                       */
/* ========================================================================== */

static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef
        sMasterConfig = {0};


    htim6.Instance =
        TIM6;


    htim6.Init.Prescaler =
        169;


    htim6.Init.CounterMode =
        TIM_COUNTERMODE_UP;


    htim6.Init.Period =
        9999;


    htim6.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if (HAL_TIM_Base_Init(
            &htim6) != HAL_OK)
    {
        Error_Handler();
    }


    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;


    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (HAL_TIMEx_MasterConfigSynchronization(
            &htim6,
            &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* USART1                                                                     */
/* ========================================================================== */

static void MX_USART1_UART_Init(void)
{
    huart1.Instance =
        USART1;


    huart1.Init.BaudRate =
        115200;


    huart1.Init.WordLength =
        UART_WORDLENGTH_8B;


    huart1.Init.StopBits =
        UART_STOPBITS_1;


    huart1.Init.Parity =
        UART_PARITY_NONE;


    huart1.Init.Mode =
        UART_MODE_TX_RX;


    huart1.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;


    huart1.Init.OverSampling =
        UART_OVERSAMPLING_16;


    huart1.Init.OneBitSampling =
        UART_ONE_BIT_SAMPLE_DISABLE;


    huart1.Init.ClockPrescaler =
        UART_PRESCALER_DIV1;


    huart1.AdvancedInit.AdvFeatureInit =
        UART_ADVFEATURE_NO_INIT;


    if (HAL_UART_Init(
            &huart1) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_UARTEx_SetTxFifoThreshold(
            &huart1,
            UART_TXFIFO_THRESHOLD_1_8)
        != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_UARTEx_SetRxFifoThreshold(
            &huart1,
            UART_RXFIFO_THRESHOLD_1_8)
        != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_UARTEx_DisableFifoMode(
            &huart1) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* USART3                                                                     */
/* ========================================================================== */

static void MX_USART3_UART_Init(void)
{
    huart3.Instance =
        USART3;


    huart3.Init.BaudRate =
        115200;


    huart3.Init.WordLength =
        UART_WORDLENGTH_8B;


    huart3.Init.StopBits =
        UART_STOPBITS_1;


    huart3.Init.Parity =
        UART_PARITY_NONE;


    huart3.Init.Mode =
        UART_MODE_TX_RX;


    huart3.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;


    huart3.Init.OverSampling =
        UART_OVERSAMPLING_16;


    huart3.Init.OneBitSampling =
        UART_ONE_BIT_SAMPLE_DISABLE;


    huart3.Init.ClockPrescaler =
        UART_PRESCALER_DIV1;


    huart3.AdvancedInit.AdvFeatureInit =
        UART_ADVFEATURE_NO_INIT;


    if (HAL_UART_Init(
            &huart3) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_UARTEx_SetTxFifoThreshold(
            &huart3,
            UART_TXFIFO_THRESHOLD_1_8)
        != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_UARTEx_SetRxFifoThreshold(
            &huart3,
            UART_RXFIFO_THRESHOLD_1_8)
        != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_UARTEx_DisableFifoMode(
            &huart3) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ========================================================================== */
/* GPIO                                                                       */
/* ========================================================================== */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef
        GPIO_InitStruct = {0};


    __HAL_RCC_GPIOC_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();


    /* BRK 초기상태 = HIGH */

    HAL_GPIO_WritePin(
        GPIOC,
        LEFT_BRK_Pin |
        RIGHT_BRK_Pin,
        GPIO_PIN_SET
    );


    /* PRECHARGE 초기상태 */

    HAL_GPIO_WritePin(
        PRECHARGE_RELAY_GPIO_Port,
        PRECHARGE_RELAY_Pin,
        GPIO_PIN_SET
    );


    /* RPM UP */

    GPIO_InitStruct.Pin =
        RPM_UP_BTN_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_FALLING;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    HAL_GPIO_Init(
        RPM_UP_BTN_GPIO_Port,
        &GPIO_InitStruct
    );


    /* BRK */

    GPIO_InitStruct.Pin =
        LEFT_BRK_Pin |
        RIGHT_BRK_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );


    /* PRECHARGE */

    GPIO_InitStruct.Pin =
        PRECHARGE_RELAY_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        PRECHARGE_RELAY_GPIO_Port,
        &GPIO_InitStruct
    );


    /* RPM DOWN */

    GPIO_InitStruct.Pin =
        RPM_DOWN_BTN_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_IT_FALLING;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    HAL_GPIO_Init(
        RPM_DOWN_BTN_GPIO_Port,
        &GPIO_InitStruct
    );


    HAL_NVIC_SetPriority(
        EXTI4_IRQn,
        3,
        0
    );

    HAL_NVIC_EnableIRQ(
        EXTI4_IRQn
    );


    HAL_NVIC_SetPriority(
        EXTI9_5_IRQn,
        3,
        0
    );

    HAL_NVIC_EnableIRQ(
        EXTI9_5_IRQn
    );
}


/* USER CODE BEGIN 4 */


/* ========================================================================== */
/* MOTOR PWM                                                                  */
/* ========================================================================== */

void Motor_Set_PWM(
    uint32_t left_pwm,
    uint32_t right_pwm)
{
    if (left_pwm >
        PWM_MAX)
    {
        left_pwm =
            PWM_MAX;
    }


    if (right_pwm >
        PWM_MAX)
    {
        right_pwm =
            PWM_MAX;
    }


    left_motor_pwm =
        left_pwm;


    right_motor_pwm =
        right_pwm;


    /*
     * 확인 완료:
     *
     * PWM 값 증가
     * -> CCR 증가
     * -> 모터 속도 증가
     */

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        left_pwm
    );


    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_2,
        right_pwm
    );
}


/* ========================================================================== */
/* USART1 RX CALLBACK                                                         */
/* ========================================================================== */

void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef *huart)
{
    if (huart->Instance ==
        USART1)
    {
        /* '\n' = Packet End */

        if (uart1_rx_byte ==
            '\n')
        {
            uart1_rx_buffer[
                uart1_rx_index
            ] =
                '\0';


            uart1_packet_ready =
                1;
        }


        /* CR은 저장하지 않음 */

        else if (uart1_rx_byte ==
                 '\r')
        {
            HAL_UART_Receive_IT(
                &huart1,
                &uart1_rx_byte,
                1
            );
        }


        else
        {
            if (uart1_rx_index <
                (UART1_RX_BUFFER_SIZE -
                 1U))
            {
                uart1_rx_buffer[
                    uart1_rx_index
                ] =
                    (char)
                    uart1_rx_byte;


                uart1_rx_index++;


                HAL_UART_Receive_IT(
                    &huart1,
                    &uart1_rx_byte,
                    1
                );
            }

            else
            {
                uart1_rx_index =
                    0;


                HAL_UART_Receive_IT(
                    &huart1,
                    &uart1_rx_byte,
                    1
                );
            }
        }
    }
}


/* ========================================================================== */
/* UART ERROR                                                                 */
/* ========================================================================== */

void HAL_UART_ErrorCallback(
    UART_HandleTypeDef *huart)
{
    if (huart->Instance ==
        USART1)
    {
        uart1_rx_index =
            0;


        uart1_packet_ready =
            0;


        HAL_UART_Receive_IT(
            &huart1,
            &uart1_rx_byte,
            1
        );
    }
}


/* ========================================================================== */
/* SHARP ADC                                                                  */
/* ========================================================================== */

uint32_t Sharp_Read_ADC(void)
{
    uint32_t adc_value =
        0;


    if (HAL_ADC_Start(
            &hadc1) != HAL_OK)
    {
        return 0;
    }


    if (HAL_ADC_PollForConversion(
            &hadc1,
            10) == HAL_OK)
    {
        adc_value =
            HAL_ADC_GetValue(
                &hadc1
            );
    }


    HAL_ADC_Stop(
        &hadc1
    );


    return
        adc_value;
}


/* ========================================================================== */
/* SHARP UPDATE                                                               */
/* ========================================================================== */

void Sharp_Update(void)
{
    if ((HAL_GetTick() -
         sharp_last_read_tick)
        >= 50U)
    {
        sharp_last_read_tick =
            HAL_GetTick();


        sharp_adc_value =
            Sharp_Read_ADC();


        sharp_voltage =
            ((float)
             sharp_adc_value *
             3.3f)
            /
            4095.0f;


        if (sharp_voltage >
            0.20f)
        {
            sharp_distance_cm =
                27.61f
                /
                (sharp_voltage -
                 0.1696f);


            if (sharp_distance_cm <
                10.0f)
            {
                sharp_distance_cm =
                    10.0f;
            }

            else if (
                sharp_distance_cm >
                80.0f)
            {
                sharp_distance_cm =
                    80.0f;
            }
        }

        else
        {
            sharp_distance_cm =
                80.0f;
        }
    }
}


/* ========================================================================== */
/* PID                                                                        */
/* ========================================================================== */

float PID_Calculate(
    float target_rpm,
    float actual_rpm,
    float kp,
    float ki,
    float kd,
    float *integral,
    float *prev_error)
{
    float error;

    float derivative;

    float output;

    float new_integral;


    /* Error */

    error =
        target_rpm -
        actual_rpm;


    /* D */

    derivative =
        (error -
         *prev_error)
        /
        CONTROL_PERIOD_SEC;


    /* I */

    new_integral =
        *integral
        +
        error *
        CONTROL_PERIOD_SEC;


    /* PID Output */

    output =
        kp * error
        +
        ki * new_integral
        +
        kd * derivative;


    /* =========================================================
     * Anti-Windup
     * ========================================================= */

    if (output >
        (float)PWM_MAX)
    {
        output =
            (float)PWM_MAX;


        if (error < 0.0f)
        {
            *integral =
                new_integral;
        }
    }


    else if (output <
             (float)PWM_MIN)
    {
        output =
            (float)PWM_MIN;


        if (error > 0.0f)
        {
            *integral =
                new_integral;
        }
    }


    else
    {
        *integral =
            new_integral;
    }


    *prev_error =
        error;


    return
        output;
}


/* ========================================================================== */
/* BRAKE ON                                                                   */
/* ========================================================================== */

void Motor_Brake_On(void)
{
    /*
     * BRK HIGH 먼저
     */

    HAL_GPIO_WritePin(
        LEFT_BRK_GPIO_Port,
        LEFT_BRK_Pin,
        GPIO_PIN_SET
    );


    HAL_GPIO_WritePin(
        RIGHT_BRK_GPIO_Port,
        RIGHT_BRK_Pin,
        GPIO_PIN_SET
    );


    /* PWM 0 */

    Motor_Set_PWM(
        0,
        0
    );


    drive_enable_request =
        0;


    left_pwm_output =
        0.0f;


    right_pwm_output =
        0.0f;


    left_integral =
        0.0f;


    right_integral =
        0.0f;


    left_prev_error =
        0.0f;


    right_prev_error =
        0.0f;
}


/* ========================================================================== */
/* BRAKE OFF                                                                  */
/* ========================================================================== */

void Motor_Brake_Off(void)
{
    /*
     * BRK LOW = Drive
     */

    HAL_GPIO_WritePin(
        LEFT_BRK_GPIO_Port,
        LEFT_BRK_Pin,
        GPIO_PIN_RESET
    );


    HAL_GPIO_WritePin(
        RIGHT_BRK_GPIO_Port,
        RIGHT_BRK_Pin,
        GPIO_PIN_RESET
    );
}


/* ========================================================================== */
/* TIM6 CONTROL LOOP                                                          */
/* ========================================================================== */

void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef *htim)
{
    if (htim->Instance ==
        TIM6)
    {
        uint32_t left_now;

        uint16_t right_now;

        uint32_t left_pwm_cmd =
            0;

        uint32_t right_pwm_cmd =
            0;


        /* =====================================================
         * EMERGENCY
         * ===================================================== */

        if (emergency_stop)
        {
            Motor_Brake_On();

            return;
        }


        /* =====================================================
         * ENCODER COUNT
         * ===================================================== */

        left_now =
            __HAL_TIM_GET_COUNTER(
                &htim2
            );


        right_now =
            (uint16_t)
            __HAL_TIM_GET_COUNTER(
                &htim3
            );


        /* =====================================================
         * DELTA COUNT
         * ===================================================== */

        left_encoder_delta =
            (int32_t)(
                left_now -
                left_encoder_prev
            );


        right_encoder_delta =
            (int16_t)(
                right_now -
                right_encoder_prev
            );


        left_encoder_prev =
            left_now;


        right_encoder_prev =
            right_now;


        /* =====================================================
         * COUNT -> RPM
         * ===================================================== */

        left_rpm =
            LEFT_ENCODER_SIGN
            *
            (
                (float)
                left_encoder_delta
                *
                60.0f
            )
            /
            (
                ENCODER_COUNTS_PER_OUTPUT_REV
                *
                CONTROL_PERIOD_SEC
            );


        right_rpm =
            RIGHT_ENCODER_SIGN
            *
            (
                (float)
                right_encoder_delta
                *
                60.0f
            )
            /
            (
                ENCODER_COUNTS_PER_OUTPUT_REV
                *
                CONTROL_PERIOD_SEC
            );


        /* =====================================================
         * LEFT PI CONTROL
         * ===================================================== */

        if (left_target_rpm >
            0.0f)
        {
            left_pwm_output =
                PID_Calculate(
                    left_target_rpm,
                    left_rpm,

                    LEFT_KP,
                    LEFT_KI,
                    LEFT_KD,

                    &left_integral,
                    &left_prev_error
                );


            left_pwm_cmd =
                (uint32_t)
                left_pwm_output;
        }

        else
        {
            left_integral =
                0.0f;


            left_prev_error =
                0.0f;


            left_pwm_output =
                0.0f;


            left_pwm_cmd =
                0;
        }


        /* =====================================================
         * RIGHT PI CONTROL
         * ===================================================== */

        if (right_target_rpm >
            0.0f)
        {
            right_pwm_output =
                PID_Calculate(
                    right_target_rpm,
                    right_rpm,

                    RIGHT_KP,
                    RIGHT_KI,
                    RIGHT_KD,

                    &right_integral,
                    &right_prev_error
                );


            right_pwm_cmd =
                (uint32_t)
                right_pwm_output;
        }

        else
        {
            right_integral =
                0.0f;


            right_prev_error =
                0.0f;


            right_pwm_output =
                0.0f;


            right_pwm_cmd =
                0;
        }


        /* =====================================================
         * PWM OUTPUT
         * ===================================================== */

        Motor_Set_PWM(
            left_pwm_cmd,
            right_pwm_cmd
        );


        /* =====================================================
         * FIRST DRIVE START
         *
         * PID PWM을 먼저 만든 후
         * BRK 해제
         * ===================================================== */

        if (drive_enable_request)
        {
            if ((left_target_rpm >
                 0.0f)
                ||
                (right_target_rpm >
                 0.0f))
            {
                Motor_Brake_Off();


                drive_enable_request =
                    0;
            }
        }
    }
}


/* ========================================================================== */
/* BUTTON                                                                     */
/* ========================================================================== */

void HAL_GPIO_EXTI_Callback(
    uint16_t GPIO_Pin)
{
    uint32_t current_tick =
        HAL_GetTick();


    if (GPIO_Pin ==
        RPM_UP_BTN_Pin)
    {
        if ((current_tick -
             rpm_up_last_tick)
            >=
            BUTTON_DEBOUNCE_MS)
        {
            rpm_up_last_tick =
                current_tick;


            rpm_up_flag =
                1;
        }
    }


    if (GPIO_Pin ==
        RPM_DOWN_BTN_Pin)
    {
        if ((current_tick -
             rpm_down_last_tick)
            >=
            BUTTON_DEBOUNCE_MS)
        {
            rpm_down_last_tick =
                current_tick;


            rpm_down_flag =
                1;
        }
    }
}


/* USER CODE END 4 */


/* ========================================================================== */
/* ERROR HANDLER                                                              */
/* ========================================================================== */

void Error_Handler(void)
{
    __disable_irq();


    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line)
{
}

#endif

