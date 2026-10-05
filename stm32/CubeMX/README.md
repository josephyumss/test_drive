CubeMX 프로젝트 결합 방법

이 디렉터리는 전자과의 단일 `main.c` 시험 코드를 기존 AMR 안전 코어와 결합하는
방법을 설명한다. `main.c.example`은 완성된 CubeMX 파일이 아니므로 통째로 복사하지
말고 `USER CODE` 영역의 호출부만 실제 프로젝트에 옮긴다.

## 파일 역할

| 파일 | 역할 |
|---|---|
| `board_config.h` | 차체 치수, 제어주기, 속도제한, PID 계수, 센서 연결 여부 |
| `motor_driver.c/.h` | Encoder, RPM↔mm/s, PID, PWM, 방향, 브레이크 |
| `board_io.c/.h` | 프리차지, E-Stop, Cliff, 손잡이, 배터리 입력 취합 |
| `jetson_uart.c/.h` | USART1 수신 링버퍼, AMR 패킷 입력, 상태 송신 |
| `app_integration.c/.h` | 안전 코어·보드 I/O·모터·통신 전체 연결 |
| 기존 `amr_*` | 프로토콜, Watchdog, 안전 상태, 안전한 목표속도 계산 |

## CubeMX 설정

- TIM1 CH1/CH2: 좌우 모터 PWM, Period `8499`
- TIM2: 왼쪽 Quadrature Encoder, 32비트 카운터
- TIM3: 오른쪽 Quadrature Encoder, 16비트 카운터
- TIM6: 10ms 주기 Update interrupt
- USART1: 115200, 8-N-1, TX/RX interrupt 활성화
- LEFT/RIGHT DIR: Push-pull output
- LEFT/RIGHT BRK: Push-pull output
- PRECHARGE RELAY: Push-pull output
- E-Stop/Cliff/손잡이: 회로에 맞는 pull-up/down 입력
- IWDG: 실제 하드웨어 통합 단계에서 별도 활성화

TIM6의 실제 주기가 `AMR_CONTROL_PERIOD_MS`와 정확히 같아야 한다. Timer clock이
170MHz라면 전자과 원본의 Prescaler 169, Period 9999 설정은 10ms가 된다.

## 실제 `main.c` 변경

`USER CODE BEGIN Includes`에 추가한다.

```c
#include "app_integration.h"
```

모든 `MX_*_Init()` 호출 뒤에 한 번 호출한다.

```c
AppIntegration_Init();
```

메인 루프는 다음처럼 유지한다.

```c
while (1)
{
    AppIntegration_Process();
}
```

`Error_Handler()`에서는 인터럽트를 끄기 전에 모터를 먼저 안전 정지한다.

```c
void Error_Handler(void)
{
    AppIntegration_ForceSafeStop();
    __disable_irq();
    while (1) {
    }
}
```

## 반드시 제거할 기존 코드

- 버튼으로 `base_target_rpm`을 직접 변경하는 `while` 본문
- 기존 `PID_Calculate()`, `Motor_Brake_On()`, `Motor_Brake_Off()`
- 기존 `HAL_TIM_PeriodElapsedCallback()`
- USART1을 다시 초기화하는 `BSP_COM_Init(COM1, ...)`
- USART1용으로 이미 작성한 RX/TX/Error callback

동일한 HAL callback을 두 파일에 정의하면 링크 오류가 발생한다. UART 로그가
필요하면 Jetson 통신용 USART1이 아닌 별도 UART를 사용한다.

## 핀 이름 연결

전자과 원본에서 이미 사용한 핀 이름은 그대로 사용한다.

```text
LEFT_DIR_GPIO_Port / LEFT_DIR_Pin
RIGHT_DIR_GPIO_Port / RIGHT_DIR_Pin
LEFT_BRK_GPIO_Port / LEFT_BRK_Pin
RIGHT_BRK_GPIO_Port / RIGHT_BRK_Pin
PRECHARGE_RELAY_GPIO_Port / PRECHARGE_RELAY_Pin
```

안전 센서는 CubeMX의 User Label을 다음 이름으로 지정한다.

```text
ESTOP_GPIO_Port / ESTOP_Pin
CLIFF_LEFT_GPIO_Port / CLIFF_LEFT_Pin
CLIFF_RIGHT_GPIO_Port / CLIFF_RIGHT_Pin
HANDLE_GPIO_Port / HANDLE_Pin
```

핀 설정 후 `board_config.h`의 `AMR_HAS_*` 값을 `1`로 변경한다. 기본값 `0`에서는
E-Stop과 사용자 감지를 안전하지 않은 상태로 판단하고, 좌우 Cliff 미연결은 중요
센서 고장으로 판단하므로 모터가 구동되지 않는 것이 정상이다.

## 반드시 실측할 설정값

```c
AMR_WHEEL_BASE_MM
AMR_WHEEL_DIAMETER_MM
AMR_ENCODER_COUNTS_PER_OUTPUT_REV
AMR_MAXIMUM_WHEEL_SPEED_MM_S
AMR_SLOW_WHEEL_SPEED_MM_S
```

`AMR_ENCODER_COUNTS_PER_OUTPUT_REV`는 모터축 값이 아니라 감속기 이후 바퀴 한 바퀴의
실제 quadrature count여야 한다.

## PID 튜닝 전 상태

모든 PID 계수는 의도적으로 `0.0f`다. 실제 계수를 넣기 전에는 PWM 출력이 0이므로
바퀴가 움직이지 않는다. 바퀴를 지면에서 띄운 상태에서 좌우를 각각 튜닝하고,
PWM 상한과 회전 방향을 확인한다.

## 최초 시험 순서

1. 모터 전원을 분리하고 빌드·UART 수신 확인
2. E-Stop/Cliff/손잡이 플래그 확인
3. 바퀴를 띄우고 PWM 0과 브레이크 확인
4. 낮은 PWM으로 좌우 방향과 Encoder 부호 확인
5. PID를 낮은 속도부터 튜닝
6. Jetson 명령 단절 300ms 후 정지 확인
7. E-Stop 및 Cliff 후 자동 재출발하지 않는지 확인

물리 E-Stop은 MCU 소프트웨어만으로 구현하지 말고 모터 Enable 또는 구동 전원을
차단하는 하드웨어 회로와 함께 사용한다.
