STM32 안전 제어 코어

실제 STM32 펌웨어에 들어갈 하드웨어 독립 C 모듈이다. 현재 단계에서는 STM32 HAL,
핀맵, 센서 및 VESC 드라이버에 의존하지 않으므로 PC에서도 컴파일하고 시험할 수
있다.

## 구현 모듈

| 모듈 | 역할 |
|---|---|
| `amr_crc16` | CRC-16/CCITT-FALSE |
| `amr_protocol` | UART 프레임 파싱 및 직렬화 |
| `amr_safety` | 안전 상태와 수동 재출발 래치 |
| `amr_watchdog` | 300ms 명령 통신 단절 감지 |
| `amr_motor` | 선속도·각속도를 좌우 바퀴 목표값으로 변환 |
| `amr_app` | 통신·안전·모터 모듈 통합 |

Python과 C는 저장소의 `protocol/protocol_constants.py`와
`protocol/protocol_constants.h`를 통해 같은 메시지 번호와 상태값을 사용한다.

## PC에서 테스트

프로젝트 루트에서:

```bash
cmake -S stm32 -B /tmp/guide-amr-stm32-build
cmake --build /tmp/guide-amr-stm32-build
ctest --test-dir /tmp/guide-amr-stm32-build --output-on-failure
```

## STM32CubeIDE 연결 방법

1. `Core/Inc`의 헤더와 `Core/Src`의 C 파일을 STM32 프로젝트에 추가한다.
2. include path에 저장소의 `protocol/` 폴더를 추가한다.
3. UART DMA 또는 인터럽트에서 받은 각 바이트를
   `AmrApp_ProcessRxByte()`에 전달한다.
4. 1ms 또는 10ms 태스크에서 `AmrApp_Tick()`을 호출한다.
5. `app.motor_targets`를 VESC 또는 실제 모터 제어 모듈에 전달한다.
6. 주기적으로 `AmrApp_EncodeStatus()` 결과를 Jetson으로 전송한다.

개념 예시:

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    AmrApp_ProcessRxByte(&g_amr_app, rx_byte, HAL_GetTick());
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

void App_10msTask(void)
{
    AmrHardwareInputs inputs = ReadHardwareInputs();
    AmrApp_Tick(&g_amr_app, HAL_GetTick(), &inputs);

    Vesc_SetWheelTargets(
        g_amr_app.motor_targets.left_target_mm_s,
        g_amr_app.motor_targets.right_target_mm_s
    );
}
```

## 아직 구현하지 않은 보드 종속 부분

- STM32 모델별 CubeMX 설정과 핀맵
- UART DMA 링버퍼
- 하향 ToF I2C 드라이버
- Cliff GPIO 또는 ADC 드라이버
- IMU 및 엔코더
- VESC UART/CAN 드라이버
- 햅틱 PWM
- 물리 E-Stop 전원 차단 회로
- 하드웨어 Independent Watchdog 설정

이 항목은 실제 MCU 모델, 센서 모델, 통신 방식과 핀맵이 확정된 후 구현한다.

## 안전 주의사항

- E-Stop은 소프트웨어 정지만으로 대체하지 않는다.
- 인터럽트 안에서 긴 연산이나 블로킹 통신을 수행하지 않는다.
- 긴급정지와 고장 상태에서는 좌우 목표속도가 항상 0이어야 한다.
- 위험이 해제돼도 `RESET_REQUEST` 후 별도 `DRIVE_ENABLE`이 있어야 재출발한다.
- 실제 VESC 제동 방식과 정지거리는 실차에서 별도로 검증한다.

