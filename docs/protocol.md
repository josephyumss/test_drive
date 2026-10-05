# Jetson–STM32 통신 규격 초안

상태: Draft  
프로토콜 버전: 2

## 프레임

| 필드 | 크기 | 설명 |
|---|---:|---|
| SOF | 2 bytes | `AA 55` |
| Version | 1 byte | 현재 `2` |
| Message ID | 1 byte | 메시지 종류 |
| Sequence | 1 byte | 0~255 순환 |
| Payload Length | 1 byte | 최대 32 |
| Payload | N bytes | Little-endian |
| CRC16 | 2 bytes | Version부터 Payload까지 계산 |

CRC는 CRC-16/CCITT-FALSE를 사용한다.

## DRIVE_COMMAND

Payload 형식: `<HhhHBB`

| 필드 | 형식 | 단위 |
|---|---|---|
| command_id | uint16 | - |
| linear_velocity | int16 | mm/s |
| angular_velocity | int16 | mrad/s |
| speed_limit | uint16 | mm/s |
| control_flags | uint8 | bit field |
| reserved | uint8 | - |

### DRIVE_COMMAND control flags

| Bit | 이름 | 설명 |
|---:|---|---|
| 0 | DRIVE_ENABLE | STM32에 주행 허가 요청 |
| 1 | CONTROLLED_STOP | 일반 감속 정지 요청 |
| 2 | NAVIGATION_ACTIVE | 자율 경로 주행 활성 |
| 3 | MANUAL_MODE | 수동 시험 모드 |
| 4 | RESET_REQUEST | 위험 해제 후 수동 리셋 |
| 5 | SLOW_MODE | 감속 상태 및 속도 제한 적용 |

`RESET_REQUEST`가 포함된 패킷은 주행 명령을 동시에 수행하지 않는다. STM32는
위험 조건이 모두 해제된 경우에만 READY로 전환하고, 이후 별도의
`DRIVE_ENABLE` 패킷을 받아야 주행한다.

## WHEEL_COMMAND

CPU가 최종 결정한 좌우 목표 RPM을 STM32에 직접 전달하는 운영 명령이다.
Payload 형식은 `<HhhBB`이며 메시지 ID는 `0x13`이다.

| 필드 | 형식 | 단위 |
|---|---|---|
| command_id | uint16 | - |
| left_target_rpm | int16 | RPM |
| right_target_rpm | int16 | RPM |
| control_flags | uint8 | bit field |
| emergency | uint8 | `0` 정상, `1` 즉시 정지 및 래치 |

Jetson은 50ms 주기로 이 명령을 반복 전송한다. STM32는 좌우 목표 RPM을
바퀴 선속도로 변환한 뒤 엔코더 PID로 추종하며, 마지막 정상 명령 이후
500ms가 지나면 명령값과 무관하게 정지한다.

## ROBOT_STATUS

Payload 형식: `<BHhhHHHHIHHH`

| 필드 | 형식 | 단위 |
|---|---|---|
| system_state | uint8 | 상태 코드 |
| safety_flags | uint16 | bit field |
| left_velocity | int16 | mm/s |
| right_velocity | int16 | mm/s |
| battery_voltage | uint16 | mV |
| motor_error | uint16 | 오류 코드 |
| last_command_id | uint16 | - |
| rx_error_count | uint16 | count |
| uptime | uint32 | ms |
| ultrasonic_front | uint16 | mm, `65535`는 invalid |
| sharp_left | uint16 | mm, `65535`는 invalid |
| sharp_right | uint16 | mm, `65535`는 invalid |
| push_switch_pressed | uint8 | `0` 해제, `1` 눌림 |
| requested_base_rpm | int16 | RPM |
| left_velocity_rpm | int16 | RPM |
| right_velocity_rpm | int16 | RPM |

## 안전 규칙

- CRC 또는 길이가 잘못된 명령은 실행하지 않는다.
- 마지막 유효 주행 명령 이후 500ms가 지나면 STM32가 정지한다.
- 통신이 복구돼도 자동으로 재출발하지 않는다.
- Cliff, E-Stop 및 STM32 고장은 Jetson 명령보다 우선한다.
