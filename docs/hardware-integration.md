# 실센서·주행·음성 통합 안내

## 데이터 흐름

```text
STM32 초음파/SHARP/엔코더
  -> Protocol v2 ROBOT_STATUS
  -> amr_mcu_bridge
  -> /ultrasonic/front, /sharp/left, /sharp/right, /mcu/status

STL-27L /scan + 근거리 Range
  -> amr_perception/range_fusion_node
  -> /obstacle/info
  -> amr_safety_node
  -> /cmd_vel_safe
  -> amr_mcu_bridge
  -> STM32 좌우 휠 PID

STM32 Push Switch + requested base RPM
  -> Protocol v2 ROBOT_STATUS (100ms)
  -> /drive/request + /drive/base_speed_mps
  -> Jetson 경로/안전 계산
  -> WHEEL_COMMAND 좌우 목표 RPM (50ms)
  -> STM32 독립 좌/우 PID (10ms)

OAK-D /oak/rgb/image_raw + STL-27L /scan + /mcu/status
  -> camera_path_node + path_guidance_node
  -> /cmd_vel_raw

/safety/state + /obstacle/info
  -> amr_audio/audio_node
  -> USB 기본 오디오 장치
```

## 반드시 실측해야 하는 값

- `board_config.h`: 바퀴 지름, 축간거리, 출력축 1회전당 엔코더 카운트
- `board_config.h`: 좌우 PID 게인. 현재 0이므로 리프트 테스트 후 조정
- SHARP 보정표: 사용 모델과 실제 ADC/거리 측정값으로 작성
- 초음파: TIM Input Capture 핀과 타이머 주파수
- `mcu.yaml`: 실제 STM32 포트(udev 별칭 권장)
- `perception.yaml`: 실제 복도와 카메라 장착 위치에서 게인 조정

## STM32 연결 지점

`range_sensors.c`는 거리 변환과 중앙값 필터를 제공한다. CubeMX 프로젝트에서
다음 weak 함수를 실제 TIM/ADC 드라이버로 override한다.

```c
uint16_t BoardIO_ReadUltrasonicFrontMm(void);
uint16_t BoardIO_ReadSharpLeftMm(void);
uint16_t BoardIO_ReadSharpRightMm(void);
```

초음파 Echo가 없거나 SHARP ADC가 보정표 밖이면 `AMR_RANGE_INVALID_MM`을
반환한다. 안전 로직은 invalid 값을 정상 거리로 간주하면 안 된다.

## Jetson 설치 및 빌드

```bash
sudo apt install -y python3-serial espeak-ng \
  ros-humble-cv-bridge ros-humble-image-transport

cd ~/guide-amr/jetson_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Protocol v2 변경으로 Jetson과 STM32는 반드시 같은 버전으로 다시 빌드한다.

## 정지 상태 통합 시험

바퀴를 지면에서 띄우고 `path_guidance_node.enabled=false`,
`safety_controller_node.drive_enable=false` 상태로 시작한다.

```bash
ros2 launch amr_bringup hardware_system.launch.py
ros2 topic hz /scan
ros2 topic echo /mcu/status
ros2 topic echo /obstacle/info
ros2 topic echo /cmd_vel_safe
```

센서 방향, 엔코더 부호, E-Stop을 모두 확인한 후에만 단계적으로 주행을 허가한다.
