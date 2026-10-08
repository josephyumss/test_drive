# Jetson 원클릭 실행

## YOLO + LiDAR only: one avoidance run

The separate launcher `scripts/start_yolo_lidar_path_avoidance.sh` uses the
OAK-D YOLO detections and STL-27L LiDAR for obstacle sensing. It does not import
Jetson.GPIO, poll ultrasonic sensors, or use SHARP readings. STM32 wheel RPM
feedback and emergency status are still required for odometry and motor control.
The original `start_fused_path_avoidance.sh` is unchanged.

On an Ubuntu 22.04 / JetPack 6 Jetson, install and build its dependencies:

```bash
./scripts/setup_yolo_lidar_path_avoidance.sh
```

Authenticate with sudo in the local terminal when requested. Setup installs ROS
2 Humble, pyserial and build tools, builds `amr_interfaces` and `amr_vision`,
builds the official STL-27L driver, downloads YOLO11n, and builds the existing
JetPack YOLO Docker image. It does not send motor commands. Driver sources and
builds default to `.runtime/ldlidar_ros2_ws`; an existing
`~/ldlidar_ros2_ws/install/setup.bash` or explicit `LIDAR_WS` is also supported.

Check the installation, then start the run:

```bash
./scripts/start_yolo_lidar_path_avoidance.sh --check
./scripts/start_yolo_lidar_path_avoidance.sh
```

Movement starts automatically after fresh YOLO, LiDAR, and full MCU status
messages arrive. The sequence is forward, stop for a confirmed fused obstacle,
follow a side path, pass the obstacle, return to the original path, drive
straight for two seconds, and stop all owned processes. The two-second segment
uses a monotonic clock with a control-cycle scheduling tolerance. A new obstacle
in that final segment ends the run with a stop. Obstacles without a YOLO match
do not initiate a normal avoidance; the independent LiDAR near-field stop still
applies.

Missing or stale data, a LiDAR hard stop, emergency status, stalled wheel
feedback, or a timeout abort the run. Defaults are 90 seconds for startup,
180 seconds for the run, 20 seconds for entry, and 40 seconds for return.
Vehicle dimensions, camera/LiDAR alignment and motion tuning retain the values
from the original controller in `scripts/test_yolo_lidar_path_avoidance.py`.

Optional overrides:

```bash
MCU_DEVICE=/dev/ttyTHS1 LIDAR_DEVICE=/dev/ttyUSB0 \
  STARTUP_TIMEOUT_S=90 MAX_RUNTIME_S=180 \
  ./scripts/start_yolo_lidar_path_avoidance.sh
```

The active ASCII STM32 firmware receives commands on USART1 and sends status
on USART3. On this robot the launcher uses `/dev/ttyTHS1` for commands and, when
present, `/dev/ttyTHS2` for status. Override the latter with
`MCU_STATUS_DEVICE` if the Jetson wiring differs. YOLO defaults to CPU inference
at image size 320 because the current JetPack 6 CUDA allocator fails during the
640-pixel model warmup. `YOLO_DEVICE=0 YOLO_HALF=1` opts back into CUDA after
that runtime is repaired.

This robot currently does not return `$STATUS` on either Jetson UART. Therefore
the YOLO/LiDAR launcher defaults to `MCU_OPEN_LOOP=1`: it requires fresh camera
and LiDAR data, sends commands on `/dev/ttyTHS1`, and integrates commanded RPM
for path odometry. It sends zero RPM for at least one second before moving. Set
`MCU_OPEN_LOOP=0` after the USART3 status wire is connected; closed-loop mode
then requires live wheel RPM and emergency telemetry before movement.

`--preflight-only` starts the sensors and checks live MCU telemetry while
commanding only zero RPM, then exits. The supplied ASCII firmware latches its
command watchdog after 500 ms without commands. During startup only, the
launcher clears that watchdog latch with `$CMD,0,0,0` and waits for a new status
confirming it cleared. An emergency reported after driving begins remains
latched and immediately stops the run.

Use Ctrl+C to stop an active launcher. When launched in the background, send
SIGTERM to the PID recorded in `.run/yolo_lidar_path.pid`. Logs are
`logs/runtime/yolo_lidar_controller.log`, `yolo_lidar_lidar.log`,
`yolo_lidar_bridge.log`, and `yolo_lidar_yolo.log`. Exit code zero indicates a
completed run (or successful check); nonzero indicates an abort or setup error.

Setup follows the [official ROS apt bootstrap](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/_Apt-Repositories.rst)
and uses the [official LDROBOT driver](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2).

## 1. 최초 한 번

```bash
git clone <REPOSITORY_URL> ~/guide-amr
cd ~/guide-amr
./scripts/setup_jetson.sh
cp config/runtime.env.example config/runtime.env
```

`config/runtime.env`에서 실제 MCU·라이다 장치명, STL-27L 패키지 및 launch 파일명을 확인합니다.
기본값은 MCU `/dev/ttyUSB_mcu`, 라이다 `/dev/ttyUSB_lidar`입니다. 고정 udev 별칭을 아직
등록하지 않았다면 현재 `/dev/ttyUSB0` 등의 실제 값을 입력할 수 있습니다.

OAK-D는 `depthai_ros_driver`만 직접 점유하고 YOLO 노드는 ROS 영상 토픽을 구독합니다.
JetPack용 CUDA PyTorch와 Ultralytics는 일반 PC용 `pip torch`로 덮어쓰지 마십시오.

## 2. 매번 실행

```bash
cd ~/guide-amr
./scripts/start_amr.sh
```

사전점검에 성공하면 OAK-D, YOLO, STL-27L, STM32, 안전제어, 센서융합, 음성안내와 관제
서버가 실행됩니다. 데스크톱 세션에서는 YOLO 영상, RViz 라이다 및 관제 브라우저도 열립니다.
로그는 `logs/runtime/`에 저장됩니다. `Ctrl+C` 또는 아래 명령으로 함께 종료합니다.

```bash
./scripts/stop_amr.sh
```

실행 완료는 모터가 즉시 회전한다는 뜻이 아니라 `READY` 상태입니다. 실제 주행은 STM32
푸시스위치, 센서 정상, 비상정지 해제 및 유효 경로 명령 조건을 모두 만족해야 시작됩니다.

## 3. 부팅 자동 실행(선택)

먼저 수동 실행을 충분히 검증한 다음 서비스 파일의 사용자명과 경로를 실제 값으로 바꿉니다.

```bash
sudo cp deploy/amr-core.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now amr-core.service
```

서비스는 GUI 없이 핵심 시스템만 시작합니다. 화면 자동 실행은 Ubuntu의 시작 프로그램에
`scripts/start_amr.sh`를 중복 등록하지 말고 별도 GUI 전용 스크립트로 구성해야 합니다.
