# Jetson 원클릭 실행

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
