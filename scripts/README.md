```markdown
# 프로젝트 실행 스크립트

이 폴더는 개발환경 설정, 빌드, 시험 및 실행을 단순화하는 스크립트를 관리한다.

## 스크립트 목록

| 스크립트 | 역할 |
|---|---|
| `setup_jetson.sh` | Jetson 최초 개발환경 구성 |
| `build_jetson.sh` | ROS2 작업공간 빌드 |
| `run_tests.sh` | 단위·통합 시험 실행 |
| `run_simulation.sh` | 가상 센서 및 시뮬레이션 실행 |
| `check_hardware.sh` | 실제 로봇 실행 전 장치 점검 |
| `run_robot.sh` | 실제 로봇 실행 |
| `stop_robot.sh` | 로봇 정상 종료 |
| `download_models.sh` | AI 모델 준비 |
| `build_firmware.sh` | STM32 펌웨어 빌드 |
| `flash_firmware.sh` | STM32 펌웨어 업로드 |

## 기본 사용 순서

최초 설치:

```bash
./scripts/setup_jetson.sh
./scripts/build_jetson.sh
./scripts/run_tests.sh
./scripts/run_simulation.sh
