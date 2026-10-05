#!/usr/bin/env bash
set -o pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$PROJECT_DIR/config/runtime.env" ]]; then
  # shellcheck disable=SC1091
  source "$PROJECT_DIR/config/runtime.env"
fi
ROS_DISTRO="${ROS_DISTRO:-humble}"
MCU_DEVICE="${MCU_DEVICE:-/dev/ttyTHS1}"
LIDAR_DEVICE="${LIDAR_DEVICE:-/dev/ttyUSB_lidar}"
VISION_MODE="${VISION_MODE:-docker}"
YOLO_DOCKER_IMAGE="${YOLO_DOCKER_IMAGE:-socialguide-amr-oak-yolo:jp6}"
errors=0

check_file() {
  if [[ ! -e "$1" ]]; then echo "[FAIL] 장치 없음: $1"; errors=$((errors + 1));
  else echo "[ OK ] 장치 확인: $1"; fi
}

[[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]] || { echo "[FAIL] ROS 2 $ROS_DISTRO 없음"; errors=$((errors + 1)); }
[[ -f "$PROJECT_DIR/jetson_ws/install/setup.bash" ]] || { echo "[FAIL] jetson_ws 빌드 필요"; errors=$((errors + 1)); }
check_file "$MCU_DEVICE"
check_file "$LIDAR_DEVICE"
if lsusb | grep -qiE 'Luxonis|03e7'; then echo "[ OK ] OAK-D USB 확인";
else echo "[FAIL] OAK-D USB를 찾지 못함"; errors=$((errors + 1)); fi
if [[ "$VISION_MODE" == "docker" ]]; then
  if ! command -v docker >/dev/null 2>&1; then
    echo "[FAIL] Docker 없음"
    errors=$((errors + 1))
  elif ! docker image inspect "$YOLO_DOCKER_IMAGE" >/dev/null 2>&1; then
    echo "[FAIL] YOLO Docker 이미지 없음: $YOLO_DOCKER_IMAGE"
    echo "       먼저 ./scripts/build_yolo_docker.sh 를 실행하세요."
    errors=$((errors + 1))
  else
    echo "[ OK ] YOLO Docker 이미지 확인"
  fi
elif ! python3 -c 'import torch, ultralytics, cv2; assert torch.cuda.is_available()' 2>/dev/null; then
  echo "[FAIL] 호스트 CUDA PyTorch/Ultralytics/OpenCV 환경 확인 실패"
  errors=$((errors + 1))
else
  echo "[ OK ] 호스트 YOLO CUDA 환경 확인"
fi

if (( errors > 0 )); then
  echo "사전 점검 실패: 모터는 활성화하지 않습니다."
  exit 1
fi
echo "사전 점검 완료"
