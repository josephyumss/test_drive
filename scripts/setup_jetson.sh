#!/usr/bin/env bash
set -Eeo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
[[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]] || { echo "ROS 2 $ROS_DISTRO 설치가 필요합니다."; exit 1; }
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pip python3-opencv \
  "ros-$ROS_DISTRO-cv-bridge" "ros-$ROS_DISTRO-rqt-image-view" \
  "ros-$ROS_DISTRO-rviz2" "ros-$ROS_DISTRO-topic-tools" \
  "ros-$ROS_DISTRO-robot-localization" "ros-$ROS_DISTRO-slam-toolbox" \
  "ros-$ROS_DISTRO-navigation2" "ros-$ROS_DISTRO-nav2-bringup" \
  "ros-$ROS_DISTRO-robot-state-publisher" "ros-$ROS_DISTRO-xacro"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
source "/opt/ros/$ROS_DISTRO/setup.bash"
if ! python3 -c 'import torch, ultralytics; assert torch.cuda.is_available()' 2>/dev/null; then
  echo "[주의] JetPack용 CUDA PyTorch/Ultralytics가 확인되지 않아 YOLO는 실행되지 않을 수 있습니다."
  echo "일반 pip torch는 CUDA를 손상시킬 수 있어 자동 설치하지 않습니다."
fi
cd "$PROJECT_DIR"
# Ubuntu 22.04의 기본 pip/build backend는 PEP 660 editable 설치를 지원하지
# 않을 수 있다. 실행 시에는 start_amr.sh가 저장소를 PYTHONPATH에 추가하므로
# 코어 패키지는 일반 사용자 설치로 충분하다.
python3 -m pip install --user .
cd "$PROJECT_DIR/jetson_ws"
rosdep install --from-paths src --ignore-src -r -y || true
colcon build --symlink-install
[[ -f "$PROJECT_DIR/config/runtime.env" ]] || cp "$PROJECT_DIR/config/runtime.env.example" "$PROJECT_DIR/config/runtime.env"
echo "설치 완료: config/runtime.env 확인 후 scripts/start_amr.sh를 실행하세요."
