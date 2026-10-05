#!/usr/bin/env bash
set -Eeo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_FILE="$PROJECT_DIR/config/runtime.env"
[[ -f "$RUNTIME_FILE" ]] && source "$RUNTIME_FILE"
ROS_DISTRO="${ROS_DISTRO:-humble}"
OAK_LAUNCH_PACKAGE="${OAK_LAUNCH_PACKAGE:-depthai_ros_driver}"
OAK_LAUNCH_FILE="${OAK_LAUNCH_FILE:-camera.launch.py}"
VISION_MODE="${VISION_MODE:-docker}"
YOLO_WEB_URL="${YOLO_WEB_URL:-http://127.0.0.1:8081}"
LIDAR_LAUNCH_PACKAGE="${LIDAR_LAUNCH_PACKAGE:-ldlidar_stl_ros2}"
LIDAR_LAUNCH_FILE="${LIDAR_LAUNCH_FILE:-stl27l.launch.py}"
MCU_DEVICE="${MCU_DEVICE:-/dev/ttyTHS1}"
LIDAR_DEVICE="${LIDAR_DEVICE:-/dev/ttyUSB_lidar}"
LIDAR_PORT_ARGUMENT="${LIDAR_PORT_ARGUMENT:-port_name}"
OPEN_GUI="${OPEN_GUI:-1}"
OPEN_YOLO_WINDOW="${OPEN_YOLO_WINDOW:-1}"
OPEN_RVIZ="${OPEN_RVIZ:-1}"
OPEN_DASHBOARD="${OPEN_DASHBOARD:-1}"
STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-60}"
TOPIC_PROBE_TIMEOUT_S="${TOPIC_PROBE_TIMEOUT_S:-8}"
RUN_DIR="$PROJECT_DIR/.run"
LOG_DIR="$PROJECT_DIR/logs/runtime"
PID_FILE="$RUN_DIR/amr.pids"
CONTROLLER_FILE="$RUN_DIR/amr.controller.pid"
mkdir -p "$RUN_DIR" "$LOG_DIR"

if [[ -f "$CONTROLLER_FILE" ]]; then
  read -r old_controller < "$CONTROLLER_FILE" || true
  if [[ "${old_controller:-}" =~ ^[0-9]+$ ]] && kill -0 "$old_controller" 2>/dev/null; then
    echo "[FAIL] AMR가 이미 실행 중입니다 (controller PID: $old_controller)."
    echo "       먼저 $PROJECT_DIR/scripts/stop_amr.sh 를 실행하세요."
    exit 1
  fi
  echo "[WARN] 오래된 실행 정보 파일을 정리합니다."
  rm -f "$CONTROLLER_FILE" "$PID_FILE"
fi

# ROS-generated setup files read these optional variables during initialization.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$PROJECT_DIR/jetson_ws/install/setup.bash"
if [[ -n "${LIDAR_SETUP_FILE:-}" && -f "$LIDAR_SETUP_FILE" ]]; then
  source "$LIDAR_SETUP_FILE"
fi
export PYTHONPATH="$PROJECT_DIR${PYTHONPATH:+:$PYTHONPATH}"
"$PROJECT_DIR/scripts/preflight_amr.sh"

echo "$$" > "$CONTROLLER_FILE"
: > "$PID_FILE"

start_process() {
  local name="$1"; shift
  echo "[START] $name"
  # Give each component its own process group so ROS launch children are also
  # stopped, including the dashboard HTTP server.
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  echo "$! $name" >> "$PID_FILE"
}

cleanup_done=0
cleanup() {
  local pid name
  (( cleanup_done == 0 )) || return 0
  cleanup_done=1
  trap - EXIT INT TERM
  echo "AMR 프로세스를 종료합니다."
  if [[ -f "$PID_FILE" ]]; then
    while read -r pid name; do
      if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
        echo "[STOP] $name ($pid)"
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
      fi
    done < "$PID_FILE"

    sleep 2
    while read -r pid name; do
      if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
        echo "[KILL] 종료되지 않은 $name ($pid)"
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
      fi
    done < "$PID_FILE"
  fi
  docker stop -t 2 socialguide-amr-yolo >/dev/null 2>&1 || true
  rm -f "$PID_FILE" "$CONTROLLER_FILE"
}
trap cleanup EXIT INT TERM

if [[ "$VISION_MODE" == "docker" ]]; then
  start_process yolo_docker "$PROJECT_DIR/scripts/start_yolo_docker.sh"
else
  start_process oak_driver ros2 launch "$OAK_LAUNCH_PACKAGE" "$OAK_LAUNCH_FILE"
fi
start_process lidar_driver ros2 launch "$LIDAR_LAUNCH_PACKAGE" "$LIDAR_LAUNCH_FILE" "$LIDAR_PORT_ARGUMENT:=$LIDAR_DEVICE"
start_process amr_core ros2 launch amr_bringup hardware_system.launch.py "mcu_port:=$MCU_DEVICE"

echo "[WAIT] 센서와 ROS 노드 준비 대기 (최대 ${STARTUP_TIMEOUT_S}초)"
topics=(/yolo/detections /scan /mcu/status /safety/state)
declare -A topic_ready=()
startup_deadline=$((SECONDS + STARTUP_TIMEOUT_S))

while (( SECONDS < startup_deadline )); do
  all_topics_ready=1
  for topic in "${topics[@]}"; do
    if [[ "${topic_ready[$topic]:-0}" == "1" ]]; then
      continue
    fi
    all_topics_ready=0
    if timeout "$TOPIC_PROBE_TIMEOUT_S" ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      topic_ready["$topic"]=1
      echo "[ OK ] 토픽 수신: $topic"
    fi
  done
  (( all_topics_ready == 0 )) || break
  sleep 1
done

topic_failures=0
for topic in "${topics[@]}"; do
  if [[ "${topic_ready[$topic]:-0}" != "1" ]]; then
    echo "[WARN] 시작 제한시간 내 토픽 미수신: $topic"
    topic_failures=$((topic_failures + 1))
  fi
done

mcu_connected=0
if [[ "${topic_ready[/mcu/status]:-0}" == "1" ]]; then
  if timeout 3 ros2 topic echo /mcu/status --once --field connected 2>/dev/null | grep -q '^true$'; then
    mcu_connected=1
    echo "[ OK ] STM32 STATUS 통신 연결"
  else
    echo "[WARN] /mcu/status 토픽은 수신되지만 STM32 STATUS 데이터가 없습니다."
    echo "       STM32 전원, 펌웨어 및 UART 송신을 확인하세요. 모터는 활성화하지 않습니다."
  fi
fi

if [[ "$OPEN_GUI" == "1" && -n "${DISPLAY:-}" ]]; then
  [[ "$OPEN_YOLO_WINDOW" == "1" ]] && xdg-open "$YOLO_WEB_URL" >/dev/null 2>&1 || true
  [[ "$OPEN_RVIZ" == "1" ]] && start_process lidar_rviz rviz2 -d "$PROJECT_DIR/config/amr_lidar.rviz"
  [[ "$OPEN_DASHBOARD" == "1" ]] && xdg-open http://127.0.0.1:8080 >/dev/null 2>&1 || true
else
  echo "GUI를 열지 않습니다. 관제 주소: http://127.0.0.1:8080"
fi

if (( topic_failures == 0 && mcu_connected == 1 )); then
  echo "AMR가 READY 상태로 실행되었습니다. 실제 주행은 안전 조건 확인 후 푸시스위치로 허용됩니다."
else
  echo "AMR가 DEGRADED 상태로 실행되었습니다. 위 경고를 해결하기 전에는 주행하지 마세요."
fi
echo "종료 명령: $PROJECT_DIR/scripts/stop_amr.sh"
wait
