#!/usr/bin/env bash
set -Eeo pipefail

on_error() {
    local exit_code=$?
    echo "[FAIL] 실행 스크립트 오류"
    echo "       line=$1 exit=$exit_code"
    echo "       command=$2"
}
trap 'on_error "$LINENO" "$BASH_COMMAND"' ERR

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
JETSON_SETUP="$PROJECT_DIR/jetson_ws/install/setup.bash"
LIDAR_SETUP="${LIDAR_WS:-$HOME/ldlidar_ros2_ws}/install/setup.bash"
LIDAR_DEVICE="${LIDAR_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
UART_DEVICE="${MCU_DEVICE:-/dev/ttyTHS1}"
LOG_DIR="$PROJECT_DIR/logs/runtime"
ROS_DOMAIN_VALUE="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_VALUE="${ROS_LOCALHOST_ONLY:-0}"

mkdir -p "$LOG_DIR"

# ROS setup scripts are not compatible with nounset in every installation.
set +u
source "$ROS_SETUP"
source "$JETSON_SETUP"
source "$LIDAR_SETUP"
set -u
export PYTHONPATH="$PROJECT_DIR${PYTHONPATH:+:$PYTHONPATH}"

LIDAR_PID=""
YOLO_PID=""
YOLO_BRIDGE_PID=""

stop_motors() {
    sudo env UART_DEVICE="$UART_DEVICE" /usr/bin/python3 - <<'PY' || true
import os
import time

try:
    import serial

    uart = serial.Serial(os.environ["UART_DEVICE"], 115200, timeout=0.05)
    for _ in range(10):
        uart.write(b"$CMD,0,0,0\r\n")
        uart.flush()
        time.sleep(0.05)
    uart.close()
except Exception:
    pass
PY
}

cleanup() {
    trap - EXIT INT TERM
    echo
    echo "[STOP] 통합 자동회피 종료"
    stop_motors
    for pid in "$YOLO_BRIDGE_PID" "$LIDAR_PID" "$YOLO_PID"; do
        if [[ "$pid" =~ ^[0-9]+$ ]]; then
            kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    sudo docker stop -t 2 socialguide-amr-yolo >/dev/null 2>&1 || true
    echo "[STOP] 모터 및 센서 프로세스 정리 완료"
}
trap cleanup EXIT INT TERM

"$PROJECT_DIR/scripts/stop_amr.sh" || true
sudo -v
sudo chmod 666 "$UART_DEVICE"

# Remove only stale processes belonging to the same LiDAR/YOLO runtime.
sudo pkill -TERM -f 'yolo_udp_bridge_node' 2>/dev/null || true
sudo pkill -TERM -f 'ldlidar_stl_ros2_node' 2>/dev/null || true
sudo docker stop -t 2 socialguide-amr-yolo >/dev/null 2>&1 || true
sleep 2

if sudo ss -lunp 2>/dev/null | grep -q ':5005 '; then
    echo "[FAIL] UDP 5005 포트가 아직 사용 중입니다."
    sudo ss -lunp | grep ':5005 ' || true
    exit 1
fi

echo "[START] STL-27L LiDAR"
setsid ros2 launch ldlidar_stl_ros2 stl27l.launch.py \
    port_name:="$LIDAR_DEVICE" \
    >"$LOG_DIR/avoidance_lidar.log" 2>&1 &
LIDAR_PID=$!

echo "[START] OAK-D YOLO Docker"
setsid sudo "$PROJECT_DIR/scripts/start_yolo_docker.sh" \
    >"$LOG_DIR/avoidance_yolo.log" 2>&1 &
YOLO_PID=$!

echo "[START] YOLO ROS 브리지"
setsid ros2 run amr_vision yolo_udp_bridge_node --ros-args \
    --params-file "$PROJECT_DIR/jetson_ws/src/amr_vision/config/yolo.yaml" \
    >"$LOG_DIR/avoidance_yolo_bridge.log" 2>&1 &
YOLO_BRIDGE_PID=$!

echo "[WAIT] LiDAR와 YOLO 준비 대기 (최대 약 90초)"
LIDAR_READY=0
YOLO_READY=0
for _ in $(seq 1 30); do
    if [[ "$LIDAR_READY" == "0" ]] \
        && timeout 2 ros2 topic echo /scan --once >/dev/null 2>&1; then
        LIDAR_READY=1
        echo "[ OK ] /scan 수신"
    fi
    if [[ "$YOLO_READY" == "0" ]] \
        && timeout 2 ros2 topic echo /yolo/detections --once >/dev/null 2>&1; then
        YOLO_READY=1
        echo "[ OK ] /yolo/detections 수신"
    fi
    if [[ "$LIDAR_READY" == "1" && "$YOLO_READY" == "1" ]]; then
        break
    fi
    sleep 1
done

if [[ "$LIDAR_READY" != "1" ]]; then
    echo "[FAIL] /scan 데이터가 없습니다."
    tail -n 40 "$LOG_DIR/avoidance_lidar.log" || true
    exit 1
fi
if [[ "$YOLO_READY" != "1" ]]; then
    echo "[FAIL] /yolo/detections 데이터가 없습니다."
    tail -n 40 "$LOG_DIR/avoidance_yolo.log" || true
    tail -n 40 "$LOG_DIR/avoidance_yolo_bridge.log" || true
    exit 1
fi

echo "[START] YOLO + LiDAR + SHARP + 초음파 자동회피"
echo "       UP 버튼으로 출발하며 종료는 Ctrl+C입니다."

sudo env \
    JETSON_MODEL_NAME=JETSON_ORIN_NANO \
    ROS_DOMAIN_ID="$ROS_DOMAIN_VALUE" \
    ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_VALUE" \
    PROJECT_DIR="$PROJECT_DIR" \
    LIDAR_SETUP="$LIDAR_SETUP" \
    /bin/bash -c '
        set +u
        source /opt/ros/humble/setup.bash
        source "$PROJECT_DIR/jetson_ws/install/setup.bash"
        source "$LIDAR_SETUP"
        export PYTHONPATH="$PROJECT_DIR${PYTHONPATH:+:$PYTHONPATH}"
        exec /usr/bin/python3 "$PROJECT_DIR/scripts/test_integrated_avoidance.py"
    '
