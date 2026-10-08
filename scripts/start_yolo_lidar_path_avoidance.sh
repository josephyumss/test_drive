#!/usr/bin/env bash
# One run: forward -> fused obstacle avoidance -> forward for 2 s -> stop.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
JETSON_SETUP="$PROJECT_DIR/jetson_ws/install/setup.bash"
if [[ -z "${LIDAR_WS:-}" ]]; then
    runtime_user_home="${SUDO_USER:+$(getent passwd "$SUDO_USER" | cut -d: -f6)}"
    runtime_user_home="${runtime_user_home:-$HOME}"
    if [[ -f "$runtime_user_home/ldlidar_ros2_ws/install/setup.bash" ]]; then
        LIDAR_WS="$runtime_user_home/ldlidar_ros2_ws"
    else
        LIDAR_WS="$PROJECT_DIR/.runtime/ldlidar_ros2_ws"
    fi
fi
LIDAR_SETUP="${LIDAR_SETUP:-$LIDAR_WS/install/setup.bash}"
LIDAR_DEVICE="${LIDAR_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
MCU_DEVICE="${MCU_DEVICE:-/dev/ttyTHS1}"
default_mcu_status_device="$MCU_DEVICE"
# The active ASCII firmware receives $CMD on STM32 USART1 and transmits
# $STATUS on STM32 USART3. The robot wiring maps the second channel to THS2.
if [[ "$MCU_DEVICE" == /dev/ttyTHS1 && -e /dev/ttyTHS2 ]]; then
    default_mcu_status_device=/dev/ttyTHS2
fi
MCU_STATUS_DEVICE="${MCU_STATUS_DEVICE:-$default_mcu_status_device}"
MCU_BAUDRATE="${MCU_BAUDRATE:-115200}"
MCU_OPEN_LOOP="${MCU_OPEN_LOOP:-1}"
STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-90}"
MAX_RUNTIME_S="${MAX_RUNTIME_S:-180}"
LOG_DIR="$PROJECT_DIR/logs/runtime"
MODE="${1:-run}"
case "$MODE" in
    run|--check|--preflight-only) ;;
    -h|--help)
        echo "Usage: $0 [--check|--preflight-only]"
        echo "Default starts movement after live YOLO, LiDAR and MCU readiness."
        echo "--check checks installation/devices; --preflight-only checks live feeds with zero RPM."
        exit 0 ;;
    *) echo "Unknown option: $MODE" >&2; exit 2 ;;
esac
(( $# <= 1 )) || { echo 'Too many arguments' >&2; exit 2; }

failures=0
require_path() {
    if [[ ! -e "$1" ]]; then
        echo "[FAIL] Missing $2: $1" >&2
        failures=$((failures + 1))
    fi
}
require_path "$ROS_SETUP" 'ROS 2 setup'
require_path "$JETSON_SETUP" 'AMR ROS workspace'
require_path "$LIDAR_SETUP" 'LiDAR ROS workspace'
require_path "$MCU_DEVICE" 'STM32 UART'
if [[ "$MCU_OPEN_LOOP" != "1" ]]; then
    require_path "$MCU_STATUS_DEVICE" 'STM32 status UART'
fi
require_path "$LIDAR_DEVICE" 'LiDAR serial device'
require_path /dev/bus/usb 'camera USB access'
for program in docker setsid flock fuser; do
    command -v "$program" >/dev/null || { echo "[FAIL] Missing command: $program" >&2; failures=$((failures + 1)); }
done
if [[ ! -s "$PROJECT_DIR/models/yolo11n.pt" ]]; then
    echo '[FAIL] Missing YOLO model: models/yolo11n.pt' >&2
    failures=$((failures + 1))
fi
if (( failures )); then
    echo "[SETUP] Run: $PROJECT_DIR/scripts/setup_yolo_lidar_path_avoidance.sh" >&2
    exit 1
fi

# Keep all children under one identity; do not change serial device permissions.
if (( EUID != 0 )) && { [[ ! -r "$MCU_DEVICE" || ! -w "$MCU_DEVICE" || ! -r "$LIDAR_DEVICE" || ! -w "$LIDAR_DEVICE" ]] || ! docker info >/dev/null 2>&1; }; then
    if [[ ! -t 0 ]] && ! sudo -n true 2>/dev/null; then
        echo "[FAIL] Local administrator authentication is required. Run in a terminal: sudo $0 $MODE" >&2
        exit 1
    fi
    exec sudo env ROS_SETUP="$ROS_SETUP" LIDAR_SETUP="$LIDAR_SETUP" \
        LIDAR_DEVICE="$LIDAR_DEVICE" MCU_DEVICE="$MCU_DEVICE" MCU_BAUDRATE="$MCU_BAUDRATE" \
        MCU_STATUS_DEVICE="$MCU_STATUS_DEVICE" \
        MCU_OPEN_LOOP="$MCU_OPEN_LOOP" \
        STARTUP_TIMEOUT_S="$STARTUP_TIMEOUT_S" MAX_RUNTIME_S="$MAX_RUNTIME_S" \
        ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
        /bin/bash "$0" "$MODE"
fi

# Generated ROS environment scripts can reference unset variables.
set +u
source "$ROS_SETUP"
source "$JETSON_SETUP"
source "$LIDAR_SETUP"
set -u
export PYTHONPATH="$PROJECT_DIR${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
/usr/bin/python3 -c 'import serial, rclpy; from sensor_msgs.msg import LaserScan; from amr_interfaces.msg import ObstacleInfo'
ros2 pkg prefix ldlidar_stl_ros2 >/dev/null
ros2 pkg prefix amr_vision >/dev/null
docker image inspect socialguide-amr-oak-yolo:jp6 >/dev/null
docker info --format '{{json .Runtimes}}' | /usr/bin/python3 -c 'import json,sys; assert "nvidia" in json.load(sys.stdin), "NVIDIA Docker runtime missing"'
echo '[ OK ] Runtime dependencies and device paths are present.'
echo "[INFO] STM32 command UART: $MCU_DEVICE"
if [[ "$MCU_OPEN_LOOP" == "1" ]]; then
    echo '[INFO] STM32 status UART:  unavailable; command-RPM odometry enabled'
else
    echo "[INFO] STM32 status UART:  $MCU_STATUS_DEVICE"
fi
[[ "$MODE" != --check ]] || exit 0

mkdir -p "$PROJECT_DIR/.run" "$LOG_DIR"
exec 9>"$PROJECT_DIR/.run/yolo_lidar_path.lock"
flock -n 9 || { echo '[FAIL] A YOLO/LiDAR run is already active.' >&2; exit 1; }
devices=("$MCU_DEVICE" "$LIDAR_DEVICE")
if [[ "$MCU_OPEN_LOOP" != "1" && "$MCU_STATUS_DEVICE" != "$MCU_DEVICE" ]]; then
    devices+=("$MCU_STATUS_DEVICE")
fi
for device in "${devices[@]}"; do
    if fuser "$device" >/dev/null 2>&1; then
        echo "[FAIL] Another process owns $device. Stop its current controller first." >&2
        exit 1
    fi
done
/usr/bin/python3 - <<'PY'
import socket
for kind, port in ((socket.SOCK_DGRAM, 5005), (socket.SOCK_STREAM, 8081)):
    with socket.socket(socket.AF_INET, kind) as probe:
        try:
            probe.bind(('0.0.0.0', port))
        except OSError as exc:
            raise SystemExit(f'[FAIL] Required port {port} unavailable: {exc}')
PY
if docker container inspect socialguide-amr-yolo >/dev/null 2>&1; then
    echo '[FAIL] socialguide-amr-yolo already exists; stop the previous run first.' >&2
    exit 1
fi

sensor_pids=()
controller_pid=''
yolo_started=0
abnormal_controller_exit=0
cleanup() {
    local result=$? pid forced=$abnormal_controller_exit controller_result=0
    trap - EXIT INT TERM
    if [[ -n "$controller_pid" ]]; then
        if kill -0 "$controller_pid" 2>/dev/null; then
            kill -TERM -- "-$controller_pid" 2>/dev/null || true
            for _ in {1..30}; do
                kill -0 "$controller_pid" 2>/dev/null || break
                sleep 0.1
            done
            if kill -0 "$controller_pid" 2>/dev/null; then
                kill -KILL -- "-$controller_pid" 2>/dev/null || true
                forced=1
            fi
        fi
        wait "$controller_pid" 2>/dev/null || controller_result=$?
        if (( controller_result >= 128 )); then forced=1; fi
    fi
    # Normal shutdown is performed by the controller. Only if it was killed,
    # send a latched emergency stop; never clear an unknown emergency state.
    if (( forced )); then
        /usr/bin/python3 - "$MCU_DEVICE" "$MCU_BAUDRATE" <<'PY' || true
import serial, sys, time
with serial.Serial(sys.argv[1], int(sys.argv[2]), timeout=0.1, write_timeout=0.2) as uart:
    for _ in range(5):
        uart.write(b'$CMD,0,0,1\r\n')
        time.sleep(0.05)
PY
    fi
    for pid in "${sensor_pids[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    if (( yolo_started )); then
        docker stop -t 2 socialguide-amr-yolo >/dev/null 2>&1 || true
    fi
    for pid in "${sensor_pids[@]}"; do
        for _ in {1..20}; do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL -- "-$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    rm -f "$PROJECT_DIR/.run/yolo_lidar_path.pid"
    echo "[STOP] Run finished (exit=$result). Logs: $LOG_DIR/yolo_lidar_*.log"
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "$$" >"$PROJECT_DIR/.run/yolo_lidar_path.pid"

echo '[START] STL-27L LiDAR'
# Upstream stl27l.launch.py hardcodes its port; set the node parameter directly.
setsid ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
    -p product_name:=LDLiDAR_STL27L -p topic_name:=scan -p frame_id:=base_laser \
    -p port_name:="$LIDAR_DEVICE" -p port_baudrate:=921600 \
    -p laser_scan_dir:=false -p enable_angle_crop_func:=false \
    -p angle_crop_min:=0.0 -p angle_crop_max:=0.0 \
    >"$LOG_DIR/yolo_lidar_lidar.log" 2>&1 9>&- &
sensor_pids+=("$!")
echo '[START] YOLO ROS bridge'
setsid ros2 run amr_vision yolo_udp_bridge_node --ros-args \
    --params-file "$PROJECT_DIR/jetson_ws/src/amr_vision/config/yolo.yaml" \
    >"$LOG_DIR/yolo_lidar_bridge.log" 2>&1 9>&- &
sensor_pids+=("$!")
echo '[START] OAK-D YOLO Docker'
setsid "$PROJECT_DIR/scripts/start_yolo_docker.sh" \
    >"$LOG_DIR/yolo_lidar_yolo.log" 2>&1 9>&- &
sensor_pids+=("$!")
yolo_started=1

args=(--port "$MCU_DEVICE" --baudrate "$MCU_BAUDRATE" \
    --startup-timeout "$STARTUP_TIMEOUT_S" --max-runtime "$MAX_RUNTIME_S")
if [[ "$MCU_OPEN_LOOP" == "1" ]]; then
    args+=(--open-loop)
else
    args+=(--status-port "$MCU_STATUS_DEVICE" --clear-startup-emergency)
fi
if [[ "$MODE" == --preflight-only ]]; then
    args+=(--preflight-only)
    echo '[CHECK] Waiting for live YOLO, LiDAR and STM32 data; zero RPM only.'
else
    echo '[RUN] Movement starts when live feeds are ready: forward -> avoidance -> forward 2 s -> stop.'
fi
setsid /usr/bin/python3 -u "$PROJECT_DIR/scripts/test_yolo_lidar_path_avoidance.py" "${args[@]}" \
    >"$LOG_DIR/yolo_lidar_controller.log" 2>&1 9>&- &
controller_pid=$!
echo "[LOG] $LOG_DIR/yolo_lidar_controller.log"
while kill -0 "$controller_pid" 2>/dev/null; do
    for pid in "${sensor_pids[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "[FAIL] Sensor process $pid exited; stopping run. See sensor logs." >&2
            exit 1
        fi
    done
    sleep 0.2
done
result=0
wait "$controller_pid" || result=$?
if (( result >= 128 )); then abnormal_controller_exit=1; fi
controller_pid=''
tail -n 15 "$LOG_DIR/yolo_lidar_controller.log"
exit "$result"
