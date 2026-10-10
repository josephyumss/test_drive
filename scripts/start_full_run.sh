#!/usr/bin/env bash
# Boot service and foreground/manual launcher for continuous button-controlled runs.
set -Eeuo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-run}"
case "$MODE" in
    run|--check|--preflight-only) ;;
    --stop) exec /bin/bash "$PROJECT_DIR/scripts/stop_full_run.sh" ;;
    --instant-stop) exec /usr/bin/python3 "$PROJECT_DIR/scripts/full_run_user_stop.py" ;;
    -h|--help)
        echo 'Usage: sudo bash scripts/start_full_run.sh [--check|--preflight-only|--instant-stop|--stop]'
        echo 'Existing STM32 firmware is used by default (MCU_PROTOCOL=legacy).'
        echo '--instant-stop pauses the live run; UP resumes the saved manoeuvre.'
        echo 'Starts in READY at zero RPM. A new physical UP press starts motion.'
        echo 'Ctrl+C/SIGTERM stops motors and owned processes. Restart manually with the same command.'
        exit 0 ;;
    *) echo "Unknown option: $MODE" >&2; exit 2 ;;
esac
(( $# <= 1 )) || { echo 'Too many arguments' >&2; exit 2; }
[[ "$(uname -s)" == Linux ]] || { echo '[FAIL] This launcher runs on the Linux Jetson.' >&2; exit 1; }

RUN_DIR="$PROJECT_DIR/.run"
LOG_ROOT="${FULL_RUN_LOG_ROOT:-$PROJECT_DIR/logs/full_run}"
umask 027
mkdir -p "$RUN_DIR" "$LOG_ROOT"
LOG_DIR="$(mktemp -d "$LOG_ROOT/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
export FULL_RUN_LOG_DIR="$LOG_DIR"
export YOLO_RUNTIME_OWNER="$(basename "$LOG_DIR")"
exec > >(tee -a "$LOG_DIR/launcher.log") 2>&1
echo "[LOG] $LOG_DIR"
trap 'code=$?; echo "[FAIL] exit=$code line=$LINENO command=$BASH_COMMAND"' ERR

# Available before environment/dependency checks, so startup errors are also
# archived. This helper imports only Python's standard library.
auto_bundle_on_exit() {
    local result="$1" reason="${2:-launcher_exit_$1}" interpreter="${PYTHON_BIN:-/usr/bin/python3}"
    local -a bundle_options=(--auto-bundle --capture-environment)
    [[ "${3:-}" != finalize ]] || bundle_options+=(--finalize-bundle)
    if (( result != 0 )) || [[ -f "$LOG_DIR/auto_bundle.request.json" ]]; then
        if ! command -v "$interpreter" >/dev/null 2>&1; then interpreter=/usr/bin/python3; fi
        echo "[BUNDLE] Automatic diagnostics: $reason"
        if "$interpreter" "$PROJECT_DIR/scripts/full_run_diagnostics.py" "$LOG_DIR" \
            "${bundle_options[@]}" --reason "$reason" >> "$LOG_DIR/bundle.log" 2>&1; then
            echo "[BUNDLE] Saved: $LOG_ROOT/$(basename "$LOG_DIR")-debug.tar.gz"
        else
            echo "[BUNDLE FAIL] Original logs kept at $LOG_DIR; see bundle.log."
        fi
    fi
    return 0
}
early_exit() {
    local result=$?
    trap - EXIT ERR
    set +e
    auto_bundle_on_exit "$result"
    exit "$result"
}
trap early_exit EXIT
source "$PROJECT_DIR/scripts/full_run_log_access.sh"
full_run_prepare_log_access "$PROJECT_DIR" "$LOG_ROOT" "$LOG_DIR"

# Optional site-specific file; no dependencies on a desktop login session.
if [[ -f "$PROJECT_DIR/config/full_run.env" ]]; then
    set -a
    source "$PROJECT_DIR/config/full_run.env"
    set +a
fi
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
JETSON_SETUP="${JETSON_SETUP:-$PROJECT_DIR/jetson_ws/install/setup.bash}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
MCU_DEVICE="${MCU_DEVICE:-/dev/ttyTHS1}"
MCU_STATUS_DEVICE="${MCU_STATUS_DEVICE:-/dev/ttyTHS2}"
MCU_BAUDRATE="${MCU_BAUDRATE:-115200}"
MCU_PROTOCOL="${MCU_PROTOCOL:-legacy}"
SIDE_SENSOR_SOURCE="${SIDE_SENSOR_SOURCE:-mcu}"
[[ "$SIDE_SENSOR_SOURCE" == mcu || "$SIDE_SENSOR_SOURCE" == gpio ]] || {
    echo '[FAIL] SIDE_SENSOR_SOURCE must be mcu or gpio' >&2; exit 1;
}
MCU_STATUS_COMMAND_FALLBACK="${MCU_STATUS_COMMAND_FALLBACK:-1}"
[[ "$MCU_STATUS_COMMAND_FALLBACK" == 0 || "$MCU_STATUS_COMMAND_FALLBACK" == 1 ]] || {
    echo '[FAIL] MCU_STATUS_COMMAND_FALLBACK must be 0 or 1' >&2; exit 1;
}
LIDAR_DEVICE="${LIDAR_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
FULL_RUN_CONFIG="${FULL_RUN_CONFIG:-$PROJECT_DIR/config/full_run.json}"
export MCU_DEVICE MCU_STATUS_DEVICE MCU_BAUDRATE MCU_PROTOCOL MCU_STATUS_COMMAND_FALLBACK
export SIDE_SENSOR_SOURCE
export LIDAR_DEVICE FULL_RUN_CONFIG ROS_SETUP JETSON_SETUP
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export PYTHONPATH="$PROJECT_DIR${PYTHONPATH:+:$PYTHONPATH}" PYTHONUNBUFFERED=1
# This project's board is Orin Nano/Super, as in the existing fused launchers.
# Export explicitly for sudo/manual runs AND systemd (no desktop environment).
export JETSON_MODEL_NAME="${JETSON_MODEL_NAME:-JETSON_ORIN_NANO}"
echo "[GPIO] JETSON_MODEL_NAME=$JETSON_MODEL_NAME (project board fallback; detected hardware still takes precedence)"
if [[ -z "${LIDAR_SETUP:-}" ]]; then
    runtime_user="${SUDO_USER:-$(stat -c %U "$PROJECT_DIR")}"
    runtime_user_home="$(getent passwd "$runtime_user" | cut -d: -f6)"
    if [[ -f "$runtime_user_home/ldlidar_ros2_ws/install/setup.bash" ]]; then
        LIDAR_SETUP="$runtime_user_home/ldlidar_ros2_ws/install/setup.bash"
    else
        LIDAR_SETUP="$PROJECT_DIR/.runtime/ldlidar_ros2_ws/install/setup.bash"
    fi
fi
export LIDAR_SETUP
fail() { echo "[FAIL] $*" >&2; exit 1; }
check_errors=0
check_fail() { echo "[CHECK FAIL] $*" >&2; check_errors=$((check_errors + 1)); }
# Collect before dependency checks: a missing device/import must not prevent
# evidence of OTHER independent problems from reaching the automatic bundle.
environment_python="$PYTHON_BIN"
command -v "$environment_python" >/dev/null 2>&1 || environment_python=/usr/bin/python3
if ! "$environment_python" "$PROJECT_DIR/jetson/amr_core/full_run_environment.py" "$PROJECT_DIR" \
    > "$LOG_DIR/environment-start.json" 2> "$LOG_DIR/environment-start.log"; then
    echo '[WARN] Detailed startup inventory failed; see environment-start.log. Continuing required checks.'
fi
for program in "$PYTHON_BIN" docker setsid flock fuser; do
    command -v "$program" >/dev/null || check_fail "Missing executable: $program"
done
for file in "$ROS_SETUP" "$JETSON_SETUP" "$LIDAR_SETUP" "$FULL_RUN_CONFIG"; do
    [[ -r "$file" ]] || check_fail "Missing/unreadable configuration or workspace: $file"
done
for device in "$MCU_DEVICE" "$MCU_STATUS_DEVICE" "$LIDAR_DEVICE"; do
    [[ -r "$device" && -w "$device" ]] || check_fail "Missing or inaccessible device: $device. Existing STM32 STATUS must reach the configured RX port."
done
[[ -d /dev/bus/usb ]] || check_fail 'Camera USB devices not present'
[[ -s "$PROJECT_DIR/models/yolo11n.pt" ]] || check_fail 'Missing models/yolo11n.pt; run setup_yolo_lidar_path_avoidance.sh'
set +u
for file in "$ROS_SETUP" "$JETSON_SETUP" "$LIDAR_SETUP"; do
    [[ ! -r "$file" ]] || source "$file"
done
set -u
if command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    "$PYTHON_BIN" "$PROJECT_DIR/scripts/full_run_controller.py" --config "$FULL_RUN_CONFIG" --log-dir "$LOG_DIR" \
        --mcu-protocol "$MCU_PROTOCOL" --side-sensor-source "$SIDE_SENSOR_SOURCE" --check-config \
        || check_fail 'Full-run configuration validation failed'
    "$PYTHON_BIN" "$PROJECT_DIR/scripts/full_run_requirements.py" > "$LOG_DIR/imports.json" 2> "$LOG_DIR/imports.log" \
        || check_fail 'Python/ROS/GPIO imports or installed-source identity failed; see imports.json and imports.log'
fi
for package in ldlidar_stl_ros2 amr_vision; do
    ros2 pkg prefix "$package" || check_fail "ROS package unavailable: $package"
done
if command -v docker >/dev/null 2>&1; then
    docker info --format '{{json .Runtimes}}' | "$PYTHON_BIN" -c 'import json,sys; assert "nvidia" in json.load(sys.stdin), "NVIDIA container runtime missing"' \
        || check_fail 'Docker daemon/NVIDIA runtime check failed'
    docker image inspect socialguide-amr-oak-yolo:jp6 --format '{{.Id}}' || check_fail 'YOLO Docker image unavailable'
fi
[[ "$MCU_PROTOCOL" == legacy || "$MCU_PROTOCOL" == ctrl ]] || check_fail 'MCU_PROTOCOL must be legacy or ctrl'
(( check_errors == 0 )) || fail "$check_errors independent startup checks failed. No robot processes were started."
echo "[INFO] command=$MCU_DEVICE status=$MCU_STATUS_DEVICE command_RX_fallback=$MCU_STATUS_COMMAND_FALLBACK side_sensors=$SIDE_SENSOR_SOURCE lidar=$LIDAR_DEVICE mode=$MODE protocol=$MCU_PROTOCOL"
echo '[INFO] Default legacy uses existing $CMD/$STATUS, without a firmware update. Live status RX is required; no open-loop fallback.'
[[ "$MODE" != --check ]] || { echo '[CHECK] Installation/configuration passed; live sensors and firmware not checked.'; exit 0; }

# Never stop another run's processes, container or serial-port owner.
exec 9>"$RUN_DIR/full_run.lock"
flock -n 9 || fail 'A full run is already active. Use --stop before starting manually.'
for device in "$MCU_DEVICE" "$MCU_STATUS_DEVICE" "$LIDAR_DEVICE"; do
    fuser "$device" >/dev/null 2>&1 && fail "Another process owns $device; stop its launcher first."
done
if docker container inspect socialguide-amr-yolo >/dev/null 2>&1; then
    fail 'socialguide-amr-yolo already exists; stop its original launcher first.'
fi
"$PYTHON_BIN" - <<'PY'
import socket
for kind, port in ((socket.SOCK_DGRAM, 5005), (socket.SOCK_STREAM, 8081)):
    with socket.socket(socket.AF_INET, kind) as probe:
        try:
            probe.bind(('0.0.0.0', port))
        except OSError as exc:
            raise SystemExit(f'Required port {port} unavailable: {exc}')
PY
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"
cp "$FULL_RUN_CONFIG" "$LOG_DIR/config.json"
tar -czf "$LOG_DIR/source.tar.gz" -C "$PROJECT_DIR" \
    scripts/start_full_run.sh scripts/stop_full_run.sh scripts/full_run_controller.py scripts/full_run_user_stop.py scripts/full_run_requirements.py \
    scripts/full_run_log_access.sh scripts/full_run_setup_steps.sh \
    jetson/amr_core/full_run.py jetson/amr_core/full_run_log.py \
    jetson/amr_core/full_run_environment.py jetson/amr_core/full_run_summary.py \
    jetson/amr_core/full_run_legacy.py jetson/amr_core/full_run_user_stop.py jetson/amr_core/full_run_uart.py jetson/amr_core/ascii_serial_bridge.py \
    jetson/amr_core/serial_bridge.py jetson/amr_core/transport.py jetson/amr_core/packet.py \
    jetson/amr_core/crc16.py protocol/protocol_constants.py \
    jetson/amr_core/full_run_ultrasonic.py jetson/amr_core/reactive_avoidance.py \
    jetson/amr_core/full_run_diagnostics.py scripts/full_run_diagnostics.py \
    stm32/Core/Src/main.c stm32/Core/Inc/full_run_control.h \
    scripts/start_yolo_docker.sh docker/oak_yolo_udp.py \
    jetson_ws/src/amr_vision/amr_vision/yolo_udp_bridge_node.py jetson_ws/src/amr_vision/config/yolo.yaml
sha256sum "$PROJECT_DIR"/scripts/*full_run* "$PROJECT_DIR"/jetson/amr_core/full_run*.py \
    "$PROJECT_DIR/jetson/amr_core/reactive_avoidance.py" "$PROJECT_DIR/stm32/Core/Src/main.c" \
    "$PROJECT_DIR/stm32/Core/Inc/full_run_control.h" > "$LOG_DIR/source.sha256"
{
    date -u --iso-8601=seconds
    uname -a
    cat /etc/os-release
    id
    ls -l "$MCU_DEVICE" "$MCU_STATUS_DEVICE" "$LIDAR_DEVICE"
    # Read-only mapping evidence. Do not change pinmux/getty or probe other
    # serial devices: the active MCU ports are the only permitted RX sources.
    for uart_device in "$MCU_DEVICE" "$MCU_STATUS_DEVICE"; do
        printf 'UART requested=%s resolved=' "$uart_device"
        uart_resolved="$(readlink -f "$uart_device")" || uart_resolved="$uart_device"
        printf '%s\n' "$uart_resolved"
        uart_name="${uart_resolved##*/}"
        for uart_metadata in /sys/class/tty/"$uart_name"/device \
            /sys/class/tty/"$uart_name"/device/driver /sys/class/tty/"$uart_name"/device/of_node; do
            printf 'UART sysfs %s: ' "$uart_metadata"
            readlink -f "$uart_metadata" || true
        done
        if command -v timeout >/dev/null 2>&1; then
            if command -v udevadm >/dev/null 2>&1; then
                timeout 3s udevadm info --query=property --name="$uart_device" || true
            fi
            if command -v systemctl >/dev/null 2>&1; then
                printf 'UART console unit serial-getty@%s.service: ' "$uart_name"
                timeout 3s systemctl is-active "serial-getty@$uart_name.service" || true
            fi
        fi
    done
    if command -v timeout >/dev/null 2>&1 && command -v systemctl >/dev/null 2>&1; then
        printf 'Jetson console unit nvgetty.service: '
        timeout 3s systemctl is-active nvgetty.service || true
    fi
    "$PYTHON_BIN" --version
    docker version
    df -h "$LOG_DIR"
    [[ ! -r /proc/device-tree/model ]] || tr '\0' '\n' < /proc/device-tree/model
    "$PYTHON_BIN" -c 'import serial, rclpy; print("pyserial", serial.__version__); print("ROS", rclpy.__file__)'
    if [[ "$SIDE_SENSOR_SOURCE" == gpio ]]; then
        "$PYTHON_BIN" -c 'import Jetson.GPIO as g; print("GPIO", getattr(g,"VERSION", "unknown")); print("GPIO board", getattr(g,"JETSON_INFO", {}))'
    else
        echo 'Side sensors: STM32 STATUS centimetres; no Jetson GPIO import/owner.'
    fi
    printf 'ROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s\n' "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY"
} > "$LOG_DIR/system.txt" 2>&1
sensor_pids=()
sensor_names=()
controller_pid=''
yolo_owned=0
diagnostics_pid=''
diagnostics_reported=0
failure_reason='runtime_exit'

cleanup() {
    local result=$? forced=0 pid
    trap - EXIT INT TERM ERR
    set +e
    if [[ -n "$controller_pid" ]]; then
        kill -TERM -- "-$controller_pid" 2>/dev/null
        for _ in {1..30}; do
            kill -0 -- "-$controller_pid" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 -- "-$controller_pid" 2>/dev/null; then
            kill -KILL -- "-$controller_pid" 2>/dev/null
            forced=1
        fi
        wait "$controller_pid"
        controller_result=$?
        echo "[CHILD EXIT] name=controller pid=$controller_pid exit=$controller_result phase=cleanup"
        (( controller_result >= 128 )) && forced=1
    fi
    # UART owner has exited. This cannot race the live controller or clear a
    # fault. The firmware watchdog also independently stops orphaned commands.
    "$PYTHON_BIN" - "$MCU_DEVICE" "$MCU_BAUDRATE" >> "$LOG_DIR/shutdown.log" 2>&1 <<'PY'
import serial, sys, time, traceback
try:
    with serial.Serial(sys.argv[1], int(sys.argv[2]), timeout=0, write_timeout=0.15, exclusive=True) as uart:
        for _ in range(3):
            stop_command = b'$CMD,0,0,1\r\n'
            written = uart.write(stop_command)
            if written != len(stop_command):
                raise IOError(f'Short fallback stop write: {written}/{len(stop_command)}')
            time.sleep(0.02)
    print('Fallback stop/brake commands sent')
except Exception:
    traceback.print_exc()
    sys.exit(1)
PY
    shutdown_result=$?
    echo "[FALLBACK STOP] exit=$shutdown_result; successful writes do not prove MCU receipt"
    if (( shutdown_result != 0 )); then
        result=1
        failure_reason="${failure_reason}_fallback_stop_failed"
    fi
    for pid in "${sensor_pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null; done
    if (( yolo_owned )); then
        owner="$(docker container inspect socialguide-amr-yolo --format '{{index .Config.Labels "amr.full_run.owner"}}' 2>/dev/null)"
        if [[ "$owner" == "$YOLO_RUNTIME_OWNER" ]]; then
            docker stop -t 2 socialguide-amr-yolo >> "$LOG_DIR/shutdown.log" 2>&1
        else
            echo "[INFO] YOLO container absent or belongs to another run; not stopping it."
        fi
    fi
    for pid in "${sensor_pids[@]}"; do
        for _ in {1..20}; do kill -0 -- "-$pid" 2>/dev/null || break; sleep 0.1; done
        if kill -0 -- "-$pid" 2>/dev/null; then
            kill -KILL -- "-$pid" 2>/dev/null
            forced=1
        fi
        wait "$pid"
        echo "[CHILD EXIT] pid=$pid exit=$? phase=cleanup"
    done
    rm -f -- "$RUN_DIR/full_run.pid"
    echo "[STOP] exit=$result forced=$forced logs=$LOG_DIR"
    # Motors and owned sensor processes are stopped before any blocking
    # compression. Wait for an already-started fault capture, then retry only
    # if it failed. Successful automatic captures never get overwritten.
    if [[ -n "$diagnostics_pid" ]]; then wait "$diagnostics_pid"; fi
    if (( forced && result == 0 )); then
        auto_bundle_on_exit 1 forced_controller_shutdown finalize
    else
        auto_bundle_on_exit "$result" "$failure_reason" finalize
    fi
    exit "$result"
}
trap cleanup EXIT
trap 'exit 0' INT TERM
printf '%s %s\n' "$$" "$(awk '{print $22}' "/proc/$$/stat")" > "$RUN_DIR/full_run.pid"

# Start the zero-RPM controller before sensor warmup. It owns the only actuator
# UART and keeps the STM32 heartbeat alive during YOLO initialization.
args=(--config "$FULL_RUN_CONFIG" --port "$MCU_DEVICE" --status-port "$MCU_STATUS_DEVICE" \
      --baudrate "$MCU_BAUDRATE" --mcu-protocol "$MCU_PROTOCOL" --side-sensor-source "$SIDE_SENSOR_SOURCE" --log-dir "$LOG_DIR" --supervisor-pid "$$" \
      --user-stop-file "$RUN_DIR/full_run.user_stop.json" \
      --supervisor-start-ticks "$(awk '{print $22}' "/proc/$$/stat")")
[[ "$MODE" != --preflight-only ]] || args+=(--preflight-only)
[[ "$MCU_STATUS_COMMAND_FALLBACK" != 0 ]] || args+=(--no-command-status-fallback)
setsid "$PYTHON_BIN" -u "$PROJECT_DIR/scripts/full_run_controller.py" "${args[@]}" \
    > >(tee -a "$LOG_DIR/controller.log" 9>&-) 2>&1 9>&- &
controller_pid=$!
setsid ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
    -p product_name:=LDLiDAR_STL27L -p topic_name:=scan -p frame_id:=base_laser \
    -p port_name:="$LIDAR_DEVICE" -p port_baudrate:=921600 \
    -p laser_scan_dir:=false -p enable_angle_crop_func:=false \
    -p angle_crop_min:=0.0 -p angle_crop_max:=0.0 > "$LOG_DIR/lidar.log" 2>&1 9>&- &
sensor_pids+=("$!"); sensor_names+=(lidar)
setsid ros2 run amr_vision yolo_udp_bridge_node --ros-args \
    --params-file "$PROJECT_DIR/jetson_ws/src/amr_vision/config/yolo.yaml" > "$LOG_DIR/bridge.log" 2>&1 9>&- &
sensor_pids+=("$!"); sensor_names+=(bridge)
setsid /bin/bash "$PROJECT_DIR/scripts/start_yolo_docker.sh" > "$LOG_DIR/yolo.log" 2>&1 9>&- &
sensor_pids+=("$!"); sensor_names+=(yolo)
yolo_owned=1
echo "[RUN] controller=$controller_pid sensors=${sensor_pids[*]}. READY is stationary; press UP only after READY."
while kill -0 "$controller_pid" 2>/dev/null; do
    if [[ -z "$diagnostics_pid" && -f "$LOG_DIR/auto_bundle.request.json" ]]; then
        echo '[BUNDLE] Fault detected; saving diagnostics in the background.'
        "$PYTHON_BIN" "$PROJECT_DIR/scripts/full_run_diagnostics.py" "$LOG_DIR" \
            --auto-bundle --capture-environment >> "$LOG_DIR/bundle.log" 2>&1 9>&- &
        diagnostics_pid=$!
    fi
    if [[ -n "$diagnostics_pid" ]] && (( diagnostics_reported == 0 )) && ! kill -0 "$diagnostics_pid" 2>/dev/null; then
        if wait "$diagnostics_pid"; then
            echo "[BUNDLE] Saved: $LOG_ROOT/$(basename "$LOG_DIR")-debug.tar.gz"
        else
            echo '[BUNDLE FAIL] Background capture failed; will retry after shutdown. Original logs are preserved.'
        fi
        diagnostics_reported=1
    fi
    for index in "${!sensor_pids[@]}"; do
        if ! kill -0 "${sensor_pids[index]}" 2>/dev/null; then
            result=0
            wait "${sensor_pids[index]}" || result=$?
            echo "[FAIL] sensor=${sensor_names[index]} pid=${sensor_pids[index]} exit=$result"
            failure_reason="sensor_${sensor_names[index]}_exit_$result"
            exit 1
        fi
    done
    sleep 0.2
done
result=0
wait "$controller_pid" || result=$?
echo "[CONTROLLER] exit=$result"
failure_reason="controller_exit_$result"
# Preserve PID for cleanup/fallback; wait on a reaped PID is harmless.
exit "$result"
