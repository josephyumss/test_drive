#!/usr/bin/env bash
# Internal, single-step worker for setup_full_run.sh. No motor/service commands.
# A new process per step stops the failing step, not the complete installation.
set -Eeuo pipefail
PROJECT_DIR="${PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
STEP="${1:?Internal usage: full_run_setup_steps.sh STEP}"
trap 'setup_code=$?; printf "[ERROR] step=%s exit=%s file=%s line=%s command=%s\n" "$STEP" "$setup_code" "${BASH_SOURCE[0]}" "$LINENO" "$BASH_COMMAND" >&2; exit "$setup_code"' ERR
BASE_PACKAGES=(curl ca-certificates software-properties-common git build-essential cmake
    python3 python3-pip python3-serial python3-numpy python3-yaml python3-opencv)
ROS_PACKAGES=(ros-humble-ros-base ros-humble-cv-bridge ros-humble-rosidl-default-generators
    ros-humble-tf2-ros python3-colcon-common-extensions)
export DEBIAN_FRONTEND=noninteractive LC_ALL=C.UTF-8 LANG=C.UTF-8
# Match the working fused/integrated launchers and the full-run boot launcher.
# Jetson.GPIO 2.1.7 supports this fallback when its device-tree match fails.
export JETSON_MODEL_NAME="${JETSON_MODEL_NAME:-JETSON_ORIN_NANO}"
export SIDE_SENSOR_SOURCE="${SIDE_SENSOR_SOURCE:-mcu}"
if [[ -f "${FULL_RUN_SETUP_CONTEXT:-}" && "$STEP" != user_context ]]; then
    source "$FULL_RUN_SETUP_CONTEXT"
fi

already() {
    printf 'SKIP\n' > "$FULL_RUN_SETUP_LOG_DIR/$STEP.status"
    echo "[REUSE] $*"
    exit 77
}
blocked() {
    printf 'BLOCKED\n' > "$FULL_RUN_SETUP_LOG_DIR/$STEP.status"
    echo "[BLOCKED] $*" >&2
    exit 78
}
packages_present() {
    local package
    for package in "$@"; do
        [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null)" == 'install ok installed' ]] || return 1
    done
}
require_command() { command -v "$1" >/dev/null 2>&1 || blocked "Missing command: $1"; }
require_ros() { [[ -r /opt/ros/humble/setup.bash ]] || blocked 'ROS Humble setup.bash missing; see ros_packages.log.'; }
as_user() { runuser -u "$BUILD_USER" -- env LC_ALL=C.UTF-8 LANG=C.UTF-8 "$@"; }

platform() {
    local problems=0 os_id='' os_version='' architecture=''
    if (( EUID != 0 )); then echo '[ERROR] Run: sudo bash scripts/setup_full_run.sh'; problems=$((problems + 1)); fi
    if [[ "$(uname -s)" != Linux ]]; then echo '[ERROR] Installation requires Linux on Jetson.'; problems=$((problems + 1)); fi
    if [[ -r /etc/os-release ]]; then
        os_id="$(. /etc/os-release; printf '%s' "${ID:-}")"
        os_version="$(. /etc/os-release; printf '%s' "${VERSION_ID:-}")"
    fi
    if [[ "$os_id" != ubuntu || "$os_version" != 22.04 ]]; then
        echo "[ERROR] Requires Ubuntu 22.04 for ROS 2 Humble; found $os_id $os_version."; problems=$((problems + 1))
    fi
    if command -v dpkg >/dev/null 2>&1; then architecture="$(dpkg --print-architecture)"; fi
    if [[ "$architecture" != arm64 ]]; then echo "[ERROR] Requires ARM64; found $architecture."; problems=$((problems + 1)); fi
    if [[ ! -r /etc/nv_tegra_release ]] || ! grep -q '^# R36 ' /etc/nv_tegra_release; then
        echo '[ERROR] Requires JetPack 6 / L4T R36 on Jetson.'; problems=$((problems + 1))
    fi
    (( problems == 0 ))
}

user_context() {
    require_command getent
    require_command runuser
    BUILD_USER="${SUDO_USER:-$(stat -c '%U' "$PROJECT_DIR")}"
    BUILD_USER_HOME="$(getent passwd "$BUILD_USER" | cut -d: -f6)"
    [[ -n "$BUILD_USER_HOME" ]] || { echo "[ERROR] Cannot find home directory for $BUILD_USER."; return 1; }
    if [[ -z "${LIDAR_WS:-}" ]]; then
        if [[ -f "$BUILD_USER_HOME/ldlidar_ros2_ws/install/setup.bash" ]]; then
            LIDAR_WS="$BUILD_USER_HOME/ldlidar_ros2_ws"
        else
            LIDAR_WS="$PROJECT_DIR/.runtime/ldlidar_ros2_ws"
        fi
    fi
    [[ "$LIDAR_WS" == /* ]] || { echo '[ERROR] LIDAR_WS must be an absolute path.'; return 1; }
    printf 'BUILD_USER=%q\nBUILD_USER_HOME=%q\nLIDAR_WS=%q\n' "$BUILD_USER" "$BUILD_USER_HOME" "$LIDAR_WS" > "$FULL_RUN_SETUP_CONTEXT"
    echo "[INFO] Build user=$BUILD_USER LiDAR workspace=$LIDAR_WS"
}

system_metadata() {
    echo "UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ) step=$STEP"
    uname -a
    for metadata_file in /etc/os-release /etc/nv_tegra_release; do
        if [[ -r "$metadata_file" ]]; then echo "--- $metadata_file"; sed -n '1,40p' "$metadata_file"; fi
    done
    echo "Project=$PROJECT_DIR Python=$PYTHON_BIN"
    echo "JETSON_MODEL_NAME=$JETSON_MODEL_NAME"
    for metadata_file in /proc/device-tree/model /proc/device-tree/compatible /proc/device-tree/chosen/ids; do
        if [[ -r "$metadata_file" ]]; then
            echo "--- $metadata_file"
            tr '\0' '\n' < "$metadata_file"
        fi
    done
    if command -v dpkg-query >/dev/null 2>&1; then dpkg-query -W nvidia-jetpack python3-jetson-gpio 2>&1 || true; fi
    if command -v "$PYTHON_BIN" >/dev/null 2>&1; then
        "$PYTHON_BIN" -m pip show Jetson.GPIO 2>&1 || true
    fi
    for metadata_file in /sys/bus/gpio/devices/gpiochip*/label; do
        if [[ -r "$metadata_file" ]]; then echo "--- $metadata_file"; head -c 256 "$metadata_file"; echo; fi
    done
    if command -v "$PYTHON_BIN" >/dev/null 2>&1; then "$PYTHON_BIN" --version || true; fi
    if command -v docker >/dev/null 2>&1; then docker --version || true; fi
    if command -v dpkg-query >/dev/null 2>&1; then
        dpkg-query -W -f='${Package}\t${Version}\t${Status}\n' "${BASE_PACKAGES[@]}" "${ROS_PACKAGES[@]}" ros2-apt-source 2>&1 || true
    fi
}

source_snapshot() {
    mkdir -p "$FULL_RUN_SETUP_LOG_DIR/source"
    local source_file
    for source_file in scripts/setup_full_run.sh scripts/full_run_setup_steps.sh scripts/full_run_log_access.sh scripts/download_models.sh \
        scripts/full_run_diagnostics.py jetson/amr_core/full_run_diagnostics.py \
        jetson/amr_core/full_run_environment.py jetson/amr_core/full_run_summary.py \
        scripts/build_yolo_docker.sh docker/Dockerfile.oak-yolo config/full_run.json; do
        mkdir -p "$FULL_RUN_SETUP_LOG_DIR/source/$(dirname "$source_file")"
        cp "$PROJECT_DIR/$source_file" "$FULL_RUN_SETUP_LOG_DIR/source/$source_file"
    done
    if command -v sha256sum >/dev/null 2>&1; then
        (cd "$FULL_RUN_SETUP_LOG_DIR/source"; find . -type f -exec sha256sum '{}' +)
    fi
}

docker_daemon() { require_command docker; docker info; }
docker_runtime() {
    require_command "$PYTHON_BIN"
    docker info --format '{{json .Runtimes}}' | "$PYTHON_BIN" -c \
        'import json,sys; assert "nvidia" in json.load(sys.stdin), "NVIDIA runtime missing; configure JetPack nvidia-container-toolkit"'
}
apt_base_update() {
    packages_present "${BASE_PACKAGES[@]}" && already 'Base apt packages present; no index download needed.'
    apt-get update
}
base_packages() {
    packages_present "${BASE_PACKAGES[@]}" && already 'Base apt packages present.'
    # Never remove JetPack packages to resolve an inconsistent apt state.
    apt-get install -y --no-remove "${BASE_PACKAGES[@]}"
}
apt_universe() {
    packages_present "${ROS_PACKAGES[@]}" && already 'ROS packages present; no repository change needed.'
    require_command add-apt-repository
    add-apt-repository -y universe
}
ros_apt_source() {
    packages_present "${ROS_PACKAGES[@]}" && already 'ROS packages present; no bootstrap needed.'
    packages_present ros2-apt-source && already 'Official ros2-apt-source installed.'
    require_command curl
    require_command "$PYTHON_BIN"
    local version
    ROS_BOOTSTRAP_DIR="$(mktemp -d /tmp/amr-ros-bootstrap.XXXXXX)"
    # Only the exact downloaded file and now-empty, owned temporary directory.
    trap 'rm -f -- "$ROS_BOOTSTRAP_DIR/ros2-apt-source.deb"; rmdir -- "$ROS_BOOTSTRAP_DIR"' EXIT
    version="${ROS_APT_SOURCE_VERSION:-}"
    if [[ -z "$version" ]]; then
        version="$(curl --fail --silent --show-error --location --retry 3 \
            https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
            | "$PYTHON_BIN" -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
    fi
    [[ "$version" =~ ^[0-9][0-9A-Za-z.+~-]*$ ]] || { echo "[ERROR] Unexpected ros2-apt-source release: $version"; return 1; }
    echo "[INFO] Official ROS apt bootstrap version=$version"
    curl --fail --location --retry 3 --output "$ROS_BOOTSTRAP_DIR/ros2-apt-source.deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.jammy_all.deb"
    dpkg -i "$ROS_BOOTSTRAP_DIR/ros2-apt-source.deb"
}
apt_ros_update() {
    packages_present "${ROS_PACKAGES[@]}" && already 'ROS apt packages present; no index download needed.'
    apt-get update
}
ros_packages() {
    packages_present "${ROS_PACKAGES[@]}" && already 'ROS apt packages present.'
    apt-get install -y --no-remove "${ROS_PACKAGES[@]}"
}

ros_bridge_build() {
    require_ros
    if [[ -f "$PROJECT_DIR/jetson_ws/install/setup.bash" ]] && verify_ros_bridge \
        > "$FULL_RUN_SETUP_LOG_DIR/ros_bridge_reuse_probe.log" 2>&1; then
        already 'Existing ROS bridge imports and installed-source identity passed.'
    fi
    as_user /bin/bash -c '
        set -Ee -o pipefail
        source /opt/ros/humble/setup.bash
        cd "$1/jetson_ws"
        export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
        colcon build --symlink-install --executor sequential \
            --packages-up-to amr_interfaces amr_vision --cmake-args -DPython3_EXECUTABLE="$2"
    ' setup-build "$PROJECT_DIR" "$PYTHON_BIN"
}
lidar_source() {
    local lidar_source="$LIDAR_WS/src/ldlidar_stl_ros2"
    [[ ! -f "$lidar_source/package.xml" ]] || already "Existing LiDAR source: $lidar_source"
    [[ ! -e "$lidar_source" ]] || { echo "[ERROR] Incomplete existing LiDAR source at $lidar_source; not overwriting it."; return 1; }
    require_command git
    as_user mkdir -p "$LIDAR_WS/src"
    as_user git clone --depth 1 https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git "$lidar_source"
    [[ -f "$lidar_source/package.xml" ]]
}
lidar_build() {
    require_ros
    if [[ -f "$LIDAR_WS/install/setup.bash" ]] && verify_lidar \
        > "$FULL_RUN_SETUP_LOG_DIR/lidar_reuse_probe.log" 2>&1; then
        already 'Existing LiDAR driver executable passed.'
    fi
    as_user /bin/bash -c '
        set -Ee -o pipefail
        source /opt/ros/humble/setup.bash
        cd "$1"
        export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
        colcon build --symlink-install --executor sequential --packages-select ldlidar_stl_ros2
    ' setup-lidar "$LIDAR_WS"
}
model_download() {
    [[ ! -s "$PROJECT_DIR/models/yolo11n.pt" ]] || already 'YOLO11n model present.'
    as_user env YOLO_MODEL_URL="${YOLO_MODEL_URL:-https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt}" \
        /bin/bash "$PROJECT_DIR/scripts/download_models.sh"
}
yolo_image() {
    if docker image inspect socialguide-amr-oak-yolo:jp6; then already 'YOLO Docker image present.'; fi
    /bin/bash "$PROJECT_DIR/scripts/build_yolo_docker.sh"
}
jetson_gpio() {
    [[ "$SIDE_SENSOR_SOURCE" != mcu ]] || already 'Side sensors are on STM32; Jetson.GPIO is not a runtime dependency.'
    require_command "$PYTHON_BIN"
    echo "[GPIO] JETSON_MODEL_NAME=$JETSON_MODEL_NAME"
    if "$PYTHON_BIN" -c 'import Jetson.GPIO'; then already 'Jetson.GPIO import passed.'; fi
    if "$PYTHON_BIN" -c 'from importlib.metadata import version; print(version("Jetson.GPIO"))'; then
        echo '[ERROR] Jetson.GPIO is installed but import failed. Not reinstalling the same package or claiming success.' >&2
        echo '[ERROR] Check model/compatible/GPIO chip metadata in system_before.log and the import traceback.' >&2
        return 1
    fi
    # Do not replace JetPack PyTorch/CUDA or upgrade unrelated pip packages.
    "$PYTHON_BIN" -m pip install --no-deps Jetson.GPIO
    # A successful pip command is not proof that GPIO actually imports.
    "$PYTHON_BIN" -c 'import Jetson.GPIO; print("[OK] Jetson.GPIO import after installation")'
}

verify_python() {
    require_command "$PYTHON_BIN"
    "$PYTHON_BIN" - <<'PY'
import importlib
import os
import sys
import traceback
failed = []
print(f"[GPIO] JETSON_MODEL_NAME={os.environ.get('JETSON_MODEL_NAME')}", flush=True)
source = os.environ.get("SIDE_SENSOR_SOURCE", "mcu")
print(f"[SIDE SENSORS] source={source}", flush=True)
names = ["serial", "numpy", "yaml", "cv2"]
if source == "gpio":
    names.append("Jetson.GPIO")
for name in names:
    try:
        module = importlib.import_module(name)
        version = getattr(module, '__version__', getattr(module, 'VERSION', 'unknown'))
        print(f"[OK] {name} version={version} file={getattr(module, '__file__', 'unknown')}", flush=True)
        if name == "Jetson.GPIO":
            print(f"[GPIO] actual_board={getattr(module, 'JETSON_INFO', 'unknown')}", flush=True)
    except Exception:
        failed.append(name)
        print(f"[ERROR] Import failed: {name}", flush=True)
        traceback.print_exc()
print(f"[SUMMARY] Failed imports: {failed}", flush=True)
sys.exit(bool(failed))
PY
}
verify_ros_bridge() {
    require_ros
    [[ -r "$PROJECT_DIR/jetson_ws/install/setup.bash" ]] || blocked 'ROS bridge setup.bash missing; see ros_bridge_build.log.'
    as_user /bin/bash -c '
        set -Ee -o pipefail
        source /opt/ros/humble/setup.bash
        source "$1/jetson_ws/install/setup.bash"
        export PYTHONPATH="$1${PYTHONPATH:+:$PYTHONPATH}"
        "$2" - "$1" <<"PY"
import hashlib, importlib, sys, traceback
from pathlib import Path
failed = []
for name in ("rclpy", "serial", "sensor_msgs.msg:LaserScan", "std_msgs.msg:String",
             "amr_interfaces.msg:ObstacleInfo", "amr_vision.yolo_udp_bridge_node"):
    try:
        module_name, _, attribute = name.partition(":")
        module = importlib.import_module(module_name)
        if attribute:
            getattr(module, attribute)
        module_file = getattr(module, "__file__", None)
        print(f"[OK] {name} file={module_file}", flush=True)
        if module_name == "amr_vision.yolo_udp_bridge_node":
            actual = Path(module.__file__).resolve()
            expected = Path(sys.argv[1]) / "jetson_ws/src/amr_vision/amr_vision/yolo_udp_bridge_node.py"
            actual_hash = hashlib.sha256(actual.read_bytes()).hexdigest()
            expected_hash = hashlib.sha256(expected.read_bytes()).hexdigest()
            print(f"[BRIDGE SOURCE] actual={actual} sha256={actual_hash} expected={expected} sha256={expected_hash}", flush=True)
            if actual_hash != expected_hash:
                raise RuntimeError("Installed ROS bridge differs from this checkout; rebuild required")
    except Exception:
        failed.append(name)
        print(f"[ERROR] Import failed: {name}", flush=True)
        traceback.print_exc()
sys.exit(bool(failed))
PY
    ' setup-verify "$PROJECT_DIR" "$PYTHON_BIN"
}
verify_lidar() {
    require_ros
    [[ -r "$LIDAR_WS/install/setup.bash" ]] || blocked 'LiDAR setup.bash missing; see lidar_source/lidar_build logs.'
    as_user /bin/bash -c '
        set -Ee -o pipefail
        source /opt/ros/humble/setup.bash
        source "$1/install/setup.bash"
        ros2 pkg executables ldlidar_stl_ros2 | grep "ldlidar_stl_ros2_node"
    ' setup-verify-lidar "$LIDAR_WS"
}
verify_model() {
    [[ -s "$PROJECT_DIR/models/yolo11n.pt" ]] || { echo '[ERROR] Missing/empty models/yolo11n.pt'; return 1; }
    ls -l "$PROJECT_DIR/models/yolo11n.pt"
}
verify_yolo_image() { require_command docker; docker image inspect socialguide-amr-oak-yolo:jp6; }
configuration() {
    require_command "$PYTHON_BIN"
    "$PYTHON_BIN" "$PROJECT_DIR/scripts/full_run_controller.py" --config "$PROJECT_DIR/config/full_run.json" \
        --log-dir "$FULL_RUN_SETUP_LOG_DIR/config-check" --side-sensor-source "$SIDE_SENSOR_SOURCE" --check-config
}

case "$STEP" in
    system_before|system_after) system_metadata ;;
    platform|user_context|source_snapshot|docker_daemon|docker_runtime|apt_base_update|base_packages|apt_universe|\
    ros_apt_source|apt_ros_update|ros_packages|ros_bridge_build|lidar_source|lidar_build|model_download|\
    yolo_image|jetson_gpio|verify_python|verify_ros_bridge|verify_lidar|verify_model|verify_yolo_image|configuration) "$STEP" ;;
    *) echo "Unknown internal setup step: $STEP" >&2; exit 2 ;;
esac
