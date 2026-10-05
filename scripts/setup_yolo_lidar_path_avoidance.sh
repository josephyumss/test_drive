#!/usr/bin/env bash
# Prepare the YOLO + LiDAR run on Ubuntu 22.04 / JetPack 6. Never sends motor commands.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    cat <<'EOF'
Usage: setup_yolo_lidar_path_avoidance.sh

Installs ROS 2 Humble and build dependencies, builds the YOLO bridge and STL-27L
driver, downloads YOLO11n, and builds the existing JetPack 6 YOLO Docker image.
Run in a local terminal; sudo may request your password. This does not drive.

Optional environment:
  LIDAR_WS                 Existing or new LiDAR workspace path
  ROS_APT_SOURCE_VERSION   Pin the official ros2-apt-source bootstrap release
  YOLO_MODEL_URL           Override the model download URL
EOF
}

case "${1:-}" in
    -h|--help) usage; exit 0 ;;
    "") ;;
    *) usage >&2; exit 2 ;;
esac
(( $# == 0 )) || { usage >&2; exit 2; }

if (( EUID != 0 )); then
    # Explicitly pass only this script's configuration through privilege elevation.
    exec sudo env \
        LIDAR_WS="${LIDAR_WS:-}" \
        ROS_APT_SOURCE_VERSION="${ROS_APT_SOURCE_VERSION:-}" \
        YOLO_MODEL_URL="${YOLO_MODEL_URL:-https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt}" \
        /bin/bash "$PROJECT_DIR/scripts/setup_yolo_lidar_path_avoidance.sh"
fi

fail() { echo "[FAIL] $*" >&2; exit 1; }
trap 'echo "[FAIL] Setup failed at line $LINENO. Resolve the error above and rerun setup." >&2' ERR

source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] \
    || fail "This setup requires Ubuntu 22.04 (ROS 2 Humble / JetPack 6)."
[[ "$(dpkg --print-architecture)" == arm64 ]] \
    || fail "The supplied YOLO image requires an ARM64 Jetson with JetPack 6."
[[ -f /etc/nv_tegra_release ]] \
    || fail "Jetson L4T was not detected; the supplied YOLO image requires JetPack 6."
grep -q '^# R36 ' /etc/nv_tegra_release \
    || fail "This YOLO image requires JetPack 6 (L4T R36)."

# Keep source downloads and build outputs owned by the invoking user.
BUILD_USER="${SUDO_USER:-$(stat -c '%U' "$PROJECT_DIR")}"
BUILD_USER_HOME="$(getent passwd "$BUILD_USER" | cut -d: -f6)"
[[ -n "$BUILD_USER_HOME" ]] || fail "Cannot find home directory for $BUILD_USER."
if [[ -z "${LIDAR_WS:-}" ]]; then
    if [[ -f "$BUILD_USER_HOME/ldlidar_ros2_ws/install/setup.bash" ]]; then
        LIDAR_WS="$BUILD_USER_HOME/ldlidar_ros2_ws"
    else
        LIDAR_WS="$PROJECT_DIR/.runtime/ldlidar_ros2_ws"
    fi
fi
[[ "$LIDAR_WS" == /* ]] || fail "LIDAR_WS must be an absolute path."

as_user() {
    runuser -u "$BUILD_USER" -- env LC_ALL=C.UTF-8 LANG=C.UTF-8 "$@"
}

command -v docker >/dev/null || fail "Install Docker with the JetPack NVIDIA runtime first."
docker info >/dev/null || fail "Docker is not running. Start the Docker service and retry."
docker info --format '{{json .Runtimes}}' | /usr/bin/python3 -c \
    'import json, sys; sys.exit(0 if "nvidia" in json.load(sys.stdin) else 1)' \
    || fail "Docker has no NVIDIA runtime. Configure JetPack's nvidia-container-toolkit first."

echo "[SETUP] Installing ROS 2 Humble prerequisites"
export DEBIAN_FRONTEND=noninteractive
export LC_ALL=C.UTF-8 LANG=C.UTF-8
apt-get update
# --no-remove prevents an old or inconsistent apt state from removing JetPack packages.
apt-get install -y --no-remove curl ca-certificates software-properties-common \
    git build-essential cmake python3 python3-pip python3-serial python3-numpy \
    python3-yaml python3-opencv
add-apt-repository -y universe

# Official ROS apt bootstrap:
# https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/_Apt-Repositories.rst
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
    BOOTSTRAP_DIR="$(mktemp -d /tmp/amr-ros-bootstrap.XXXXXX)"
    trap 'rm -rf -- "$BOOTSTRAP_DIR"' EXIT
    if [[ -z "${ROS_APT_SOURCE_VERSION:-}" ]]; then
        ROS_APT_SOURCE_VERSION="$(
            curl --fail --silent --show-error --location --retry 3 \
                https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
                | /usr/bin/python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])'
        )"
    fi
    [[ "$ROS_APT_SOURCE_VERSION" =~ ^[0-9][0-9A-Za-z.+~-]*$ ]] \
        || fail "Unexpected ros2-apt-source release: $ROS_APT_SOURCE_VERSION"
    curl --fail --location --retry 3 \
        --output "$BOOTSTRAP_DIR/ros2-apt-source.deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb"
    dpkg -i "$BOOTSTRAP_DIR/ros2-apt-source.deb"
    rm -rf -- "$BOOTSTRAP_DIR"
    trap - EXIT
fi

apt-get update
apt-get install -y --no-remove ros-humble-ros-base ros-humble-cv-bridge \
    ros-humble-rosidl-default-generators ros-humble-tf2-ros \
    python3-colcon-common-extensions

echo "[BUILD] ROS messages and YOLO UDP bridge (user: $BUILD_USER)"
as_user /bin/bash -c '
    set -Ee -o pipefail
    source /opt/ros/humble/setup.bash
    cd "$1/jetson_ws"
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
    colcon build --symlink-install --executor sequential \
        --packages-up-to amr_interfaces amr_vision \
        --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
' setup-build "$PROJECT_DIR"

echo "[BUILD] STL-27L driver: $LIDAR_WS"
as_user mkdir -p "$LIDAR_WS/src"
LIDAR_SOURCE="$LIDAR_WS/src/ldlidar_stl_ros2"
if [[ ! -d "$LIDAR_SOURCE" ]]; then
    as_user git clone --depth 1 \
        https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git "$LIDAR_SOURCE"
fi
[[ -f "$LIDAR_SOURCE/package.xml" ]] || fail "Not a LiDAR source checkout: $LIDAR_SOURCE"
as_user /bin/bash -c '
    set -Ee -o pipefail
    source /opt/ros/humble/setup.bash
    cd "$1"
    export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
    colcon build --symlink-install --executor sequential \
        --packages-select ldlidar_stl_ros2
' setup-lidar "$LIDAR_WS"

echo "[SETUP] YOLO model and JetPack 6 Docker image"
as_user env \
    YOLO_MODEL_URL="${YOLO_MODEL_URL:-https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt}" \
    /bin/bash "$PROJECT_DIR/scripts/download_models.sh"
/bin/bash "$PROJECT_DIR/scripts/build_yolo_docker.sh"

echo "[CHECK] ROS imports and driver executable"
as_user /bin/bash -c '
    set -Ee -o pipefail
    source /opt/ros/humble/setup.bash
    source "$1/jetson_ws/install/setup.bash"
    source "$2/install/setup.bash"
    export PYTHONPATH="$1${PYTHONPATH:+:$PYTHONPATH}"
    /usr/bin/python3 -c "import rclpy, serial; from amr_interfaces.msg import ObstacleInfo; import amr_vision.yolo_udp_bridge_node"
    ros2 pkg executables ldlidar_stl_ros2 | grep -q "ldlidar_stl_ros2_node"
' setup-check "$PROJECT_DIR" "$LIDAR_WS"

echo "[READY] Setup complete. No motor commands were sent."
printf 'Run: LIDAR_WS=%q %q\n' "$LIDAR_WS" "$PROJECT_DIR/scripts/start_yolo_lidar_path_avoidance.sh"
