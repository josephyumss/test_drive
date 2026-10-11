#!/usr/bin/env bash
# ROS 2's generated setup files intentionally probe variables that may not
# exist yet, so nounset (-u) cannot remain enabled while they are sourced.
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -r "$PROJECT_DIR/config/full_run.env" ]]; then
    set -a
    source "$PROJECT_DIR/config/full_run.env"
    set +a
fi
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
JETSON_SETUP="${JETSON_SETUP:-$PROJECT_DIR/jetson_ws/install/setup.bash}"
FULL_RUN_CONFIG="${FULL_RUN_CONFIG:-$PROJECT_DIR/config/full_run.json}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

source "$ROS_SETUP"
[[ ! -r "$JETSON_SETUP" ]] || source "$JETSON_SETUP"

exec "$PYTHON_BIN" -u "$PROJECT_DIR/scripts/visualize_front_lidar.py" \
    --config "$FULL_RUN_CONFIG" "$@"
