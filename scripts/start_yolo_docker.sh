#!/usr/bin/env bash
set -Eeo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$PROJECT_DIR/models"
docker run --rm \
  --name socialguide-amr-yolo \
  --privileged \
  --runtime=nvidia \
  --network=host \
  --ipc=host \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$PROJECT_DIR/models:/models" \
  socialguide-amr-oak-yolo:jp6 --model /models/yolo11n.pt "$@"
