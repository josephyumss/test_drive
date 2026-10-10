#!/usr/bin/env bash
set -Eeo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL_PATH="$PROJECT_DIR/models/yolo11n.pt"
YOLO_DEVICE="${YOLO_DEVICE:-cpu}"
YOLO_IMGSZ="${YOLO_IMGSZ:-320}"
YOLO_HALF="${YOLO_HALF:-0}"

mkdir -p "$PROJECT_DIR/models"

if [[ ! -s "$MODEL_PATH" ]]; then
  echo "[FAIL] YOLO 모델 파일이 없거나 비어 있습니다: $MODEL_PATH" >&2
  echo "       먼저 bash ./scripts/download_models.sh 를 실행하세요." >&2
  exit 1
fi

extra_args=()
runtime_labels=()
if [[ -n "${YOLO_RUNTIME_OWNER:-}" ]]; then
  runtime_labels+=(--label "amr.full_run.owner=$YOLO_RUNTIME_OWNER")
  if [[ -n "${FULL_RUN_PROJECT_DIR:-}" && -n "${FULL_RUN_SUPERVISOR_PID:-}" \
      && -n "${FULL_RUN_SUPERVISOR_START_TICKS:-}" ]]; then
    runtime_labels+=(--label "amr.full_run.project_dir=$FULL_RUN_PROJECT_DIR")
    runtime_labels+=(--label "amr.full_run.supervisor_pid=$FULL_RUN_SUPERVISOR_PID")
    runtime_labels+=(--label "amr.full_run.supervisor_start_ticks=$FULL_RUN_SUPERVISOR_START_TICKS")
  fi
fi
if [[ "$YOLO_HALF" == "1" ]]; then
  extra_args+=(--half)
fi

docker run --rm \
  --name socialguide-amr-yolo \
  "${runtime_labels[@]}" \
  --privileged \
  --runtime=nvidia \
  --network=host \
  --ipc=host \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$PROJECT_DIR/models:/models" \
  -v "$PROJECT_DIR/docker/oak_yolo_udp.py:/opt/amr/oak_yolo_udp.py:ro" \
  socialguide-amr-oak-yolo:jp6 \
  --model /models/yolo11n.pt \
  --device "$YOLO_DEVICE" \
  --imgsz "$YOLO_IMGSZ" \
  "${extra_args[@]}" \
  "$@"
