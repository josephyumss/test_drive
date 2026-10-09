#!/usr/bin/env bash
set -Eeo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL_DIR="$PROJECT_DIR/models"
MODEL_PATH="$MODEL_DIR/yolo11n.pt"
MODEL_URL="${YOLO_MODEL_URL:-https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt}"

mkdir -p "$MODEL_DIR"

if [[ -s "$MODEL_PATH" ]]; then
  echo "[ OK ] YOLO 모델이 이미 준비됨: $MODEL_PATH"
  exit 0
fi

temporary_path="$(mktemp "$MODEL_DIR/.yolo11n.pt.XXXXXX")"
cleanup() {
  rm -f "$temporary_path"
}
trap cleanup EXIT

echo "[DOWN] YOLO11n 모델 다운로드"
if command -v curl >/dev/null 2>&1; then
  curl --fail --location --retry 3 --output "$temporary_path" "$MODEL_URL"
elif command -v wget >/dev/null 2>&1; then
  wget --tries=3 --output-document="$temporary_path" "$MODEL_URL"
else
  echo "[FAIL] curl 또는 wget이 필요합니다." >&2
  exit 1
fi

if [[ ! -s "$temporary_path" ]]; then
  echo "[FAIL] 다운로드한 모델 파일이 비어 있습니다." >&2
  exit 1
fi

mv "$temporary_path" "$MODEL_PATH"
trap - EXIT
echo "[ OK ] YOLO 모델 준비 완료: $MODEL_PATH"
