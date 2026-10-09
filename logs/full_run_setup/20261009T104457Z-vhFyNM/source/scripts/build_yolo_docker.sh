#!/usr/bin/env bash
set -Eeo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker build -f "$PROJECT_DIR/docker/Dockerfile.oak-yolo" -t socialguide-amr-oak-yolo:jp6 "$PROJECT_DIR"
