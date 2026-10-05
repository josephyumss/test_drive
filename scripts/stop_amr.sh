#!/usr/bin/env bash
set -o pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="$PROJECT_DIR/.run/amr.pids"
CONTROLLER_FILE="$PROJECT_DIR/.run/amr.controller.pid"
stopped=0

# Prefer asking start_amr.sh to run its own cleanup trap.
if [[ -f "$CONTROLLER_FILE" ]]; then
  read -r controller_pid < "$CONTROLLER_FILE" || true
  if [[ "${controller_pid:-}" =~ ^[0-9]+$ ]] && kill -0 "$controller_pid" 2>/dev/null; then
    echo "[STOP] AMR controller ($controller_pid)"
    kill -TERM "$controller_pid" 2>/dev/null || true
    stopped=1
    for _ in 1 2 3 4 5; do
      kill -0 "$controller_pid" 2>/dev/null || break
      sleep 1
    done
  fi
fi

# Recovery path for an interrupted controller or an older PID file.
if [[ -f "$PID_FILE" ]]; then
  while read -r pid name; do
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      echo "[STOP] 남아 있는 $name ($pid)"
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
      stopped=1
    fi
  done < "$PID_FILE"
fi

docker stop -t 2 socialguide-amr-yolo >/dev/null 2>&1 || true
rm -f "$PID_FILE" "$CONTROLLER_FILE"

# Clean up a dashboard left by the older launcher, but only when its working
# directory belongs to this project.
while read -r dashboard_pid; do
  [[ "$dashboard_pid" =~ ^[0-9]+$ ]] || continue
  dashboard_cwd="$(readlink -f "/proc/$dashboard_pid/cwd" 2>/dev/null || true)"
  if [[ "$dashboard_cwd" == "$PROJECT_DIR" || "$dashboard_cwd" == "$PROJECT_DIR/"* ]]; then
    echo "[STOP] 이전 버전에서 남은 dashboard ($dashboard_pid)"
    kill -TERM "$dashboard_pid" 2>/dev/null || true
    stopped=1
  fi
done < <(pgrep -f 'python3 .*\-m monitoring\.server' 2>/dev/null || true)

if (( stopped == 0 )); then
  echo "실행 중인 AMR 프로세스가 없습니다."
else
  echo "AMR 종료 요청을 완료했습니다."
fi

# Never kill an unknown port owner automatically.
sleep 1
if command -v ss >/dev/null 2>&1; then
  port_owner="$(ss -ltnp 2>/dev/null | grep ':8080 ' || true)"
  if [[ -n "$port_owner" ]]; then
    echo "[WARN] 8080 포트를 사용하는 프로세스가 아직 있습니다:"
    echo "$port_owner"
    echo "       위 PID가 monitoring.server인지 확인한 뒤 종료하세요."
  else
    echo "[ OK ] 통합 관제 포트 8080 정리 완료"
  fi
fi
