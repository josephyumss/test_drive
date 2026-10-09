#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="$PROJECT_DIR/.run/full_run.pid"
[[ -f "$PID_FILE" ]] || { echo '[STOP] No full-run PID file; already stopped.'; exit 0; }
read -r pid start_ticks < "$PID_FILE"
[[ "$pid" =~ ^[0-9]+$ && "$start_ticks" =~ ^[0-9]+$ ]] || { echo 'Invalid PID record'; exit 1; }
[[ -f "/proc/$pid/stat" ]] || { echo '[STOP] Recorded process no longer exists.'; exit 0; }
actual_ticks="$(awk '{print $22}' "/proc/$pid/stat")"
[[ "$actual_ticks" == "$start_ticks" ]] || { echo 'PID was reused; refusing to signal it.'; exit 1; }
matched=0
while IFS= read -r -d '' argument; do
    if [[ "$argument" == /* ]]; then
        candidate="$argument"
    else
        candidate="$(readlink "/proc/$pid/cwd")/$argument"
    fi
    [[ "$(readlink -f -- "$candidate" 2>/dev/null || true)" == "$PROJECT_DIR/scripts/start_full_run.sh" ]] && matched=1
done < "/proc/$pid/cmdline"
(( matched )) || { echo 'Recorded PID does not belong to this launcher; refusing to signal it.'; exit 1; }
if command -v systemctl >/dev/null && [[ "$(systemctl show amr-full-run.service -p MainPID --value 2>/dev/null || true)" == "$pid" ]]; then
    systemctl stop amr-full-run.service
else
    kill -TERM "$pid"
fi
for _ in {1..100}; do
    kill -0 "$pid" 2>/dev/null || { echo '[STOP] Full run stopped. Restart with sudo bash scripts/start_full_run.sh'; exit 0; }
    sleep 0.1
done
echo '[WARN] Stop requested; launcher cleanup has not finished. Inspect latest/launcher.log.'
exit 1
