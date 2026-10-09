#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
umask 027
LOG_ROOT="$PROJECT_DIR/logs/full_run_setup"
mkdir -p "$LOG_ROOT"
LOG_DIR="$(mktemp -d "$LOG_ROOT/$(date -u +%Y%m%dT%H%M%SZ)-service-XXXXXX")"
exec 3>&1
exec > >(tee -a "$LOG_DIR/launcher.log") 2>&1
service_logger_pid=$!
temporary_unit=''
temporary_dir=''
finish_service_install() {
    local result=$? logger_result interpreter=/usr/bin/python3
    trap - EXIT ERR INT TERM
    set +e
    if [[ -n "$temporary_unit" ]]; then rm -f -- "$temporary_unit"; fi
    if [[ -n "$temporary_dir" ]]; then rmdir -- "$temporary_dir"; fi
    echo "[SERVICE INSTALL RESULT] exit=$result UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ) logs=$LOG_DIR"
    exec 1>&3 2>&3
    wait "$service_logger_pid"
    logger_result=$?
    (( logger_result == 0 )) || result=1
    if (( result != 0 )); then
        command -v "$interpreter" >/dev/null 2>&1 || interpreter="$(command -v python3 || true)"
        if [[ -n "$interpreter" ]] && "$interpreter" "$PROJECT_DIR/scripts/full_run_diagnostics.py" "$LOG_DIR" \
            --auto-bundle --capture-environment --reason "service_install_exit_$result" > "$LOG_DIR/bundle.log" 2>&1; then
            echo "[BUNDLE] Saved: $LOG_DIR-debug.tar.gz"
        else
            echo "[BUNDLE FAIL] Original service-install output preserved in $LOG_DIR/launcher.log"
        fi
    fi
    exec 3>&-
    exit "$result"
}
trap finish_service_install EXIT
trap 'result=$?; echo "[SERVICE INSTALL FAIL] exit=$result line=$LINENO command=$BASH_COMMAND"; exit "$result"' ERR
trap 'echo "[SERVICE INSTALL INTERRUPTED] SIGINT"; exit 130' INT
trap 'echo "[SERVICE INSTALL INTERRUPTED] SIGTERM"; exit 143' TERM
source "$PROJECT_DIR/scripts/full_run_log_access.sh"
full_run_prepare_log_access "$PROJECT_DIR" "$LOG_ROOT" "$LOG_DIR"
echo "[LOG] $LOG_DIR"
(( EUID == 0 )) || { echo 'Run: sudo bash scripts/install_full_run_service.sh'; exit 1; }
[[ "$PROJECT_DIR" != *$'\n'* && "$PROJECT_DIR" != *'"'* && "$PROJECT_DIR" != *'%'* ]] || { echo 'Unsupported character in installation path'; exit 1; }
if systemctl is-active --quiet amr-core.service || systemctl is-enabled --quiet amr-core.service; then
    echo 'amr-core.service is active and owns the devices. Stop/disable it before installing the full-run service.'
    exit 1
fi
replacement="$(printf '%s' "$PROJECT_DIR" | sed 's/[\\&|]/\\&/g')"
temporary_dir="$(mktemp -d)"
temporary_unit="$temporary_dir/amr-full-run.service"
sed "s|@PROJECT_DIR@|$replacement|g" "$PROJECT_DIR/deploy/amr-full-run.service" > "$temporary_unit"
systemd-analyze verify "$temporary_unit" || { echo 'Generated systemd unit did not validate'; exit 1; }
install -m 644 "$temporary_unit" /etc/systemd/system/amr-full-run.service
systemctl daemon-reload
systemctl enable amr-full-run.service
echo 'Boot auto-start enabled. To start now: sudo systemctl start amr-full-run.service'
echo 'Manual foreground run: sudo bash scripts/start_full_run.sh'
