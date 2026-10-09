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
service_stage=initialize_logging
generated_unit="$LOG_DIR/amr-full-run.service"
service_step() {
    local started_utc result status
    service_stage="$1"
    shift
    started_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "[SERVICE INSTALL STEP] stage=$service_stage UTC=$started_utc command=$*"
    if "$@" > "$LOG_DIR/$service_stage.log" 2>&1; then result=0; status=OK;
    else result=$?; status=FAILED; fi
    sed -n '1,200p' "$LOG_DIR/$service_stage.log"
    printf '%s\t%s\t%s\t%s\t%s\n' "$service_stage" "$status" "$result" "$started_utc" \
        "$(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$LOG_DIR/stages.tsv"
    echo "[SERVICE INSTALL STEP RESULT] stage=$service_stage exit=$result full_output=$LOG_DIR/$service_stage.log"
    return "$result"
}
finish_service_install() {
    local result=$? logger_result interpreter=/usr/bin/python3
    trap - EXIT ERR INT TERM
    set +e
    echo "[SERVICE INSTALL RESULT] exit=$result stage=$service_stage UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ) logs=$LOG_DIR"
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
trap 'result=$?; echo "[SERVICE INSTALL FAIL] exit=$result stage=$service_stage line=$LINENO command=$BASH_COMMAND"; exit "$result"' ERR
trap 'echo "[SERVICE INSTALL INTERRUPTED] SIGINT"; exit 130' INT
trap 'echo "[SERVICE INSTALL INTERRUPTED] SIGTERM"; exit 143' TERM
source "$PROJECT_DIR/scripts/full_run_log_access.sh"
full_run_prepare_log_access "$PROJECT_DIR" "$LOG_ROOT" "$LOG_DIR"
echo "[LOG] $LOG_DIR"
printf 'step\tstatus\texit_code\tstarted_utc\tfinished_utc\n' > "$LOG_DIR/stages.tsv"
service_stage=check_privileges_and_path
(( EUID == 0 )) || { echo 'Run: sudo bash scripts/install_full_run_service.sh'; exit 1; }
[[ "$PROJECT_DIR" != *$'\n'* && "$PROJECT_DIR" != *$'\r'* && "$PROJECT_DIR" != *'"'* \
    && "$PROJECT_DIR" != *'%'* && "$PROJECT_DIR" != *'\'* && "$PROJECT_DIR" != *[[:space:]] ]] || {
    echo 'Unsupported character or trailing whitespace in installation path'; exit 1;
}
service_stage=check_old_device_owner
echo "[SERVICE INSTALL STEP] stage=$service_stage"
# Missing old units are normal. Retain their query output without presenting
# 'No such file' as the installation failure. Never stop/disable them implicitly.
if { systemctl is-active --quiet amr-core.service || systemctl is-enabled --quiet amr-core.service; } \
    > "$LOG_DIR/old-service-check.log" 2>&1; then
    echo 'amr-core.service is active or enabled and could own the devices. Stop/disable it before installing the full-run service.'
    exit 1
fi
service_step systemd_version systemd-analyze --version
service_stage=render_unit
echo "[SERVICE INSTALL STEP] stage=$service_stage project=$PROJECT_DIR"
cp -- "$PROJECT_DIR/deploy/amr-full-run.service" "$LOG_DIR/unit-template.service"
replacement="$(printf '%s' "$PROJECT_DIR" | sed 's/[\\&|]/\\&/g')"
sed "s|@PROJECT_DIR@|$replacement|g" "$PROJECT_DIR/deploy/amr-full-run.service" > "$generated_unit"
# Keep the EXACT generated file in both successful logs and failure archives.
# WorkingDirectory is a raw path setting (unlike ExecStart's argument list):
# surrounding quotes are treated as part of the path by Jetson systemd 249.
nl -ba "$generated_unit"
sha256sum "$generated_unit" "$LOG_DIR/unit-template.service" > "$LOG_DIR/unit-source.sha256"
service_step verify_unit systemd-analyze verify "$generated_unit"
service_step install_unit install -m 644 "$generated_unit" /etc/systemd/system/amr-full-run.service
service_step daemon_reload systemctl daemon-reload
service_step enable_unit systemctl enable amr-full-run.service
echo 'Boot auto-start enabled. To start now: sudo systemctl start amr-full-run.service'
echo 'Manual foreground run: sudo bash scripts/start_full_run.sh'
