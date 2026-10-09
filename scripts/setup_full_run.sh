#!/usr/bin/env bash
# Dependency preparation only. Each independent step runs even after failures.
# Does not command motors, flash MCU, start a service, or change CUDA/PyTorch.
set -Eeuo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-}" in
    -h|--help)
        echo 'Usage: sudo bash scripts/setup_full_run.sh'
        echo 'Continue independent installation steps, save all results, exit nonzero if incomplete.'
        exit 0 ;;
    "") ;;
    *) echo 'Usage: sudo bash scripts/setup_full_run.sh' >&2; exit 2 ;;
esac
(( $# == 0 )) || { echo 'Too many arguments' >&2; exit 2; }

LOG_ROOT="$PROJECT_DIR/logs/full_run_setup"
mkdir -p "$LOG_ROOT"
LOG_DIR="$(mktemp -d "$LOG_ROOT/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
export PROJECT_DIR PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
export FULL_RUN_SETUP_LOG_DIR="$LOG_DIR"
export FULL_RUN_SETUP_CONTEXT="$LOG_DIR/context.env"
export PYTHONUNBUFFERED=1
STEPS_SCRIPT="$PROJECT_DIR/scripts/full_run_setup_steps.sh"
printf 'step\tstatus\texit_code\tstarted_utc\tfinished_utc\tlog_file\tnote\n' > "$LOG_DIR/stages.tsv"

passed=0 skipped=0 failed=0 blocked=0 completed=0
current_step=initialization
declare -A step_status=()
# Keep a direct terminal FD; wait for tee before archiving so the last error and
# summary are really in launcher.log, not still buffered in a pipe.
exec 3>&1
exec > >(tee -a "$LOG_DIR/launcher.log") 2>&1
LOGGER_PID=$!

record_result() {
    local name="$1" status="$2" code="$3" started="$4" finished="$5" note="${6:-}"
    step_status["$name"]="$status"
    case "$status" in
        OK) passed=$((passed + 1)) ;;
        SKIP) skipped=$((skipped + 1)) ;;
        FAILED) failed=$((failed + 1)) ;;
        BLOCKED) blocked=$((blocked + 1)) ;;
    esac
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$name" "$status" "$code" \
        "$started" "$finished" "$name.log" "$note" >> "$LOG_DIR/stages.tsv"
    echo "[$status] $name (exit=$code) ${note}"
}

run_step() {
    local name="$1" dependencies="${2:-}" dependency started finished code status note=''
    local -a codes
    current_step="$name"
    started="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    for dependency in $dependencies; do
        if [[ "${step_status[$dependency]:-BLOCKED}" != OK && \
              "${step_status[$dependency]:-BLOCKED}" != SKIP ]]; then
            note="Prerequisite $dependency is ${step_status[$dependency]:-missing}; no unsafe/dependent operation attempted."
            echo "$note" > "$LOG_DIR/$name.log"
            record_result "$name" BLOCKED 78 "$started" "$started" "$note"
            return 0
        fi
    done
    echo "[STEP] $started $name -> $LOG_DIR/$name.log"
    # A fresh Bash process retains errexit INSIDE the step, unlike calling a
    # shell function as an if-condition (which disables errexit in that function).
    if /bin/bash "$STEPS_SCRIPT" "$name" 2>&1 | tee "$LOG_DIR/$name.log"; then
        codes=("${PIPESTATUS[@]}")
    else
        codes=("${PIPESTATUS[@]}")
    fi
    code="${codes[0]}"
    finished="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    if (( codes[1] != 0 )); then
        record_result "$name" FAILED "${codes[1]}" "$started" "$finished" 'Log write failed; cannot safely collect further diagnostics.'
        exit 1
    fi
    case "$code" in
        0) status=OK ;;
        77|78)
            # curl also uses 77/78 for genuine errors. Only an explicit worker
            # marker may classify these codes as reuse or prerequisite failure.
            status=FAILED
            note='Command failed; continuing other steps.'
            if [[ -f "$LOG_DIR/$name.status" ]]; then
                if (( code == 77 )) && [[ "$(< "$LOG_DIR/$name.status")" == SKIP ]]; then
                    status=SKIP; note='Already installed/usable; existing installation reused.'
                elif (( code == 78 )) && [[ "$(< "$LOG_DIR/$name.status")" == BLOCKED ]]; then
                    status=BLOCKED; note='Required resource missing; see step log for the reason.'
                fi
            fi ;;
        *) status=FAILED; note='See step log for command, line and error output; continuing other steps.' ;;
    esac
    record_result "$name" "$status" "$code" "$started" "$finished" "$note"
}

archive_failure() {
    local interpreter="$PYTHON_BIN" temporary archive output code
    archive="$LOG_DIR-debug.tar.gz"
    if ! command -v "$interpreter" >/dev/null 2>&1; then
        interpreter="$(command -v python3 || true)"
    fi
    if [[ -n "$interpreter" ]] && "$interpreter" "$PROJECT_DIR/scripts/full_run_diagnostics.py" "$LOG_DIR" \
        --auto-bundle --reason "installation_failed_${failed}_blocked_${blocked}" > "$LOG_DIR/bundle.log" 2>&1; then
        echo "[BUNDLE] Saved: $archive" | tee -a "$LOG_DIR/launcher.log"
        return 0
    fi
    # A broken/missing Python installation must not prevent collecting its own
    # diagnosis. tar captures only settled files; its stderr is saved afterwards.
    echo '[BUNDLE] Python bundler unavailable/failed; trying tar fallback.' | tee -a "$LOG_DIR/launcher.log"
    temporary="$(mktemp "$LOG_ROOT/.setup-debug-XXXXXX.part")" || return 1
    if output="$(tar -czf "$temporary" -C "$LOG_ROOT" "$(basename "$LOG_DIR")" 2>&1)"; then
        code=0
    else
        code=$?
    fi
    printf '%s\n' "$output" >> "$LOG_DIR/bundle.log"
    if (( code == 0 )) && mv -- "$temporary" "$archive"; then
        echo "[BUNDLE] Saved: $archive (tar fallback)" | tee -a "$LOG_DIR/launcher.log"
    else
        rm -f -- "$temporary"
        echo "[BUNDLE FAIL] Original logs kept at $LOG_DIR; see bundle.log." | tee -a "$LOG_DIR/launcher.log"
    fi
}

finish() {
    local result=$? logger_result
    local -a summary_codes
    trap - EXIT ERR INT TERM
    set +e
    if (( completed == 0 )); then
        echo "[ABORTED] Installation interrupted/unexpected failure during $current_step; not all steps ran."
        printf '%s\tFAILED\t%s\t%s\t%s\tlauncher.log\tInstallation aborted before all steps completed.\n' \
            "${current_step}_aborted" "$result" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$LOG_DIR/stages.tsv"
        failed=$((failed + 1))
    fi
    (( failed == 0 && blocked == 0 && completed == 1 )) || { (( result != 0 )) || result=1; }
    {
        echo "Installation finished: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "completed=$completed exit=$result OK=$passed SKIP=$skipped FAILED=$failed BLOCKED=$blocked"
        echo "Logs: $LOG_DIR"
        if (( result != 0 )); then
            echo 'Installation is NOT complete. Repair the recorded problems and rerun the same setup command.'
            echo 'Failed or blocked steps (all command output is in their .log files):'
            awk -F '\t' 'NR > 1 && ($2 == "FAILED" || $2 == "BLOCKED") {print "  " $1 " / " $2 " / exit=" $3 " / " $6 " / " $7}' "$LOG_DIR/stages.tsv"
        else
            echo 'Dependencies prepared. No motors or services were started.'
            echo 'Flash updated STM32 main.c + full_run_control.h and connect USART3 status before preflight.'
            echo 'Next: sudo bash scripts/start_full_run.sh --preflight-only'
        fi
    } | tee "$LOG_DIR/summary.txt"
    summary_codes=("${PIPESTATUS[@]}")
    if (( summary_codes[0] != 0 || summary_codes[1] != 0 )); then
        echo '[FAIL] Final summary could not be written completely; setup is incomplete.'
        result=1
    fi
    if (( result != 0 )); then echo "[BUNDLE] Automatic archive after all attempted steps: $LOG_DIR-debug.tar.gz"; fi
    exec 1>&3 2>&3
    wait "$LOGGER_PID"
    logger_result=$?
    if (( logger_result != 0 )); then
        echo "[FAIL] Combined log writer exited $logger_result; original step logs remain at $LOG_DIR."
        result=1
    fi
    (( result == 0 )) || archive_failure
    exec 3>&-
    exit "$result"
}
trap finish EXIT
trap 'setup_code=$?; echo "[FAIL] exit=$setup_code line=$LINENO command=$BASH_COMMAND"; exit "$setup_code"' ERR
trap 'echo "[INTERRUPTED] User stopped installation."; exit 130' INT
trap 'echo "[INTERRUPTED] Installation received SIGTERM."; exit 143' TERM
echo "[LOG] $LOG_DIR"
echo '[INFO] Independent steps continue after errors; FAILED/BLOCKED means setup is incomplete.'

# Preparation is independent of Docker health. For example, a dead Docker
# daemon must not hide a separate ROS/GPIO problem. Unsupported platform or
# invalid user context blocks only operations that would mutate that system.
run_step system_before
run_step platform
run_step user_context
run_step source_snapshot
run_step docker_daemon
run_step docker_runtime 'docker_daemon'
run_step apt_base_update 'platform'
# Old cached apt indices may still work when update failed; try installation.
run_step base_packages 'platform'
run_step apt_universe 'platform'
run_step ros_apt_source 'platform'
run_step apt_ros_update 'platform ros_apt_source'
run_step ros_packages 'platform'
run_step ros_bridge_build 'platform user_context'
run_step lidar_source 'platform user_context'
run_step lidar_build 'platform user_context lidar_source'
run_step model_download 'platform user_context'
run_step yolo_image 'platform docker_daemon'
run_step jetson_gpio 'platform'
# Read-only checks still run after failed installs: sometimes part of an
# existing installation is usable, and these reveal independent missing pieces.
run_step verify_python
run_step verify_ros_bridge 'user_context'
run_step verify_lidar 'user_context'
run_step verify_model
run_step verify_yolo_image
run_step configuration
run_step system_after
current_step=finished
completed=1
if (( failed > 0 || blocked > 0 )); then exit 1; fi
