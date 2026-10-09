#!/usr/bin/env bash
# Sourced by both launchers. Keep root ownership; grant the checkout group read
# access so sudo/systemd diagnostics can be added to Git without sudo git.
full_run_prepare_log_access() {
    local project_dir="$1" log_root="$2" log_dir="$3" log_group directory early_file
    local -a directories=("$log_root" "$log_dir")
    log_group="$(stat -c %g -- "$project_dir")"
    if [[ "$log_root" == "$project_dir/logs/"* ]]; then
        directories=("$project_dir/logs" "${directories[@]}")
    fi
    for directory in "${directories[@]}"; do
        [[ -d "$directory" && ! -L "$directory" ]] || {
            echo "[LOG_ACCESS FAIL] Expected a real log directory: $directory" >&2
            return 1
        }
        if (( EUID == 0 )); then chgrp -- "$log_group" "$directory"; fi
        # setgid makes future files/subdirectories inherit the checkout group.
        # Group read/traverse only: no world-read grant and no group write grant.
        chmod g+rx,g+s -- "$directory"
    done
    # tee/stages.tsv can be opened before traps and this helper are installed.
    # Initialize the tee path to avoid a race, then fix only these early files.
    touch -- "$log_dir/launcher.log"
    for early_file in "$log_dir/launcher.log" "$log_dir/stages.tsv"; do
        if [[ -f "$early_file" && ! -L "$early_file" ]]; then
            if (( EUID == 0 )); then chgrp -- "$log_group" "$early_file"; fi
            chmod g+r -- "$early_file"
        fi
    done
    echo "[LOG_ACCESS] checkout_group=$log_group directories=group_read_traverse files=group_read"
}
