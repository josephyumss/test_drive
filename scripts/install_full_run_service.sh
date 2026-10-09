#!/usr/bin/env bash
set -Eeuo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
(( EUID == 0 )) || { echo 'Run: sudo bash scripts/install_full_run_service.sh'; exit 1; }
[[ "$PROJECT_DIR" != *$'\n'* && "$PROJECT_DIR" != *'"'* && "$PROJECT_DIR" != *'%'* ]] || { echo 'Unsupported character in installation path'; exit 1; }
if systemctl is-active --quiet amr-core.service || systemctl is-enabled --quiet amr-core.service; then
    echo 'amr-core.service is active and owns the devices. Stop/disable it before installing the full-run service.'
    exit 1
fi
replacement="$(printf '%s' "$PROJECT_DIR" | sed 's/[\\&|]/\\&/g')"
temporary_dir="$(mktemp -d)"
temporary_unit="$temporary_dir/amr-full-run.service"
trap 'rm -f -- "$temporary_unit"; rmdir -- "$temporary_dir"' EXIT
sed "s|@PROJECT_DIR@|$replacement|g" "$PROJECT_DIR/deploy/amr-full-run.service" > "$temporary_unit"
systemd-analyze verify "$temporary_unit" || { echo 'Generated systemd unit did not validate'; exit 1; }
install -m 644 "$temporary_unit" /etc/systemd/system/amr-full-run.service
systemctl daemon-reload
systemctl enable amr-full-run.service
echo 'Boot auto-start enabled. To start now: sudo systemctl start amr-full-run.service'
echo 'Manual foreground run: sudo bash scripts/start_full_run.sh'
