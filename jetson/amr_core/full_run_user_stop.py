"""Atomic, local stop requests addressed to one running launcher's identity."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import secrets
import tempfile


def process_start_ticks(stat):
    return stat.rsplit(")", 1)[1].split()[19]


def request_stop(project, *, proc_root="/proc"):
    project = Path(project).resolve()
    pid, ticks = (project / ".run/full_run.pid").read_text().split()
    if not pid.isdigit() or not ticks.isdigit():
        raise ValueError("Invalid full-run PID record")
    process = Path(proc_root) / pid
    if process_start_ticks((process / "stat").read_text()) != ticks:
        raise ValueError("PID was reused; stop request refused")
    cwd = (process / "cwd").resolve(strict=True)
    arguments = (process / "cmdline").read_bytes().split(b"\0")
    expected = project / "scripts/start_full_run.sh"
    if not any((Path(os.fsdecode(arg)) if Path(os.fsdecode(arg)).is_absolute() else cwd / os.fsdecode(arg)).resolve() == expected
               for arg in arguments if arg):
        raise ValueError("PID is not this full-run launcher; stop request refused")
    payload = {"pid": int(pid), "start_ticks": ticks, "request_id": secrets.token_hex(16),
               "requested_utc": datetime.now(timezone.utc).isoformat(), "action": "instant_stop"}
    target = project / ".run/full_run.user_stop.json"
    fd, name = tempfile.mkstemp(prefix=".full-run-user-stop-", dir=target.parent)
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf8") as stream:
            json.dump(payload, stream)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
    return payload


def read_stop_request(path, pid, ticks, last_id):
    try:
        payload = json.loads(Path(path).read_text(encoding="utf8"))
    except FileNotFoundError:
        return None
    if not isinstance(payload, dict):
        raise ValueError("Stop request must be a JSON object")
    if payload.get("pid") != pid or payload.get("start_ticks") != str(ticks):
        return None
    if (payload.get("action") != "instant_stop" or not isinstance(payload.get("request_id"), str)
            or not payload["request_id"]):
        raise ValueError("Invalid stop request")
    if payload["request_id"] == last_id:
        return None
    return payload
