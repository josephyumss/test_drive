"""Stream all retained flight-record rotations; distinguish evidence from guesses."""
from collections import Counter, deque
import csv
from datetime import datetime, timezone
import json
from pathlib import Path


IMPORTANT = {"state", "phase", "exception", "resume_denied", "CTRL_invalid", "STATUS_invalid",
             "legacy_button_limit", "legacy_initial_speed_ignored", "legacy_instant_stop",
             "user_instant_stop_request", "scan_invalid", "yolo_invalid", "shutdown", "loop_exit",
             "startup_step", "ultrasonic_exception", "gpio_setup", "gpio_cleanup",
             "uart_open", "uart_status_source", "uart_health", "uart_candidate_invalid", "runtime_health",
             "MCU_schema_mismatch"}
IMPORTANT.update({"legacy_status_format", "legacy_stop_zero_sent", "legacy_stop_standstill_observed",
                  "side_sensor_source", "mcu_side_input_invalid"})


def summarize_run(folder):
    folder = Path(folder)
    counts, recent, bad, read_errors = Counter(), deque(maxlen=40), 0, {}
    latest_uart = latest_snapshot = latest_runtime = None
    first_fault = None
    echoes = {side: Counter() for side in ("left", "right")}
    def order(path):
        suffix = path.name.removeprefix("events.jsonl")
        return int(suffix[1:]) if suffix.startswith(".") and suffix[1:].isdigit() else 0
    for path in sorted(folder.glob("events.jsonl*"), key=order, reverse=True):
        try:
            with path.open(encoding="utf8", errors="replace") as stream:
                for line in stream:
                    try:
                        record = json.loads(line)
                        if not isinstance(record, dict) or not isinstance(record.get("event"), str):
                            raise ValueError("Not an event object")
                    except (ValueError, TypeError):
                        bad += 1
                        continue
                    event = record["event"]
                    counts[event] += 1
                    if event == "state" and record.get("after") == "FAULT_STOP" and first_fault is None:
                        first_fault = record
                    if event == "uart_health":
                        latest_uart = record
                    elif isinstance(record.get("uart_rx"), dict):
                        latest_uart = record["uart_rx"]
                    if event == "runtime_health":
                        latest_runtime = record
                    if event == "snapshot":
                        latest_snapshot = record
                    if event == "ultrasonic":
                        reading = record.get("reading")
                        if isinstance(reading, dict) and reading.get("side") in echoes:
                            echoes[reading["side"]][str(reading.get("status"))] += 1
                    if event in IMPORTANT:
                        recent.append(record)
        except OSError as exc:
            read_errors[path.name] = str(exc)
    stages = []
    if (folder / "stages.tsv").exists():
        try:
            with (folder / "stages.tsv").open(encoding="utf8", errors="replace") as stream:
                stages = list(csv.DictReader(stream, delimiter="\t"))
        except OSError as exc:
            read_errors["stages.tsv"] = str(exc)
    return {"directory": str(folder), "summarized_utc": datetime.now(timezone.utc).isoformat(),
            "event_counts": dict(counts), "unreadable_lines": bad, "file_read_errors": read_errors,
            "first_retained_fault": first_fault, "latest_uart_health": latest_uart,
            "latest_runtime_health": latest_runtime, "latest_snapshot": latest_snapshot,
            "ultrasonic_counts": {side: dict(v) for side, v in echoes.items()},
            "setup_stages": stages,
            "failed_or_blocked_setup_stages": [s for s in stages if s.get("status") in ("FAILED", "BLOCKED")],
            "recent_transitions_and_errors": list(recent),
            "limitations": ["A successful UART write is not MCU receipt acknowledgement.",
                            "NO_ECHO cannot distinguish open space from sensor/wiring faults.",
                            "Missing RX cannot by itself identify firmware, wire, power, baud or pinmux cause.",
                            "Only retained files are summarized; collection time may precede final archive capture."]}
