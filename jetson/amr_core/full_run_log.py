"""Timestamped, bounded JSONL flight recorder shared by controller threads."""
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
import json
import logging
from logging.handlers import RotatingFileHandler
import math
from pathlib import Path
import time


def json_safe(value):
    if is_dataclass(value):
        return json_safe(asdict(value))
    if isinstance(value, float) and not math.isfinite(value):
        return "NaN" if math.isnan(value) else ("+Inf" if value > 0 else "-Inf")
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


class RequiredFileHandler(RotatingFileHandler):
    def handleError(self, record):
        # A full disk must stop motion rather than silently lose evidence.
        raise IOError("Flight recorder write/rotation failed")


class FlightRecorder:
    def __init__(self, folder, max_bytes=16 * 1024 * 1024, backups=24):
        Path(folder).mkdir(parents=True, exist_ok=True)
        self.logger = logging.Logger(f"full_run.{id(self)}", logging.INFO)
        self.handler = RequiredFileHandler(Path(folder) / "events.jsonl", maxBytes=max_bytes,
                                           backupCount=backups, encoding="utf-8")
        self.handler.setFormatter(logging.Formatter("%(message)s"))
        self.logger.addHandler(self.handler)
        self.origin = time.monotonic()

    def emit(self, event, **data):
        record = {"utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                  "monotonic_s": time.monotonic(), "elapsed_s": time.monotonic() - self.origin,
                  "event": event, **data}
        self.logger.info(json.dumps(json_safe(record), ensure_ascii=False, allow_nan=False,
                                    separators=(",", ":")))
        if event in {"state", "phase", "exception", "ready", "resume_denied", "shutdown", "handshake"}:
            print(json.dumps(json_safe(record), ensure_ascii=False), flush=True)

    def close(self):
        self.handler.flush()
        self.handler.close()
        self.logger.removeHandler(self.handler)
