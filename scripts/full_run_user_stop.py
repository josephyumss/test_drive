#!/usr/bin/env python3
"""Request an immediate, UP-resumable pause, without exiting the robot run."""
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from jetson.amr_core.full_run_user_stop import request_stop


def main():
    try:
        request = request_stop(PROJECT)
    except (OSError, ValueError, IndexError) as exc:
        print(f"[STOP REFUSED] {exc}", file=sys.stderr)
        return 1
    print(f"[INSTANT STOP] Requested for running full-run process {request['pid']}. "
          "The process stays alive. Wait for zero-target acknowledgement (~0.2s), "
          "then press UP to resume after the space is clear.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
