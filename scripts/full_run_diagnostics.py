#!/usr/bin/env python3
"""Summarize a flight record or bundle a complete run for offline debugging."""
import argparse
from collections import Counter, deque
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jetson.amr_core.full_run_diagnostics import create_bundle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_dir", nargs="?", default=str(Path(__file__).resolve().parents[1] / "logs/full_run/latest"))
    parser.add_argument("--bundle", action="store_true")
    parser.add_argument("--auto-bundle", action="store_true", help="Create one automatic bundle per failed run")
    parser.add_argument("--reason", default="runtime_failure")
    args = parser.parse_args()
    folder = Path(args.log_dir).resolve(strict=True)
    if not folder.is_dir() or not (folder / "launcher.log").exists():
        parser.error("Expected one full-run log directory containing launcher.log")
    if args.bundle or args.auto_bundle:
        print(create_bundle(folder, automatic=args.auto_bundle, reason=args.reason))
        return
    counts, recent, bad = Counter(), deque(maxlen=25), 0
    files = list(folder.glob("events.jsonl*"))
    def order(path):
        return int(path.suffix[1:]) if path.name != "events.jsonl" else 0
    for path in sorted(files, key=order, reverse=True):
        for line in path.read_text(encoding="utf8", errors="replace").splitlines():
            try:
                record = json.loads(line)
            except ValueError:
                bad += 1
                continue
            counts[record["event"]] += 1
            if record["event"] in ("state", "phase", "exception", "resume_denied", "CTRL_invalid", "STATUS_invalid",
                                    "legacy_button_limit", "legacy_initial_speed_ignored", "legacy_instant_stop",
                                    "user_instant_stop_request", "scan_invalid", "shutdown"):
                recent.append(record)
    print(json.dumps({"directory": str(folder), "event_counts": counts, "unreadable_lines": bad,
                      "recent_transitions_and_errors": list(recent)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
