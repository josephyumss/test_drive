#!/usr/bin/env python3
"""Summarize a flight record or bundle a complete run for offline debugging."""
import argparse
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
    parser.add_argument("--finalize-bundle", action="store_true",
                        help="After shutdown, add final logs while preserving the original first-fault archive inside")
    parser.add_argument("--capture-environment", action="store_true",
                        help="Collect read-only bounded system evidence in this separate diagnostic process")
    parser.add_argument("--reason", default="runtime_failure")
    args = parser.parse_args()
    folder = Path(args.log_dir).resolve(strict=True)
    if not folder.is_dir() or not (folder / "launcher.log").exists():
        parser.error("Expected one full-run log directory containing launcher.log")
    if args.capture_environment:
        try:
            from jetson.amr_core.full_run_environment import collect_environment
            phase = "shutdown" if args.finalize_bundle else "fault"
            evidence = collect_environment(Path(__file__).resolve().parents[1], detailed=True)
            (folder / f"environment-{phase}.json").write_text(
                json.dumps(evidence, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
        except Exception:
            import traceback
            traceback.print_exc(file=sys.stderr)
    if args.bundle or args.auto_bundle or args.finalize_bundle:
        print(create_bundle(folder, automatic=args.auto_bundle or args.finalize_bundle,
                            reason=args.reason, finalize=args.finalize_bundle))
        return
    from jetson.amr_core.full_run_summary import summarize_run
    print(json.dumps(summarize_run(folder), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
