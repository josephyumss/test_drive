"""Standard-library-only, atomic diagnostic archives for a single robot run."""
from datetime import datetime, timezone
import io
import json
import os
from pathlib import Path
import stat
import tarfile
import tempfile


REQUEST_FILE = "auto_bundle.request.json"
STATUS_FILE = "auto_bundle.status.json"


def request_auto_bundle(folder, reason, source="controller"):
    """Create one request per run. Never overwrite the original fault reason."""
    folder = Path(folder)
    request = {"requested_utc": datetime.now(timezone.utc).isoformat(),
               "reason": str(reason), "source": source}
    try:
        with (folder / REQUEST_FILE).open("x", encoding="utf-8") as stream:
            json.dump(request, stream, ensure_ascii=False)
            stream.write("\n")
    except FileExistsError:
        return False
    return True


def _read_request(folder):
    try:
        return json.loads((folder / REQUEST_FILE).read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {}
    except (OSError, ValueError) as exc:
        return {"request_read_error": str(exc)}


def create_bundle(folder, *, automatic=False, reason="runtime_failure"):
    """Archive a stable prefix of each open log, then publish by atomic rename.

    File handles survive log rotation. Appends after a file is opened are not
    included; the captured sizes/time are recorded in bundle_metadata.json.
    Temporary archives live outside the run folder, never inside themselves.
    """
    folder = Path(folder).resolve(strict=True)
    if not folder.is_dir() or not (folder / "launcher.log").is_file():
        raise ValueError("Expected a run directory containing launcher.log")
    destination = folder.parent / f"{folder.name}-debug.tar.gz"
    if automatic:
        request_auto_bundle(folder, reason, source="launcher")
        try:
            status = json.loads((folder / STATUS_FILE).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            status = {}
        if isinstance(status, dict) and status.get("complete") and destination.is_file():
            return destination
    metadata = {"capture_started_utc": datetime.now(timezone.utc).isoformat(),
                "automatic": automatic, "trigger": _read_request(folder),
                "captured_files": {}, "skipped_files": {}}
    fd, temporary_name = tempfile.mkstemp(prefix=f".{folder.name}-debug-", suffix=".part", dir=folder.parent)
    os.close(fd)
    temporary = Path(temporary_name)
    try:
        with tarfile.open(temporary, "w:gz", compresslevel=1) as archive:
            for path in sorted(folder.rglob("*")):
                if not path.is_file() or path.is_symlink():
                    continue
                relative = path.relative_to(folder).as_posix()
                try:
                    stream = path.open("rb")
                except OSError as exc:
                    metadata["skipped_files"][relative] = str(exc)
                    continue
                with stream:
                    info = os.fstat(stream.fileno())
                    entry = tarfile.TarInfo(f"{folder.name}/{relative}")
                    entry.size, entry.mtime = info.st_size, info.st_mtime
                    entry.mode = stat.S_IMODE(info.st_mode)
                    archive.addfile(entry, stream)
                    metadata["captured_files"][relative] = info.st_size
            metadata["capture_finished_utc"] = datetime.now(timezone.utc).isoformat()
            payload = json.dumps(metadata, ensure_ascii=False, indent=2).encode("utf-8")
            entry = tarfile.TarInfo(f"{folder.name}/bundle_metadata.json")
            entry.size = len(payload)
            archive.addfile(entry, io.BytesIO(payload))
        # mkstemp uses 0600 even under a readable umask. The launcher's setgid
        # log root supplies the checkout group; let that group read the finished
        # archive without handing it root's credentials or write permissions.
        os.chmod(temporary, stat.S_IMODE(temporary.stat().st_mode) | stat.S_IRGRP)
        os.replace(temporary, destination)
        if automatic:
            # A completed archive is preserved even if fault-state logs later
            # rotate or normal user shutdown happens hours after the fault.
            completed = {"complete": True, "archive": str(destination),
                         "created_utc": metadata["capture_finished_utc"],
                         "trigger": metadata["trigger"]}
            (folder / STATUS_FILE).write_text(json.dumps(completed, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return destination
    finally:
        temporary.unlink(missing_ok=True)
