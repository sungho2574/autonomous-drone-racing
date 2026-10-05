"""Crash-safe flight manifest creation; no ROS dependency."""

import json
import os
from pathlib import Path
import subprocess
from datetime import datetime, timezone
from uuid import uuid4


def atomic_json(path, data):
    path = Path(path)
    tmp = path.with_suffix(".tmp")
    with tmp.open("w") as f:
        json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    fd = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def repository_root(*hints):
    for hint in hints:
        p = Path(hint).resolve()
        for candidate in (p, *p.parents):
            if (candidate / "adr_ws/src/adr_bringup").is_dir() and (
                candidate / "scripts"
            ).is_dir():
                return candidate
    raise RuntimeError(
        "저장소 루트를 찾지 못했습니다. record_root:=<저장소>/flight_logs 를 지정하세요."
    )


def new_session(
    record_root, track, parameters, snapshots, message_definitions, repo=None
):
    root = Path(record_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    now = datetime.now(timezone.utc)
    safe = "".join(c for c in track if c.isalnum() or c in "-_") or "track"
    folder = root / (
        now.strftime("%Y%m%dT%H%M%SZ") + "_" + safe + "_" + uuid4().hex[:8]
    )
    folder.mkdir()
    git = {}
    if repo:
        for key, args in [
            ("commit", ["rev-parse", "HEAD"]),
            ("status", ["status", "--porcelain"]),
        ]:
            try:
                git[key] = subprocess.check_output(
                    ["git", "-C", str(repo), *args], text=True, timeout=3
                ).strip()
            except (OSError, subprocess.SubprocessError):
                git[key] = None
    meta = {
        "schema_version": 1,
        "id": folder.name,
        "created_utc": now.isoformat(),
        "track": track,
        "status": "starting",
        "parameters": parameters,
        "git": git,
        "configs": snapshots,
        "message_definitions": message_definitions,
        "coordinate_frame": "map / ENU; gate front horizontal axis = (sin(yaw), -cos(yaw), 0)",
        "notes": [
            "One step1 invocation is one flight session.",
            "SLAM cloud count is in-state landmarks, not all KLT tracks.",
            "Corrected trajectory is gate-PnP drift correction, not loop closure.",
        ],
    }
    atomic_json(folder / "session.json", meta)
    return folder
