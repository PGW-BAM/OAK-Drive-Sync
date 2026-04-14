"""
Drive position time-series log.

Appends JSONL entries to data/drive_positions.jsonl on every state-change
publish. Supports startup recovery (seed last known position) and
offline postprocessing.

Log rotation: keep lines from the last MAX_AGE_DAYS days OR at most
MAX_LINES lines, whichever limit is hit first. Rotation runs at startup.
"""

from __future__ import annotations

import json
import os
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import TYPE_CHECKING

import structlog

if TYPE_CHECKING:
    from src.pi_controller.main import DriveManager

logger = structlog.get_logger()

LOG_PATH = Path("data/drive_positions.jsonl")
DRIVE_KEYS = ["cam1:a", "cam1:b", "cam2:a", "cam2:b"]
MAX_LINES = 100_000
MAX_AGE_DAYS = 7


def _key_to_field(key: str) -> str:
    """'cam1:a' → 'cam1_a'"""
    return key.replace(":", "_")


def append_position_entry(drives: dict) -> None:
    """Append one JSONL line with all drive positions and states.

    Called synchronously from _publish_position() — file I/O is fast
    enough at our publish rate (< 20 writes/sec).
    """
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    entry = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        **{_key_to_field(k): drives[k].current_position for k in DRIVE_KEYS if k in drives},
        "states": {k: drives[k].state.value for k in DRIVE_KEYS if k in drives},
    }
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps(entry) + "\n")


def load_last_entry() -> dict | None:
    """Return the last valid JSON line from the log.

    Uses seek-from-end to handle large files without reading all lines.
    Returns None if file is absent, empty, or the last line is corrupt.
    """
    if not LOG_PATH.exists():
        return None
    try:
        with open(LOG_PATH, "rb") as f:
            f.seek(0, 2)  # seek to EOF
            size = f.tell()
            if size == 0:
                return None

            # Walk backwards byte-by-byte to find the last complete line
            pos = size - 1
            buf = b""
            while pos >= 0:
                f.seek(pos)
                ch = f.read(1)
                if ch == b"\n" and buf.strip():
                    break
                buf = ch + buf
                pos -= 1

            line = buf.strip().decode("utf-8")
            if not line:
                return None
            return json.loads(line)
    except Exception as exc:
        logger.warning("position_log.load_failed", error=str(exc))
        return None


def seed_positions_from_log(drive_mgr: DriveManager) -> bool:
    """Pre-seed _current_position on each drive from the last log entry.

    Returns True if at least one drive position was restored.
    Call this AFTER setup_all() and BEFORE any movement.
    """
    entry = load_last_entry()
    if entry is None:
        logger.info("position_log.no_prior_entry")
        return False

    restored: list[str] = []
    for key in DRIVE_KEYS:
        drive = drive_mgr.drives.get(key)
        field = _key_to_field(key)
        if drive and field in entry:
            value = float(entry[field])
            drive.seed_position(value)
            restored.append(f"{key}={value:.1f}")

    if restored:
        logger.info("position_log.positions_restored", drives=restored)
    return bool(restored)


def rotate_log() -> None:
    """Trim the log: drop entries older than MAX_AGE_DAYS, keep at most MAX_LINES.

    Uses atomic write (temp file + os.replace) to avoid corrupt state on crash.
    Call once at startup before setup_all().
    """
    if not LOG_PATH.exists():
        return

    cutoff = datetime.now(timezone.utc) - timedelta(days=MAX_AGE_DAYS)

    try:
        with open(LOG_PATH, "r", encoding="utf-8") as f:
            lines = f.readlines()

        original_count = len(lines)

        def is_recent(line: str) -> bool:
            try:
                ts = datetime.fromisoformat(json.loads(line)["timestamp"])
                return ts >= cutoff
            except Exception:
                return False  # drop corrupt lines

        lines = [ln for ln in lines if ln.strip() and is_recent(ln)]

        # Trim to max lines (keep newest)
        if len(lines) > MAX_LINES:
            lines = lines[-MAX_LINES:]

        tmp = LOG_PATH.with_suffix(".jsonl.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            f.writelines(lines)
        os.replace(tmp, LOG_PATH)

        logger.info(
            "position_log.rotated",
            before=original_count,
            after=len(lines),
        )

    except Exception as exc:
        logger.warning("position_log.rotate_failed", error=str(exc))
