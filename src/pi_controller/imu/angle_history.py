"""
Per-camera (target_angle → motor_position) history.

The radial drives drift over long runs: the motor step count that once put
the IMU at −75° will gradually need +20 or −30 steps of correction to land
on the same angle a few hours later. To avoid re-running the full Newton
search every time a sequence revisits a taught angle, we remember the last
successful motor position per bucketed target angle per camera. The next
move to that angle seeds the drive open-loop to the stored position at
fast speed, then the Newton loop only has to correct a few steps of drift.

Storage lives inside the same `config/imu_calibration.yaml` file that the
rest of the per-cam geometric state lives in — one file, one writer
(DriftDetector._save_calibration). We don't own the YAML path directly;
we read/mutate a sub-dict of the calibration dict and call the save
callback supplied by DriftDetector.

Bucket resolution is 0.5°. Keys look like "-75.0", "-74.5", "90.0". This
avoids float-key churn in the YAML and keeps the file diff-readable.
"""

from __future__ import annotations

from typing import Callable

import structlog


logger = structlog.get_logger()


class AngleHistory:
    def __init__(
        self,
        calibration_dict: dict,
        save_callback: Callable[[], None],
    ) -> None:
        """`calibration_dict` is the shared dict loaded by DriftDetector from
        imu_calibration.yaml. `save_callback` is DriftDetector._save_calibration
        — called after every record/invalidate so the history survives a
        restart.
        """
        self._cal = calibration_dict
        self._save = save_callback
        self._cal.setdefault("angle_history", {})

    # ── Bucket helpers ────────────────────────────────────────────────

    @staticmethod
    def _bucket_key(angle_deg: float) -> str:
        """0.5° buckets: -74.73° → '-74.5', -75.12° → '-75.0'."""
        bucket = round(angle_deg * 2) / 2
        return f"{bucket:.1f}"

    def _axis_dict(self, cam_id: str) -> dict[str, int]:
        history = self._cal.setdefault("angle_history", {})
        return history.setdefault(f"{cam_id}_b", {})

    # ── Public API ────────────────────────────────────────────────────

    def seed_for(
        self,
        cam_id: str,
        target_angle_deg: float,
        reference_position: int | None,
        reference_angle_deg: float | None,
        steps_per_degree: float,
        direction_sign: int,
    ) -> int | None:
        """Best guess for the motor position that will put the IMU at
        `target_angle_deg`.

        Priority:
          1. Exact 0.5° bucket hit in history → stored position.
          2. Reference-anchor formula, if reference_position is known:
                 ref + (target - ref_angle) * steps_per_degree * direction_sign
          3. None (caller should skip the pre-move and let the Newton loop
             start from wherever the drive currently sits).
        """
        hist = self._axis_dict(cam_id)
        key = self._bucket_key(target_angle_deg)
        if key in hist:
            seed = int(hist[key])
            logger.info(
                "angle_history.seed_hit",
                cam_id=cam_id,
                target_angle_deg=target_angle_deg,
                bucket=key,
                seed_position=seed,
            )
            return seed

        if reference_position is not None and reference_angle_deg is not None:
            seed = int(
                round(
                    reference_position
                    + (target_angle_deg - reference_angle_deg)
                    * steps_per_degree
                    * direction_sign
                )
            )
            logger.info(
                "angle_history.seed_formula",
                cam_id=cam_id,
                target_angle_deg=target_angle_deg,
                reference_position=reference_position,
                reference_angle_deg=reference_angle_deg,
                steps_per_degree=round(steps_per_degree, 3),
                direction_sign=direction_sign,
                seed_position=seed,
            )
            return seed

        logger.info(
            "angle_history.seed_none",
            cam_id=cam_id,
            target_angle_deg=target_angle_deg,
            reason="no_history_no_reference",
        )
        return None

    def record(
        self,
        cam_id: str,
        target_angle_deg: float,
        position: int,
    ) -> None:
        """Write-through save of the motor position observed to produce
        `target_angle_deg` on the IMU for `cam_id`. Overwrites any existing
        entry for the bucket.
        """
        hist = self._axis_dict(cam_id)
        key = self._bucket_key(target_angle_deg)
        prev = hist.get(key)
        hist[key] = int(position)
        logger.info(
            "angle_history.record",
            cam_id=cam_id,
            target_angle_deg=target_angle_deg,
            bucket=key,
            position=int(position),
            prev_position=prev,
        )
        try:
            self._save()
        except Exception:
            logger.exception("angle_history.save_failed", cam_id=cam_id)

    def invalidate_stale(
        self,
        cam_id: str,
        reference_position: int,
        reference_angle_deg: float,
        steps_per_degree: float,
        direction_sign: int,
        tolerance_steps: int,
    ) -> list[str]:
        """Prune entries whose stored position now differs from the
        reference-anchor-derived position by more than `tolerance_steps`.

        Called on auto-cal success: if the new reference_position is close
        to the old one, most history entries stay valid (drift was small).
        If the reference shifted a lot, the stale entries are dropped and
        the next move to that angle re-runs the full Newton search.
        Returns the list of dropped bucket keys for logging.
        """
        hist = self._axis_dict(cam_id)
        dropped: list[str] = []
        for key, stored_pos in list(hist.items()):
            try:
                angle = float(key)
            except ValueError:
                # Corrupted key — drop it.
                dropped.append(key)
                del hist[key]
                continue
            expected = int(
                round(
                    reference_position
                    + (angle - reference_angle_deg)
                    * steps_per_degree
                    * direction_sign
                )
            )
            if abs(int(stored_pos) - expected) > tolerance_steps:
                dropped.append(key)
                del hist[key]

        if dropped:
            logger.warning(
                "angle_history.pruned_stale",
                cam_id=cam_id,
                reference_position=reference_position,
                reference_angle_deg=reference_angle_deg,
                tolerance_steps=tolerance_steps,
                keys=dropped,
            )
            try:
                self._save()
            except Exception:
                logger.exception("angle_history.save_failed", cam_id=cam_id)

        return dropped

    def count(self, cam_id: str | None = None) -> int:
        """Return number of stored entries, optionally filtered by cam."""
        history = self._cal.get("angle_history", {}) or {}
        if cam_id is not None:
            return len(history.get(f"{cam_id}_b", {}) or {})
        return sum(len(v or {}) for v in history.values())
