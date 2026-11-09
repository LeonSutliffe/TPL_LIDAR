"""Mount-calibration staleness tracking (HANDOFF.md "Calibration staleness
tracking").

mount_roll_deg/mount_pitch_deg (the "Calibrate Mount" tool) fix the
sensor's mounting angle on the tilt puck, but nothing used to record *when*
that was last done, or how much the rig has been used since. This module
keeps that record so the GUI can show e.g. "last calibrated 6 days / 14
scans ago" and flag it once it's past a configurable limit, instead of an
operator having to remember.

Pure Python, no ROS imports -- scan_aggregator's node feeds it events
(a calibration solve finished, vlp16_config's mount angles changed, a scan
was saved) and publishes summary() as JSON. Kept separate so it can be
unit-tested off the rig.

What counts as "the calibration":
  - "applied" is the reference point: the moment vlp16_config's live
    mount_roll_deg/mount_pitch_deg last actually changed. If the new
    values match the most recent successful Calibrate Mount solve, it's
    recorded as source "calibrate_mount" (with that solve's fit quality,
    i.e. "against what"); otherwise "manual" (Config tab edit, Load
    Settings, CLI). Either way the mount is now at a known, deliberately
    set value, so age and scan count restart from zero.
  - A solve that was computed but never applied doesn't refresh anything
    -- the scanner is still running on the old angles. It's reported as
    "pending" so the GUI can point out an unapplied result.

Stored in its own file, not ~/.lidar_scanner_settings.json: that file is
read/written wholesale by the GUI's Save/Load Settings, and this record is
history, not a setting -- loading an old settings file shouldn't rewind
it, and a scan finishing shouldn't race a settings write.
"""

from __future__ import annotations

import json
import os
import threading
import time

STATE_PATH = os.path.expanduser("~/.lidar_calibration_state.json")

# Live-param vs. solve-result comparison tolerance. The GUI applies the
# solve's own float values verbatim (no rounding), so a real "Apply" lands
# well inside this; a hand-typed value that happens to fall within a
# thousandth of a degree of the solve is, for every practical purpose, the
# same calibration anyway.
MATCH_TOL_DEG = 1e-3

# Fields copied from mount_calibration.calibrate_roll_pitch's result into
# the record -- the "against what" provenance shown in the GUI.
_RESULT_FIELDS = (
    "roll_deg", "pitch_deg", "initial_roll_deg", "initial_pitch_deg",
    "before_rms_mm", "after_rms_mm", "n_points", "tilt_range_deg",
)


class CalibrationTracker:
    def __init__(self, path: str = STATE_PATH, clock=time.time):
        self._path = path
        self._clock = clock
        # Called from both the executor thread (param events, solves) and
        # scan_aggregator's background save thread (scan counted), so every
        # read-modify-write goes through this.
        self._lock = threading.Lock()
        self._state = self._load()
        # Live mount angles as last seen on vlp16_config -- not persisted,
        # re-synced from the node at startup (see sync_live_mount).
        self._live_roll: float | None = None
        self._live_pitch: float | None = None
        # True when the live angles at startup don't match the recorded
        # applied ones (changed while this node wasn't running, or the
        # settings file was edited by hand) -- the record can't vouch for
        # what's actually live, so it's reported as unknown.
        self._mismatch = False

    # ---- persistence ----

    def _load(self) -> dict:
        try:
            with open(self._path) as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def _save(self) -> None:
        tmp = self._path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self._state, f, indent=1)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self._path)

    # ---- events ----

    def record_solve(self, result: dict, sweep_min_deg: float, sweep_max_deg: float) -> None:
        """A Calibrate Mount solve finished (successfully or not)."""
        now = self._clock()
        with self._lock:
            if result.get("success"):
                entry = {k: result[k] for k in _RESULT_FIELDS if k in result}
                entry["time"] = now
                entry["sweep_min_deg"] = sweep_min_deg
                entry["sweep_max_deg"] = sweep_max_deg
                self._state["last_solve"] = entry
            else:
                self._state["last_failed_solve"] = {"time": now, "error": result.get("error")}
            self._save()

    def sync_live_mount(self, roll_deg: float, pitch_deg: float) -> None:
        """Startup sync with vlp16_config's current values -- doesn't count
        as a calibration, just establishes the baseline later changes are
        compared against."""
        with self._lock:
            self._live_roll, self._live_pitch = roll_deg, pitch_deg
            applied = self._state.get("applied")
            self._mismatch = bool(applied) and not (
                _close(applied.get("roll_deg"), roll_deg)
                and _close(applied.get("pitch_deg"), pitch_deg)
            )

    def on_mount_params_changed(self, roll_deg: float | None, pitch_deg: float | None) -> bool:
        """vlp16_config's mount_roll_deg and/or mount_pitch_deg were set
        (None = not part of this change). Returns True if this counted as
        a new calibration (values actually changed)."""
        now = self._clock()
        with self._lock:
            new_roll = roll_deg if roll_deg is not None else self._live_roll
            new_pitch = pitch_deg if pitch_deg is not None else self._live_pitch
            prev_roll, prev_pitch = self._live_roll, self._live_pitch
            self._live_roll, self._live_pitch = new_roll, new_pitch
            if new_roll is None or new_pitch is None:
                # Only half the pair known (startup sync hasn't landed yet)
                # -- can't record a complete calibration.
                return False
            if (
                prev_roll is not None and prev_pitch is not None
                and new_roll == prev_roll and new_pitch == prev_pitch
            ):
                # Re-set to identical values (e.g. Load Settings re-pushing
                # every field) -- nothing about the mount changed.
                return False

            solve = self._state.get("last_solve")
            from_solve = bool(solve) and _close(solve.get("roll_deg"), new_roll) and _close(
                solve.get("pitch_deg"), new_pitch
            )
            self._state["applied"] = {
                "time": now,
                "roll_deg": new_roll,
                "pitch_deg": new_pitch,
                "source": "calibrate_mount" if from_solve else "manual",
                "solve": dict(solve) if from_solve else None,
                "scans_since": 0,
            }
            self._mismatch = False
            self._save()
            return True

    def on_scan_saved(self) -> None:
        with self._lock:
            applied = self._state.get("applied")
            if applied is None:
                self._state["scans_without_calibration"] = (
                    int(self._state.get("scans_without_calibration", 0)) + 1
                )
            else:
                applied["scans_since"] = int(applied.get("scans_since", 0)) + 1
            self._save()

    # ---- reporting ----

    def summary(self, max_age_days: float, max_scans: int) -> dict:
        """JSON-able snapshot for the GUI. `level` is "ok", "stale",
        "never", or "unknown"; `reasons` are short human-readable strings.
        Ages are computed here on the Pi's clock (not the browser's) so a
        phone/laptop with a different clock can't skew them."""
        now = self._clock()
        with self._lock:
            applied = self._state.get("applied")
            solve = self._state.get("last_solve")
            out: dict = {
                "level": "ok",
                "reasons": [],
                "max_age_days": max_age_days,
                "max_scans": max_scans,
                "applied": None,
                "pending_solve": None,
            }

            if applied is None:
                out["level"] = "never"
                out["reasons"].append("no Calibrate Mount run has been recorded on this scanner")
                out["scans_without_calibration"] = int(
                    self._state.get("scans_without_calibration", 0)
                )
            else:
                age_s = now - float(applied["time"])
                a = dict(applied)
                # Negative age = the Pi's clock went backwards since (no
                # RTC; time can jump when NTP syncs after an offline boot).
                a["age_s"] = age_s if age_s >= 0 else None
                out["applied"] = a
                if self._mismatch:
                    out["level"] = "unknown"
                    out["reasons"].append(
                        "live mount angles differ from the last recorded calibration "
                        "(changed while scan_aggregator wasn't running?)"
                    )
                if a["age_s"] is None:
                    out["reasons"].append("calibration age unknown (Pi clock changed)")
                elif max_age_days > 0 and age_s > max_age_days * 86400.0:
                    out["level"] = "stale"
                    out["reasons"].append(
                        f"{age_s / 86400.0:.0f} days since last calibration "
                        f"(limit {max_age_days:g})"
                    )
                scans = int(a.get("scans_since", 0))
                if max_scans > 0 and scans > max_scans:
                    out["level"] = "stale"
                    out["reasons"].append(
                        f"{scans} scans since last calibration (limit {max_scans})"
                    )

            if solve and (applied is None or float(solve["time"]) > float(applied["time"])):
                p = dict(solve)
                age_s = now - float(solve["time"])
                p["age_s"] = age_s if age_s >= 0 else None
                out["pending_solve"] = p
            return out


def _close(a, b) -> bool:
    try:
        return abs(float(a) - float(b)) <= MATCH_TOL_DEG
    except (TypeError, ValueError):
        return False
