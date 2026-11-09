"""Off-rig tests for calibration_tracker (no ROS needed):
    python3 -m pytest ros2_ws/src/scan_aggregator/test"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scan_aggregator"))

from calibration_tracker import CalibrationTracker  # noqa: E402

DAY = 86400.0
SOLVE = {
    "success": True, "roll_deg": 90.4, "pitch_deg": -0.7, "initial_roll_deg": 90.0,
    "initial_pitch_deg": 0.0, "before_rms_mm": 12.0, "after_rms_mm": 2.1,
    "n_points": 50000, "tilt_range_deg": 120.0,
}


class Clock:
    def __init__(self):
        self.t = 1_000_000.0

    def __call__(self):
        return self.t


def make(tmp_path, clock):
    return CalibrationTracker(path=str(tmp_path / "state.json"), clock=clock)


def test_never_calibrated_counts_scans(tmp_path):
    t = make(tmp_path, Clock())
    t.on_scan_saved()
    s = t.summary(30, 50)
    assert s["level"] == "never"
    assert s["scans_without_calibration"] == 1


def test_solve_then_apply_is_calibrate_mount_and_persists(tmp_path):
    clock = Clock()
    t = make(tmp_path, clock)
    t.sync_live_mount(90.0, 0.0)
    t.record_solve(SOLVE, 10.0, 170.0)
    assert t.summary(30, 50)["pending_solve"]["roll_deg"] == 90.4
    clock.t += 5
    assert t.on_mount_params_changed(90.4, -0.7)
    s = t.summary(30, 50)
    assert s["level"] == "ok"
    assert s["applied"]["source"] == "calibrate_mount"
    assert s["applied"]["solve"]["after_rms_mm"] == 2.1
    assert s["pending_solve"] is None

    # survives a restart
    t2 = make(tmp_path, clock)
    t2.sync_live_mount(90.4, -0.7)
    assert t2.summary(30, 50)["applied"]["source"] == "calibrate_mount"


def test_stale_by_age_and_scans(tmp_path):
    clock = Clock()
    t = make(tmp_path, clock)
    t.on_mount_params_changed(90.0, 0.0)
    for _ in range(3):
        t.on_scan_saved()
    assert t.summary(30, 3)["level"] == "ok"
    t.on_scan_saved()
    s = t.summary(30, 3)
    assert s["level"] == "stale" and "4 scans" in s["reasons"][0]
    clock.t += 31 * DAY
    s = t.summary(30, 0)
    assert s["level"] == "stale" and "days" in s["reasons"][0]
    assert t.summary(0, 0)["level"] == "ok"  # both checks disabled


def test_identical_reset_does_not_refresh(tmp_path):
    clock = Clock()
    t = make(tmp_path, clock)
    t.on_mount_params_changed(90.0, 0.0)
    t.on_scan_saved()
    clock.t += DAY
    assert not t.on_mount_params_changed(90.0, 0.0)
    assert t.summary(30, 50)["applied"]["scans_since"] == 1


def test_partial_change_and_manual_source(tmp_path):
    t = make(tmp_path, Clock())
    t.sync_live_mount(90.0, 0.0)
    assert t.on_mount_params_changed(None, 0.5)
    a = t.summary(30, 50)["applied"]
    assert (a["roll_deg"], a["pitch_deg"], a["source"]) == (90.0, 0.5, "manual")


def test_mismatch_at_startup_is_unknown(tmp_path):
    clock = Clock()
    t = make(tmp_path, clock)
    t.on_mount_params_changed(90.0, 0.0)
    t2 = make(tmp_path, clock)
    t2.sync_live_mount(91.0, 0.0)
    assert t2.summary(30, 50)["level"] == "unknown"


def test_clock_going_backwards(tmp_path):
    clock = Clock()
    t = make(tmp_path, clock)
    t.on_mount_params_changed(90.0, 0.0)
    clock.t -= DAY
    s = t.summary(30, 50)
    assert s["applied"]["age_s"] is None
    assert any("clock" in r for r in s["reasons"])


def test_failed_solve_is_not_pending(tmp_path):
    t = make(tmp_path, Clock())
    t.record_solve({"success": False, "error": "no wall"}, 0, 90)
    assert t.summary(30, 50)["pending_solve"] is None
