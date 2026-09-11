"""Scan controller for the terrestrial LiDAR scanner -- two modes.

Step-and-stare: drives tilt_axis_bridge through a sequence of discrete tilt
stops (home, then step across the configured range), and at each stop --
once tilt_axis_bridge reports "settled" -- captures the VLP-16 point cloud
for a fixed number of azimuth revolutions before moving to the next stop.
This is the brief's validated, motion-blur-free approach.

Continuous sweep scan: homes once, then hands tilt_axis_bridge's own
existing continuous-sweep feature (sweep_enable/sweep_min_rad/etc, the same
thing the GUI's Motor tab drives directly) the sweep range/speed, and
merges the point cloud live while it runs, rather than at fixed stops.
Faster coverage, but points are captured while the axis is moving -- unlike
step-and-stare, this trades some motion blur/smear (worse at higher sweep
speeds) for speed, exactly the tradeoff the project brief flagged when it
chose step-and-stare as the first validated mode. By default, tf2's buffer
interpolates the tilt angle between its ~10 Hz joint_state samples for each
point cloud's own timestamp, but that's still one transform per cloud
message (~1 rotation of points), not per individual point -- some smear
within each cloud is inherent to this mode unless enable_sweep_deskew is on
(see _try_transform_and_accumulate_deskewed), which instead rotates each
point by its own individually-interpolated tilt angle using the point's own
per-point `time` field. Clouds captured within sweep_edge_margin_deg of
either turnaround are dropped entirely (see _in_sweep_edge_margin) -- the
axis is changing direction right at the edges, on top of the smear that's
already inherent to this mode elsewhere in the sweep (deskewing doesn't
change this -- edge clouds are still dropped, not rescued).

Both modes transform each cloud into a common frame via tf2 (using the tilt
joint's own published tf, so the lever-arm/tilt-angle math lives in one
place: scanner_description's URDF + robot_state_publisher) and accumulate
it into a merged cloud, written to a PCD file when the run completes. A
sweep scan with no duration cap keeps accumulating points in RAM for as
long as it runs -- fine for a bounded session, but avoid leaving an
unbounded sweep running for a very long time on a memory-constrained field
unit (see the Pi field-recording setup: prefer rosbag2 raw-packet recording
over this in-memory merge for open-ended sessions).

Talks to tilt_axis_bridge purely over its public ROS interface (cmd_position,
status, ~/home, ~/stop, ~/sweep_enable, ~/set_parameters) rather than
importing anything from it, so this node has no build-time coupling to that
package.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import threading
import time
import zipfile
from collections import deque
from datetime import datetime
from typing import Callable

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from numpy.lib import recfunctions as rfn
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Bool, Float64, Header, String
from std_srvs.srv import Trigger
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from . import mount_calibration
from .e57_writer import read_pcd_points, write_e57
from .pcd_writer import fsync_durable

# x, y, z, intensity as contiguous float32 -- matches the layout of the
# Nx4 float32 arrays already accumulated in _merged_points, so building a
# preview PointCloud2 is a straight .tobytes() with no per-point packing.
_PREVIEW_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]

# Shared settings file -- same file, same format, as tilt_axis_bridge's own
# copy of these two helpers (see that module for the full rationale).
# Duplicated rather than imported: this package has no build-time coupling
# to tilt_axis_bridge (see this file's own docstring above), and these two
# functions are the only thing that would otherwise need one. This node
# owns the "scan_aggregator" top-level section of the file; tilt_axis_bridge
# owns "tilt_axis_bridge" and "mks_driver".
SETTINGS_PATH = os.path.expanduser("~/.lidar_scanner_settings.json")

# Local-first scan output -- not a parameter/setting (see README's local
# scans step): lives inside the GUI's own served static folder (see
# tpl-gui-http.service, a plain `python3 -m http.server 8080` rooted at
# web/tilt_axis_gui/) so every finished scan is downloadable at
# http://<pi-ip>:8080/scans/<name>.e57 for free, no separate download
# server needed. Writing straight to removable USB storage used to be the
# only option here (this was the old output_dir default) -- measured this
# session at ~12 MB/s on this rig's actual stick vs. ~36 MB/s on the Pi's
# own SD card, a real ~3x gap that used to sit on the scan-completion
# critical path. USB is now purely an explicit, on-demand export target
# (see export_to_usb_request below), never the live write path.
OUTPUT_DIR = os.path.expanduser("~/TPL_LIDAR/web/tilt_axis_gui/scans")

# Where "Export to USB" copies finished scans to, on request -- this rig's
# real removable-storage mount point (see README's USB-storage step), the
# same value output_dir itself used to default to before local-first saving.
USB_EXPORT_DIR = "/media/tpl/LIDAR"


# Extra per-point fields captured alongside x/y/z/intensity, when the
# raw /velodyne_points message actually carries them -- the
# ros-drivers/velodyne ROS2 driver's own PointXYZIRT convention. Added
# 2026-09-10 for future per-point sweep deskewing (see "Feature
# roadmap" in HANDOFF.md): ring identifies which of the 16 laser
# channels produced a point (also useful for the still-open per-laser-
# calibration item, independent of deskewing), and time is a per-point
# capture-time offset -- the actual ingredient deskewing needs, since
# today's transform is one tf2 lookup per whole ~0.1s cloud message
# (see _try_transform_and_accumulate), not one per point.
# NOT YET CONFIRMED against real hardware that these are the exact
# field names this project's specific driver/version publishes -- see
# _read_points_with_extra_fields, which degrades to NaN-filled ring/
# time columns rather than failing outright if they turn out wrong or
# absent, precisely because this couldn't be checked without hardware.
EXTRA_POINT_FIELDS = ("ring", "time")
POINT_FIELD_NAMES = ("x", "y", "z", "intensity") + EXTRA_POINT_FIELDS


def _read_points_with_extra_fields(cloud_msg: PointCloud2) -> np.ndarray:
    """Reads x/y/z/intensity plus, best-effort, ring/time (see
    EXTRA_POINT_FIELDS) from a cloud message into one Nx6 float32
    array. All fields are read in a single pc2.read_points call (one
    skip_nans pass) specifically so every column stays row-aligned to
    the same point -- reading ring/time from a second, separate call
    (e.g. against the pre-transform message) would risk two
    independently-NaN-filtered arrays of different lengths silently
    lining up wrong, attaching the wrong ring/time to the wrong point.

    Falls back to NaN-filled ring/time columns if the message doesn't
    actually carry those two fields, so every array this produces is
    always the same width regardless (np.concatenate in
    _write_output_in_background needs that), and a wrong guess about
    the real field names degrades to "no ring/time data", not a crashed
    scan -- this project's actual driver/version hasn't been checked
    against this assumption yet, see EXTRA_POINT_FIELDS above."""
    available = {f.name for f in cloud_msg.fields}
    if all(name in available for name in EXTRA_POINT_FIELDS):
        structured = pc2.read_points(cloud_msg, field_names=POINT_FIELD_NAMES, skip_nans=True)
        return rfn.structured_to_unstructured(structured, dtype=np.float32)

    structured = pc2.read_points(cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
    arr = rfn.structured_to_unstructured(structured, dtype=np.float32)
    pad = np.full((arr.shape[0], len(EXTRA_POINT_FIELDS)), np.nan, dtype=np.float32)
    return np.concatenate([arr, pad], axis=1)


def _sanitize_filename_part(text: str) -> str:
    """Collapses whitespace to underscores and drops anything not
    alphanumeric/dash/underscore, so a free-typed project_name (e.g. "St
    Mary's Cathedral") is safe to use directly in a filename/URL rather
    than needing the user to pre-sanitize it themselves. Trimmed to a
    sane length -- this is a filename component, not a full description."""
    collapsed = re.sub(r"\s+", "_", text.strip())
    safe = re.sub(r"[^A-Za-z0-9_-]", "", collapsed)
    return safe[:60]


def _build_output_basename(project_name: str, extension: str) -> str:
    """<sanitized project>_<timestamp>.<ext> when a project is set (see
    project_name's own declare_parameter comment); falls back to today's
    plain scan_<timestamp>.<ext> when it isn't, so leaving project_name
    blank is a no-op, not a forced workflow change. No per-station
    counter in the name -- each capture within a project is distinguished
    by its own timestamp alone, per explicit request."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    project = _sanitize_filename_part(project_name)
    if not project:
        return f"scan_{stamp}.{extension}"
    return f"{project}_{stamp}.{extension}"


def _load_settings_section(section: str) -> dict:
    try:
        with open(SETTINGS_PATH) as f:
            return dict(json.load(f).get(section, {}))
    except (OSError, ValueError):
        return {}


STATE_IDLE = "idle"
STATE_HOMING = "homing"
STATE_MOVING = "moving"
STATE_SETTLING_EXTRA = "settling_extra"
STATE_CAPTURING = "capturing"
STATE_SWEEP_HOMING = "sweep_homing"
STATE_SWEEP_SCANNING = "sweep_scanning"
STATE_SAVING = "saving"
STATE_MOUNT_SOLVING = "mount_solving"
STATE_DONE = "done"
STATE_ABORTED = "aborted"

MODE_STEP_AND_STARE = "step_and_stare"
MODE_SWEEP = "sweep"
# Reuses the sweep motion state machine (home, then sweep across
# sweep_min_deg/sweep_max_deg) unchanged -- only _on_pointcloud's handling
# of each arriving cloud, and what _finish_run does at the end, differ
# from a normal MODE_SWEEP run. See mount_calibration.py.
MODE_CALIBRATE = "calibrate"
# See _accumulate_calibration_cloud -- every 20th point is kept, cutting a
# real multi-million-point sweep down to a size every step of
# mount_calibration.py handles in seconds rather than minutes, with no
# meaningful loss of calibration precision (a plane fit doesn't need
# every point a wall reflects).
CALIBRATION_POINT_STRIDE = 20

# How long a cloud that fails its first tf lookup gets retried on
# subsequent ticks before being dropped for real -- see
# _try_transform_and_accumulate/_retry_pending_transforms.
TF_RETRY_TIMEOUT_S = 1.0

# How many (stamp, tilt_rad) samples _on_joint_state keeps for
# _try_transform_and_accumulate_deskewed's own per-point interpolation --
# see that function. tilt_axis_bridge republishes joint_state at its own
# joint_state_rate_hz (50Hz default), so 300 samples is a good 6s of
# history -- comfortably more than one cloud message's own ~0.1s span
# plus TF_RETRY_TIMEOUT_S's worth of retry delay, with margin to spare.
JOINT_STATE_HISTORY_MAXLEN = 300


def deg_to_rad(deg: float) -> float:
    return deg * 3.14159265358979 / 180.0


def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / 3.14159265358979


# Coverage bin width for sweep mode's live coverage map (see
# _init_coverage/_record_coverage) -- step-and-stare uses its own
# step_deg instead (one bin per stop, an exact match), but sweep has no
# equivalent natural bin size since tilt moves continuously, so this is
# just a fixed, reasonable resolution.
SWEEP_COVERAGE_BIN_DEG = 2.0


class ScanAggregatorNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_aggregator")

        # Ensured at startup, not lazily on first scan, so a fresh install
        # with zero scans yet still has list_local_scans_request return an
        # empty list rather than an OSError from a missing directory.
        os.makedirs(OUTPUT_DIR, exist_ok=True)

        # Every parameter below defaults to whatever was last persisted to
        # the shared settings file (see SETTINGS_PATH, shared with
        # tilt_axis_bridge) rather than a hardcoded literal, falling back to
        # that literal if nothing's been saved yet -- see tilt_axis_bridge's
        # own __init__ for the full rationale (same pattern, same file,
        # this node's own top-level section).
        persisted = _load_settings_section("scan_aggregator")

        def _default(name: str, fallback):
            return persisted.get(name, fallback)

        self.declare_parameter("tilt_node_name", _default("tilt_node_name", "/tilt_axis_bridge"))
        self.declare_parameter(
            "pointcloud_topic", _default("pointcloud_topic", "/velodyne_points")
        )
        # Only used by mount calibration, to read/write vlp16_config's
        # mount_roll_deg/mount_pitch_deg -- see _on_start_mount_calibration.
        self.declare_parameter(
            "vlp16_node_name", _default("vlp16_node_name", "/vlp16_config")
        )
        self.declare_parameter("output_frame", _default("output_frame", "base_link"))
        # Negates the Z coordinate of every merged point before it's kept.
        # Exists because of a real, confirmed-empirically inversion this
        # session: with the tilt axis vertical and the VLP-16 mounted on
        # its side (see vlp16_config's mount_roll_deg/mount_pitch_deg),
        # every scan came out upside-down regardless of which mount
        # rotation was tried -- all four 90-degree-spaced twist candidates
        # around the confirmed-horizontal spin axis were tested and all
        # failed, and RViz's Orbit view's own "Invert Z Axis" (a camera-only
        # setting, doesn't touch data) was found to fully correct the
        # *view* of an otherwise-unchanged file. That's the tell: a real
        # Z-only inversion, with X/Y otherwise correct, is mathematically
        # impossible to produce from mount_roll_deg/mount_pitch_deg/
        # mount_yaw_deg -- those only ever compose proper rotations
        # (determinant +1) via _quaternion_from_euler, and a proper
        # rotation can never flip exactly one axis while leaving the other
        # two untouched (that's a reflection, determinant -1) -- so no
        # mount-offset value could ever have fixed this, and the exhaustive
        # four-candidate search was never going to succeed. The actual
        # reflection is presumably introduced upstream of this node (the
        # VLP-16 driver/calibration, or a real-world convention mismatch
        # this project hasn't chased down) -- this parameter cancels it out
        # at the one point everything funnels through (_transform_and_
        # accumulate) rather than fixing it at its real source, which is
        # still unidentified. Defaults to True -- confirmed needed for
        # this rig's actual current mount; re-derive (see this comment's
        # own reasoning above) if the physical mount ever changes.
        self.declare_parameter("invert_z_axis", _default("invert_z_axis", True))
        # Same idea, X and Y -- added once invert_z_axis alone left the
        # scan mirrored in X/Y (expected: fixing only Z is one reflection:
        # determinant -1; the *real* underlying issue apparently needs an
        # odd number of axis flips overall, and one wasn't enough). All
        # three are independent toggles rather than baking in "flip all
        # three" as a single option, since which combination is actually
        # needed depends on this rig's specific mount and isn't assumed to
        # be the same for anyone else's. x/z True, y False confirmed
        # needed for this rig's actual current mount.
        self.declare_parameter("invert_x_axis", _default("invert_x_axis", True))
        self.declare_parameter("invert_y_axis", _default("invert_y_axis", False))
        # Session/project grouping (added 2026-09-11, HANDOFF.md Feature
        # roadmap; redesigned same day per explicit request -- see git
        # history for the earlier venue_name+station_number version).
        # project_name identifies the current job/site (e.g. a cathedral
        # name) and feeds into the auto-generated output filename (see
        # _write_output_in_background) as "<project>_<timestamp>" instead
        # of every run getting an anonymous scan_<timestamp> name that
        # only means anything if someone manually renames it afterward --
        # no per-station counter in the name, each capture is
        # distinguished by its own timestamp alone. Blank project_name
        # (the default) falls back to today's plain scan_<timestamp>
        # naming unchanged -- this is additive, not a forced workflow
        # change. The GUI drives this via a dropdown of previously-used
        # project names (derived from existing filenames, see
        # index.html's refreshProjectDropdown -- no separate "known
        # projects" list is persisted here) plus a "New Project" button,
        # rather than a free-typed field on every run.
        self.declare_parameter("project_name", _default("project_name", ""))
        # float(...) around every _default() call below that's meant to be
        # a DOUBLE parameter: JSON doesn't distinguish 0 from 0.0, so a
        # whole-number value previously persisted by _save_settings_section
        # loads back as a Python int, and declare_parameter() locks the
        # parameter's type to whatever Python type its default arg has --
        # silently declaring it INTEGER instead of DOUBLE. Confirmed this
        # session as a real, node-killing bug: scan_aggregator crashed at
        # startup with InvalidParameterTypeException because sweep_min_deg
        # had been persisted as bare `0`, got declared INTEGER, and then
        # collided with this launch's own params.yaml override (a real
        # 0.0/DOUBLE). float() here makes the declared type always DOUBLE
        # regardless of what shape happens to be sitting in the settings
        # file. sweep_speed_rpm/sweep_accel/preview_max_points are
        # genuinely integer params and are deliberately left alone.
        # 5-185 deg with a few degrees of margin off each end-stop --
        # this rig's actual confirmed-working range with the real homing
        # direction/offset now known (see mks_driver's set_home_params,
        # applied in tilt_axis_bridge -- home_direction was the fix for a
        # real stall, see HANDOFF.md).
        self.declare_parameter("tilt_start_deg", float(_default("tilt_start_deg", 5.0)))
        self.declare_parameter("tilt_end_deg", float(_default("tilt_end_deg", 185.0)))
        self.declare_parameter("step_deg", float(_default("step_deg", 0.65)))
        self.declare_parameter("rotation_rate_hz", float(_default("rotation_rate_hz", 10.0)))
        self.declare_parameter(
            "revolutions_per_stop", float(_default("revolutions_per_stop", 2.0))
        )
        self.declare_parameter("settle_extra_s", float(_default("settle_extra_s", 0.0)))
        self.declare_parameter("homing_timeout_s", float(_default("homing_timeout_s", 60.0)))
        self.declare_parameter("move_timeout_s", float(_default("move_timeout_s", 30.0)))
        # Keep sweep_min_deg >= 0: home (0 deg) sits close to the mechanical
        # hard stop (see tilt_start_deg's 5 deg margin above), and a negative
        # bound drives the arm into that physical limit instead of a
        # controlled PID stop -- confirmed a much harder halt at that end
        # than at the safe end.
        self.declare_parameter("sweep_min_deg", float(_default("sweep_min_deg", 5.0)))
        self.declare_parameter("sweep_max_deg", float(_default("sweep_max_deg", 205.0)))
        self.declare_parameter(
            "sweep_speed_rpm", _default("sweep_speed_rpm", 1)
        )  # see MksDriver.MAX_ALLOWED_RPM -- this rig's own real sweep speed
        self.declare_parameter("sweep_accel", _default("sweep_accel", 2))
        self.declare_parameter(
            "sweep_duration_s", float(_default("sweep_duration_s", 35.0))
        )  # 0 = run until stopped; 35s is this rig's own real default
        # Near each turnaround the axis is changing direction (decelerating
        # into it, re-accelerating out), so clouds captured there are the
        # most likely to carry extra smear/jitter beyond the smear that's
        # already inherent to sweep mode. 0 disables discarding.
        self.declare_parameter(
            "sweep_edge_margin_deg", float(_default("sweep_edge_margin_deg", 10.0))
        )
        # Off by default -- new, not yet field-proven the way the plain
        # sweep path is. When on, MODE_SWEEP rotates each point by its own
        # individually-interpolated tilt angle (see
        # _try_transform_and_accumulate_deskewed) instead of one shared
        # angle per whole cloud message. Inert for step-and-stare (already
        # stationary during capture, nothing to deskew) and for
        # MODE_CALIBRATE (_accumulate_calibration_cloud never looks at
        # this parameter at all).
        self.declare_parameter(
            "enable_sweep_deskew", _default("enable_sweep_deskew", False)
        )
        self.declare_parameter("preview_enabled", _default("preview_enabled", True))
        self.declare_parameter(
            "preview_publish_period_s", float(_default("preview_publish_period_s", 1.0))
        )
        # Fraction of points to keep, applied unconditionally via a fixed
        # stride (not just once preview_max_points is hit) -- a live
        # preview is for getting a look at coverage/shape while a scan
        # runs, not a faithful render, so it's fine to always be coarser
        # than the final .e57. 1.0 = full density, disables this stride.
        self.declare_parameter("preview_decimation", float(_default("preview_decimation", 0.5)))
        # Extra safety cap on top of preview_decimation, for when even the
        # decimated count is still large on a long scan. 0 disables the cap.
        self.declare_parameter("preview_max_points", _default("preview_max_points", 500000))
        # Live coverage map (~/coverage) -- how often it's republished
        # while a scan runs. Cheap (a few hundred ints as JSON at most),
        # so this can run faster than the preview without real cost.
        self.declare_parameter(
            "coverage_publish_period_s", float(_default("coverage_publish_period_s", 0.5))
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        tilt_node = self.get_parameter("tilt_node_name").value
        pointcloud_topic = self.get_parameter("pointcloud_topic").value
        vlp16_node = self.get_parameter("vlp16_node_name").value

        self._state = STATE_IDLE
        self._mode: str | None = None
        self._targets_rad: list[float] = []
        self._target_idx = 0
        # "" is a deliberate sentinel distinct from every real status
        # tilt_axis_bridge ever publishes ("disconnected"/"idle"/"homing"/
        # "moving"/"settling"/"settled"/"stalled"), used by
        # _invalidate_tilt_status -- see that method's own docstring.
        self._tilt_status = "disconnected"
        # Raw ~/status passthrough from vlp16_config, i.e. an unparsed
        # /cgi/status.json response (see that node's own module docstring
        # -- its exact field schema isn't confirmed against the real
        # sensor yet). Cached for E57 export metadata (_on_export_e57_request)
        # rather than parsed into named fields this module can't yet
        # verify the names of.
        self._vlp16_status_json: str | None = None
        self._phase_deadline = 0.0
        self._capture_deadline = 0.0
        # None until the first non-edge-margin cloud of a sweep is actually
        # captured -- see _on_pointcloud/_sweep_data_started. sweep_duration_s
        # counts from real data starting to be gathered, not from when
        # sweep motion starts (homing was already excluded before this;
        # the initial transit from home to sweep_min_rad, which produces no
        # usable data since it's within the edge margin, was not).
        self._sweep_deadline: float | None = None
        self._sweep_data_started = False
        self._capture_buffer: list[PointCloud2] = []
        # Clouds whose first tf lookup failed -- retried once per tick
        # (see _retry_pending_transforms) rather than dropped immediately.
        # Each entry is (cloud_msg, time of first failed attempt, the
        # _try_transform_and_accumulate* variant it was submitted through
        # -- so a retry uses the same one, not whatever enable_sweep_deskew
        # happens to say by the time the retry actually runs).
        self._pending_transforms: list[
            tuple[PointCloud2, float, Callable[[PointCloud2], "Exception | None"]]
        ] = []
        self._merged_points: list[np.ndarray] = []
        # MODE_CALIBRATE's own accumulators -- raw (still velodyne-frame)
        # x/y/z + intensity + the tilt reading at capture time, same shape
        # scripts/calibration/capture_raw_for_mount_calibration.py already
        # produces. Kept entirely separate from _merged_points: calibration
        # skips _transform_and_accumulate/tf2 completely (see _on_pointcloud)
        # since mount_calibration.py needs to redo that transform itself
        # under candidate roll/pitch values, not once with whatever's
        # currently configured.
        self._calib_points: list[np.ndarray] = []
        self._calib_intensity: list[np.ndarray] = []
        self._calib_tilt: list[np.ndarray] = []
        self._stops_done = 0
        self._error: str | None = None
        self._last_output_path: str | None = None
        self._dropped_edge_clouds = 0
        self._last_preview_publish = 0.0
        # Set at the start of every scan (see _start_scan_impl/
        # _start_sweep_scan_impl) -- read only by _publish_status, to word
        # a completed run's done text so it never matches index.html's
        # rename-popup trigger for a scan the front panel started.
        self._started_from_panel = False
        # Same lifecycle as _started_from_panel just above -- see
        # _start_scan_impl/_start_sweep_scan_impl for where it's really
        # set; None here only matters if _finish_run were ever somehow
        # reached without a prior start (shouldn't happen given the
        # STATE_IDLE/DONE/ABORTED guard both impls already have).
        self._run_start_time: float | None = None

        # Live coverage map -- one bin count per tilt slice, incremented
        # each time a cloud lands in that slice during a real scan (see
        # _record_coverage). (Re-)sized at the start of every scan by
        # _init_coverage, never during MODE_CALIBRATE (see
        # _on_start_mount_calibration, which never calls it) -- so a
        # calibration run just leaves the previous real scan's map
        # untouched rather than clearing or polluting it, since it's not
        # itself something this map is meant to represent.
        self._coverage_start_deg = 0.0
        self._coverage_bin_deg = 1.0
        self._coverage_counts: np.ndarray = np.zeros(0, dtype=np.int32)
        self._last_coverage_publish = 0.0

        # Updated from tilt_axis_bridge's own joint_state as it arrives
        # (~10 Hz real samples, republished at joint_state_rate_hz); used
        # to decide, at the coarse "within N degrees of a turnaround"
        # granularity that feature needs, whether to discard a just-
        # arrived sweep cloud.
        self._current_tilt_rad = 0.0
        self._sweep_min_rad = 0.0
        self._sweep_max_rad = 0.0
        # Rolling (stamp_sec, tilt_rad) history for
        # _try_transform_and_accumulate_deskewed's own per-point angle
        # interpolation when enable_sweep_deskew is on -- see that
        # function and JOINT_STATE_HISTORY_MAXLEN. Kept regardless of
        # whether deskewing is actually enabled (cheap, a plain append
        # each callback) so turning the parameter on mid-session doesn't
        # start with an empty buffer.
        self._joint_state_history: deque[tuple[float, float]] = deque(
            maxlen=JOINT_STATE_HISTORY_MAXLEN
        )

        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._status_pub = self.create_publisher(String, "~/status", 10)
        self._cmd_position_pub = self.create_publisher(Float64, f"{tilt_node}/cmd_position", 10)
        self._sweep_enable_pub = self.create_publisher(Bool, f"{tilt_node}/sweep_enable", 10)
        # Request/response over a topic pair rather than a service, matching
        # tilt_axis_bridge's driver_command/driver_response pattern -- avoids
        # needing a custom .srv interface package just for "list this dir".
        self._list_dir_response_pub = self.create_publisher(String, "~/list_dir_response", 10)
        self._rename_output_response_pub = self.create_publisher(
            String, "~/rename_output_response", 10
        )
        # Backs the new "Scans" GUI tab (local-first save -> browse/download/
        # export/delete) -- same request/response-over-topic pattern as
        # list_dir_request/rename_output_request above.
        self._list_local_scans_response_pub = self.create_publisher(
            String, "~/list_local_scans_response", 10
        )
        self._export_to_usb_response_pub = self.create_publisher(
            String, "~/export_to_usb_response", 10
        )
        # Separate from the response: an export can move multiple large
        # (multi-hundred-MB+) files at this rig's own measured USB write
        # speed (~12 MB/s), so this is published repeatedly while one
        # export_to_usb_request is in flight, not just once at the end --
        # same "make a slow operation visible" philosophy as ~/status
        # during STATE_SAVING.
        self._export_to_usb_progress_pub = self.create_publisher(
            String, "~/export_to_usb_progress", 10
        )
        self._delete_local_scan_response_pub = self.create_publisher(
            String, "~/delete_local_scan_response", 10
        )
        # Converts an already-saved local .pcd to .e57 on request (see
        # e57_writer.py for the format itself and why it's hand-rolled) --
        # a legacy-file migration tool now, not part of the normal save
        # path any more (every scan is saved natively as .e57, see
        # _finish_run/HANDOFF.md), kept reachable for .pcd files that
        # already existed before that change. Same request/response
        # pattern as export_to_usb above, including a background thread,
        # though in practice a single conversion is fast enough (~7s for
        # a real 15M-point scan, measured) that no progress topic was
        # added for this one -- export_to_usb's slow part is USB write
        # speed (~12 MB/s measured), which this doesn't share since it
        # writes back into OUTPUT_DIR (local, fast) same as the original
        # .pcd.
        self._export_e57_response_pub = self.create_publisher(
            String, "~/export_e57_response", 10
        )
        # Bundles every locally-saved file for one project into a single
        # .zip -- see project_name's own declare_parameter comment and
        # _on_bundle_project_request. Same request/response pattern as
        # export_e57 above (background thread, no separate progress topic
        # -- zipping several already-local files is fast enough not to
        # need one, same reasoning as export_e57's own).
        self._bundle_project_response_pub = self.create_publisher(
            String, "~/bundle_project_response", 10
        )
        # One-shot result of a ~/start_mount_calibration run -- see
        # _finish_mount_calibration.
        self._mount_calibration_response_pub = self.create_publisher(
            String, "~/mount_calibration_response", 10
        )
        # depth 1: only the latest accumulated cloud matters, a viewer that
        # missed one publish just picks up the next (larger) one.
        self._preview_pub = self.create_publisher(PointCloud2, "~/preview_points", 1)
        self._coverage_pub = self.create_publisher(String, "~/coverage", 10)

        self.create_subscription(String, f"{tilt_node}/status", self._on_tilt_status, 10)
        self.create_subscription(String, f"{vlp16_node}/status", self._on_vlp16_status, 10)
        self.create_subscription(JointState, f"{tilt_node}/joint_state", self._on_joint_state, 10)
        self.create_subscription(PointCloud2, pointcloud_topic, self._on_pointcloud, 10)
        self.create_subscription(String, "~/list_dir_request", self._on_list_dir_request, 10)
        self.create_subscription(
            String, "~/rename_output_request", self._on_rename_output_request, 10
        )
        self.create_subscription(
            String, "~/list_local_scans_request", self._on_list_local_scans_request, 10
        )
        self.create_subscription(
            String, "~/export_to_usb_request", self._on_export_to_usb_request, 10
        )
        self.create_subscription(
            String, "~/delete_local_scan_request", self._on_delete_local_scan_request, 10
        )
        self.create_subscription(
            String, "~/export_e57_request", self._on_export_e57_request, 10
        )
        self.create_subscription(
            String, "~/bundle_project_request", self._on_bundle_project_request, 10
        )

        self._home_client = self.create_client(Trigger, f"{tilt_node}/home")
        self._tilt_stop_client = self.create_client(Trigger, f"{tilt_node}/stop")
        self._tilt_set_params_client = self.create_client(
            SetParameters, f"{tilt_node}/set_parameters"
        )
        # Mount calibration reads the current mount_roll_deg/mount_pitch_deg
        # from vlp16_config to use as its search starting point -- see
        # _on_start_mount_calibration. It does NOT write the result back:
        # the GUI's own "Apply" button does that, via the same generic
        # set_parameters call it already uses for every other vlp16_config
        # field -- deliberately not auto-applied, see the GUI-side comment
        # on that button for why.
        self._vlp16_get_params_client = self.create_client(
            GetParameters, f"{vlp16_node}/get_parameters"
        )

        self.create_service(Trigger, "~/start_scan", self._on_start_scan)
        self.create_service(Trigger, "~/start_sweep_scan", self._on_start_sweep_scan)
        # Front-panel (status.html) equivalents of the two above -- see
        # _on_start_scan_from_panel's own comment for why these exist as
        # separate services rather than a flag on the same one (std_srvs/
        # Trigger's request carries no fields to flag with).
        self.create_service(
            Trigger, "~/start_scan_from_panel", self._on_start_scan_from_panel
        )
        self.create_service(
            Trigger, "~/start_sweep_scan_from_panel", self._on_start_sweep_scan_from_panel
        )
        self.create_service(Trigger, "~/start_mount_calibration", self._on_start_mount_calibration)
        self.create_service(Trigger, "~/stop_scan", self._on_stop_scan)

        self.create_timer(0.1, self._tick)
        self.get_logger().info("scan_aggregator ready")

    # ---- ROS callbacks ----

    def _on_set_parameters(self, params) -> SetParametersResult:
        for p in params:
            self._save_settings_section("scan_aggregator", p.name, p.value)
        return SetParametersResult(successful=True)

    def _save_settings_section(self, section: str, key: str, value) -> None:
        """Pairs with _load_settings_section -- see SETTINGS_PATH.
        Read-modify-write so tilt_axis_bridge's sections of the same file
        (written independently, possibly around the same time) aren't
        clobbered. Best-effort: a failed write is logged but doesn't fail
        the parameter change itself."""
        try:
            try:
                with open(SETTINGS_PATH) as f:
                    settings = json.load(f)
            except (OSError, ValueError):
                settings = {}
            settings.setdefault(section, {})[key] = value
            with open(SETTINGS_PATH, "w") as f:
                json.dump(settings, f)
        except OSError as exc:
            self.get_logger().warning(f"Failed to persist {section}.{key}: {exc}")

    def _on_tilt_status(self, msg: String) -> None:
        self._tilt_status = msg.data

    def _invalidate_tilt_status(self) -> None:
        """Call this immediately after issuing any command (home, or a new
        cmd_position target) whose completion _tick's state machine will
        later confirm by checking self._tilt_status == "settled".

        Real bug this fixes (found live, 2026-09-11): self._tilt_status is
        just whatever tilt_axis_bridge's own ~/status last said, cached
        here via _on_tilt_status -- it's NOT re-validated against "has a
        fresh status update actually arrived since the command I just
        issued". A run finishes with the axis settled, so self._tilt_status
        is already "settled" going into the next command; the home/move
        service call is async (tilt_axis_bridge's own next tick is what
        actually starts real motion, not this call), so there's a real
        window -- up to a full tilt_axis_bridge tick, observed live as
        long enough to matter -- where _tick here can run first and see
        that stale "settled" before any fresh status reflecting the new
        command has arrived, concluding the wait is already over before
        the axis has even started moving. For STATE_SWEEP_HOMING this
        meant _start_sweep() fired immediately: the axis was still
        genuinely homing (visible motion, tilt_axis_bridge's own state
        machine unaffected), but scan_aggregator had already moved on to
        STATE_SWEEP_SCANNING and started accepting clouds as real sweep
        data -- no "homing"/"sweep_homing" ever visible in the GUI (the
        transition happened within one ~100ms tick), and the coverage
        map/preview lighting up as if real data was arriving during what
        was still, physically, the homing move. Same race applies to
        every STATE_MOVING wait too (step-and-stare's inter-stop moves),
        just less visible there since a genuinely early
        STATE_SETTLING_EXTRA still waits settle_extra_s before capturing,
        which usually (not always -- a big step could still lose real
        settle time) covers for it.

        "" is guaranteed to never equal "settled" (or any other real
        status string), so the next _tick after a command is issued can't
        mistake old news for confirmation -- it has to wait for a
        genuinely fresh /tilt_axis_bridge/status message to arrive."""
        self._tilt_status = ""

    def _on_vlp16_status(self, msg: String) -> None:
        self._vlp16_status_json = msg.data

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            idx = msg.name.index("tilt_axis")
        except ValueError:
            return
        self._current_tilt_rad = msg.position[idx]
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._joint_state_history.append((stamp_sec, self._current_tilt_rad))

    def _in_sweep_edge_margin(self) -> bool:
        margin_rad = deg_to_rad(float(self.get_parameter("sweep_edge_margin_deg").value))
        if margin_rad <= 0.0:
            return False
        pos = self._current_tilt_rad
        return pos <= self._sweep_min_rad + margin_rad or pos >= self._sweep_max_rad - margin_rad

    def _init_coverage(self, bin_deg: float) -> None:
        """(Re)starts the live coverage map for a new real scan -- see the
        map's own state comment in __init__. bin_deg is step_deg itself
        for step-and-stare (one bin per stop at that resolution, an exact
        match with no rounding slop) or SWEEP_COVERAGE_BIN_DEG for sweep
        (no equally natural bin size there).

        Always spans the full 0-360deg compass now, not just the scan's
        own configured tilt range -- see _record_coverage for why: the
        VLP-16 spins a full revolution on every single capture regardless
        of where the external tilt axis has it pointed, so a cloud
        captured at tilt position X also genuinely contains real points
        180deg around the compass from X, not just at X itself (found
        2026-09-11, real user-reported case: a sweep from 5-195deg with a
        5deg edge margin -- an effective 180deg span -- was reported to
        genuinely produce data across the full 360deg, not just the
        configured range, and the coverage map was silently discarding
        that other half by only ever sizing/recording within the
        configured range)."""
        self._coverage_start_deg = 0.0
        self._coverage_bin_deg = bin_deg
        n_bins = max(1, round(360.0 / bin_deg))
        self._coverage_counts = np.zeros(n_bins, dtype=np.int32)
        self._last_coverage_publish = 0.0

    def _record_coverage(self, tilt_rad: float) -> None:
        """Records a capture at the tilt axis's own real position, AND at
        its mirror 180deg around the compass -- the VLP-16's own spin
        covers both simultaneously on every single revolution (its 360deg
        azimuth FOV has no "front" or "back", it always sees all the way
        around itself), so a cloud captured while the tilt axis points at
        X is real data at X's mirror too, not a guess. This assumes the
        VLP-16's own azimuth FOV is left at its default, uncropped 360deg
        (the View width field on the VLP-16 tab, default/hint value) --
        a deliberately narrowed FOV crop there isn't modeled here, since
        that's a much rarer configuration and checking it would mean
        reading vlp16_config's own live parameters on every capture."""
        if self._coverage_counts.size == 0:
            return
        n = self._coverage_counts.size
        deg = rad_to_deg(tilt_rad) % 360.0
        idx = int(round(deg / self._coverage_bin_deg)) % n
        self._coverage_counts[idx] += 1
        mirror_deg = (deg + 180.0) % 360.0
        mirror_idx = int(round(mirror_deg / self._coverage_bin_deg)) % n
        self._coverage_counts[mirror_idx] += 1

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        if self._state == STATE_CAPTURING:
            self._capture_buffer.append(msg)
            self._record_coverage(self._current_tilt_rad)
        elif self._state == STATE_SWEEP_SCANNING:
            if self._in_sweep_edge_margin():
                self._dropped_edge_clouds += 1
                return
            if not self._sweep_data_started:
                # First cloud that actually counts as real data (past the
                # initial home->sweep_min transit, which is always within
                # the edge margin) -- start sweep_duration_s's countdown
                # from here rather than from when sweep motion began, so
                # a requested duration means duration of actual coverage.
                self._sweep_data_started = True
                duration = float(self.get_parameter("sweep_duration_s").value)
                self._sweep_deadline = self._now() + duration if duration > 0.0 else None
            if self._mode == MODE_CALIBRATE:
                self._accumulate_calibration_cloud(msg)
            else:
                # No fixed capture window in this mode -- transform and
                # fold in each cloud as it arrives, rather than buffering
                # raw messages, since a sweep has no natural end to flush
                # a buffer at.
                self._transform_and_accumulate(msg)
                self._record_coverage(self._current_tilt_rad)

    def _accumulate_calibration_cloud(self, msg: PointCloud2) -> None:
        """MODE_CALIBRATE's own per-cloud handling -- deliberately does
        NOT call _transform_and_accumulate (that path needs a tf2 lookup
        and folds straight into _merged_points using whatever mount angles
        are *currently* configured; mount_calibration.py instead needs the
        raw, still-velodyne-frame points plus the tilt reading at capture
        time, exactly like scripts/calibration/
        capture_raw_for_mount_calibration.py already captures, so it can
        redo that transform itself under many candidate angles).

        Decimated by CALIBRATION_POINT_STRIDE -- found live, against a real
        capture on this rig, that a full-density ~40s sweep is several
        million points, and every downstream step (transform, RANSAC,
        refine/trim, the final optimizer) scales with that count. A plane
        fit doesn't need every point a real wall reflects -- this module's
        own synthetic tests fit exactly as precisely from ~10-20k points as
        from millions -- so cutting density at the source (a plain fixed-
        stride slice, cheap and still evenly spread across each cloud's own
        azimuth/elevation ordering) shrinks every later step proportionally,
        not just the RANSAC search find_best_plane already bounds on its
        own."""
        structured = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        if not structured.size:
            return
        arr = rfn.structured_to_unstructured(structured, dtype=np.float32)[::CALIBRATION_POINT_STRIDE]
        self._calib_points.append(arr[:, :3])
        self._calib_intensity.append(arr[:, 3])
        self._calib_tilt.append(
            np.full(arr.shape[0], self._current_tilt_rad, dtype=np.float64)
        )

    def _on_start_scan(self, request: Trigger.Request, response: Trigger.Response):
        return self._start_scan_impl(response, from_panel=False)

    def _on_start_scan_from_panel(self, request: Trigger.Request, response: Trigger.Response):
        """Identical to ~/start_scan -- see _start_scan_impl -- except the
        run it starts is flagged as front-panel-originated, so its done
        status is worded to never trigger index.html's post-scan rename
        popup (see _publish_status): the kiosk display (status.html) has
        no such dialog at all, so a scan started there should just save
        under its default name, full stop, not pop a naming prompt on
        some *other* GUI that happens to be connected at the time."""
        return self._start_scan_impl(response, from_panel=True)

    def _start_scan_impl(self, response: Trigger.Response, from_panel: bool) -> Trigger.Response:
        if self._state not in (STATE_IDLE, STATE_DONE, STATE_ABORTED):
            response.success = False
            response.message = f"already running (state={self._state})"
            return response

        start_deg = float(self.get_parameter("tilt_start_deg").value)
        end_deg = float(self.get_parameter("tilt_end_deg").value)
        step_deg = float(self.get_parameter("step_deg").value)
        if step_deg <= 0.0 or end_deg <= start_deg:
            response.success = False
            response.message = "invalid tilt_start_deg/tilt_end_deg/step_deg"
            return response

        steps = int(round((end_deg - start_deg) / step_deg))
        self._targets_rad = [deg_to_rad(start_deg + i * step_deg) for i in range(steps + 1)]
        self._target_idx = 0
        self._merged_points = []
        self._pending_transforms = []
        self._stops_done = 0
        self._error = None
        self._last_output_path = None
        self._last_preview_publish = 0.0
        self._init_coverage(step_deg)
        self._mode = MODE_STEP_AND_STARE
        self._started_from_panel = from_panel
        # Real wall-clock capture start, for the automatically-saved E57's
        # own acquisitionStart -- see _finish_run/_assemble_e57_metadata.
        # time.time() (wall clock), not self._now() (time.monotonic(),
        # only meaningful for this node's own internal deadline math).
        self._run_start_time = time.time()

        if not self._home_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "tilt_axis_bridge ~/home service not available"
            return response
        self._invalidate_tilt_status()
        self._home_client.call_async(Trigger.Request())
        self._state = STATE_HOMING
        self._phase_deadline = self._now() + float(self.get_parameter("homing_timeout_s").value)

        response.success = True
        response.message = f"scan started: {len(self._targets_rad)} stops"
        return response

    def _on_start_sweep_scan(self, request: Trigger.Request, response: Trigger.Response):
        return self._start_sweep_scan_impl(response, from_panel=False)

    def _on_start_sweep_scan_from_panel(self, request: Trigger.Request, response: Trigger.Response):
        """See _on_start_scan_from_panel's own comment -- identical
        reasoning, sweep mode."""
        return self._start_sweep_scan_impl(response, from_panel=True)

    def _start_sweep_scan_impl(self, response: Trigger.Response, from_panel: bool) -> Trigger.Response:
        if self._state not in (STATE_IDLE, STATE_DONE, STATE_ABORTED):
            response.success = False
            response.message = f"already running (state={self._state})"
            return response

        min_deg = float(self.get_parameter("sweep_min_deg").value)
        max_deg = float(self.get_parameter("sweep_max_deg").value)
        if max_deg <= min_deg:
            response.success = False
            response.message = "invalid sweep_min_deg/sweep_max_deg"
            return response

        self._merged_points = []
        self._pending_transforms = []
        self._stops_done = 0
        self._error = None
        self._last_output_path = None
        self._last_preview_publish = 0.0
        self._dropped_edge_clouds = 0
        self._init_coverage(SWEEP_COVERAGE_BIN_DEG)
        self._mode = MODE_SWEEP
        self._started_from_panel = from_panel
        self._run_start_time = time.time()

        if not self._home_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "tilt_axis_bridge ~/home service not available"
            return response
        self._invalidate_tilt_status()
        self._home_client.call_async(Trigger.Request())
        self._state = STATE_SWEEP_HOMING
        self._phase_deadline = self._now() + float(self.get_parameter("homing_timeout_s").value)

        response.success = True
        response.message = "sweep scan started"
        return response

    def _on_start_mount_calibration(self, request: Trigger.Request, response: Trigger.Response):
        """Reuses the exact same sweep motion (home, then sweep across
        sweep_min_deg/sweep_max_deg) as _on_start_sweep_scan above -- a
        wide range is what gives real leverage on roll/pitch (see
        mount_calibration.py), and this project already has a perfectly
        good "sweep across a wide range" primitive, no new motion params
        needed. Only _mode differs, which _on_pointcloud/_finish_run
        branch on to capture raw points instead of merging a scan output."""
        if self._state not in (STATE_IDLE, STATE_DONE, STATE_ABORTED):
            response.success = False
            response.message = f"already running (state={self._state})"
            return response

        min_deg = float(self.get_parameter("sweep_min_deg").value)
        max_deg = float(self.get_parameter("sweep_max_deg").value)
        if max_deg <= min_deg:
            response.success = False
            response.message = "invalid sweep_min_deg/sweep_max_deg"
            return response

        self._calib_points = []
        self._calib_intensity = []
        self._calib_tilt = []
        self._pending_transforms = []
        self._error = None
        self._last_preview_publish = 0.0
        self._dropped_edge_clouds = 0
        self._mode = MODE_CALIBRATE

        if not self._home_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "tilt_axis_bridge ~/home service not available"
            return response
        self._invalidate_tilt_status()
        self._home_client.call_async(Trigger.Request())
        self._state = STATE_SWEEP_HOMING
        self._phase_deadline = self._now() + float(self.get_parameter("homing_timeout_s").value)

        response.success = True
        response.message = "mount calibration started"
        return response

    def _on_stop_scan(self, request: Trigger.Request, response: Trigger.Response):
        # Used to always _abort() here, discarding every point captured so
        # far with no file ever written and no rename-popup opportunity --
        # a manual Stop mid-run lost everything. Flush whatever's sitting
        # in the current stop's not-yet-transformed capture buffer (sweep
        # mode has no such buffer -- points are transformed and
        # accumulated as each cloud arrives, so there's nothing to flush
        # there) and hand off to _finish_run(), the same path a normal
        # completed run takes: writes a .e57 from whatever's in
        # _merged_points and publishes the same "done: ... -> path" status
        # the GUI's rename popup already watches for -- so a manually
        # stopped scan now gets the same naming opportunity a completed
        # one does. _finish_run() itself still falls back to _abort() if
        # truly nothing was captured yet (e.g. stopped during homing).
        for cloud_msg in self._capture_buffer:
            self._transform_and_accumulate(cloud_msg)
        self._capture_buffer = []
        self._finish_run()

        if self._tilt_stop_client.wait_for_service(timeout_sec=1.0):
            self._tilt_stop_client.call_async(Trigger.Request())
        response.success = True
        response.message = "stopped"
        return response

    def _on_list_dir_request(self, msg: String) -> None:
        """Backs the GUI's settings-file browser (Save Settings As / Load
        Settings), which always passes file_filter (e.g. ".json") to also
        list files ending in it alongside subdirectories. Also supports
        creating+entering a new folder in one round trip, for the "Save
        Settings As" case."""
        try:
            request = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self._publish_list_dir_response(None, None, None, [], None, f"bad request: {exc}")
            return

        req_id = request.get("id")
        path = os.path.abspath(os.path.expanduser(request.get("path") or "~"))
        new_folder = request.get("new_folder")
        file_filter = request.get("file_filter")

        try:
            if new_folder:
                path = os.path.join(path, new_folder)
                os.makedirs(path, exist_ok=True)
            if not os.path.isdir(path):
                path = os.path.expanduser("~")
            entries = []
            files = [] if file_filter else None
            for e in os.scandir(path):
                if e.name.startswith("."):
                    continue
                try:
                    # One inaccessible entry (permission-denied system
                    # folders are common under Windows profile dirs browsed
                    # via WSL2's /mnt/c) used to raise inside the generator
                    # and abort the whole listing -- skip just that entry
                    # instead, follow_symlinks default (True) so NTFS
                    # junctions/symlinked folders (also common there) still
                    # show up as browsable.
                    if e.is_dir():
                        entries.append(e.name)
                    elif file_filter and e.is_file() and e.name.endswith(file_filter):
                        files.append(e.name)
                except OSError:
                    continue
            entries.sort(key=str.casefold)
            if files is not None:
                files.sort(key=str.casefold)
            parent = os.path.dirname(path) if path != os.path.sep else None
            self._publish_list_dir_response(req_id, path, parent, entries, files, None)
        except OSError as exc:
            self._publish_list_dir_response(req_id, path, None, [], None, str(exc))

    def _publish_list_dir_response(
        self,
        req_id,
        path: str | None,
        parent: str | None,
        entries: list,
        files: list | None,
        error: str | None,
    ) -> None:
        msg = String()
        payload = {"id": req_id, "path": path, "parent": parent, "entries": entries, "error": error}
        if files is not None:
            payload["files"] = files
        msg.data = json.dumps(payload)
        self._list_dir_response_pub.publish(msg)

    def _on_rename_output_request(self, msg: String) -> None:
        """Backs the GUI's post-scan naming popup. The file is already on
        disk under its auto-generated name by the time this can be called
        (_finish_run hands the write off to a background thread as soon as
        a run completes -- see that method -- and the GUI only ever opens
        this popup in response to the "done: ... -> path" status text that
        thread publishes once its write has actually finished) -- this
        just renames it in place, rather than deferring the write until a
        name is confirmed, so a closed browser tab/dropped connection before
        naming never loses scan data. An empty name is a deliberate no-op
        (not an error): the popup's "leave blank to keep default" case
        doesn't need a request at all, but routing it through here too keeps
        the GUI-side logic simple."""
        try:
            request = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self._publish_rename_output_response(None, None, f"bad request: {exc}")
            return

        req_id = request.get("id")
        new_name = (request.get("name") or "").strip()

        if not self._last_output_path or not os.path.isfile(self._last_output_path):
            self._publish_rename_output_response(
                req_id, self._last_output_path, "no completed scan output to rename"
            )
            return

        if not new_name:
            self._publish_rename_output_response(req_id, self._last_output_path, None)
            return

        # basename only -- a typed-in name can't relocate the file via a
        # path traversal, it can only rename it within its own directory.
        # Extension taken from the actual file being renamed, not
        # hardcoded -- this used to always force ".pcd" back when that
        # was the only format _finish_run ever produced; now that every
        # scan saves natively as .e57 (see HANDOFF.md), hardcoding would
        # have silently mis-renamed every completed run's real .e57 file
        # into a wrongly-suffixed "<name>.pcd" that doesn't actually
        # exist in that format.
        new_name = os.path.basename(new_name)
        _, real_ext = os.path.splitext(self._last_output_path)
        if not new_name.lower().endswith(real_ext.lower()):
            new_name += real_ext

        new_path = os.path.join(os.path.dirname(self._last_output_path), new_name)
        if os.path.exists(new_path):
            self._publish_rename_output_response(
                req_id, self._last_output_path, f"{new_name} already exists"
            )
            return

        try:
            os.rename(self._last_output_path, new_path)
            # rename() only updates the directory entry in the OS's cache --
            # like write_pcd() before its own fsync fix (see pcd_writer.py
            # and HANDOFF.md), a successful rename() is not itself a
            # durability guarantee. The file's *contents* were already
            # fsync'd when it was written; what's missing here is the
            # *directory* entry pointing at the new name, which needs its
            # own fsync on the containing directory's own file descriptor
            # -- fsync'ing the renamed file itself doesn't cover this,
            # directory metadata is a distinct object from file data.
            # Confirmed the hard way: a user unplugged a USB drive right
            # after this GUI action reported success, and the file was
            # intact but had silently reverted to its original name --
            # the rename had never actually reached the physical device.
            dir_fd = os.open(os.path.dirname(new_path), os.O_RDONLY)
            try:
                os.fsync(dir_fd)
            finally:
                os.close(dir_fd)
        except OSError as exc:
            self._publish_rename_output_response(req_id, self._last_output_path, str(exc))
            return

        self._last_output_path = new_path
        self._publish_rename_output_response(req_id, new_path, None)

    def _publish_rename_output_response(self, req_id, path: str | None, error: str | None) -> None:
        msg = String()
        msg.data = json.dumps({"id": req_id, "path": path, "error": error})
        self._rename_output_response_pub.publish(msg)

    def _on_list_local_scans_request(self, msg: String) -> None:
        """Backs the Scans tab's file list. Always lists OUTPUT_DIR (it's
        no longer a setting -- see that constant's own comment) rather than
        taking a path in the request, unlike list_dir_request's generic
        browser. Each entry's "exported" flag is just "a same-named file
        already exists at the USB export target" -- good enough for a
        badge, not a byte-for-byte guarantee, and deliberately doesn't
        block re-export (see _on_export_to_usb_request). Deliberately
        does NOT list .pcd -- every scan is saved natively as .e57 now
        (see _finish_run/HANDOFF.md), so a .pcd only exists at all if it
        predates that change, and this project's own policy since is
        that .pcd is no longer something the GUI offers to download or
        export at all, full stop -- see _on_export_e57_request's own
        comment if one of those legacy files ever needs converting.
        .zip (a project bundle, see _on_bundle_project_request) is
        listed alongside .e57 so it can be downloaded/exported/deleted
        through the exact same generic, extension-agnostic paths every
        other listed file already uses."""
        try:
            request = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self._publish_list_local_scans_response(None, [], None, None, f"bad request: {exc}")
            return

        req_id = request.get("id")
        scans = []
        try:
            with os.scandir(OUTPUT_DIR) as entries:
                for entry in entries:
                    if not entry.name.lower().endswith((".e57", ".zip")) or not entry.is_file():
                        continue
                    stat = entry.stat()
                    exported_path = os.path.join(USB_EXPORT_DIR, entry.name)
                    scans.append(
                        {
                            "name": entry.name,
                            "size": stat.st_size,
                            "mtime": stat.st_mtime,
                            "exported": os.path.isfile(exported_path),
                        }
                    )
        except OSError as exc:
            self._publish_list_local_scans_response(req_id, [], None, None, str(exc))
            return
        scans.sort(key=lambda s: s["mtime"], reverse=True)

        local_free = shutil.disk_usage(OUTPUT_DIR).free
        usb_free = shutil.disk_usage(USB_EXPORT_DIR).free if os.path.isdir(USB_EXPORT_DIR) else None
        self._publish_list_local_scans_response(req_id, scans, local_free, usb_free, None)

    def _publish_list_local_scans_response(
        self, req_id, scans: list, local_free_bytes, usb_free_bytes, error: str | None
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "id": req_id,
                "scans": scans,
                "local_free_bytes": local_free_bytes,
                "usb_free_bytes": usb_free_bytes,
                "error": error,
            }
        )
        self._list_local_scans_response_pub.publish(msg)

    def _on_export_to_usb_request(self, msg: String) -> None:
        """Copies named scans from OUTPUT_DIR (local, fast) to USB_EXPORT_DIR
        (removable, slow -- ~12 MB/s measured on this rig) on request,
        rather than that ever being the live scan-completion path (see
        OUTPUT_DIR's comment). Runs on a background thread, same reasoning
        as _write_output_in_background: this can take minutes for large/
        multiple files and must not block this node's single-threaded
        executor. Names are basenames only, same path-traversal guard as
        _on_rename_output_request."""
        try:
            request = json.loads(msg.data)
            names = [os.path.basename(n) for n in request["names"]]
        except (TypeError, ValueError, KeyError) as exc:
            self._publish_export_to_usb_response(None, [], f"bad request: {exc}")
            return

        req_id = request.get("id")
        if not os.path.isdir(USB_EXPORT_DIR):
            self._publish_export_to_usb_response(
                req_id, [], f"no USB drive mounted at {USB_EXPORT_DIR}"
            )
            return

        threading.Thread(
            target=self._export_to_usb_in_background,
            args=(req_id, names),
            daemon=True,
        ).start()

    def _export_to_usb_in_background(self, req_id, names: list[str]) -> None:
        chunk_size = 4 * 1024 * 1024
        results = []
        for file_index, name in enumerate(names):
            src = os.path.join(OUTPUT_DIR, name)
            dst = os.path.join(USB_EXPORT_DIR, name)
            total = os.path.getsize(src) if os.path.isfile(src) else 0
            copied = 0
            try:
                if not os.path.isfile(src):
                    raise OSError(f"{name}: not found in {OUTPUT_DIR}")
                with open(src, "rb") as fsrc, open(dst, "wb") as fdst:
                    while True:
                        chunk = fsrc.read(chunk_size)
                        if not chunk:
                            break
                        fdst.write(chunk)
                        copied += len(chunk)
                        self._publish_export_to_usb_progress(
                            name, copied, total, file_index, len(names)
                        )
                    fdst.flush()
                    os.fsync(fdst.fileno())
                fsync_durable(dst)
                results.append({"name": name, "error": None})
            except OSError as exc:
                results.append({"name": name, "error": str(exc)})
        self._publish_export_to_usb_response(req_id, results, None)

    def _publish_export_to_usb_progress(
        self, name: str, copied: int, total: int, file_index: int, file_count: int
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "name": name,
                "bytes_copied": copied,
                "bytes_total": total,
                "file_index": file_index,
                "file_count": file_count,
            }
        )
        self._export_to_usb_progress_pub.publish(msg)

    def _publish_export_to_usb_response(self, req_id, results: list, error: str | None) -> None:
        msg = String()
        msg.data = json.dumps({"id": req_id, "results": results, "error": error})
        self._export_to_usb_response_pub.publish(msg)

    # Fetched from vlp16_config to embed as E57 provenance (both the
    # automatic per-scan save, see _finish_run, and the legacy on-demand
    # .pcd conversion below) -- separate from mount_calibration's own use
    # of the same client/params (that one NEEDS them as its search
    # starting point; this is best-effort metadata either way).
    _EXPORT_E57_MOUNT_PARAM_NAMES = (
        "mount_roll_deg", "mount_pitch_deg", "mount_yaw_deg",
        "mount_x", "mount_y", "mount_z",
    )

    def _fetch_mount_params_then(self, continuation) -> None:
        """Fetches vlp16_config's current mount-calibration parameters
        (best-effort E57 provenance) via the same async client/pattern
        _finish_mount_calibration uses for its own, stricter use of the
        identical service -- a service call's response only ever arrives
        via a callback on this executor thread, never by blocking a call
        from a background thread. `continuation(mount_params)` runs once
        that's back, with mount_params `None` if the service didn't
        respond in time -- non-critical here (unlike mount calibration's
        own use of it), so a miss still calls continuation rather than
        failing whatever's waiting on it."""
        if not self._vlp16_get_params_client.wait_for_service(timeout_sec=1.0):
            continuation(None)
            return

        request_params = GetParameters.Request(names=list(self._EXPORT_E57_MOUNT_PARAM_NAMES))
        future = self._vlp16_get_params_client.call_async(request_params)

        def on_done(f):
            try:
                values = f.result().values
                mount_params = {
                    param_name: value.double_value
                    for param_name, value in zip(self._EXPORT_E57_MOUNT_PARAM_NAMES, values)
                }
            except Exception:  # noqa: BLE001 -- non-critical, see this method's own docstring
                mount_params = None
            continuation(mount_params)

        future.add_done_callback(on_done)

    # Every scan_aggregator parameter worth recording as "what this run
    # was configured to do" -- deliberately everything relevant to both
    # modes at once (unused-for-this-run fields, e.g. sweep_* on a
    # step-and-stare save, are harmless to include) rather than trying to
    # guess which mode produced a given run from its name alone.
    _EXPORT_E57_SCAN_PARAM_NAMES = (
        "tilt_start_deg", "tilt_end_deg", "step_deg", "revolutions_per_stop",
        "settle_extra_s", "sweep_min_deg", "sweep_max_deg", "sweep_speed_rpm",
        "sweep_accel", "sweep_duration_s", "sweep_edge_margin_deg",
        "invert_x_axis", "invert_y_axis", "invert_z_axis", "output_frame",
    )

    def _assemble_e57_metadata(
        self,
        station_name: str,
        description_note: str,
        acquisition_start: float,
        acquisition_end: float,
        mount_params: dict | None,
    ) -> dict:
        """Gathers real provenance shared by every E57 this node ever
        writes: scan_aggregator's own current parameters, vlp16_config's
        current mount calibration (if it answered in time, see
        _fetch_mount_params_then), and vlp16_config's raw sensor status
        passthrough (cached from _on_vlp16_status). `description_note`
        and the two acquisition timestamps are the part that differs by
        caller -- see _finish_run (a real run's own true start/end times)
        vs the legacy .pcd-conversion path below (a source file's mtime,
        the best available proxy for something no longer actually being
        captured)."""
        scan_config = {
            param_name: self.get_parameter(param_name).value
            for param_name in self._EXPORT_E57_SCAN_PARAM_NAMES
        }

        extra_string_fields = {"tplScanConfigJson": json.dumps(scan_config)}
        if mount_params is not None:
            extra_string_fields["tplMountCalibrationJson"] = json.dumps(mount_params)
        if self._vlp16_status_json:
            extra_string_fields["tplVlp16StatusJson"] = self._vlp16_status_json

        description_lines = [
            "TPL (Terrestrial Panning Lidar) scan_aggregator output.",
            description_note,
        ]
        if mount_params is None:
            description_lines.append(
                "Mount calibration unavailable: vlp16_config's get_parameters "
                "service didn't respond in time."
            )

        return {
            "station_name": station_name,
            "description": " ".join(description_lines),
            "extra_string_fields": extra_string_fields,
            "acquisition_start_unix": acquisition_start,
            "acquisition_end_unix": acquisition_end,
        }

    # ---- Legacy .pcd -> .e57 conversion (no GUI button any more -- see
    # HANDOFF.md: every scan is now saved natively as .e57, see
    # _finish_run -- but kept reachable over ROS as a migration tool for
    # .pcd files that already existed on disk before that change, same
    # "escape hatch with no dedicated button" reasoning as e.g.
    # tilt_axis_bridge's ~/driver_command.) ----

    def _on_export_e57_request(self, msg: String) -> None:
        """Converts an already-saved local .pcd (named by basename, same
        path-traversal guard as _on_rename_output_request/
        _on_export_to_usb_request) into a sibling .e57 file in OUTPUT_DIR
        -- see e57_writer.py for the format itself and why it's hand-
        rolled rather than a dependency."""
        try:
            request = json.loads(msg.data)
            name = os.path.basename(request["name"])
        except (TypeError, ValueError, KeyError) as exc:
            self._publish_export_e57_response(None, None, f"bad request: {exc}")
            return

        req_id = request.get("id")
        if not name.lower().endswith(".pcd"):
            self._publish_export_e57_response(req_id, name, f"{name}: not a .pcd file")
            return

        self._fetch_mount_params_then(
            lambda mount_params: self._start_export_e57_thread(req_id, name, mount_params)
        )

    def _start_export_e57_thread(self, req_id, name: str, mount_params: dict | None) -> None:
        src = os.path.join(OUTPUT_DIR, name)
        try:
            mtime = os.path.getmtime(src)
        except OSError:
            mtime = time.time()
        metadata = self._assemble_e57_metadata(
            station_name=name[: -len(".pcd")],
            description_note=(
                "Converted from a legacy .pcd file. Scan config/mount "
                "calibration/sensor status fields reflect live values at "
                "CONVERSION time, not necessarily what was active during "
                "the original capture."
            ),
            acquisition_start=mtime,
            acquisition_end=mtime,
            mount_params=mount_params,
        )
        threading.Thread(
            target=self._export_e57_in_background,
            args=(req_id, name, metadata),
            daemon=True,
        ).start()

    def _export_e57_in_background(self, req_id, name: str, metadata: dict) -> None:
        src = os.path.join(OUTPUT_DIR, name)
        out_name = name[: -len(".pcd")] + ".e57"
        dst = os.path.join(OUTPUT_DIR, out_name)
        try:
            points, field_names = read_pcd_points(src)
            write_e57(dst, points, field_names=field_names, metadata=metadata)
            fsync_durable(dst)
        except (OSError, ValueError) as exc:
            self._publish_export_e57_response(req_id, name, str(exc))
            return
        self._publish_export_e57_response(req_id, out_name, None)

    def _publish_export_e57_response(self, req_id, name: str | None, error: str | None) -> None:
        msg = String()
        msg.data = json.dumps({"id": req_id, "name": name, "error": error})
        self._export_e57_response_pub.publish(msg)

    def _on_bundle_project_request(self, msg: String) -> None:
        """Zips every locally-saved .e57 file whose name belongs
        to a given project (see _build_output_basename's own naming
        convention) into one archive in OUTPUT_DIR, so the GUI can offer
        "download this whole job" as a single link instead of one file
        at a time. Runs on a background thread, same reasoning as
        export_to_usb/export_e57: zipping a job's worth of multi-hundred-
        MB scans is real I/O work that must not block this node's
        single-threaded executor."""
        try:
            request = json.loads(msg.data)
            project = request["project"]
        except (TypeError, ValueError, KeyError) as exc:
            self._publish_bundle_project_response(None, None, f"bad request: {exc}")
            return

        req_id = request.get("id")
        prefix = _sanitize_filename_part(project)
        if not prefix:
            self._publish_bundle_project_response(req_id, None, "project name is empty after sanitizing")
            return

        threading.Thread(
            target=self._bundle_project_in_background,
            args=(req_id, prefix),
            daemon=True,
        ).start()

    def _bundle_project_in_background(self, req_id, prefix: str) -> None:
        # .e57 only -- every scan is saved natively as .e57 now (see
        # _finish_run), so that's the only extension _build_output_basename
        # ever actually produces for a real project. Full-match, not
        # startswith -- a plain prefix check would let "Test" incorrectly
        # pull in "Test2_<timestamp>.e57" too.
        pattern = re.compile(
            rf"^{re.escape(prefix)}_\d{{8}}_\d{{6}}\.e57$", re.IGNORECASE
        )
        matches = []
        try:
            with os.scandir(OUTPUT_DIR) as entries:
                for entry in entries:
                    if not entry.is_file():
                        continue
                    if not pattern.match(entry.name):
                        continue
                    matches.append(entry.name)
        except OSError as exc:
            self._publish_bundle_project_response(req_id, None, str(exc))
            return

        if not matches:
            self._publish_bundle_project_response(req_id, None, f"no files found for project '{prefix}'")
            return

        # Always rebuilt fresh on every request rather than cached/reused
        # -- simplest correct behavior given new scans can be added to
        # the same project between one download and the next, and
        # zipping already-local files is cheap. ZIP_STORED (no
        # compression): PCD/E57 are already dense binary float data, not
        # meaningfully compressible, and compression would only slow down
        # a multi-hundred-MB operation for no real size benefit -- same
        # reasoning pcd_writer.py already gives for binary over ASCII.
        zip_name = f"{prefix}_bundle.zip"
        zip_path = os.path.join(OUTPUT_DIR, zip_name)
        try:
            with open(zip_path, "wb") as f:
                with zipfile.ZipFile(f, "w", zipfile.ZIP_STORED) as zf:
                    for name in matches:
                        zf.write(os.path.join(OUTPUT_DIR, name), arcname=name)
                f.flush()
                os.fsync(f.fileno())
            fsync_durable(zip_path)
        except OSError as exc:
            self._publish_bundle_project_response(req_id, None, str(exc))
            return

        self._publish_bundle_project_response(req_id, zip_name, None)

    def _publish_bundle_project_response(self, req_id, name: str | None, error: str | None) -> None:
        msg = String()
        msg.data = json.dumps({"id": req_id, "name": name, "error": error})
        self._bundle_project_response_pub.publish(msg)

    def _on_delete_local_scan_request(self, msg: String) -> None:
        """Deletes one scan from OUTPUT_DIR -- closes the local-first-save
        loop so reclaiming space never requires SSH access. Deliberately
        only ever touches OUTPUT_DIR (never USB_EXPORT_DIR): a scan already
        exported to USB is untouched by deleting the local copy."""
        try:
            request = json.loads(msg.data)
            name = os.path.basename(request["name"])
        except (TypeError, ValueError, KeyError) as exc:
            self._publish_delete_local_scan_response(None, None, f"bad request: {exc}")
            return

        req_id = request.get("id")
        path = os.path.join(OUTPUT_DIR, name)
        try:
            os.remove(path)
        except OSError as exc:
            self._publish_delete_local_scan_response(req_id, name, str(exc))
            return
        self._publish_delete_local_scan_response(req_id, name, None)

    def _publish_delete_local_scan_response(
        self, req_id, name: str | None, error: str | None
    ) -> None:
        msg = String()
        msg.data = json.dumps({"id": req_id, "name": name, "error": error})
        self._delete_local_scan_response_pub.publish(msg)

    def _now(self) -> float:
        return time.monotonic()

    def _abort(self, reason: str) -> None:
        self._error = reason
        self._state = STATE_ABORTED
        self._capture_buffer = []
        self._pending_transforms = []
        self.get_logger().error(f"scan aborted: {reason}")

    # ---- State machine ----

    def _tick(self) -> None:
        if self._state == STATE_HOMING:
            if self._now() > self._phase_deadline:
                self._abort("homing timed out (no limit switch installed?)")
            elif self._tilt_status == "settled":
                self._advance_to_next_target()

        elif self._state == STATE_MOVING:
            if self._now() > self._phase_deadline:
                self._abort("move timed out")
            elif self._tilt_status == "settled":
                self._state = STATE_SETTLING_EXTRA
                extra = float(self.get_parameter("settle_extra_s").value)
                self._phase_deadline = self._now() + extra

        elif self._state == STATE_SETTLING_EXTRA:
            if self._now() >= self._phase_deadline:
                self._start_capture()

        elif self._state == STATE_CAPTURING:
            if self._now() >= self._capture_deadline:
                self._finish_capture()

        elif self._state == STATE_SWEEP_HOMING:
            if self._now() > self._phase_deadline:
                self._abort("homing timed out (no limit switch installed?)")
            elif self._tilt_status == "settled":
                self._start_sweep()

        elif self._state == STATE_SWEEP_SCANNING:
            if self._sweep_deadline is not None and self._now() >= self._sweep_deadline:
                self._finish_sweep()

        self._retry_pending_transforms()
        self._maybe_publish_preview()
        self._maybe_publish_coverage()
        self._publish_status()

    def _maybe_publish_coverage(self) -> None:
        """Publishes the live coverage map (~/coverage) -- a bin count per
        tilt slice across the current scan's own configured range, so a
        GUI can render which of it has been captured (and, importantly,
        which hasn't) while the run is still going, not just after the
        fact. Runs after every tick like _maybe_publish_preview, same
        reasoning: the last state stays visible (and gets one final
        publish) through STATE_DONE/STATE_ABORTED too, so reviewing a
        just-finished or just-aborted run's actual coverage doesn't need
        a separate code path."""
        if self._coverage_counts.size == 0:
            return
        now = self._now()
        period = float(self.get_parameter("coverage_publish_period_s").value)
        if period > 0.0 and now - self._last_coverage_publish < period:
            return
        self._last_coverage_publish = now
        msg = String()
        msg.data = json.dumps(
            {
                "start_deg": self._coverage_start_deg,
                "bin_deg": self._coverage_bin_deg,
                "counts": self._coverage_counts.tolist(),
            }
        )
        self._coverage_pub.publish(msg)

    def _maybe_publish_preview(self) -> None:
        """Publish the in-progress merged cloud so a viewer (rviz2, Foxglove
        Studio over rosbridge) can watch a scan build up live rather than
        only seeing the final .e57. Throttled by preview_publish_period_s
        and decimated (preview_decimation, then preview_max_points as a
        second cap) -- the full merge is re-built from _merged_points on
        every publish (simplest correct thing), so these knobs exist to
        bound that cost as a scan grows, not just the wire payload/render
        load on the viewer. Runs after every tick regardless of state, so
        the completed cloud stays visible (and keeps being refreshed) once
        a run reaches STATE_DONE too."""
        if not self.get_parameter("preview_enabled").value or not self._merged_points:
            return
        now = self._now()
        period = float(self.get_parameter("preview_publish_period_s").value)
        if period > 0.0 and now - self._last_preview_publish < period:
            return
        self._last_preview_publish = now

        merged = np.concatenate(self._merged_points, axis=0)
        stride = 1
        decimation = float(self.get_parameter("preview_decimation").value)
        if 0.0 < decimation < 1.0:
            stride = max(1, round(1.0 / decimation))
        max_points = int(self.get_parameter("preview_max_points").value)
        if max_points > 0:
            # -(-n // stride): points remaining after the fraction stride,
            # without materializing it first -- widen the stride further if
            # that's still over the cap.
            remaining_after_stride = -(-merged.shape[0] // stride)
            if remaining_after_stride > max_points:
                stride = max(stride, merged.shape[0] // max_points + 1)
        if stride > 1:
            merged = merged[::stride]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter("output_frame").value
        # _make_preview_cloud/_PREVIEW_FIELDS are hardcoded to x/y/z/
        # intensity (point_step=16) -- merged is Nx6 now that ring/time
        # ride along (see POINT_FIELD_NAMES), but a live viewer has no
        # use for either, so just drop them here rather than widening
        # the preview's own wire format to match.
        self._preview_pub.publish(self._make_preview_cloud(merged[:, :4], header))

    def _make_preview_cloud(self, points_xyzi: np.ndarray, header: Header) -> PointCloud2:
        points_xyzi = np.ascontiguousarray(points_xyzi, dtype=np.float32)
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = points_xyzi.shape[0]
        msg.fields = _PREVIEW_FIELDS
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * points_xyzi.shape[0]
        msg.is_dense = True
        msg.data = points_xyzi.tobytes()
        return msg

    def _advance_to_next_target(self) -> None:
        if self._target_idx >= len(self._targets_rad):
            self._finish_run()
            return
        target = self._targets_rad[self._target_idx]
        msg = Float64()
        msg.data = target
        self._invalidate_tilt_status()
        self._cmd_position_pub.publish(msg)
        self._state = STATE_MOVING
        self._phase_deadline = self._now() + float(self.get_parameter("move_timeout_s").value)

    def _start_capture(self) -> None:
        self._capture_buffer = []
        rate_hz = float(self.get_parameter("rotation_rate_hz").value)
        revs = float(self.get_parameter("revolutions_per_stop").value)
        duration = revs / rate_hz if rate_hz > 0 else 1.0
        self._capture_deadline = self._now() + duration
        self._state = STATE_CAPTURING

    def _try_transform_and_accumulate(self, cloud_msg: PointCloud2) -> Exception | None:
        """One attempt at transforming a single cloud into output_frame via
        tf2 and folding it into the running merge. Returns None on success,
        or the caught tf2 exception on failure -- never raises. Split out
        from _transform_and_accumulate so a failed attempt can be retried
        (see _retry_pending_transforms) without redoing the whole "queue
        it or drop it" decision on every attempt.

        No timeout on the lookup itself -- this runs on the same
        single-threaded executor as this node's own TransformListener, so
        a nonzero timeout *inside one lookup call* is self-defeating: the
        only thing that could deliver the awaited /tf message is the
        executor thread that's blocked waiting for it. Confirmed against
        real hardware: with a 0.2s timeout, step-and-stare's tight
        per-buffered-cloud loop in _finish_capture burned up to
        buffer-length x 0.2s per stop with the executor starved the whole
        time (so no /tf could arrive to satisfy even the *next* lookup in
        the same loop either) -- compounding stop over stop into a
        growing, then roughly steady-state, multi-second lag and every
        cloud getting dropped. That's a different failure mode from the
        retry loop below: retrying *across* ticks lets ordinary spinning
        between attempts actually deliver a fresher /tf sample, rather
        than blocking the one thread that could deliver it."""
        target_frame = self.get_parameter("output_frame").value
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            return exc

        transformed = do_transform_cloud(cloud_msg, transform)
        arr = _read_points_with_extra_fields(transformed)
        self._finish_accumulate_array(arr)
        return None

    def _finish_accumulate_array(self, arr: np.ndarray) -> None:
        """Shared tail for both _try_transform_and_accumulate and its
        deskewed sibling below -- applies the axis-invert flags and folds
        the result into the running merge. Split out so the two transform
        paths (whole-cloud vs per-point) share this rather than each
        reimplementing it."""
        if not arr.size:
            return
        # See invert_z_axis's own declare_parameter comment -- columns
        # are x, y, z, intensity, ring, time (0-5; the last two are
        # NaN if the source topic didn't have them, see
        # EXTRA_POINT_FIELDS). Applied here, the one place every capture
        # path funnels every point through, so both the live preview
        # (built from _merged_points) and the final .e57 get it.
        if self.get_parameter("invert_x_axis").value:
            arr[:, 0] = -arr[:, 0]
        if self.get_parameter("invert_y_axis").value:
            arr[:, 1] = -arr[:, 1]
        if self.get_parameter("invert_z_axis").value:
            arr[:, 2] = -arr[:, 2]
        self._merged_points.append(arr)

    def _try_transform_and_accumulate_deskewed(self, cloud_msg: PointCloud2) -> Exception | None:
        """Sweep-mode alternative to _try_transform_and_accumulate, used
        instead of it when enable_sweep_deskew is on (see
        _pick_transform_fn) -- rotates each point by its own individually
        interpolated tilt angle rather than applying one shared transform
        to the whole ~0.1s cloud message.

        Splits the base_link<-velodyne transform into its two real parts
        instead of treating it as tf2's one composed whole:

        - tilt_link<-velodyne (the mount calibration -- translation +
          roll/pitch/yaw from vlp16_config, see that node's
          _publish_mount_transform) is fixed for the whole message, so
          this is still exactly one tf2 lookup + do_transform_cloud, same
          cost as the non-deskewed path.
        - base_link<-tilt_link (the tilt joint itself) is a pure rotation
          about Z by the joint's own current angle (scanner.urdf.xacro's
          "tilt_axis" joint: origin xyz/rpy all zero, axis 0 0 1 -- no
          fixed offset to account for, just Rz(angle)) -- this is the
          only part that actually varies within one cloud message during
          a sweep, and tf2 has no per-point API to vary it, so it's
          computed here directly instead: each point's own absolute
          capture time (cloud_msg.header.stamp + that point's own `time`
          field) is looked up against _joint_state_history via linear
          interpolation (np.interp), then every point gets its own Rz
          applied via plain vectorized cos/sin over the whole array --
          not a per-point Python loop, and not one tf2 lookup per point
          either (which is what the original "might not be practical on
          the Pi" caveat in HANDOFF.md assumed -- this sidesteps that
          entirely by only ever doing tf2 work for the fixed part).

        Falls back to the plain (non-deskewed) path entirely -- not a
        cruder approximation of its own -- when there isn't enough
        _joint_state_history to interpolate against yet (e.g. the first
        cloud or two right at scan start, before two real samples have
        arrived): reuses that already-correct single-stamp tf2 lookup
        rather than inventing a less-precise one here."""
        if len(self._joint_state_history) < 2:
            return self._try_transform_and_accumulate(cloud_msg)

        try:
            mount_transform = self._tf_buffer.lookup_transform(
                "tilt_link",
                cloud_msg.header.frame_id,
                cloud_msg.header.stamp,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            return exc

        in_tilt_link = do_transform_cloud(cloud_msg, mount_transform)
        arr = _read_points_with_extra_fields(in_tilt_link)
        if not arr.size:
            return None

        time_col = arr[:, 5]
        if np.all(np.isnan(time_col)):
            # No real per-point time -- older/different driver, see
            # EXTRA_POINT_FIELDS (confirmed present on this project's own
            # hardware, so this is a defensive fallback, not the expected
            # path). Same shared-angle behavior as the non-deskewed path,
            # just already past the mount-only lookup above so redone
            # here rather than re-delegating and doing that lookup twice.
            tilt_rad = np.full(arr.shape[0], self._current_tilt_rad, dtype=np.float64)
        else:
            header_stamp = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9
            point_times = header_stamp + np.nan_to_num(time_col, nan=0.0).astype(np.float64)
            hist_t = np.fromiter((t for t, _ in self._joint_state_history), dtype=np.float64)
            hist_rad = np.fromiter((r for _, r in self._joint_state_history), dtype=np.float64)
            tilt_rad = np.interp(point_times, hist_t, hist_rad)

        cos_t = np.cos(tilt_rad)
        sin_t = np.sin(tilt_rad)
        x = arr[:, 0].astype(np.float64)
        y = arr[:, 1].astype(np.float64)
        arr[:, 0] = (x * cos_t - y * sin_t).astype(np.float32)
        arr[:, 1] = (x * sin_t + y * cos_t).astype(np.float32)
        # z untouched: the tilt joint's own rotation axis is Z, per the
        # URDF -- see this function's own docstring.

        self._finish_accumulate_array(arr)
        return None

    def _pick_transform_fn(self) -> Callable[[PointCloud2], "Exception | None"]:
        if self._mode == MODE_SWEEP and bool(self.get_parameter("enable_sweep_deskew").value):
            return self._try_transform_and_accumulate_deskewed
        return self._try_transform_and_accumulate

    def _transform_and_accumulate(self, cloud_msg: PointCloud2) -> None:
        """Entry point shared by both modes: step-and-stare calls this once
        per buffered cloud when a capture window closes; sweep mode calls
        it directly as each cloud arrives, since there's no window to
        buffer against. A first-attempt failure (typically the requested
        stamp being a few ms newer than the newest /tf sample the executor
        had processed by the time this ran -- see _try_transform_and_accumulate)
        is queued for a bounded number of retries on later ticks rather
        than dropped on the spot; see _retry_pending_transforms."""
        try_fn = self._pick_transform_fn()
        exc = try_fn(cloud_msg)
        if exc is not None:
            self._pending_transforms.append((cloud_msg, self._now(), try_fn))

    def _retry_pending_transforms(self) -> None:
        """Called once per tick (10 Hz) -- by then robot_state_publisher
        (60 Hz) has had several chances to publish a /tf sample newer than
        whatever was cached at the failed cloud's first attempt, so most
        retries here succeed on the very next tick. A cloud still failing
        after TF_RETRY_TIMEOUT_S is a genuine drop (tf truly never
        resolved, not just momentarily behind) and gets the same warning
        _transform_and_accumulate used to log immediately on any failure."""
        if not self._pending_transforms:
            return
        now = self._now()
        still_pending: list[
            tuple[PointCloud2, float, Callable[[PointCloud2], "Exception | None"]]
        ] = []
        for cloud_msg, first_attempt, try_fn in self._pending_transforms:
            exc = try_fn(cloud_msg)
            if exc is None:
                continue
            if now - first_attempt >= TF_RETRY_TIMEOUT_S:
                self.get_logger().warning(
                    f"tf lookup failed, dropping one cloud after retrying for "
                    f"{TF_RETRY_TIMEOUT_S:.1f}s: {exc}"
                )
            else:
                still_pending.append((cloud_msg, first_attempt, try_fn))
        self._pending_transforms = still_pending

    def _finish_capture(self) -> None:
        for cloud_msg in self._capture_buffer:
            self._transform_and_accumulate(cloud_msg)

        self._capture_buffer = []
        self._stops_done += 1
        self._target_idx += 1
        self._advance_to_next_target()

    def _start_sweep(self) -> None:
        min_deg = float(self.get_parameter("sweep_min_deg").value)
        max_deg = float(self.get_parameter("sweep_max_deg").value)
        speed_rpm = int(self.get_parameter("sweep_speed_rpm").value)
        accel = int(self.get_parameter("sweep_accel").value)
        # Cached for _in_sweep_edge_margin() rather than re-reading params
        # on every cloud -- these don't change mid-sweep.
        self._sweep_min_rad = deg_to_rad(min_deg)
        self._sweep_max_rad = deg_to_rad(max_deg)

        if not self._tilt_set_params_client.wait_for_service(timeout_sec=1.0):
            self._abort("tilt_axis_bridge ~/set_parameters service not available")
            return

        def _param(name: str, ptype: int, **value_kwargs) -> Parameter:
            return Parameter(name=name, value=ParameterValue(type=ptype, **value_kwargs))

        request = SetParameters.Request(
            parameters=[
                _param(
                    "sweep_min_rad",
                    ParameterType.PARAMETER_DOUBLE,
                    double_value=deg_to_rad(min_deg),
                ),
                _param(
                    "sweep_max_rad",
                    ParameterType.PARAMETER_DOUBLE,
                    double_value=deg_to_rad(max_deg),
                ),
                _param(
                    "sweep_speed_rpm", ParameterType.PARAMETER_INTEGER, integer_value=speed_rpm
                ),
                _param("sweep_accel", ParameterType.PARAMETER_INTEGER, integer_value=accel),
            ]
        )
        # Fire-and-forget, matching this node's existing pattern for
        # cmd_position/home/stop -- these are plain numeric parameter
        # writes on a node we know is alive (we just got "settled" from
        # it), not a driver round-trip that can meaningfully fail here.
        self._tilt_set_params_client.call_async(request)

        enable_msg = Bool()
        enable_msg.data = True
        self._sweep_enable_pub.publish(enable_msg)

        # Deadline is computed once real data starts arriving, not here --
        # see _on_pointcloud/_sweep_data_started.
        self._sweep_deadline = None
        self._sweep_data_started = False
        self._state = STATE_SWEEP_SCANNING

    def _finish_sweep(self) -> None:
        enable_msg = Bool()
        enable_msg.data = False
        self._sweep_enable_pub.publish(enable_msg)
        self._finish_run()

    def _finish_run(self) -> None:
        if self._mode == MODE_CALIBRATE:
            self._finish_mount_calibration()
            return

        if not self._merged_points:
            self._abort("run completed but no points were captured")
            return

        # The actual concatenate+write used to happen right here, inline
        # on this node's single-threaded executor. Confirmed (timed a
        # synthetic 5M-point write) that a realistic-sized merge takes
        # real, multi-second time to write -- fine for a normal
        # end-of-run completion (just a hitch in status publishing/tilt
        # communication), but a real bug once Stop Scan started calling
        # this too (see _on_stop_scan): the stop_scan service call would
        # block for that same time, past the GUI's own service-call
        # timeout, so Stop Scan reported "timed out" even though the
        # stop+save had actually (eventually) succeeded. Snapshotting and
        # handing the heavy work to a background thread fixes both: the
        # service call (and every other callback) returns promptly, and
        # no new points can land in the snapshot after this point
        # regardless -- _on_pointcloud only appends while self._state is
        # STATE_CAPTURING/STATE_SWEEP_SCANNING, and it's about to become
        # STATE_SAVING below. Deliberately NOT clearing self._merged_points
        # here (list(...) makes points_snapshot its own list object,
        # sharing the same underlying array references -- clearing the
        # original wouldn't affect it either way): leaving it populated is
        # what makes _maybe_publish_preview's existing "completed cloud
        # stays visible" behavior keep working through STATE_SAVING/
        # STATE_DONE, same as before this change. A new scan's own
        # _on_start_scan/_on_start_sweep_scan already resets it to [] when
        # one actually starts (and can't start any earlier than that --
        # STATE_SAVING isn't in the idle-state tuple those check), so
        # there's nothing left for this method to protect.
        points_snapshot = list(self._merged_points)
        mode = self._mode
        stops_done = self._stops_done
        dropped_edge_clouds = self._dropped_edge_clouds
        project_name = str(self.get_parameter("project_name").value)
        run_end_time = time.time()
        run_start_time = self._run_start_time if self._run_start_time is not None else run_end_time
        # Computed synchronously here, not inside the async callback below
        # -- _build_output_basename's own datetime.now() call needs to
        # land at essentially the same moment as run_end_time above, not
        # whenever the mount-params fetch happens to come back.
        basename = _build_output_basename(project_name, "e57")
        self._state = STATE_SAVING

        # Every scan is saved natively as E57 now, not PCD (see
        # HANDOFF.md) -- fetches vlp16_config's current mount calibration
        # first (best-effort provenance, see _fetch_mount_params_then),
        # since a service response only ever arrives via a callback on
        # this executor thread, never by blocking a call from the
        # background thread that will do the actual write.
        def start_write(mount_params: dict | None) -> None:
            metadata = self._assemble_e57_metadata(
                station_name=basename[: -len(".e57")],
                description_note=(
                    "Native scan_aggregator output, saved automatically "
                    "as this run's own output format."
                ),
                acquisition_start=run_start_time,
                acquisition_end=run_end_time,
                mount_params=mount_params,
            )
            threading.Thread(
                target=self._write_output_in_background,
                args=(
                    points_snapshot, mode, stops_done, dropped_edge_clouds,
                    OUTPUT_DIR, basename, metadata,
                ),
                daemon=True,
            ).start()

        self._fetch_mount_params_then(start_write)

    def _write_output_in_background(
        self,
        points_snapshot: list[np.ndarray],
        mode: str,
        stops_done: int,
        dropped_edge_clouds: int,
        out_dir: str,
        basename: str,
        metadata: dict,
    ) -> None:
        """Runs off the executor thread -- see _finish_run. Takes
        everything it needs as arguments rather than reading self.* (bar
        the final status-relevant writes below) so it never touches
        mutable node state the executor thread could be concurrently
        changing; the three attribute writes at the end are each a single
        Python-level assignment, safe enough under the GIL without a lock
        for this node's read patterns (_tick/_publish_status only ever
        read them, never read-modify-write)."""
        merged = np.concatenate(points_snapshot, axis=0)
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, basename)
        write_e57(out_path, merged, field_names=POINT_FIELD_NAMES, metadata=metadata)
        # write_e57 fsyncs the file itself but not the containing
        # directory entry -- see fsync_durable's own docstring for why
        # that's a separate, real durability gap on removable/slow media.
        fsync_durable(out_path)

        self._last_output_path = out_path
        self._state = STATE_DONE
        if mode == MODE_SWEEP:
            self.get_logger().info(
                f"sweep scan complete: {merged.shape[0]} points -> {out_path} "
                f"({dropped_edge_clouds} clouds dropped near sweep edges)"
            )
        else:
            self.get_logger().info(
                f"scan complete: {stops_done} stops, {merged.shape[0]} points -> {out_path}"
            )

    def _finish_mount_calibration(self) -> None:
        if not self._calib_tilt:
            self._abort("mount calibration completed but no points were captured")
            return

        points_snapshot = list(self._calib_points)
        tilt_snapshot = list(self._calib_tilt)
        self._state = STATE_MOUNT_SOLVING

        if not self._vlp16_get_params_client.wait_for_service(timeout_sec=1.0):
            self._publish_mount_calibration_response(
                {"success": False, "error": "vlp16_config get_parameters service not available"}
            )
            self._state = STATE_DONE
            return

        request = GetParameters.Request(names=["mount_roll_deg", "mount_pitch_deg"])
        future = self._vlp16_get_params_client.call_async(request)
        future.add_done_callback(
            lambda f: self._on_vlp16_mount_params_received(f, points_snapshot, tilt_snapshot)
        )

    def _on_vlp16_mount_params_received(
        self, future, points_snapshot: list[np.ndarray], tilt_snapshot: list[np.ndarray]
    ) -> None:
        try:
            values = future.result().values
            initial_roll_deg = values[0].double_value
            initial_pitch_deg = values[1].double_value
        except Exception as exc:  # noqa: BLE001 -- any failure here just means "can't calibrate this time"
            self._publish_mount_calibration_response(
                {"success": False, "error": f"failed to read current mount angles: {exc}"}
            )
            self._state = STATE_DONE
            return

        # Off the executor thread -- see _write_output_in_background's own
        # comment for the identical reasoning (RANSAC + compass_search over
        # a real capture's worth of points is real work, must not block
        # every other callback in this node while it runs).
        threading.Thread(
            target=self._mount_calibration_in_background,
            args=(points_snapshot, tilt_snapshot, initial_roll_deg, initial_pitch_deg),
            daemon=True,
        ).start()

    def _mount_calibration_in_background(
        self,
        points_snapshot: list[np.ndarray],
        tilt_snapshot: list[np.ndarray],
        initial_roll_deg: float,
        initial_pitch_deg: float,
    ) -> None:
        points = np.concatenate(points_snapshot, axis=0)
        tilt = np.concatenate(tilt_snapshot, axis=0)
        result = mount_calibration.calibrate_roll_pitch(points, tilt, initial_roll_deg, initial_pitch_deg)
        result["initial_roll_deg"] = initial_roll_deg
        result["initial_pitch_deg"] = initial_pitch_deg
        self._publish_mount_calibration_response(result)
        self._state = STATE_DONE

    def _publish_mount_calibration_response(self, result: dict) -> None:
        msg = String()
        msg.data = json.dumps(result)
        self._mount_calibration_response_pub.publish(msg)

    def _publish_status(self) -> None:
        if self._state in (STATE_MOVING, STATE_CAPTURING, STATE_SETTLING_EXTRA):
            text = (
                f"{self._state} ({self._target_idx + 1}/{len(self._targets_rad)}, "
                f"tilt_status={self._tilt_status})"
            )
        elif self._state == STATE_SWEEP_SCANNING:
            duration = float(self.get_parameter("sweep_duration_s").value)
            # Calibration mode captures into _calib_tilt, not
            # _merged_points (see _accumulate_calibration_cloud) -- report
            # the count it's actually filling, not an always-zero one.
            n_clouds = len(self._calib_tilt) if self._mode == MODE_CALIBRATE else len(self._merged_points)
            label = "calibrating" if self._mode == MODE_CALIBRATE else "sweep_scanning"
            if duration <= 0.0:
                text = (
                    f"{label} (no duration set, {n_clouds} clouds captured, "
                    f"{self._dropped_edge_clouds} dropped near edges, stop manually)"
                )
            elif self._sweep_deadline is None:
                # Still transiting from home to sweep_min_rad -- duration
                # hasn't started counting down yet, see _on_pointcloud.
                text = (
                    f"{label} (moving to start, {duration:.0f}s once data "
                    f"begins, {self._dropped_edge_clouds} dropped near edges)"
                )
            else:
                remaining = max(0.0, self._sweep_deadline - self._now())
                text = (
                    f"{label} ({remaining:.0f}s left, {n_clouds} clouds "
                    f"captured, {self._dropped_edge_clouds} dropped near edges)"
                )
        elif self._state == STATE_SAVING:
            text = "saving (writing merged cloud to disk, this can take a while for a large scan)"
        elif self._state == STATE_MOUNT_SOLVING:
            text = "mount_solving (finding a flat surface and fitting roll/pitch, a few seconds)"
        elif self._state == STATE_ABORTED:
            text = f"aborted: {self._error}"
        elif self._state == STATE_DONE and self._mode == MODE_CALIBRATE:
            # Deliberately not "done: ... -> ..." -- the GUI's rename-scan
            # popup (extractDoneOutputPath) watches for exactly that shape
            # and this isn't a scan output to rename. The actual result
            # lives on ~/mount_calibration_response, not this status text.
            text = "mount_calibration_done"
        elif self._state == STATE_DONE and self._last_output_path and self._started_from_panel:
            # Deliberately no "->" -- index.html's rename-popup trigger
            # (extractDoneOutputPath) only matches that exact shape, and a
            # front-panel-started scan has no popup to show it on
            # (status.html has no such dialog) -- saved under its default
            # name is the whole point here, not something to prompt about
            # on whatever *other* GUI happens to be connected right now.
            saved_as = os.path.basename(self._last_output_path)
            if self._mode == MODE_SWEEP:
                text = f"done: sweep complete, saved as {saved_as} (started from front panel)"
            else:
                text = (
                    f"done: {self._stops_done} stops complete, saved as {saved_as} "
                    "(started from front panel)"
                )
        elif self._state == STATE_DONE and self._last_output_path:
            if self._mode == MODE_SWEEP:
                text = f"done: sweep -> {self._last_output_path}"
            else:
                text = f"done: {self._stops_done} stops -> {self._last_output_path}"
        else:
            text = self._state
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
