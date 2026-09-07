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
chose step-and-stare as the first validated mode. tf2's buffer interpolates
the tilt angle between its ~10 Hz joint_state samples for each point cloud's
own timestamp, but that's still one transform per cloud message (~1 rotation
of points), not per individual point -- some smear within each cloud is
inherent to this mode, not a bug. Clouds captured within sweep_edge_margin_deg
of either turnaround are dropped entirely (see _in_sweep_edge_margin) -- the
axis is changing direction right at the edges, on top of the smear that's
already inherent to this mode elsewhere in the sweep.

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
import shutil
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from numpy.lib import recfunctions as rfn
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Bool, Float64, Header, String
from std_srvs.srv import Trigger
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from .pcd_writer import fsync_durable, write_pcd

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
# http://<pi-ip>:8080/scans/<name>.pcd for free, no separate download
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
STATE_DONE = "done"
STATE_ABORTED = "aborted"

MODE_STEP_AND_STARE = "step_and_stare"
MODE_SWEEP = "sweep"

# How long a cloud that fails its first tf lookup gets retried on
# subsequent ticks before being dropped for real -- see
# _try_transform_and_accumulate/_retry_pending_transforms.
TF_RETRY_TIMEOUT_S = 1.0


def deg_to_rad(deg: float) -> float:
    return deg * 3.14159265358979 / 180.0


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
        self.declare_parameter("preview_enabled", _default("preview_enabled", True))
        self.declare_parameter(
            "preview_publish_period_s", float(_default("preview_publish_period_s", 1.0))
        )
        # Fraction of points to keep, applied unconditionally via a fixed
        # stride (not just once preview_max_points is hit) -- a live
        # preview is for getting a look at coverage/shape while a scan
        # runs, not a faithful render, so it's fine to always be coarser
        # than the final .pcd. 1.0 = full density, disables this stride.
        self.declare_parameter("preview_decimation", float(_default("preview_decimation", 0.5)))
        # Extra safety cap on top of preview_decimation, for when even the
        # decimated count is still large on a long scan. 0 disables the cap.
        self.declare_parameter("preview_max_points", _default("preview_max_points", 500000))

        self.add_on_set_parameters_callback(self._on_set_parameters)

        tilt_node = self.get_parameter("tilt_node_name").value
        pointcloud_topic = self.get_parameter("pointcloud_topic").value

        self._state = STATE_IDLE
        self._mode: str | None = None
        self._targets_rad: list[float] = []
        self._target_idx = 0
        self._tilt_status = "disconnected"
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
        # Each entry is (cloud_msg, time of first failed attempt).
        self._pending_transforms: list[tuple[PointCloud2, float]] = []
        self._merged_points: list[np.ndarray] = []
        self._stops_done = 0
        self._error: str | None = None
        self._last_output_path: str | None = None
        self._dropped_edge_clouds = 0
        self._last_preview_publish = 0.0

        # Updated from tilt_axis_bridge's own joint_state as it arrives
        # (~10 Hz); used only to decide, at the coarse "within N degrees of
        # a turnaround" granularity this feature needs, whether to discard
        # a just-arrived sweep cloud -- not precise enough to need a
        # per-cloud tf lookup of its own.
        self._current_tilt_rad = 0.0
        self._sweep_min_rad = 0.0
        self._sweep_max_rad = 0.0

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
        # depth 1: only the latest accumulated cloud matters, a viewer that
        # missed one publish just picks up the next (larger) one.
        self._preview_pub = self.create_publisher(PointCloud2, "~/preview_points", 1)

        self.create_subscription(String, f"{tilt_node}/status", self._on_tilt_status, 10)
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

        self._home_client = self.create_client(Trigger, f"{tilt_node}/home")
        self._tilt_stop_client = self.create_client(Trigger, f"{tilt_node}/stop")
        self._tilt_set_params_client = self.create_client(
            SetParameters, f"{tilt_node}/set_parameters"
        )

        self.create_service(Trigger, "~/start_scan", self._on_start_scan)
        self.create_service(Trigger, "~/start_sweep_scan", self._on_start_sweep_scan)
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

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            idx = msg.name.index("tilt_axis")
        except ValueError:
            return
        self._current_tilt_rad = msg.position[idx]

    def _in_sweep_edge_margin(self) -> bool:
        margin_rad = deg_to_rad(float(self.get_parameter("sweep_edge_margin_deg").value))
        if margin_rad <= 0.0:
            return False
        pos = self._current_tilt_rad
        return pos <= self._sweep_min_rad + margin_rad or pos >= self._sweep_max_rad - margin_rad

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        if self._state == STATE_CAPTURING:
            self._capture_buffer.append(msg)
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
            # No fixed capture window in this mode -- transform and fold in
            # each cloud as it arrives, rather than buffering raw messages,
            # since a sweep has no natural end to flush a buffer at.
            self._transform_and_accumulate(msg)

    def _on_start_scan(self, request: Trigger.Request, response: Trigger.Response):
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
        self._mode = MODE_STEP_AND_STARE

        if not self._home_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "tilt_axis_bridge ~/home service not available"
            return response
        self._home_client.call_async(Trigger.Request())
        self._state = STATE_HOMING
        self._phase_deadline = self._now() + float(self.get_parameter("homing_timeout_s").value)

        response.success = True
        response.message = f"scan started: {len(self._targets_rad)} stops"
        return response

    def _on_start_sweep_scan(self, request: Trigger.Request, response: Trigger.Response):
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
        self._mode = MODE_SWEEP

        if not self._home_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            response.message = "tilt_axis_bridge ~/home service not available"
            return response
        self._home_client.call_async(Trigger.Request())
        self._state = STATE_SWEEP_HOMING
        self._phase_deadline = self._now() + float(self.get_parameter("homing_timeout_s").value)

        response.success = True
        response.message = "sweep scan started"
        return response

    def _on_stop_scan(self, request: Trigger.Request, response: Trigger.Response):
        # Used to always _abort() here, discarding every point captured so
        # far with no file ever written and no rename-popup opportunity --
        # a manual Stop mid-run lost everything. Flush whatever's sitting
        # in the current stop's not-yet-transformed capture buffer (sweep
        # mode has no such buffer -- points are transformed and
        # accumulated as each cloud arrives, so there's nothing to flush
        # there) and hand off to _finish_run(), the same path a normal
        # completed run takes: writes a .pcd from whatever's in
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
        new_name = os.path.basename(new_name)
        if not new_name.lower().endswith(".pcd"):
            new_name += ".pcd"

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
        block re-export (see _on_export_to_usb_request)."""
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
                    if not entry.name.lower().endswith(".pcd") or not entry.is_file():
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
        self._publish_status()

    def _maybe_publish_preview(self) -> None:
        """Publish the in-progress merged cloud so a viewer (rviz2, Foxglove
        Studio over rosbridge) can watch a scan build up live rather than
        only seeing the final .pcd. Throttled by preview_publish_period_s
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
        self._preview_pub.publish(self._make_preview_cloud(merged, header))

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
        structured = pc2.read_points(
            transformed, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )
        if structured.size:
            arr = rfn.structured_to_unstructured(structured, dtype=np.float32)
            # See invert_z_axis's own declare_parameter comment -- columns
            # are x, y, z, intensity (0, 1, 2, 3). Applied here, the one
            # place both step-and-stare and sweep mode funnel every point
            # through, so both the live preview (built from
            # _merged_points) and the final .pcd get it.
            if self.get_parameter("invert_x_axis").value:
                arr[:, 0] = -arr[:, 0]
            if self.get_parameter("invert_y_axis").value:
                arr[:, 1] = -arr[:, 1]
            if self.get_parameter("invert_z_axis").value:
                arr[:, 2] = -arr[:, 2]
            self._merged_points.append(arr)
        return None

    def _transform_and_accumulate(self, cloud_msg: PointCloud2) -> None:
        """Entry point shared by both modes: step-and-stare calls this once
        per buffered cloud when a capture window closes; sweep mode calls
        it directly as each cloud arrives, since there's no window to
        buffer against. A first-attempt failure (typically the requested
        stamp being a few ms newer than the newest /tf sample the executor
        had processed by the time this ran -- see _try_transform_and_accumulate)
        is queued for a bounded number of retries on later ticks rather
        than dropped on the spot; see _retry_pending_transforms."""
        exc = self._try_transform_and_accumulate(cloud_msg)
        if exc is not None:
            self._pending_transforms.append((cloud_msg, self._now()))

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
        still_pending: list[tuple[PointCloud2, float]] = []
        for cloud_msg, first_attempt in self._pending_transforms:
            exc = self._try_transform_and_accumulate(cloud_msg)
            if exc is None:
                continue
            if now - first_attempt >= TF_RETRY_TIMEOUT_S:
                self.get_logger().warning(
                    f"tf lookup failed, dropping one cloud after retrying for "
                    f"{TF_RETRY_TIMEOUT_S:.1f}s: {exc}"
                )
            else:
                still_pending.append((cloud_msg, first_attempt))
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
        if not self._merged_points:
            self._abort("run completed but no points were captured")
            return

        # The actual concatenate+write used to happen right here, inline
        # on this node's single-threaded executor. Confirmed (timed a
        # synthetic 5M-point write) that write_pcd's np.savetxt takes
        # ~10s for a realistic-sized merge -- fine for a normal
        # end-of-run completion (just a multi-second hitch in status
        # publishing/tilt communication), but a real bug once Stop Scan
        # started calling this too (see _on_stop_scan): the stop_scan
        # service call would block for that same 10+s, past the GUI's
        # own service-call timeout, so Stop Scan reported "timed out"
        # even though the stop+save had actually (eventually) succeeded.
        # Snapshotting and handing the heavy work to a background thread
        # fixes both: the service call (and every other callback) returns
        # promptly, and no new points can land in the snapshot after this
        # point regardless -- _on_pointcloud only appends while
        # self._state is STATE_CAPTURING/STATE_SWEEP_SCANNING, and it's
        # about to become STATE_SAVING below. Deliberately NOT clearing
        # self._merged_points here (list(...) makes points_snapshot its
        # own list object, sharing the same underlying array references --
        # clearing the original wouldn't affect it either way): leaving it
        # populated is what makes _maybe_publish_preview's existing
        # "completed cloud stays visible" behavior keep working through
        # STATE_SAVING/STATE_DONE, same as before this change. A new
        # scan's own _on_start_scan/_on_start_sweep_scan already resets it
        # to [] when one actually starts (and can't start any earlier
        # than that -- STATE_SAVING isn't in the idle-state tuple those
        # check), so there's nothing left for this method to protect.
        points_snapshot = list(self._merged_points)
        mode = self._mode
        stops_done = self._stops_done
        dropped_edge_clouds = self._dropped_edge_clouds
        self._state = STATE_SAVING
        threading.Thread(
            target=self._write_output_in_background,
            args=(points_snapshot, mode, stops_done, dropped_edge_clouds, OUTPUT_DIR),
            daemon=True,
        ).start()

    def _write_output_in_background(
        self,
        points_snapshot: list[np.ndarray],
        mode: str,
        stops_done: int,
        dropped_edge_clouds: int,
        out_dir: str,
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
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(out_dir, f"scan_{stamp}.pcd")
        write_pcd(out_path, merged)

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

    def _publish_status(self) -> None:
        if self._state in (STATE_MOVING, STATE_CAPTURING, STATE_SETTLING_EXTRA):
            text = (
                f"{self._state} ({self._target_idx + 1}/{len(self._targets_rad)}, "
                f"tilt_status={self._tilt_status})"
            )
        elif self._state == STATE_SWEEP_SCANNING:
            duration = float(self.get_parameter("sweep_duration_s").value)
            if duration <= 0.0:
                text = (
                    f"sweep_scanning (no duration set, {len(self._merged_points)} clouds merged, "
                    f"{self._dropped_edge_clouds} dropped near edges, stop manually)"
                )
            elif self._sweep_deadline is None:
                # Still transiting from home to sweep_min_rad -- duration
                # hasn't started counting down yet, see _on_pointcloud.
                text = (
                    f"sweep_scanning (moving to start, {duration:.0f}s once data "
                    f"begins, {self._dropped_edge_clouds} dropped near edges)"
                )
            else:
                remaining = max(0.0, self._sweep_deadline - self._now())
                text = (
                    f"sweep_scanning ({remaining:.0f}s left, {len(self._merged_points)} clouds "
                    f"merged, {self._dropped_edge_clouds} dropped near edges)"
                )
        elif self._state == STATE_SAVING:
            text = "saving (writing merged cloud to disk, this can take a while for a large scan)"
        elif self._state == STATE_ABORTED:
            text = f"aborted: {self._error}"
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
