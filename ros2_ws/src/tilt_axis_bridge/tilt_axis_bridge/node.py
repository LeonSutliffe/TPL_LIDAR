"""PC-side ROS2 node for the LiDAR tilt axis.

Replaces the ESP32/micro-ROS firmware for this subsystem after an extensive
RS485-RX-path fault was isolated to the ESP32 board itself (confirmed via a
pure GPIO3->GPIO18 loopback, bypassing RS485 entirely, receiving nothing).
A transparent USB<->RS485 bridge was proven working end-to-end with the real
driver instead, so all protocol logic and the homing/move/sweep state
machine live here rather than in firmware -- this node talks to the bridge
over a plain serial port, no micro-ROS/DDS-agent involved.

The bridge hardware itself has changed since: originally a Raspberry Pi
Pico running custom transparent-bridge firmware
(D:\\Downloads\\MKS_Servo_Tester), replaced 2026-08-31 with an off-the-shelf
USB<->RS485 adapter (confirmed working; FTDI FT232-family per VID:PID,
not CH340 despite initially being described as one -- see BRIDGE_VID_PID's
own comment for how that was confirmed) -- see BRIDGE_VID_PID below.
Either way this node only ever assumes "a plain serial port carrying MKS
protocol bytes," so nothing here is coupled to which specific bridge is
plugged in beyond the VID:PID autodetect convenience.

States: idle -> homing -> settled; idle/settled -> moving -> settling ->
settled; idle/settled -> sweeping (until disabled) -> settling -> settled.
"""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import threading

import rclpy
import serial.tools.list_ports
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from .commands import UnknownCommandError, dispatch
from .mks_driver import MksCommandError, MksDriver

COUNTS_PER_REV = 16384
MOTOR_STATUS_HOMING = 5

# Same VID:PID start_scanner.ps1 already greps `usbipd list` for when
# attaching the bridge to WSL2 -- busid isn't stable across replugs there,
# and the /dev/ttyUSB<N> node isn't stable either: a reset (or a fresh
# usbipd attach after a disconnect) can bring the bridge back under a
# different number than before. Without re-resolving this, the reconnect
# loop below would retry the stale path forever and never recover without
# a full restart -- confirmed with the original Pico bridge, and the
# mechanism itself is bridge-hardware-agnostic (just needs the right
# VID:PID for whatever's actually plugged in).
#
# Confirmed live against the actual off-the-shelf adapter now in use
# (replaced the Pico 2026-08-31, was 0x2E8A/0x000A) -- checked via
# Get-PnpDevice on the PC side, the only serial device actually reporting
# Status: OK (every other VID:PID, including the old Pico's, showed up as
# a stale/disconnected entry). This is an FTDI FT232-family VID:PID
# (0403:6001), *not* CH340 (initially described as CH340-based -- the
# live check above is what it actually reports, so that's what's used
# here). If a different specific adapter module ever reports something
# else, update this to match (check `usbipd list` on the PC side, or
# `lsusb` on the Pi side).
BRIDGE_VID_PID = (0x0403, 0x6001)

RECONNECT_INTERVAL_S = 5.0

# Shared settings file -- NOT tilt_axis_bridge-specific despite living in
# this module: scan_aggregator reads/writes its own top-level section of
# the same file (duplicated load/save helpers over there rather than an
# import, matching this project's existing no-build-time-coupling stance
# between the two packages -- see scan_aggregator's own module docstring).
# Exists so every live-settable parameter (both nodes) and the raw
# mks_driver config fields (Config tab's raw driver-command escape hatch)
# survive a restart, not just reverse_direction (the first, narrower
# version of this) -- ROS2 parameters don't persist across a node restart
# on their own, only whatever a node was launched with (declared defaults,
# or a launch-time params file) does. Also doubles as the save/load-able
# file the GUI's Settings panel reads and writes via
# ~/read_settings_file_request / ~/write_settings_file_request below, and
# will be presets' underlying file format too when that feature lands --
# a preset is just one of these files saved somewhere other than this
# default path.
SETTINGS_PATH = os.path.expanduser("~/.lidar_scanner_settings.json")


def _load_settings_section(section: str) -> dict:
    """Best-effort: a missing/corrupt settings file, or a missing section,
    just returns {} -- same as if nothing had ever been saved."""
    try:
        with open(SETTINGS_PATH) as f:
            return dict(json.load(f).get(section, {}))
    except (OSError, ValueError):
        return {}

# Hard rotation limit, logical-degree space (0 = home). Asymmetric on
# purpose: home already sits close to the mechanical hard stop on the
# negative side (see the "don't sweep into negative degrees" gotcha), so
# unlike MksDriver.MAX_ROTATION_DEG (a raw, symmetric safety net a layer
# below this, guarding the raw driver-command escape hatch that bypasses
# this node's rad<->axis conversion entirely) this is the real operating
# range: never negative, never past TILT_MAX_DEG. Checked here rather
# than in MksDriver because it has to be reverse_direction-aware to mean
# the same physical thing regardless of that setting -- reverse_direction
# flips the sign between this logical space and the driver's raw axis
# counts (see _rad_to_axis), so a fixed bound in raw-axis space would
# silently swap which physical direction it protects depending on that
# parameter. Enforced in _rad_to_axis() -- this node's own docstring
# already called that function out as the one thing "everything funnels
# through" for anything with a destination: Go to degree, step-and-stare,
# both sweep modes, and jog (_apply_jog_speed), which targets whichever
# boundary the commanded direction points at rather than running an
# open-ended speed-mode jog -- see that function's docstring for why a
# separate per-tick watchdog approach was tried first and replaced.
# 270 initially, tightened to 260 after real-hardware testing.
TILT_MIN_DEG = 0.0
TILT_MAX_DEG = 260.0

STATE_IDLE = "idle"
STATE_HOMING = "homing"
STATE_MOVING = "moving"
STATE_SWEEPING = "sweeping"
STATE_JOGGING = "jogging"
STATE_SETTLING = "settling"
STATE_SETTLED = "settled"
# Reached when the driver reports Stopped (status 1) but is also disabled
# -- confirmed on real hardware (2026-08-21) via read_enable_status()/
# read_stall_status() that a move away from home can stall the motor,
# which auto-disables it. Stopped-and-disabled looks identical to
# Stopped-and-arrived to a bare read_motor_status() check, so without this
# state a stall was being silently reported as a normal "settled"/"done"
# -- worse, the exact same blind spot in the STATE_HOMING branch below
# meant a stalled-before-ever-homing motor would immediately call
# set_zero_point() on whatever arbitrary position it was disabled at,
# silently miscalibrating home. See _tick_state_machine.
STATE_STALLED = "stalled"

# Hardcoded baseline for _apply_persisted_mks_driver_settings, replayed
# against the driver on every connect *before* whatever's actually in the
# settings file overrides it command-by-command (see that method). Exists
# because a fresh install with an empty/missing mks_driver settings
# section used to apply nothing at all to the driver -- it would just run
# with whatever home_direction/etc. happened to already be in the
# driver's own EEPROM. That caused a real stall on the Pi's first-ever
# scan (home_direction searched the wrong physical side into a hard
# stop -- see HANDOFF.md) purely because nobody had imported a working
# settings file yet. These values are this rig's actual current,
# confirmed-working hardware configuration, not placeholders -- treat a
# change here the same as any other real calibration change, not a
# casual edit.
DEFAULT_MKS_DRIVER_SETTINGS = {
    "set_work_mode": {"mode": 4},
    "set_microstep": {"microstep": 128},
    "set_working_current": {"milliamps": 1000, "save": True},
    "set_holding_current_percent": {"ratio": 4},
    "set_enable_pin_level": {"level": 1},
    "set_direction": {"direction": 1},
    "set_auto_screen_off": {"enable": False},
    "set_overcurrent_protect": {"enable": True},
    "set_microstep_interpolation": {"enable": True},
    "set_baud_rate": {"baud_code": 6},
    "set_position_protect": {"enable": False, "tim": 20, "errors": 14000},
    "set_heartbeat_protect_time": {"milliseconds": 0},
    "set_home_params": {
        "trigger_high": True,
        "home_direction": 0,
        "home_speed_rpm": 2,
        "end_limit_enable": False,
    },
    "set_home_torque_offset": {
        "origin_offset": 300,
        "home_mode": 1,
        "home_current_ma": 350,
    },
    "set_single_turn_zero_return": {
        "mode": 0,
        "zero_action": 2,
        "speed_tier": 2,
        "direction": 0,
    },
    "set_position_threshold": {"enable": False, "threshold": 200},
    "set_limit_port_remap": {"enable": False},
    "set_respond_and_active": {"respond": True, "active": True},
    "set_group_address": {"group_address": 0},
    "set_key_lock": {"locked": False},
    "set_modbus_rtu": {"enable": False},
    "set_auto_run_on_power_on": {"action": 202},
}


def rad_to_axis(rad: float) -> int:
    return round(rad / (2.0 * math.pi) * COUNTS_PER_REV)


def axis_to_rad(axis: int) -> float:
    return axis / COUNTS_PER_REV * 2.0 * math.pi


class TiltAxisNode(Node):
    def __init__(self) -> None:
        super().__init__("tilt_axis_bridge")

        # Every parameter below defaults to whatever was last persisted
        # (see SETTINGS_PATH) rather than a hardcoded literal, falling back
        # to that literal if nothing's been saved yet (fresh install, or a
        # settings file that predates a given key). declare_parameter's
        # default is a low-precedence value -- an explicit launch-time
        # params file (scanner_bringup's own) still wins over this, so a
        # deployment-level override never gets silently clobbered by a
        # stale persisted value.
        persisted = _load_settings_section("tilt_axis_bridge")

        def _default(name: str, fallback):
            return persisted.get(name, fallback)

        self.declare_parameter("serial_port", _default("serial_port", "/dev/ttyUSB0"))
        # PC<->bridge link speed. Must match whatever the driver's own baud
        # is currently set to (function 0x8A SetBaudRate / the "Baud rate"
        # dropdown) -- live-settable via set_parameters (not just a startup
        # default), which reopens the host-side serial connection at the
        # new rate and reconnects. With the current FTDI-based adapter
        # this *is* the RS485 bus rate directly (a passive USB<->RS485
        # level-shifter, one hop, no separate bridge-side rate to keep in
        # sync). The original Pico bridge had an extra hop -- its own
        # UART1 baud, independently reconfigurable via custom firmware --
        # that this same live-reconnect happened to also keep in sync with;
        # simpler now, not more complicated, since there's nothing left on
        # the bridge side to desync from. See _on_set_parameters below.
        self.declare_parameter("serial_baud", _default("serial_baud", 115200))
        self.declare_parameter("slave_address", _default("slave_address", 1))
        # Flips the sign of every position-based command AND the encoder
        # readback that feeds joint_state/the GUI's position display, so
        # "+30 deg" (Go to degree, sweep min/max, step-and-stare targets --
        # everything funnels through _rad_to_axis/_axis_to_rad below)
        # consistently means the opposite physical direction on both ends,
        # rather than commands and the displayed position disagreeing.
        # Deliberately does not touch home_direction or the Raw Motion
        # tab's explicit Direction (bus) dropdown -- those are raw
        # CW/CCW selections the user makes directly each time, not part of
        # this coordinate-sign convention. A physically-backwards mount
        # doesn't change between restarts, so losing this every time the
        # stack restarts (a normal, frequent occurrence) was a real
        # reported annoyance -- persisted like everything else here now.
        self.declare_parameter("reverse_direction", _default("reverse_direction", True))
        # Capped at MksDriver.MAX_ALLOWED_RPM (40) -- that's the actual
        # enforcement point; this default just avoids shipping a value that
        # would immediately fail there.
        self.declare_parameter("move_speed_rpm", _default("move_speed_rpm", 40))
        self.declare_parameter("move_accel", _default("move_accel", 2))
        # Home (0 rad) sits close to the mechanical hard stop -- see
        # tilt_start_deg=5.0 in scan_aggregator's step-and-stare config for
        # the same margin. A negative sweep_min_rad drives the arm into
        # that physical limit instead of a controlled PID stop, which
        # showed up this session as a much harder/abrupt halt at that end
        # than at the (safe) positive end. Default kept non-negative for
        # that reason.
        # float(...) here (and on settle_time_s/poll_rate_hz below) forces
        # these to always declare as DOUBLE params -- see scan_aggregator's
        # node.py for the full story: a whole-number value round-tripped
        # through the shared JSON settings file loses its float-ness (JSON
        # doesn't distinguish 0 from 0.0), and declare_parameter() locks a
        # parameter's type to whatever Python type its default happens to
        # be, so an unguarded _default() here can silently declare these
        # INTEGER instead. That's already visibly corrupted this node's own
        # persisted section (sweep_min_rad saved as bare `0`) -- confirmed
        # to actually crash a node (scan_aggregator, same pattern) when a
        # launch-time params-file override collides with the wrong locked
        # type.
        # 0.0873/3.578 rad = ~5deg/~205deg -- this rig's actual confirmed
        # sweep range, matching scan_aggregator's own sweep_min_deg/
        # sweep_max_deg defaults above.
        self.declare_parameter("sweep_min_rad", float(_default("sweep_min_rad", 0.08726646259971639)))
        self.declare_parameter("sweep_max_rad", float(_default("sweep_max_rad", 3.5779249665883723)))
        self.declare_parameter(
            "sweep_speed_rpm", _default("sweep_speed_rpm", 1)
        )  # see MksDriver.MAX_ALLOWED_RPM -- this rig's own real sweep speed
        self.declare_parameter("sweep_accel", _default("sweep_accel", 2))
        self.declare_parameter("settle_time_s", float(_default("settle_time_s", 0.3)))
        self.declare_parameter("poll_rate_hz", float(_default("poll_rate_hz", 10.0)))
        # Deliberately decoupled from poll_rate_hz (which paces serial
        # round-trips to the real driver) and published on its own timer
        # instead of at the tail of _tick(). Found live: at the old 10 Hz
        # joint_state/TF rate, /velodyne_points (20 Hz) outran the TF
        # buffer -- tf2 lookup_transform for a cloud's stamp usually landed
        # ahead of the newest available tilt_link transform ("extrapolation
        # into the future"), so RViz's message filter dropped/errored on
        # most clouds. joint_state carries no fresh hardware data between
        # serial polls anyway (it just republishes _last_position_rad), so
        # there's no cost to publishing it faster than the poll rate --
        # this only needs enough headroom over the fastest TF consumer.
        self.declare_parameter("joint_state_rate_hz", float(_default("joint_state_rate_hz", 50.0)))

        self._port = self.get_parameter("serial_port").value
        address = int(self.get_parameter("slave_address").value)

        self._driver = MksDriver(port=self._port, address=address)
        self._driver_lock = threading.Lock()
        self._state = STATE_IDLE
        self._settle_deadline = 0.0
        self._pending_home = False
        self._pending_target_rad: float | None = None
        self._sweep_enabled = False
        self._sweep_target_is_max = True
        self._last_position_rad = 0.0
        # Signed RPM: positive = direction 1, negative = direction 0, 0 =
        # stop. None means "no new jog command since last tick" -- distinct
        # from 0.0, which is an explicit stop request. See _apply_jog_speed.
        self._pending_jog_speed_rpm: float | None = None
        self._motor_ready = False
        self._next_reconnect_attempt = 0.0
        # Applies the persisted mks_driver settings section once, at the
        # first successful connect after this node starts -- deliberately
        # NOT on every reconnect (_try_connect_driver also runs after a
        # dropped USB-CDC link mid-session). This node already has a
        # gotcha on record for exactly that mistake: _try_connect_driver
        # used to force a hardcoded work mode on every reconnect, silently
        # reverting a live-but-unsaved change. Re-applying a settings file
        # on every reconnect would be the same anti-pattern with extra
        # steps -- the driver already remembers its own state across power
        # cycles, this only needs to run once per session to satisfy
        # "restore what was last saved at the start of a session."
        self._mks_driver_settings_applied = False

        self._status_pub = self.create_publisher(String, "~/status", 10)
        self._joint_state_pub = self.create_publisher(JointState, "~/joint_state", 10)

        self.create_subscription(Float64, "~/cmd_position", self._on_cmd_position, 10)
        self.create_subscription(Bool, "~/sweep_enable", self._on_sweep_enable, 10)
        self.create_subscription(Float64, "~/jog_speed", self._on_jog_speed, 10)
        self.create_service(Trigger, "~/home", self._on_home_request)
        self.create_service(Trigger, "~/stop", self._on_stop_request)
        self.create_service(Trigger, "~/shutdown_stack", self._on_shutdown_stack)

        # Generic escape hatch for the advanced-options GUI: every MksDriver
        # method is reachable here without needing a custom .srv per command
        # (see commands.py for why). Not used by the automated scan flow.
        self._driver_response_pub = self.create_publisher(String, "~/driver_response", 10)
        self.create_subscription(String, "~/driver_command", self._on_driver_command, 10)

        # Backs the GUI's Settings panel (Save Settings As / Load Settings)
        # -- arbitrary-path JSON read/write of the shared settings file
        # format, same request/response-over-topic pattern as
        # scan_aggregator's list_dir_request/rename_output_request.
        self._read_settings_file_response_pub = self.create_publisher(
            String, "~/read_settings_file_response", 10
        )
        self._write_settings_file_response_pub = self.create_publisher(
            String, "~/write_settings_file_response", 10
        )
        self.create_subscription(
            String, "~/read_settings_file_request", self._on_read_settings_file_request, 10
        )
        self.create_subscription(
            String, "~/write_settings_file_request", self._on_write_settings_file_request, 10
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._try_connect_driver()

        rate_hz = float(self.get_parameter("poll_rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._tick)

        joint_state_rate_hz = float(self.get_parameter("joint_state_rate_hz").value)
        self.create_timer(1.0 / joint_state_rate_hz, self._publish_joint_state)

    def _on_set_parameters(self, params) -> SetParametersResult:
        if any(p.name in ("serial_port", "serial_baud") for p in params):
            # Force an immediate reconnect on the next tick rather than
            # waiting up to RECONNECT_INTERVAL_S, and regardless of whether
            # we're currently connected -- the old link is stale the moment
            # either of these changes.
            self._motor_ready = False
            self._next_reconnect_attempt = 0.0
        for p in params:
            self._save_settings_section("tilt_axis_bridge", p.name, p.value)
        return SetParametersResult(successful=True)

    def _save_settings_section(self, section: str, key: str, value) -> None:
        """Pairs with _load_settings_section -- see SETTINGS_PATH.
        Read-modify-write so other sections (scan_aggregator's, or
        mks_driver) and other keys already in this one aren't clobbered.
        Best-effort: a failed write is logged but doesn't fail the
        parameter change itself (the in-memory value this session is still
        correct either way, it just wouldn't survive a future restart)."""
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

    def destroy_node(self) -> bool:
        if self._motor_ready:
            self._driver.close()
        return super().destroy_node()

    def _autodetect_bridge_port(self, configured_port: str) -> str:
        """Prefer whatever port currently matches the bridge's VID:PID over
        the possibly-stale configured one -- see BRIDGE_VID_PID. Falls back
        to configured_port unchanged if no matching device is present
        (e.g. still disconnected, or a different bridge on purpose), so an
        explicit serial_port override always still works."""
        for p in serial.tools.list_ports.comports():
            if (p.vid, p.pid) == BRIDGE_VID_PID:
                if p.device != configured_port:
                    self.get_logger().info(
                        f"Bridge found at {p.device} (configured serial_port is "
                        f"{configured_port}) -- using {p.device}"
                    )
                return p.device
        return configured_port

    def _try_connect_driver(self) -> None:
        """(Re)open the serial link and re-arm the driver. Safe to call
        repeatedly -- used at startup, by _tick's reconnect loop after a
        link drop (a USB-serial bridge can reset mid-session; without this
        the node would stay permanently "disconnected" until manually
        restarted), and immediately after serial_port/serial_baud change
        via _on_set_parameters."""
        self._port = self._autodetect_bridge_port(self.get_parameter("serial_port").value)
        baud = int(self.get_parameter("serial_baud").value)
        self._driver.reconfigure(self._port, baud)
        try:
            self._driver.open()
            # Deliberately does NOT force a work mode here -- it used to
            # call set_work_mode(BUS_VFOC) unconditionally on every
            # connect, which silently reverted a manual switch to Close
            # mode back to FOC on every restart/reconnect (reported this
            # session). The driver already remembers its own work mode
            # across power cycles; this code has no business reasserting
            # one every time it reconnects. Whatever mode is currently set
            # (via the GUI's Work mode dropdown, or the driver's own
            # onboard menu) is left alone.
            self._driver.set_enable(True)
            if not self._mks_driver_settings_applied:
                # Once per session only -- see _mks_driver_settings_applied.
                self._apply_persisted_mks_driver_settings()
                self._mks_driver_settings_applied = True
            # Prime the driver's cached microstep (used by every RPM->wire
            # conversion, see mks_driver._rpm_to_wire_speed) from whatever
            # the hardware actually has -- it otherwise defaults to 16 on
            # every fresh MksDriver/node restart, silently making RPM
            # commands wrong by hardware_microstep/16x until something
            # calls read_config_params() (previously only the GUI's manual
            # "Refresh from driver", which nothing guarantees happens after
            # every reconnect). Reading it here, after any persisted
            # settings were just applied above, also means the cache
            # reflects their effect (e.g. a persisted set_microstep) rather
            # than going stale the instant they're applied.
            self._driver.read_config_params()
            self._motor_ready = True
            self._state = STATE_IDLE
            self.get_logger().info(f"Connected to MKS driver on {self._port} @ {baud} baud")
        except Exception as exc:  # noqa: BLE001 - report and keep the node alive
            self._motor_ready = False
            self.get_logger().error(
                f"Failed to open MKS driver on {self._port} @ {baud} baud: {exc}"
            )

    # ---- ROS callbacks: just record intent, the timer tick does the work ----

    def _on_cmd_position(self, msg: Float64) -> None:
        if self._sweep_enabled:
            self.get_logger().warning("Ignoring cmd_position while sweep is enabled")
            return
        self._pending_target_rad = msg.data

    def _on_sweep_enable(self, msg: Bool) -> None:
        self._sweep_enabled = msg.data
        if not msg.data and self._state == STATE_SWEEPING:
            self._state = STATE_SETTLING
            self._settle_deadline = self._now() + self.get_parameter("settle_time_s").value

    def _on_jog_speed(self, msg: Float64) -> None:
        if self._sweep_enabled:
            self.get_logger().warning("Ignoring jog_speed while sweep is enabled")
            return
        self._pending_jog_speed_rpm = msg.data

    def _on_home_request(self, request: Trigger.Request, response: Trigger.Response):
        if not self._motor_ready:
            response.success = False
            response.message = "driver not connected"
            return response
        self._pending_home = True
        response.success = True
        response.message = "homing started"
        return response

    def _on_stop_request(self, request: Trigger.Request, response: Trigger.Response):
        if not self._motor_ready:
            response.success = False
            response.message = "driver not connected"
            return response
        self._pending_home = False
        self._pending_target_rad = None
        self._sweep_enabled = False
        try:
            self._driver.emergency_stop()
            self._state = STATE_IDLE
            response.success = True
            response.message = "stopped"
        except MksCommandError as exc:
            response.success = False
            response.message = str(exc)
        except Exception as exc:  # noqa: BLE001 - link itself is gone, see _tick
            self._motor_ready = False
            self._next_reconnect_attempt = self._now() + RECONNECT_INTERVAL_S
            response.success = False
            response.message = f"driver communication lost: {exc}"
        return response

    def _on_shutdown_stack(self, request: Trigger.Request, response: Trigger.Response):
        """Tear down the whole running stack (this node included) by
        signaling the `ros2 launch scanner_bringup bringup.launch.py`
        process this node was started under -- SIGINT there triggers the
        same graceful cascade-shutdown of every child node that Ctrl+C on
        that launch's own terminal would. Found by process name rather
        than a stored PID: this node has no reliable way to know its own
        launch process's PID (it's several layers up through micromamba
        run's wrapper), and pattern-matching the well-known launch command
        line is simpler than threading a PID through every layer of
        start_scanner.ps1 -> launch_stack.sh -> ros2 launch.

        The response is best-effort -- rosbridge itself is one of the
        nodes that gets torn down by this same cascade, so the GUI should
        expect its connection to simply drop shortly after, not necessarily
        a clean acknowledgement.
        """
        try:
            found = subprocess.run(
                ["pgrep", "-f", "ros2 launch scanner_bringup"],
                capture_output=True, text=True, timeout=2.0,
            )
            pids = [int(p) for p in found.stdout.split()]
        except (subprocess.SubprocessError, ValueError) as exc:
            response.success = False
            response.message = f"failed to find launch process: {exc}"
            return response

        if not pids:
            response.success = False
            response.message = "no running 'ros2 launch scanner_bringup' process found"
            return response

        for pid in pids:
            os.kill(pid, signal.SIGINT)

        self.get_logger().warning(f"shutdown_stack: sent SIGINT to launch process(es) {pids}")
        response.success = True
        response.message = f"SIGINT sent to {pids} -- stack shutting down"
        return response

    def _on_driver_command(self, msg: String) -> None:
        try:
            request = json.loads(msg.data)
            command = request["command"]
            params = request.get("params", {})
        except (json.JSONDecodeError, KeyError) as exc:
            self._publish_driver_response(None, "", False, None, f"bad request: {exc}")
            return

        if not self._motor_ready:
            self._publish_driver_response(
                request.get("id"), command, False, None, "driver not connected"
            )
            return

        try:
            result = dispatch(self._driver, command, params)
            self._publish_driver_response(request.get("id"), command, True, result, None)
        except (MksCommandError, UnknownCommandError, KeyError, ValueError) as exc:
            self._publish_driver_response(request.get("id"), command, False, None, str(exc))
        except Exception as exc:  # noqa: BLE001 - link itself is gone, see _tick
            self._motor_ready = False
            self._next_reconnect_attempt = self._now() + RECONNECT_INTERVAL_S
            self._publish_driver_response(
                request.get("id"), command, False, None, f"driver communication lost: {exc}"
            )

    def _publish_driver_response(
        self, request_id, command: str, success: bool, result, error: str | None
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {"id": request_id, "command": command, "success": success, "result": result, "error": error}
        )
        self._driver_response_pub.publish(msg)

    def _apply_persisted_mks_driver_settings(self) -> None:
        """Replays the mks_driver section of the shared settings file (see
        SETTINGS_PATH) against the just-opened driver. Entries are keyed
        the same as commands.py's dispatch table (e.g. "set_work_mode":
        {"mode": 4}) so this is a direct replay through the same dispatch()
        the GUI's driver-command escape hatch already uses, not a second
        mapping to keep in sync. That section is populated only by the
        GUI's explicit Save Settings action (not automatically on every
        driver_command, unlike the ROS-parameter sections' _on_set_parameters
        hook -- most driver_command traffic is read-only queries that have
        no business being replayed as a setting). Best-effort per entry:
        one bad/stale entry (e.g. a command removed since the file was
        saved) is logged and skipped rather than aborting the rest or
        failing the connect.

        Starts from DEFAULT_MKS_DRIVER_SETTINGS rather than the settings
        file alone -- a real stall happened once on a fresh install whose
        settings file had no mks_driver section at all, so nothing here
        ran and the driver kept whatever home_direction/etc. happened to
        already be in its own EEPROM (see that constant's own comment).
        Persisted entries still override the hardcoded baseline
        command-by-command -- this only fills in whatever hasn't been
        explicitly saved yet, it never overrides a real user change."""
        settings = {**DEFAULT_MKS_DRIVER_SETTINGS, **_load_settings_section("mks_driver")}
        for command, params in settings.items():
            try:
                dispatch(self._driver, command, params)
            except (MksCommandError, UnknownCommandError, KeyError, ValueError) as exc:
                self.get_logger().warning(
                    f"Failed to apply persisted mks_driver.{command}: {exc}"
                )

    def _on_read_settings_file_request(self, msg: String) -> None:
        """Backs the GUI's Load Settings action -- reads an arbitrary path
        (chosen via the same WSL2 folder-browser mechanism as output_dir,
        extended to show files) and returns its parsed JSON content."""
        try:
            request = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self._publish_settings_file_response(
                self._read_settings_file_response_pub, None, None, None, f"bad request: {exc}"
            )
            return

        req_id = request.get("id")
        path = os.path.expanduser(request.get("path") or "")
        try:
            with open(path) as f:
                content = json.load(f)
            self._publish_settings_file_response(
                self._read_settings_file_response_pub, req_id, path, content, None
            )
        except (OSError, ValueError) as exc:
            self._publish_settings_file_response(
                self._read_settings_file_response_pub, req_id, path, None, str(exc)
            )

    def _on_write_settings_file_request(self, msg: String) -> None:
        """Backs the GUI's Save Settings As action -- writes the given
        (already-assembled client-side) JSON content to an arbitrary path.
        Not used for the auto-load default path directly (that's written
        one field at a time by each node's own _on_set_parameters as
        things change); this is for an explicit save/import to a path the
        user picked, and for Load's "also make this the new default"
        step (see the GUI)."""
        try:
            request = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self._publish_settings_file_response(
                self._write_settings_file_response_pub, None, None, None, f"bad request: {exc}"
            )
            return

        req_id = request.get("id")
        path = os.path.expanduser(request.get("path") or "")
        content = request.get("content")
        try:
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(path, "w") as f:
                json.dump(content, f, indent=2)
            self._publish_settings_file_response(
                self._write_settings_file_response_pub, req_id, path, None, None
            )
        except OSError as exc:
            self._publish_settings_file_response(
                self._write_settings_file_response_pub, req_id, path, None, str(exc)
            )

    def _publish_settings_file_response(
        self, publisher, req_id, path: str | None, content, error: str | None
    ) -> None:
        msg = String()
        msg.data = json.dumps({"id": req_id, "path": path, "content": content, "error": error})
        publisher.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _within_tilt_limits(self, rad: float) -> bool:
        return TILT_MIN_DEG <= math.degrees(rad) <= TILT_MAX_DEG

    def _rad_to_axis(self, rad: float) -> int:
        # See TILT_MIN_DEG/TILT_MAX_DEG -- checked against the logical
        # (pre-flip) value on purpose, so the bound means the same physical
        # thing regardless of reverse_direction.
        if not self._within_tilt_limits(rad):
            raise ValueError(
                f"target {math.degrees(rad):.1f} deg is outside this project's "
                f"allowed {TILT_MIN_DEG}-{TILT_MAX_DEG} deg range"
            )
        if self.get_parameter("reverse_direction").value:
            rad = -rad
        return rad_to_axis(rad)

    def _axis_to_rad(self, axis: int) -> float:
        rad = axis_to_rad(axis)
        if self.get_parameter("reverse_direction").value:
            rad = -rad
        return rad

    # ---- State machine, driven by a timer so serial round-trips never block a callback ----

    def _tick(self) -> None:
        if not self._motor_ready:
            if self._now() >= self._next_reconnect_attempt:
                self._next_reconnect_attempt = self._now() + RECONNECT_INTERVAL_S
                self._try_connect_driver()
            self._publish_status()
            return

        try:
            # Read position before running the state machine, not after --
            # keeps _last_position_rad one tick fresher for anything that
            # reads it (currently just _publish_joint_state; an earlier
            # jog watchdog design also depended on this ordering and was
            # since replaced, see _apply_jog_speed, but the fresher
            # ordering is harmless to keep regardless).
            self._last_position_rad = self._axis_to_rad(self._driver.read_cumulative_encoder())
            self._tick_state_machine()
        except MksCommandError as exc:
            self.get_logger().warning(f"MKS command failed: {exc}")
        except ValueError as exc:
            # A rejected command (MksDriver.MAX_ALLOWED_RPM, MAX_ROTATION_DEG)
            # -- raised client-side before any wire I/O, so the link itself
            # is fine. Without this branch it used to fall into the broad
            # except below and spuriously mark the node disconnected over a
            # value that was never sent to the driver at all.
            self.get_logger().warning(f"Command rejected: {exc}")
        except Exception as exc:  # noqa: BLE001 - see _try_connect_driver's docstring
            # Anything below MksCommandError (serial port vanished, USB-CDC
            # link reset, termios I/O error, ...) means the link itself is
            # gone, not just one bad frame. Degrade to disconnected and let
            # the reconnect loop above take over instead of letting this
            # propagate out of the timer callback -- an unhandled exception
            # here kills rclpy.spin() and the whole node with it.
            self.get_logger().error(f"Driver communication lost: {exc}")
            self._motor_ready = False
            self._next_reconnect_attempt = self._now() + RECONNECT_INTERVAL_S

        self._publish_status()

    def _tick_state_machine(self) -> None:
        if self._pending_home:
            self._pending_home = False
            self._driver.go_home()
            self._state = STATE_HOMING
            return

        if self._state == STATE_HOMING:
            if self._driver.read_motor_status() != MOTOR_STATUS_HOMING:
                if not self._driver.read_enable_status():
                    # Never actually entered the driver's Homing status (or
                    # stalled and dropped out of it) -- set_zero_point()
                    # here would zero whatever arbitrary position it's
                    # sitting at. See STATE_STALLED.
                    self._state = STATE_STALLED
                    self.get_logger().error(
                        "Homing stopped without the motor enabling/homing -- "
                        "motor is disabled (stalled?); NOT zeroing to this "
                        "position. Re-enable and home again."
                    )
                    return
                self._driver.set_zero_point()
                self._state = STATE_SETTLED
            return

        if self._pending_jog_speed_rpm is not None:
            speed = self._pending_jog_speed_rpm
            self._pending_jog_speed_rpm = None
            self._apply_jog_speed(speed)
            return

        if self._sweep_enabled:
            self._tick_sweep()
            return

        if self._pending_target_rad is not None:
            target = self._pending_target_rad
            self._pending_target_rad = None
            self._start_move(target)
            return

        if self._state in (STATE_MOVING, STATE_SWEEPING, STATE_JOGGING):
            if self._driver.read_motor_status() in (1,):  # Stopped
                if not self._driver.read_enable_status():
                    # Stopped-and-disabled, not Stopped-and-arrived -- a
                    # stall (confirmed on real hardware moving away from
                    # home). See STATE_STALLED.
                    prior_state = self._state
                    self._state = STATE_STALLED
                    self.get_logger().error(
                        f"Motor stopped while {prior_state} and is now "
                        "disabled -- likely stalled (check current/load for "
                        "this direction). Re-enable and re-home before "
                        "trusting position."
                    )
                    return
                self._state = STATE_SETTLING
                self._settle_deadline = self._now() + self.get_parameter("settle_time_s").value
            return

        if self._state == STATE_SETTLING:
            if self._now() >= self._settle_deadline:
                self._state = STATE_SETTLED
            return

    def _start_move(self, target_rad: float) -> None:
        speed = int(self.get_parameter("move_speed_rpm").value)
        acc = int(self.get_parameter("move_accel").value)
        self._driver.move_absolute_axis(speed, acc, self._rad_to_axis(target_rad))
        self._state = STATE_MOVING

    def _apply_jog_speed(self, signed_rpm: float) -> None:
        """Drives jog as a position-mode-4 move toward whichever end of the
        allowed range (TILT_MIN_DEG/TILT_MAX_DEG) the commanded sign points
        at, rather than the open-ended speed-mode run_speed jog this used
        to be. That earlier version needed its own hand-rolled
        reverse_direction flip and a per-tick watchdog to enforce the same
        limit a second, less reliable way -- both were real bugs (the
        watchdog's tick-rate lag meant it could report the axis "continuing
        to move until the jog is stopped" rather than actually stopping it,
        and once past the limit it refused *both* directions instead of
        allowing the recovery one). This reuses _rad_to_axis completely
        unchanged instead -- the same logical-target, reverse_direction-
        aware, already-bound-checked pipeline Go to degree already uses --
        so a jog command can never even ask for an out-of-range target in
        the first place, and completion (whether by reaching the boundary
        or an explicit keyup-triggered stop) is detected by the same
        generic "driver reports Stopped" check every other move already
        uses (see STATE_MOVING/STATE_SWEEPING/STATE_JOGGING above), not a
        second bespoke watchdog. It also fixes "won't move in either
        direction once past the limit" for free: the target is always one
        of the two valid boundaries, so a jog issued from any out-of-range
        position (however it got there) is still a legal move back toward
        range, never rejected -- there's nothing to special-case.
        """
        acc = int(self.get_parameter("move_accel").value)
        if signed_rpm == 0:
            if self._state == STATE_JOGGING:
                self._driver.stop_absolute_axis(acc)
                self._state = STATE_SETTLING
                self._settle_deadline = self._now() + self.get_parameter("settle_time_s").value
            return

        target_deg = TILT_MAX_DEG if signed_rpm > 0 else TILT_MIN_DEG
        self._driver.move_absolute_axis(
            abs(signed_rpm), acc, self._rad_to_axis(math.radians(target_deg))
        )
        self._state = STATE_JOGGING

    def _tick_sweep(self) -> None:
        """Drives one leg of the sweep at a time, reversing at each end.

        Found and fixed (2026-08-28): every other motion path in this file
        (a single move, a jog, homing) goes through STATE_SETTLING --
        settle_time_s of dwell after the driver reports Stopped -- before
        that position is trusted, specifically to let real mechanical
        ring-down/backlash finish. This turnaround-reversal path never got
        that treatment: it used to command the reversed move on the exact
        same tick "Stopped" was first seen, with no settle dwell and no
        stall check. Root-caused live from a real artifact: point clouds
        from a continuous sweep showed several duplicate "ghost" copies of
        the same real structure, fanned out at different angles, appearing
        only right at the sweep's two ends -- one ghost per turnaround
        reversal. With sweep_accel left at its near-default-instant value
        (per the MKS manual: acc=0 is an immediate stop/start, and higher
        values ramp *more* gradually -- so a small value like the default
        2 is already close to the abrupt end of that scale), reversing a
        loaded tilt axis from full speed in one direction to full speed in
        the other, instantly, is a genuinely violent mechanical event for
        a real geared/lever mechanism -- plausible real backlash/ringing
        in the linkage between the motor's own encoder (which read_
        cumulative_encoder() reports accurately) and the physical VLP-16
        puck it's not perfectly rigidly coupled to. tf2/joint_state
        faithfully reports what the *motor* thinks its position is, but if
        the puck is still physically oscillating through a range of *true*
        angles while the encoder already reports "arrived," every point
        captured in that window gets transformed with an angle that
        doesn't match where the sensor actually was -- exactly the
        "duplicate copies at the wrong angle, clustered right at each
        turnaround" symptom observed. Fixed by giving the reversal the
        same settle_time_s dwell (and the same Stopped-but-disabled stall
        check, since a full-speed reversal is also plausibly the highest-
        torque moment of the whole sweep) every other motion type already
        gets -- scan_aggregator's existing sweep_edge_margin_deg
        (angle-proximity-based) then naturally also covers this dwell,
        since the axis sits stationary at the boundary angle throughout
        it, so no change was needed on that side."""
        speed = int(self.get_parameter("sweep_speed_rpm").value)
        acc = int(self.get_parameter("sweep_accel").value)

        if self._state == STATE_SETTLING:
            if self._now() < self._settle_deadline:
                return
            self._sweep_target_is_max = not self._sweep_target_is_max
            next_target = (
                self.get_parameter("sweep_max_rad").value
                if self._sweep_target_is_max
                else self.get_parameter("sweep_min_rad").value
            )
            self._driver.move_absolute_axis(speed, acc, self._rad_to_axis(next_target))
            self._state = STATE_SWEEPING
            return

        if self._state != STATE_SWEEPING:
            target_rad = (
                self.get_parameter("sweep_max_rad").value
                if self._sweep_target_is_max
                else self.get_parameter("sweep_min_rad").value
            )
            self._driver.move_absolute_axis(speed, acc, self._rad_to_axis(target_rad))
            self._state = STATE_SWEEPING
            return

        if self._driver.read_motor_status() == 1:  # Stopped -> reached this waypoint
            if not self._driver.read_enable_status():
                # Stopped-and-disabled, not Stopped-and-arrived -- see
                # STATE_STALLED elsewhere in this file for the same check
                # on every other motion path. A full-speed reversal is a
                # real stall risk, arguably the highest-torque moment in
                # the whole sweep.
                self._state = STATE_STALLED
                self.get_logger().error(
                    "Motor stopped while sweeping and is now disabled -- "
                    "likely stalled at a turnaround (check current/load "
                    "for this direction). Sweep halted; re-enable and "
                    "re-home before trusting position."
                )
                return
            self._state = STATE_SETTLING
            self._settle_deadline = self._now() + self.get_parameter("settle_time_s").value

    # ---- Publishing ----

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self._state if self._motor_ready else "disconnected"
        self._status_pub.publish(msg)

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["tilt_axis"]
        msg.position = [self._last_position_rad]
        self._joint_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TiltAxisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
