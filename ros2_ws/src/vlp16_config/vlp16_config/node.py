"""VLP-16 configuration node -- two distinct layers, see HANDOFF.md.

Layer 2 (sensor hardware, "won't change" GUI tab): motor RPM, return type,
field of view, network settings -- set over the sensor's own embedded
web/CGI API (sensor_client.py), NOT ROS parameters. Reached via a generic
~/hw_command (request) / ~/hw_response (result) topic pair (commands.py),
same pattern as tilt_axis_bridge's ~/driver_command for the MKS driver.
Also polls ~/cgi/status.json on a timer and republishes it raw on
~/status -- status.json's exact field schema isn't confirmed from
documentation (see sensor_client.py), so this is the raw parsed response,
not named fields.

Layer 1 (ROS driver, "might change per scan" GUI tab): this node also owns
velodyne_transform_node's min_range/max_range/view_direction/view_width as
its OWN declared ROS parameters, live-pushed into that node via
set_parameters whenever they change (mirroring scan_aggregator's own push
of sweep params into tilt_axis_bridge) and persisted+replayed at startup
the same way every other node in this project persists its settings --
velodyne_transform_node is a third-party node with no settings-persistence
hook of its own, so this node does that on its behalf rather than forking
it. The GUI edits these directly via the standard get_parameters/
set_parameters calls against THIS node, same as any other ROS parameter
elsewhere in the project -- no custom request/response plumbing needed for
this half.

Two real, documented/derived facts this automates on a successful set_rpm,
so the operator never has to keep them in sync by hand:
  - The ROS driver's own `rpm` parameter is descriptive, not prescriptive
    (VLP-16 User Manual) -- it does not command the sensor, it just needs
    to match whatever the sensor is actually spinning at for the driver's
    own timing math. Pushed into velodyne_driver_node's `rpm`.
  - scan_aggregator's `rotation_rate_hz` (rpm / 60) drives its *actual*
    per-stop capture duration (`_start_capture`: duration = revs /
    rotation_rate_hz) -- unlike the driver's rpm, getting this wrong
    doesn't just mean a wrong number sitting somewhere unused, it means a
    real scan captures too few or too many revolutions per stop relative
    to what revolutions_per_stop was meant to represent. Pushed into
    scan_aggregator's `rotation_rate_hz`.

This node also owns the tilt_link -> velodyne static transform (the
tilt-axis-to-sensor-origin mount offset/lever arm) -- see mount_x/y/z/
mount_roll_deg/mount_pitch_deg/mount_yaw_deg below and
scanner_description/urdf/scanner.urdf.xacro's frames comment for why this
moved out of the URDF: a fixed xacro property needed an edit + rebuild to
change, these are live-tunable ROS parameters (persisted the same as
everything else on this node) published via a StaticTransformBroadcaster
instead.
"""

from __future__ import annotations

import json
import math
import os

import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .commands import UnknownCommandError, dispatch
from .sensor_client import SensorHttpError, Vlp16SensorClient

# Shared settings file -- same file/format as tilt_axis_bridge and
# scan_aggregator's own copies of these two helpers (duplicated rather
# than imported, matching this project's no-build-time-coupling stance
# between packages). This node owns the "vlp16_config" top-level section.
SETTINGS_PATH = os.path.expanduser("~/.lidar_scanner_settings.json")


def _load_settings_section(section: str) -> dict:
    try:
        with open(SETTINGS_PATH) as f:
            return dict(json.load(f).get(section, {}))
    except (OSError, ValueError):
        return {}


# velodyne_transform_node parameters this node pushes/persists on that
# node's behalf -- see module docstring.
CAPTURE_PARAM_NAMES = ("min_range", "max_range", "view_direction", "view_width")

# tilt_link -> velodyne mount offset -- see module docstring.
MOUNT_PARAM_NAMES = (
    "mount_x", "mount_y", "mount_z", "mount_roll_deg", "mount_pitch_deg", "mount_yaw_deg",
)


def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Radians in, (x, y, z, w) out. Standard roll-pitch-yaw -> quaternion
    conversion (matches tf_transformations.quaternion_from_euler's default
    'sxyz' convention) -- written out by hand rather than adding a
    dependency for one formula."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Vlp16ConfigNode(Node):
    def __init__(self) -> None:
        super().__init__("vlp16_config")

        persisted = _load_settings_section("vlp16_config")

        def _default(name: str, fallback):
            return persisted.get(name, fallback)

        self.declare_parameter("device_ip", _default("device_ip", "192.168.1.201"))
        self.declare_parameter("http_timeout_s", float(_default("http_timeout_s", 2.0)))
        self.declare_parameter("status_poll_period_s", float(_default("status_poll_period_s", 2.0)))
        self.declare_parameter(
            "velodyne_driver_node_name", _default("velodyne_driver_node_name", "/velodyne_driver_node")
        )
        self.declare_parameter(
            "velodyne_transform_node_name",
            _default("velodyne_transform_node_name", "/velodyne_transform_node"),
        )
        self.declare_parameter(
            "scan_aggregator_node_name", _default("scan_aggregator_node_name", "/scan_aggregator")
        )
        # Layer 1 capture-window params -- defaults match
        # scanner_bringup/config/vlp16.yaml's velodyne_transform_node
        # section (this node's own independent default, not read from
        # that file -- same no-cross-package-coupling stance as every
        # other pair of "sensible defaults that happen to agree" already
        # in this project).
        # float(...) around every _default() below -- see scan_aggregator's
        # node.py for the full story: JSON doesn't distinguish 0 from 0.0,
        # so a whole-number value round-tripped through the shared settings
        # file loses its float-ness, and declare_parameter() locks a
        # parameter's type to whatever Python type its default happens to
        # be -- silently declaring these INTEGER instead of DOUBLE. This
        # node's own persisted section was already visibly corrupted this
        # way (mount_roll_deg saved as bare `90`, not `90.0`); confirmed
        # this exact pattern crashes a node outright (scan_aggregator) when
        # a launch-time params-file override collides with the wrong
        # locked type.
        self.declare_parameter("min_range", float(_default("min_range", 0.4)))
        self.declare_parameter("max_range", float(_default("max_range", 130.0)))
        self.declare_parameter("view_direction", float(_default("view_direction", 0.0)))
        self.declare_parameter("view_width", float(_default("view_width", 2.0 * math.pi)))
        # tilt_link -> velodyne mount offset -- see module docstring.
        # Translation in metres, rotation in degrees (GUI convention
        # everywhere else in this project) -- converted to radians only at
        # the point of building the quaternion below.
        self.declare_parameter("mount_x", float(_default("mount_x", 0.0)))
        self.declare_parameter("mount_y", float(_default("mount_y", 0.0)))
        self.declare_parameter("mount_z", float(_default("mount_z", 0.0)))
        self.declare_parameter("mount_roll_deg", float(_default("mount_roll_deg", 0.0)))
        self.declare_parameter("mount_pitch_deg", float(_default("mount_pitch_deg", 0.0)))
        self.declare_parameter("mount_yaw_deg", float(_default("mount_yaw_deg", 0.0)))

        # Read once at startup, not re-read live -- matches
        # scan_aggregator's own tilt_node_name handling: a wiring
        # parameter that isn't expected to change without a restart.
        self._velodyne_driver_node_name = self.get_parameter("velodyne_driver_node_name").value
        self._velodyne_transform_node_name = self.get_parameter("velodyne_transform_node_name").value
        self._scan_aggregator_node_name = self.get_parameter("scan_aggregator_node_name").value

        self._sensor = Vlp16SensorClient(
            self.get_parameter("device_ip").value,
            float(self.get_parameter("http_timeout_s").value),
        )

        self._status_pub = self.create_publisher(String, "~/status", 10)
        self._hw_response_pub = self.create_publisher(String, "~/hw_response", 10)
        self.create_subscription(String, "~/hw_command", self._on_hw_command, 10)

        self._transform_set_params_client = self.create_client(
            SetParameters, f"{self._velodyne_transform_node_name}/set_parameters"
        )
        self._driver_set_params_client = self.create_client(
            SetParameters, f"{self._velodyne_driver_node_name}/set_parameters"
        )
        self._scan_set_params_client = self.create_client(
            SetParameters, f"{self._scan_aggregator_node_name}/set_parameters"
        )

        # Not tf2_ros.StaticTransformBroadcaster -- confirmed (the hard
        # way) that its sendTransform() only ever *adds* a transform for a
        # given child_frame_id once; every later call for the same child
        # frame is silently dropped and it just re-publishes whatever was
        # sent first. Fine for "declare a fixed set once at startup", not
        # for "this specific one needs to change live" as required here.
        # A plain publisher on the same topic/QoS that
        # StaticTransformBroadcaster itself uses (TRANSIENT_LOCAL,
        # depth=1) gets the same "late subscribers still see the latest
        # value" latching behavior, just without that one-shot-per-frame
        # restriction -- each publish() here correctly replaces what's
        # latched.
        self._mount_tf_pub = self.create_publisher(
            TFMessage,
            "/tf_static",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST),
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Replay persisted capture-window params once at startup --
        # velodyne_transform_node's own ROS parameters don't persist
        # across a restart on their own, only this node's
        # settings-file-backed defaults do. Best-effort: if that node
        # isn't up yet, this is a no-op and the values simply apply
        # whenever the operator next touches any of these four fields
        # (which re-triggers the same push via _on_set_parameters).
        self._push_capture_params_to_transform_node()

        # Publish the persisted (or default, all-zero) mount offset once
        # at startup too -- unlike the capture-window push above this
        # doesn't depend on another node being up (it's this node
        # broadcasting the transform itself), so it's unconditional.
        self._publish_mount_transform()

        poll_period = float(self.get_parameter("status_poll_period_s").value)
        if poll_period > 0.0:
            self.create_timer(poll_period, self._tick_status)

        self.get_logger().info("vlp16_config ready")

    # ---- Settings persistence (see SETTINGS_PATH) ----

    def _on_set_parameters(self, params) -> SetParametersResult:
        # add_on_set_parameters_callback runs as a *validator*, before
        # rclpy actually commits the new value -- self.get_parameter(...)
        # in here still returns the OLD value. Read the new one straight
        # off `params` instead; only fall back to get_parameter() for
        # anything NOT part of this particular change (see the `overrides`
        # param of _push_capture_params_to_transform_node below).
        changed = {p.name: p.value for p in params}
        for name, value in changed.items():
            self._save_settings_section("vlp16_config", name, value)
        if any(name in changed for name in CAPTURE_PARAM_NAMES):
            self._push_capture_params_to_transform_node(overrides=changed)
        if any(name in changed for name in MOUNT_PARAM_NAMES):
            self._publish_mount_transform(overrides=changed)
        if "device_ip" in changed:
            self._sensor.device_ip = changed["device_ip"]
        if "http_timeout_s" in changed:
            self._sensor.timeout_s = float(changed["http_timeout_s"])
        return SetParametersResult(successful=True)

    def _save_settings_section(self, section: str, key: str, value) -> None:
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

    # ---- Layer 1: push this node's capture-window params into velodyne_transform_node ----

    def _push_capture_params_to_transform_node(self, overrides: dict | None = None) -> None:
        """overrides carries any of CAPTURE_PARAM_NAMES that are being
        changed *right now* by the _on_set_parameters call this was
        invoked from -- those aren't committed on self yet (see that
        method's comment), so they must come from here rather than
        get_parameter(). Any name not in overrides (e.g. the other three,
        unchanged, or all four at the startup replay call) falls back to
        the current committed value as usual."""
        overrides = overrides or {}
        if not self._transform_set_params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(
                f"{self._velodyne_transform_node_name} set_parameters service not "
                "available -- capture-window params not pushed (will retry on next change)"
            )
            return

        def _param(name: str, value: float) -> Parameter:
            return Parameter(
                name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            )

        def _current(name: str) -> float:
            return overrides[name] if name in overrides else self.get_parameter(name).value

        request = SetParameters.Request(
            parameters=[_param(name, float(_current(name))) for name in CAPTURE_PARAM_NAMES]
        )
        # Fire-and-forget, matching scan_aggregator's own push of sweep
        # params into tilt_axis_bridge -- a plain numeric parameter write
        # on a node assumed alive, not a round-trip that meaningfully
        # fails here.
        self._transform_set_params_client.call_async(request)

    # ---- tilt_link -> velodyne mount offset (see module docstring) ----

    def _publish_mount_transform(self, overrides: dict | None = None) -> None:
        """Same overrides pattern as _push_capture_params_to_transform_node
        above -- values being changed *right now* by the _on_set_parameters
        call this was invoked from aren't committed on self yet, so those
        must come from overrides rather than get_parameter(). Safe to call
        on every mount-param change, not just at startup -- see
        _mount_tf_pub's own comment for why this publishes directly rather
        than through tf2_ros.StaticTransformBroadcaster."""
        overrides = overrides or {}

        def _current(name: str) -> float:
            return overrides[name] if name in overrides else self.get_parameter(name).value

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "tilt_link"
        transform.child_frame_id = "velodyne"
        transform.transform.translation.x = float(_current("mount_x"))
        transform.transform.translation.y = float(_current("mount_y"))
        transform.transform.translation.z = float(_current("mount_z"))
        qx, qy, qz, qw = _quaternion_from_euler(
            math.radians(float(_current("mount_roll_deg"))),
            math.radians(float(_current("mount_pitch_deg"))),
            math.radians(float(_current("mount_yaw_deg"))),
        )
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._mount_tf_pub.publish(TFMessage(transforms=[transform]))

    # ---- Layer 2: generic hw_command / hw_response dispatch ----

    def _on_hw_command(self, msg: String) -> None:
        try:
            request = json.loads(msg.data)
            command = request["command"]
            params = request.get("params", {})
        except (json.JSONDecodeError, KeyError) as exc:
            self._publish_hw_response(None, "", False, None, f"bad request: {exc}")
            return

        try:
            result = dispatch(self._sensor, command, params)
            if command == "set_rpm":
                self._sync_rpm(params.get("rpm"))
            self._publish_hw_response(request.get("id"), command, True, result, None)
        except (SensorHttpError, UnknownCommandError, KeyError, ValueError) as exc:
            self._publish_hw_response(request.get("id"), command, False, None, str(exc))

    def _sync_rpm(self, rpm) -> None:
        """See module docstring -- pushes a just-set RPM into both
        velodyne_driver_node's descriptive `rpm` and scan_aggregator's
        derived `rotation_rate_hz` (rpm / 60), so neither silently goes
        stale relative to what the sensor was just told. Both
        best-effort/fire-and-forget, matching the capture-param push
        above -- a missing node just means that half no-ops."""
        if rpm is None:
            return
        if self._driver_set_params_client.wait_for_service(timeout_sec=1.0):
            request = SetParameters.Request(
                parameters=[
                    Parameter(
                        name="rpm",
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(rpm)),
                    )
                ]
            )
            self._driver_set_params_client.call_async(request)
        if self._scan_set_params_client.wait_for_service(timeout_sec=1.0):
            request = SetParameters.Request(
                parameters=[
                    Parameter(
                        name="rotation_rate_hz",
                        value=ParameterValue(
                            type=ParameterType.PARAMETER_DOUBLE, double_value=float(rpm) / 60.0
                        ),
                    )
                ]
            )
            self._scan_set_params_client.call_async(request)

    def _publish_hw_response(
        self, request_id, command: str, success: bool, result, error: str | None
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {"id": request_id, "command": command, "success": success, "result": result, "error": error}
        )
        self._hw_response_pub.publish(msg)

    # ---- Live status ----

    def _tick_status(self) -> None:
        try:
            raw = self._sensor.get_status()
            payload = {"reachable": True, "raw": raw, "error": None}
        except SensorHttpError as exc:
            payload = {"reachable": False, "raw": None, "error": str(exc)}
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Vlp16ConfigNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
