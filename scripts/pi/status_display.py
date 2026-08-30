#!/usr/bin/env python3
"""Always-on status readout for the Pi's onboard GPIO/SPI panel.

All actual control happens through the web GUI (from a phone/tablet/
laptop on the hotspot, or the desktop workflow) -- this only ever shows
current scan/tilt status plus where to point a browser to reach that
GUI, nothing else. See README.md's "Onboard status display" section.

Standalone script, not a colcon package: nothing here needs its own
topics/services, just two subscriptions and a render loop, so the extra
ceremony of a full ROS2 package isn't worth it for something this small.

Panel-specific rendering (the actual write-to-screen call, marked below)
is a TODO -- depends on which small GPIO/SPI panel this ends up being
(SPI bus, controller chip, and Python library all vary by panel/vendor),
left as a placeholder until that's known. Everything else here --
gathering the ROS2 status and the current IP -- is hardware-independent
and has actually been verified, not just written and assumed correct:
an isolated-ROS_DOMAIN_ID test constructed this node alongside a
throwaway publisher for both status topics and confirmed the callbacks
received and stored real messages correctly, and current_ip() was
checked against both a real interface (returns a real address) and a
nonexistent one (degrades to "no IP" rather than raising). Also
confirmed the shutdown path (see main()'s comment) doesn't raise a
spurious second exception under a SIGTERM the way an earlier version of
this file did.
"""

from __future__ import annotations

import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def current_ip(iface: str = "wlan0") -> str:
    """Whatever address `iface` currently has -- works unchanged whether
    that's the hotspot's fixed 10.42.0.1 or a DHCP-assigned home-WiFi
    one, so this doesn't need to know or care which mode is currently
    active (see README.md's "Switching between home WiFi and the
    hotspot"). Returns "no IP" rather than raising if the interface has
    none yet (e.g. still associating) or the `ip` command itself fails --
    a status display should degrade to a clear placeholder, not crash."""
    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", iface],
            capture_output=True,
            text=True,
            timeout=2,
        )
        for line in result.stdout.splitlines():
            parts = line.split()
            if "inet" in parts:
                return parts[parts.index("inet") + 1].split("/")[0]
    except Exception:
        pass
    return "no IP"


class StatusDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("status_display")
        self._tilt_status = "-"
        self._scan_status = "-"
        self.create_subscription(String, "/tilt_axis_bridge/status", self._on_tilt, 10)
        self.create_subscription(String, "/scan_aggregator/status", self._on_scan, 10)
        self.create_timer(1.0, self._render)

    def _on_tilt(self, msg: String) -> None:
        self._tilt_status = msg.data

    def _on_scan(self, msg: String) -> None:
        self._scan_status = msg.data

    def _render(self) -> None:
        lines = [
            f"scan: {self._scan_status}",
            f"tilt: {self._tilt_status}",
            f"connect: http://{current_ip()}:8080",
        ]
        # TODO: draw `lines` to the panel -- see this file's own module
        # docstring for why this is still a placeholder.
        self.get_logger().debug("\n".join(lines))


def main() -> None:
    rclpy.init()
    node = StatusDisplayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # The latter is what a SIGTERM (Ctrl-C, or systemd stopping this
        # unit -- the normal way this actually exits once deployed) comes
        # through as, not KeyboardInterrupt alone. Confirmed live: without
        # this, systemctl stop/restart raised a second, spurious "rcl_
        # shutdown already called" error out of the finally block below,
        # since an external shutdown already tears down the context
        # before this function's own rclpy.shutdown() call runs.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
