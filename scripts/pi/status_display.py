#!/usr/bin/env python3
"""Status readout for the Pi's onboard 3.5" GPIO/SPI panel (Elecrow
RR035 / ELEGOO 3.5" -- confirmed the same hardware, 480x320, XPT2046
touch).

That panel is configured (see README.md's "Onboard status display") to
register as a normal Linux console via the mainline `piscreen` DRM
overlay, not a bespoke framebuffer target -- so this needs no
panel-specific graphics library at all, just plain ANSI terminal output.
The console itself already gives boot messages, a login prompt, and a
full interactive shell for general debugging, the same as an HDMI
monitor would; this script is a small optional extra for a compact live
status view, meant to be run when wanted, not something that replaces
normal console/login access.

Standalone script, not a colcon package: nothing here needs its own
topics/services, just two subscriptions and a render loop, so the extra
ceremony of a full ROS2 package isn't worth it for something this small.

Everything here has actually been verified, not just written and
assumed correct: an isolated-ROS_DOMAIN_ID test constructed this node
alongside a throwaway publisher for both status topics and confirmed the
callbacks received and stored real messages correctly, and current_ip()
was checked against both a real interface (returns a real address) and a
nonexistent one (degrades to "no IP" rather than raising). Also
confirmed the shutdown path (see main()'s comment) doesn't raise a
spurious second exception under a SIGTERM the way an earlier version of
this file did. The terminal rendering itself (plain ANSI clear-screen +
print) hasn't been eyeballed against the real console yet -- no
hardware -- but there's no panel-specific unknown left in it to verify;
standard escape codes any ANSI terminal (including the Linux console)
already understands.
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
            "TPL scanner status",
            "-------------------",
            f"scan: {self._scan_status}",
            f"tilt: {self._tilt_status}",
            f"connect: http://{current_ip()}:8080",
        ]
        # \x1b[2J\x1b[H: clear screen + cursor home -- keeps this a
        # clean, single-screen readout each tick on a real terminal
        # rather than scrolling a fresh block every second. Standard
        # ANSI, understood by the Linux console (what the panel becomes
        # via the piscreen overlay) same as any other terminal.
        print("\x1b[2J\x1b[H" + "\n".join(lines), flush=True)


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
