#!/usr/bin/env python3
"""GUI-facing bridge for this rig's two NetworkManager WiFi profiles --
home WiFi (client mode, for internet/SSH at a fixed location) and the
Pi's own fallback hotspot (AP mode, for the field with no other network
around). See README.md's "Wi-Fi hotspot" setup step for how both profiles
were created; this just lets the GUI read and change their SSID/password
afterward instead of requiring SSH + nmcli by hand.

Standalone script, not a colcon package -- same reasoning as
scripts/pi/status_display.py: nothing here needs its own build, just a
couple of request/response topic pairs on plain std_msgs/String (JSON
bodies, no custom .srv/.msg type), so the extra ceremony of a full ROS2
package isn't worth it for something this small. Not part of
scanner_bringup's launch tree either -- ROS2 discovery is domain-wide,
not launch-file-scoped, so this reaches rosbridge/the GUI exactly the
same way whether it's launched from there or, as here, its own
independent systemd service (tpl-wifi-config.service).

Hardcodes this rig's two real profile names -- same "hardcode this rig's
real values" convention already used throughout this project (OUTPUT_DIR,
USB_EXPORT_DIR, the mount angle defaults) rather than a settings-file
lookup for something that isn't expected to change without a deliberate
code edit.

Deliberately never runs `nmcli connection up` after a change: modifying
an active connection's SSID can make NetworkManager reconnect it
immediately, which would drop whatever session (SSH, or the GUI's own
WiFi if it's currently joined to the profile being changed) is currently
using it -- see README.md step 8's own documented version of this exact
risk. A saved change takes effect the next time NetworkManager
(auto-)connects to that profile on its own, not forced here.
"""

from __future__ import annotations

import json
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HOME_WIFI_PROFILE = "netplan-wlan0-LiFi"
HOTSPOT_PROFILE = "TPL-Hotspot"

# WPA-PSK passphrase length limits -- nmcli/wpa_supplicant reject anything
# outside this range outright, so check here and return a clear message
# rather than surfacing that raw failure.
MIN_PSK_LEN = 8
MAX_PSK_LEN = 63

# SSID is a 1-32 byte field per the 802.11 spec.
MAX_SSID_LEN = 32


def _run(args: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(args, capture_output=True, text=True, check=False)


def _get_field(profile: str, field: str) -> str | None:
    """One nmcli call per field rather than parsing a multi-value -g
    line -- simpler and more robust than getting the separator convention
    exactly right for a handful of fields called this infrequently.
    sudo is required here, not just for the modify path below -- confirmed
    live (a plain --show-secrets read came back empty for the psk fields,
    sudo the same call returned the real value): both profiles were
    created via `sudo nmcli connection add` per README's setup step, which
    makes them root-owned system connections, so even reading their own
    secrets back needs the same privilege as changing them."""
    result = _run(["sudo", "nmcli", "--show-secrets", "-g", field, "connection", "show", profile])
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def _get_active_connections() -> set[str]:
    result = _run(["nmcli", "-t", "-f", "NAME", "connection", "show", "--active"])
    if result.returncode != 0:
        return set()
    return set(result.stdout.strip().split("\n")) if result.stdout.strip() else set()


def _validate_ssid_password(ssid: str, password: str) -> str | None:
    """Returns an error string, or None if both are valid."""
    if not ssid or len(ssid.encode("utf-8")) > MAX_SSID_LEN:
        return f"ssid must be 1-{MAX_SSID_LEN} bytes"
    if not (MIN_PSK_LEN <= len(password) <= MAX_PSK_LEN):
        return f"password must be {MIN_PSK_LEN}-{MAX_PSK_LEN} characters (WPA-PSK requirement)"
    return None


def _set_profile(profile: str, ssid: str, password: str) -> str | None:
    """Returns an error string, or None on success."""
    error = _validate_ssid_password(ssid, password)
    if error:
        return error
    result = _run(
        [
            "sudo", "nmcli", "connection", "modify", profile,
            "802-11-wireless.ssid", ssid,
            "wifi-sec.key-mgmt", "wpa-psk",
            "wifi-sec.psk", password,
        ]
    )
    if result.returncode != 0:
        return result.stderr.strip() or f"nmcli exited {result.returncode}"
    return None


class WifiConfigNode(Node):
    def __init__(self) -> None:
        super().__init__("wifi_config")

        self._response_pub = self.create_publisher(String, "~/get_network_config_response", 10)
        self._set_home_response_pub = self.create_publisher(String, "~/set_home_wifi_response", 10)
        self._set_hotspot_response_pub = self.create_publisher(String, "~/set_hotspot_response", 10)

        self.create_subscription(
            String, "~/get_network_config_request", self._on_get_network_config, 10
        )
        self.create_subscription(String, "~/set_home_wifi_request", self._on_set_home_wifi, 10)
        self.create_subscription(String, "~/set_hotspot_request", self._on_set_hotspot, 10)

        self.get_logger().info("wifi_config ready")

    def _on_get_network_config(self, msg: String) -> None:
        try:
            request = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self._publish(self._response_pub, {"id": None, "error": f"bad request: {exc}"})
            return

        active = _get_active_connections()
        payload = {
            "id": request.get("id"),
            "home_ssid": _get_field(HOME_WIFI_PROFILE, "802-11-wireless.ssid"),
            "home_psk": _get_field(HOME_WIFI_PROFILE, "802-11-wireless-security.psk"),
            "hotspot_ssid": _get_field(HOTSPOT_PROFILE, "802-11-wireless.ssid"),
            "hotspot_psk": _get_field(HOTSPOT_PROFILE, "802-11-wireless-security.psk"),
            "home_active": HOME_WIFI_PROFILE in active,
            "hotspot_active": HOTSPOT_PROFILE in active,
            "error": None,
        }
        self._publish(self._response_pub, payload)

    def _on_set_home_wifi(self, msg: String) -> None:
        self._handle_set(msg, HOME_WIFI_PROFILE, self._set_home_response_pub)

    def _on_set_hotspot(self, msg: String) -> None:
        self._handle_set(msg, HOTSPOT_PROFILE, self._set_hotspot_response_pub)

    def _handle_set(self, msg: String, profile: str, response_pub) -> None:
        try:
            request = json.loads(msg.data)
            ssid = request["ssid"]
            password = request["password"]
        except (TypeError, ValueError, KeyError) as exc:
            self._publish(response_pub, {"id": None, "error": f"bad request: {exc}"})
            return

        req_id = request.get("id")
        error = _set_profile(profile, ssid, password)
        self._publish(response_pub, {"id": req_id, "error": error})

    def _publish(self, pub, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = WifiConfigNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
