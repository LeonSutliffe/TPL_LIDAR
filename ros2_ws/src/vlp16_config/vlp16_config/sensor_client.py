"""HTTP client for the VLP-16's own embedded web configuration server.

Entirely separate from the ROS driver: velodyne_driver_node just reads UDP
packets the sensor is already broadcasting, it never talks to this HTTP
API. Endpoints/param names below come straight from the VLP-16 User
Manual's web-interface/curl-command sections -- see HANDOFF.md's VLP-16
config section for sourcing and the exact manual pages.

Two things this deliberately does NOT do, both because the documentation
available while building this didn't cover them clearly enough to be
confident:
  - Validate rpm/fov ranges client-side beyond "is an int" -- the sensor
    itself clamps/rejects out-of-range values (300-1200 step 60 for rpm,
    0-359 for fov), so let it be the one source of truth rather than
    duplicating (and risking drifting from) rules read off a manual PDF.
  - Parse get_status()'s response into named fields -- no confirmed
    status.json schema was found (categories are documented: GPS/PPS,
    motor state, rotation phase, laser state -- exact key names aren't).
    Returns the raw parsed dict as-is; map specific fields once real
    hardware is available to check a real response against.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.parse
import urllib.request
from typing import Any


class SensorHttpError(Exception):
    pass


class Vlp16SensorClient:
    def __init__(self, device_ip: str, timeout_s: float = 2.0) -> None:
        self.device_ip = device_ip
        self.timeout_s = timeout_s

    def _post(self, path: str, data: dict) -> None:
        url = f"http://{self.device_ip}{path}"
        body = urllib.parse.urlencode(data).encode()
        req = urllib.request.Request(url, data=body, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                resp.read()
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            raise SensorHttpError(f"POST {path} failed: {exc}") from exc

    def _get_json(self, path: str) -> Any:
        url = f"http://{self.device_ip}{path}"
        try:
            with urllib.request.urlopen(url, timeout=self.timeout_s) as resp:
                raw = resp.read()
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            raise SensorHttpError(f"GET {path} failed: {exc}") from exc
        try:
            return json.loads(raw)
        except ValueError as exc:
            raise SensorHttpError(f"GET {path} returned invalid JSON: {exc}") from exc

    # ---- Layer 2 (sensor hardware) settings ----

    def set_rpm(self, rpm: int) -> None:
        self._post("/cgi/setting", {"rpm": int(rpm)})

    def set_returns(self, returns: str) -> None:
        if returns not in ("Strongest", "Last", "Dual"):
            raise SensorHttpError(f"invalid return type: {returns!r}")
        self._post("/cgi/setting", {"returns": returns})

    def set_fov(self, start: int, end: int) -> None:
        self._post("/cgi/setting/fov", {"start": int(start), "end": int(end)})

    def set_net(self, addr: str, mask: str, gateway: str, dhcp: str) -> None:
        # Always sent together (matches this project's other multi-field
        # "Apply" actions) -- riskiest command here, can strand the sensor
        # at an unreachable address if mistyped. See the GUI's warning copy
        # next to this fieldset.
        self._post("/cgi/setting/net", {"addr": addr, "mask": mask, "gateway": gateway, "dhcp": dhcp})

    def set_host(self, dport: int, tport: int) -> None:
        self._post("/cgi/setting/host", {"dport": int(dport), "tport": int(tport)})

    def save_config(self) -> None:
        self._post("/cgi/save", {"submit": "submit"})

    def reset_sensor(self) -> None:
        self._post("/cgi/reset", {"reset_system": "reset_system"})

    def get_status(self) -> Any:
        return self._get_json("/cgi/status.json")
