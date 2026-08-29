"""Generic command dispatch: {"command": "...", "params": {...}} -> JSON-safe result.

Same rationale as tilt_axis_bridge/commands.py: drives every
Vlp16SensorClient method over one ~/hw_command (request) / ~/hw_response
(result) topic pair instead of a custom .srv per command.
"""

from __future__ import annotations

from typing import Any, Callable

from .sensor_client import Vlp16SensorClient

DispatchFn = Callable[[Vlp16SensorClient, dict], Any]

# command name -> (client method name, ordered param names).
_SIMPLE: dict[str, tuple[str, list[str]]] = {
    "set_rpm": ("set_rpm", ["rpm"]),
    "set_returns": ("set_returns", ["returns"]),
    "set_fov": ("set_fov", ["start", "end"]),
    "set_net": ("set_net", ["addr", "mask", "gateway", "dhcp"]),
    "set_host": ("set_host", ["dport", "tport"]),
    "save_config": ("save_config", []),
    "reset_sensor": ("reset_sensor", []),
}

_CUSTOM: dict[str, DispatchFn] = {
    "get_status": lambda client, params: client.get_status(),
}


class UnknownCommandError(Exception):
    pass


def dispatch(client: Vlp16SensorClient, command: str, params: dict) -> Any:
    if command in _CUSTOM:
        return _CUSTOM[command](client, params)

    if command in _SIMPLE:
        method_name, param_names = _SIMPLE[command]
        args = [params[name] for name in param_names]
        return getattr(client, method_name)(*args)

    raise UnknownCommandError(f"unknown command: {command}")
