"""Generic command dispatch: {"command": "...", "params": {...}} -> JSON-safe result.

Exists so the advanced-options GUI can drive every MksDriver method over a
single ~/driver_command (request) / ~/driver_response (result) topic pair
instead of needing ~40 individually-typed ROS2 services (which would each
need a custom .srv interface package). Trades some type safety for a much
smaller, entirely-Python implementation -- appropriate for an operator GUI
where params come from a handful of known form fields, not arbitrary input.
"""

from __future__ import annotations

from typing import Any, Callable

from .mks_driver import MksDriver

DispatchFn = Callable[[MksDriver, dict], Any]


def _dispatch_encoder(driver: MksDriver, params: dict) -> Any:
    carry, value = driver.read_encoder()
    return {"carry": carry, "value": value}


def _dispatch_version(driver: MksDriver, params: dict) -> Any:
    calibrated, hw, fw = driver.read_version()
    return {"calibrated": calibrated, "hardware_version": hw, "firmware_version": fw.hex()}


def _dispatch_all_config_params(driver: MksDriver, params: dict) -> Any:
    return {"hex": driver.read_all_config_params().hex()}


def _dispatch_config_fields(driver: MksDriver, params: dict) -> Any:
    return driver.read_config_params()


def _dispatch_all_status_params(driver: MksDriver, params: dict) -> Any:
    return {"hex": driver.read_all_status_params().hex()}


def _dispatch_write_all_config_params(driver: MksDriver, params: dict) -> Any:
    return driver.write_all_config_params(bytes.fromhex(params["hex"]))


# command name -> (driver method name, ordered param names).
# Params are pulled from the request dict in this order; anything the
# handler above doesn't override falls through to a plain positional call.
_SIMPLE: dict[str, tuple[str, list[str]]] = {
    # Motion / bus control
    "set_enable": ("set_enable", ["enable"]),
    "emergency_stop": ("emergency_stop", []),
    "read_motor_status": ("read_motor_status", []),
    "run_speed": ("run_speed", ["direction", "speed_rpm", "acc"]),
    "stop_speed": ("stop_speed", ["acc"]),
    "move_relative_pulses": ("move_relative_pulses", ["direction", "speed_rpm", "acc", "pulses"]),
    "stop_relative_pulses": ("stop_relative_pulses", ["acc"]),
    "move_absolute_pulses": ("move_absolute_pulses", ["speed_rpm", "acc", "absolute_pulses"]),
    "stop_absolute_pulses": ("stop_absolute_pulses", ["acc"]),
    "move_relative_axis": ("move_relative_axis", ["speed_rpm", "acc", "relative_axis"]),
    "stop_relative_axis": ("stop_relative_axis", ["acc"]),
    "move_absolute_axis": ("move_absolute_axis", ["speed_rpm", "acc", "absolute_axis"]),
    "stop_absolute_axis": ("stop_absolute_axis", ["acc"]),
    "go_home": ("go_home", ["coordinate_homing"]),
    "set_zero_point": ("set_zero_point", []),
    "release_stall": ("release_stall", []),
    # Read-only status
    "read_speed": ("read_speed", []),
    "read_pulses": ("read_pulses", []),
    "read_angle_error": ("read_angle_error", []),
    "read_enable_status": ("read_enable_status", []),
    "read_stall_status": ("read_stall_status", []),
    "read_io_status": ("read_io_status", []),
    # Diagnostics / identity
    "calibrate_encoder": ("calibrate_encoder", []),
    "restore_factory_defaults": ("restore_factory_defaults", []),
    "reset_and_restart": ("reset_and_restart", []),
    "write_user_id": ("write_user_id", ["user_id"]),
    "read_user_id": ("read_user_id", []),
    # Motor configuration
    "set_work_mode": ("set_work_mode", ["mode"]),
    "set_working_current": ("set_working_current", ["milliamps", "save"]),
    "set_microstep": ("set_microstep", ["microstep"]),
    "set_slave_address": ("set_slave_address", ["new_address"]),
    "set_enable_pin_level": ("set_enable_pin_level", ["level"]),
    "set_direction": ("set_direction", ["direction"]),
    "set_auto_screen_off": ("set_auto_screen_off", ["enable"]),
    "set_overcurrent_protect": ("set_overcurrent_protect", ["enable"]),
    "set_microstep_interpolation": ("set_microstep_interpolation", ["enable"]),
    "set_baud_rate": ("set_baud_rate", ["baud_code"]),
    "set_holding_current_percent": ("set_holding_current_percent", ["ratio"]),
    # Protection
    "set_position_protect": ("set_position_protect", ["enable", "tim", "errors"]),
    "set_heartbeat_protect_time": ("set_heartbeat_protect_time", ["milliseconds"]),
    # Homing / limits
    "set_home_params": (
        "set_home_params",
        ["trigger_high", "home_direction", "home_speed_rpm", "end_limit_enable"],
    ),
    "set_home_torque_offset": (
        "set_home_torque_offset",
        ["origin_offset", "home_mode", "home_current_ma"],
    ),
    "set_single_turn_zero_return": (
        "set_single_turn_zero_return",
        ["mode", "zero_action", "speed_tier", "direction"],
    ),
    "set_position_threshold": ("set_position_threshold", ["enable", "threshold"]),
    "set_limit_port_remap": ("set_limit_port_remap", ["enable"]),
    # Closed-loop tuning (5.3) -- factory-tuned, change with caution.
    # set_pid_vfoc deliberately not exposed -- FOC work mode isn't
    # reachable from this GUI anymore (see Work mode select), so its PID
    # register is dead weight.
    "set_pid_close": ("set_pid_close", ["kp", "ki", "kd", "kv"]),
    # Bus / multi-motor
    "set_respond_and_active": ("set_respond_and_active", ["respond", "active"]),
    "set_group_address": ("set_group_address", ["group_address"]),
    "set_modbus_rtu": ("set_modbus_rtu", ["enable"]),
    "set_key_lock": ("set_key_lock", ["locked"]),
    "set_auto_run_on_power_on": ("set_auto_run_on_power_on", ["action"]),
}

_CUSTOM: dict[str, DispatchFn] = {
    "read_encoder": _dispatch_encoder,
    "read_version": _dispatch_version,
    "read_all_config_params": _dispatch_all_config_params,
    "read_config_fields": _dispatch_config_fields,
    "read_all_status_params": _dispatch_all_status_params,
    "write_all_config_params": _dispatch_write_all_config_params,
    "read_cumulative_encoder": lambda d, p: d.read_cumulative_encoder(),
}


class UnknownCommandError(Exception):
    pass


def dispatch(driver: MksDriver, command: str, params: dict) -> Any:
    if command in _CUSTOM:
        return _CUSTOM[command](driver, params)

    if command in _SIMPLE:
        method_name, param_names = _SIMPLE[command]
        args = [params[name] for name in param_names]
        return getattr(driver, method_name)(*args)

    raise UnknownCommandError(f"unknown command: {command}")
