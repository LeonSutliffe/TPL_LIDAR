"""Blocking MKS SERVO42/57D driver over the Pico USB-CDC<->RS485 bridge.

Ported from D:\\Downloads\\MKS_Servo_Tester\\app\\MksServoTester.Protocol\\
MksDriver.cs (manual: "MKS SERVO42&57D_RS485 User Manual" V1.0.9 -- section
numbers in comments refer to it). The Pico bridge
(D:\\Downloads\\MKS_Servo_Tester\\src\\main.c) is a transparent byte pipe --
all protocol logic lives here, same as it did in the C# tester.
"""

from __future__ import annotations

import struct
import threading
import time

import serial

from .mks_protocol import (
    UPLINK_HEADER,
    FunctionCode,
    MksProtocolError,
    build_downlink,
    parse_uplink,
)

DEFAULT_TIMEOUT_S = 0.5


class MksCommandError(Exception):
    pass


class MksDriver:
    # Project-wide hard safety ceiling -- this axis must never be commanded
    # faster than this, full stop. Enforced in _rpm_to_wire_speed(), the one
    # choke point every speed-taking method (run_speed, move_*, set_home_params)
    # already converts real RPM through, so this covers the whole program
    # (state machine moves, sweep, autotune, homing, and the GUI's raw
    # driver-command escape hatch) without needing to be checked anywhere
    # else. Not configurable at runtime -- deliberately a code change, not a
    # parameter, so it can't be raised by mistake from the GUI.
    MAX_ALLOWED_RPM = 40

    # Same philosophy as MAX_ALLOWED_RPM: a hard, code-level ceiling rather
    # than a runtime parameter. Caps total physical excursion from the
    # home/zero position (set by go_home + set_zero_point) to this many deg
    # in *either* direction (a raw, symmetric bound in this driver's own
    # axis-count coordinate system -- it has no concept of reverse_direction,
    # so it can't express the real, asymmetric 0-TILT_MAX_DEG operating
    # range that tilt_axis_bridge's TILT_MIN_DEG/TILT_MAX_DEG enforce a
    # layer above
    # this one; see that node's _rad_to_axis for the actual authoritative
    # limit, applied to *everything* with a destination there including
    # jog, which targets whichever boundary its commanded direction points
    # at rather than running an open-ended speed-mode run_speed jog -- see
    # _apply_jog_speed's docstring for why). This is the defense-in-depth
    # layer underneath: it exists specifically to still catch a wildly
    # out-of-range raw command from the GUI's driver-command escape hatch
    # (Config tab's raw Position mode 3/4 fields), which bypasses
    # tilt_axis_bridge's conversion layer entirely and so isn't reachable
    # by that node-level check at all. Enforced in _check_rotation_limit(),
    # called from move_absolute_axis/move_relative_axis (11.3/11.4,
    # "position mode 3/4") -- those share read_cumulative_encoder's fixed
    # 16384 counts/turn coordinate system (5.1.2). Deliberately NOT
    # enforced on the pulse-count position modes (move_absolute_pulses/
    # move_relative_pulses, 11.1/11.2) or run_speed's open-ended continuous
    # jog (10.1) at all -- pulses are microstep-scaled and this driver
    # doesn't track the motor's steps/rev, and nothing in this project
    # calls run_speed anymore (jog moved off it, see above). Both stay
    # manual-testing-only paths, same as this codebase's existing
    # bulk-config-write escape hatch (see commands.py).
    # Kept equal to tilt_axis_bridge's TILT_MAX_DEG so this coarser layer
    # is never looser than the authoritative one above it -- 270 initially,
    # tightened to 260 after real-hardware testing (see that node's own
    # TILT_MAX_DEG for the up-to-date value if these ever drift apart).
    MAX_ROTATION_DEG = 260.0
    COUNTS_PER_REV = 16384  # 5.1.2 -- same fixed encoder resolution move_*_axis targets
    MAX_ROTATION_AXIS = round(MAX_ROTATION_DEG / 360.0 * COUNTS_PER_REV)

    def __init__(self, port: str, address: int = 1, baudrate: int = 115200):
        self._address = address
        # The wire `speed` field used by run_speed/move_*/set_home_params is
        # calibrated against 16 microsteps (manual 9.1): actual RPM =
        # wire_speed * 16 / microstep. Every method on this class takes real
        # RPM and converts via _rpm_to_wire_speed() using whatever
        # microstep this driver last knew about, so callers never have to
        # think about it. Defaults to the hardware's own default (16);
        # kept in sync by set_microstep() and by read_config_params()
        # picking up whatever the driver actually reports.
        self._microstep = 16
        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = baudrate
        self._serial.timeout = DEFAULT_TIMEOUT_S
        # The Pico's USB-CDC stdio only starts forwarding bytes to a host
        # that has asserted DTR (confirmed this session) -- without this the
        # link silently drops every request.
        self._serial.dtr = True
        self._serial.rts = True
        self._lock = threading.Lock()

    def open(self) -> None:
        self._serial.open()

    def close(self) -> None:
        self._serial.close()

    def reconfigure(self, port: str, baudrate: int) -> None:
        """Repoint this driver at a (possibly different) port/baud before
        the next open() -- e.g. after the Pico bridge's own UART1 speed was
        changed to follow a SetBaudRate (0x8A) issued to the driver itself.
        Safe to call whether currently open or closed."""
        if self._serial.is_open:
            self._serial.close()
        self._serial.port = port
        self._serial.baudrate = baudrate

    def __enter__(self) -> "MksDriver":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _send(self, function_code: int, data: bytes, expected_data_length: int) -> bytes:
        request = build_downlink(self._address, function_code, data)
        reply_length = 3 + expected_data_length + 1

        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(request)
            self._serial.flush()

            deadline = time.monotonic() + DEFAULT_TIMEOUT_S

            # Resync on the uplink header instead of assuming byte-aligned
            # reads -- the Pico's USB-CDC link reliably prepends one stray
            # 0x00 byte before the real frame (confirmed reproducible, not
            # random corruption), so the first byte read is often not the
            # header.
            header_byte = b""
            while header_byte != bytes([UPLINK_HEADER]) and time.monotonic() < deadline:
                header_byte = self._serial.read(1)
                if not header_byte:
                    break

            reply = bytearray(header_byte)
            while len(reply) < reply_length and time.monotonic() < deadline:
                chunk = self._serial.read(reply_length - len(reply))
                if not chunk:
                    break
                reply.extend(chunk)

        if len(reply) != reply_length:
            raise MksCommandError(
                f"function 0x{function_code:02X}: expected {reply_length} bytes, "
                f"got {len(reply)}"
            )

        try:
            frame = parse_uplink(bytes(reply), expected_data_length)
        except MksProtocolError as exc:
            raise MksCommandError(f"function 0x{function_code:02X}: {exc}") from exc

        return frame.data

    def _rpm_to_wire_speed(self, rpm: int) -> int:
        """Real target RPM -> raw wire `speed` field, correcting for the
        current microstep (see __init__). Raises rather than silently
        clamping/truncating: at high microstep the achievable real RPM
        range shrinks (wire field is capped 0-3000), so a request that
        doesn't fit is a real "can't get there from here", not a rounding
        detail to paper over. Same reasoning applies to MAX_ALLOWED_RPM --
        a caller asking for more than the project's hard limit is a bug to
        surface loudly, not a value to quietly cap.
        """
        if rpm < 0 or rpm > self.MAX_ALLOWED_RPM:
            raise ValueError(
                f"{rpm} RPM exceeds this project's hard limit of "
                f"{self.MAX_ALLOWED_RPM} RPM (MksDriver.MAX_ALLOWED_RPM)"
            )
        wire_speed = round(rpm * self._microstep / 16)
        if not (0 <= wire_speed <= 3000):
            max_rpm = 3000 * 16 / self._microstep
            raise ValueError(
                f"{rpm} RPM at microstep={self._microstep} needs wire speed "
                f"{wire_speed}, outside the driver's valid 0-3000 range "
                f"(max achievable is ~{max_rpm:.0f} RPM at this microstep)"
            )
        return wire_speed

    def _wire_speed_to_rpm(self, wire_speed: int) -> int:
        """Inverse of _rpm_to_wire_speed(), for fields read back from the
        driver (e.g. home_speed_rpm in read_config_params)."""
        return round(wire_speed * 16 / self._microstep)

    def _encode_dir_speed(self, direction: int, speed_rpm: int) -> bytes:
        wire_speed = self._rpm_to_wire_speed(speed_rpm)
        hi = ((direction & 1) << 7) | ((wire_speed >> 8) & 0x0F)
        lo = wire_speed & 0xFF
        return bytes([hi, lo])

    def _check_rotation_limit(self, absolute_axis: int) -> None:
        """See MAX_ROTATION_DEG. Raises rather than clamping, same
        reasoning as _rpm_to_wire_speed: a target past this project's hard
        limit is a bug to surface loudly, not a value to quietly cap."""
        if abs(absolute_axis) > self.MAX_ROTATION_AXIS:
            deg = absolute_axis / self.COUNTS_PER_REV * 360.0
            raise ValueError(
                f"target position {absolute_axis} counts ({deg:.1f} deg from home) "
                f"exceeds this project's hard rotation limit of "
                f"{self.MAX_ROTATION_DEG} deg (MksDriver.MAX_ROTATION_DEG)"
            )

    # ---- 9.2 Bus control general instructions ----

    def set_enable(self, enable: bool) -> bool:
        """9.2.2 SetEnable (F3H) -- only meaningful in SR_OPEN/SR_CLOSE/SR_vFOC work modes."""
        data = self._send(FunctionCode.SET_ENABLE, bytes([1 if enable else 0]), 1)
        return data[0] == 1

    def emergency_stop(self) -> bool:
        """9.2.3 EmergencyStop (F7H). Manual warns against this above 1000 RPM."""
        data = self._send(FunctionCode.EMERGENCY_STOP, b"", 1)
        return data[0] == 1

    def read_motor_status(self) -> int:
        """9.2.1 ReadMotorStatus (F1H) -- only valid in bus control modes.

        0=QueryFailed 1=Stopped 2=SpeedingUp 3=SpeedingDown 4=FullSpeed
        5=Homing 6=Calibrating
        """
        data = self._send(FunctionCode.READ_MOTOR_STATUS, b"", 1)
        return data[0]

    # ---- Part 10: speed control mode ----

    def run_speed(self, direction: int, speed_rpm: int, acc: int) -> int:
        """10.1 Run the motor continuously in speed mode (F6H)."""
        data = self._encode_dir_speed(direction, speed_rpm) + bytes([acc])
        resp = self._send(FunctionCode.RUN_SPEED_MODE, data, 1)
        return resp[0]

    def stop_speed(self, acc: int = 0) -> int:
        """10.2 Stop the motor in speed mode. acc=0 stops immediately."""
        resp = self._send(FunctionCode.RUN_SPEED_MODE, bytes([0, 0, acc]), 1)
        return resp[0]

    # ---- Part 11: position control modes ----

    def move_relative_pulses(self, direction: int, speed_rpm: int, acc: int, pulses: int) -> int:
        """11.1.1 Position mode 1: relative motion by pulse count (FDH)."""
        data = self._encode_dir_speed(direction, speed_rpm) + bytes([acc]) + struct.pack(">I", pulses)
        resp = self._send(FunctionCode.RELATIVE_MOVE_BY_PULSES, data, 1)
        return resp[0]

    def stop_relative_pulses(self, acc: int = 0) -> int:
        data = bytes([0, 0, acc]) + struct.pack(">I", 0)
        resp = self._send(FunctionCode.RELATIVE_MOVE_BY_PULSES, data, 1)
        return resp[0]

    def move_absolute_pulses(self, speed_rpm: int, acc: int, absolute_pulses: int) -> int:
        """11.2.1 Position mode 2: absolute motion by pulse count (FEH)."""
        data = (
            struct.pack(">H", self._rpm_to_wire_speed(speed_rpm))
            + bytes([acc])
            + struct.pack(">i", absolute_pulses)
        )
        resp = self._send(FunctionCode.ABSOLUTE_MOVE_BY_PULSES, data, 1)
        return resp[0]

    def stop_absolute_pulses(self, acc: int = 0) -> int:
        data = bytes([0, 0, acc]) + struct.pack(">i", 0)
        resp = self._send(FunctionCode.ABSOLUTE_MOVE_BY_PULSES, data, 1)
        return resp[0]

    def move_relative_axis(self, speed_rpm: int, acc: int, relative_axis: int) -> int:
        """11.3.1 Position mode 3: relative motion by axis/coordinate value (F4H)."""
        # Needs an extra round trip to know where "relative to" actually is
        # -- see MAX_ROTATION_DEG.
        current = self.read_cumulative_encoder()
        self._check_rotation_limit(current + relative_axis)
        data = (
            struct.pack(">H", self._rpm_to_wire_speed(speed_rpm))
            + bytes([acc])
            + struct.pack(">i", relative_axis)
        )
        resp = self._send(FunctionCode.RELATIVE_MOVE_BY_AXIS, data, 1)
        return resp[0]

    def stop_relative_axis(self, acc: int = 0) -> int:
        data = bytes([0, 0, acc]) + struct.pack(">i", 0)
        resp = self._send(FunctionCode.RELATIVE_MOVE_BY_AXIS, data, 1)
        return resp[0]

    def move_absolute_axis(self, speed_rpm: int, acc: int, absolute_axis: int) -> int:
        """11.4.1 Position mode 4: absolute motion by axis/coordinate value (F5H)."""
        self._check_rotation_limit(absolute_axis)
        data = (
            struct.pack(">H", self._rpm_to_wire_speed(speed_rpm))
            + bytes([acc])
            + struct.pack(">i", absolute_axis)
        )
        resp = self._send(FunctionCode.ABSOLUTE_MOVE_BY_AXIS, data, 1)
        return resp[0]

    def stop_absolute_axis(self, acc: int = 0) -> int:
        data = bytes([0, 0, acc]) + struct.pack(">i", 0)
        resp = self._send(FunctionCode.ABSOLUTE_MOVE_BY_AXIS, data, 1)
        return resp[0]

    # ---- 5.1 Read-only parameters ----

    def read_encoder(self) -> tuple[int, int]:
        """5.1.1 ReadEncoder (30H) -- single-turn value with a separate turn-carry counter."""
        data = self._send(FunctionCode.READ_ENCODER, b"", 6)
        carry = struct.unpack(">i", data[0:4])[0]
        value = struct.unpack(">H", data[4:6])[0]
        return carry, value

    def read_cumulative_encoder(self) -> int:
        """5.1.2 ReadCumulativeEncoder (31H) -- int48 coordinate value, 16384 counts/turn."""
        data = self._send(FunctionCode.READ_CUMULATIVE_ENCODER, b"", 6)
        return int.from_bytes(data, byteorder="big", signed=True)

    def read_speed(self) -> int:
        """5.1.3 ReadSpeed (32H) -- real-time RPM, negative = clockwise."""
        data = self._send(FunctionCode.READ_SPEED, b"", 2)
        return struct.unpack(">h", data)[0]

    def read_pulses(self) -> int:
        """5.1.4 ReadPulses (33H) -- pulses received."""
        data = self._send(FunctionCode.READ_PULSES, b"", 4)
        return struct.unpack(">i", data)[0]

    def read_angle_error(self) -> int:
        """5.1.6 ReadAngleError (39H) -- units of 51200 per 360 degrees."""
        data = self._send(FunctionCode.READ_ANGLE_ERROR, b"", 4)
        return struct.unpack(">i", data)[0]

    def read_enable_status(self) -> bool:
        """5.1.7 ReadEnableStatus (3AH)."""
        data = self._send(FunctionCode.READ_ENABLE_STATUS, b"", 1)
        return data[0] == 1

    def read_stall_status(self) -> bool:
        """5.4.3 ReadStallStatus (3EH)."""
        data = self._send(FunctionCode.READ_STALL_STATUS, b"", 1)
        return data[0] == 1

    def release_stall(self) -> bool:
        """5.4.4 ReleaseStall (3DH)."""
        data = self._send(FunctionCode.RELEASE_STALL, b"", 1)
        return data[0] == 1

    # ---- 5.2 Write-only parameter commands (configuration) ----

    def set_work_mode(self, mode: int) -> bool:
        """5.2.2 SetWorkMode (82H). Bus control commands (F-series) need a Bus* mode."""
        data = self._send(FunctionCode.SET_WORK_MODE, bytes([mode]), 1)
        return data[0] == 1

    def set_working_current(self, milliamps: int, save: bool = True) -> int:
        """5.2.3 SetWorkingCurrent (83H). 0=Failed 1=SetAndSaved 2=SetWithoutSaving."""
        data = struct.pack(">H", milliamps)
        if not save:
            data += bytes([0x00])
        resp = self._send(FunctionCode.SET_WORKING_CURRENT, data, 1)
        return resp[0]

    def set_microstep(self, microstep: int) -> bool:
        """5.2.5 SetMicrostep (84H), 1-256.

        The wire field is a single byte, which can only hold 0-255 -- 256
        doesn't fit, so (matching the on-screen MStep menu, which offers
        256 as a normal option) it's sent as byte value 0. Without this,
        `bytes([256])` raises ValueError before anything reaches the wire,
        which is why entering 256 used to fail outright.
        """
        if not 1 <= microstep <= 256:
            raise ValueError(f"microstep must be 1-256, got {microstep}")
        data = self._send(FunctionCode.SET_MICROSTEP, bytes([microstep % 256]), 1)
        ok = data[0] == 1
        if ok:
            # Keep _rpm_to_wire_speed's calibration current -- see __init__.
            self._microstep = microstep
        return ok

    def set_slave_address(self, new_address: int) -> bool:
        """5.2.11 SetSlaveAddress (8BH). Takes effect immediately."""
        data = self._send(FunctionCode.SET_SLAVE_ADDRESS, bytes([new_address]), 1)
        ok = data[0] == 1
        if ok:
            self._address = new_address
        return ok

    def set_zero_point(self) -> bool:
        """7.3.5 SetZeroPoint (92H) -- marks the current position as the zero coordinate."""
        data = self._send(FunctionCode.SET_ZERO_POINT, b"", 1)
        return data[0] == 1

    def go_home(self, coordinate_homing: bool = False) -> int:
        """7.3.6 GoHome (91H). 0=fail 1=started 2=success."""
        data = self._send(FunctionCode.GO_HOME, bytes([1 if coordinate_homing else 0]), 1)
        return data[0]

    # ---- Calibration ----

    def calibrate_encoder(self) -> int:
        """5.2.1 CalibrateEncoder (80H). Motor must be unloaded. 0=in progress 1=success 2=failed."""
        data = self._send(FunctionCode.CALIBRATE_ENCODER, bytes([0x00]), 1)
        return data[0]

    # ---- Diagnostics / identity ----

    def read_version(self) -> tuple[bool, int, bytes]:
        """5.1.11 ReadVersion (40H)."""
        data = self._send(FunctionCode.READ_VERSION, b"", 4)
        calibrated = (data[0] >> 4) != 0
        hardware_version = data[0] & 0x0F
        firmware_version = data[1:4]
        return calibrated, hardware_version, firmware_version

    def read_io_status(self) -> int:
        """6.1 ReadIoStatus (34H). Bit0=IN_1 bit1=IN_2 bit2=OUT_1 bit3=OUT_2."""
        data = self._send(FunctionCode.READ_IO_STATUS, b"", 1)
        return data[0]

    def write_user_id(self, user_id: int) -> bool:
        """5.1.12 WriteUserId (42H)."""
        data = self._send(FunctionCode.READ_WRITE_USER_ID, struct.pack(">I", user_id), 1)
        return data[0] == 1

    def read_user_id(self) -> int:
        """5.1.12 ReadUserId (42H)."""
        data = self._send(FunctionCode.READ_WRITE_USER_ID, b"", 4)
        return struct.unpack(">I", data)[0]

    # ---- Reset ----

    def restore_factory_defaults(self) -> bool:
        """5.5.1 RestoreFactoryDefaults (3FH). Driver auto-restarts."""
        data = self._send(FunctionCode.RESTORE_FACTORY_DEFAULTS, b"", 1)
        return data[0] == 1

    def reset_and_restart(self) -> bool:
        """5.5.2 ResetAndRestart (41H)."""
        data = self._send(FunctionCode.RESET_AND_RESTART, b"", 1)
        return data[0] == 1

    # ---- Motor configuration ----

    def set_enable_pin_level(self, level: int) -> bool:
        """5.2.6 SetEnablePinLevel (85H). 0=ActiveLow 1=ActiveHigh 2=AlwaysHold."""
        data = self._send(FunctionCode.SET_ENABLE_PIN_LEVEL, bytes([level]), 1)
        return data[0] == 1

    def set_direction(self, direction: int) -> bool:
        """5.2.7 SetDirection (86H) -- pulse-interface only. 0=CW 1=CCW."""
        data = self._send(FunctionCode.SET_DIRECTION, bytes([direction]), 1)
        return data[0] == 1

    def set_auto_screen_off(self, enable: bool) -> bool:
        """5.2.8 SetAutoScreenOff (87H)."""
        data = self._send(FunctionCode.SET_AUTO_SCREEN_OFF, bytes([1 if enable else 0]), 1)
        return data[0] == 1

    def set_overcurrent_protect(self, enable: bool) -> bool:
        """5.4.1 SetOvercurrentProtect (88H) -- "Protect" on screen."""
        data = self._send(FunctionCode.SET_OVERCURRENT_PROTECT, bytes([1 if enable else 0]), 1)
        return data[0] == 1

    def set_microstep_interpolation(self, enable: bool) -> bool:
        """5.2.9 SetMicrostepInterpolation (89H) -- "MPlyer" on screen."""
        data = self._send(FunctionCode.SET_MICROSTEP_INTERPOLATION, bytes([1 if enable else 0]), 1)
        return data[0] == 1

    def set_baud_rate(self, baud_code: int) -> bool:
        """5.2.10 SetBaudRate (8AH). Takes effect immediately -- must match the Pico
        bridge's hardcoded UART1 baud afterwards or comms break."""
        data = self._send(FunctionCode.SET_BAUD_RATE, bytes([baud_code]), 1)
        return data[0] == 1

    def set_holding_current_percent(self, ratio: int) -> bool:
        """5.2.4 SetHoldingCurrentPercent (9BH). ratio: 0=10% ... 8=90%. OPEN/CLOSE only."""
        data = self._send(FunctionCode.SET_HOLDING_CURRENT_PERCENT, bytes([ratio]), 1)
        return data[0] == 1

    # ---- Protection ----

    def set_position_protect(self, enable: bool, tim: int, errors: int) -> bool:
        """5.4.2 SetPositionProtect (9DH). tim ~15ms units, errors = threshold (28000=360deg)."""
        data = bytes([1 if enable else 0]) + struct.pack(">H", tim) + struct.pack(">H", errors)
        resp = self._send(FunctionCode.SET_POSITION_PROTECT, data, 1)
        return resp[0] == 1

    def set_heartbeat_protect_time(self, milliseconds: int) -> bool:
        """5.2.16 SetHeartbeatProtectTime (98H). 0 disables."""
        data = self._send(FunctionCode.SET_HEARTBEAT_PROTECT_TIME, struct.pack(">I", milliseconds), 1)
        return data[0] == 1

    # ---- Homing / limits ----

    def set_home_params(
        self, trigger_high: bool, home_direction: int, home_speed_rpm: int, end_limit_enable: bool
    ) -> bool:
        """7.3.2 SetHomeParams (90H) -- endstop trigger level/direction/speed, limit-fn enable."""
        data = (
            bytes([1 if trigger_high else 0, home_direction])
            + struct.pack(">H", self._rpm_to_wire_speed(home_speed_rpm))
            + bytes([1 if end_limit_enable else 0])
        )
        resp = self._send(FunctionCode.SET_HOME_PARAMS, data, 1)
        return resp[0] == 1

    def set_home_torque_offset(self, origin_offset: int, home_mode: int, home_current_ma: int) -> bool:
        """7.3.3 SetHomeTorqueOffset (94H). offset: 0x4000=360deg 0x2000=180deg (default)."""
        data = struct.pack(">i", origin_offset) + bytes([home_mode]) + struct.pack(">H", home_current_ma)
        resp = self._send(FunctionCode.SET_HOME_TORQUE_OFFSET, data, 1)
        return resp[0] == 1

    def set_single_turn_zero_return(
        self, mode: int, zero_action: int, speed_tier: int, direction: int
    ) -> bool:
        """7.3.4 SetSingleTurnZeroReturn (9AH) -- "0_Mode" etc on screen."""
        data = self._send(
            FunctionCode.SET_SINGLE_TURN_ZERO_RETURN,
            bytes([mode, zero_action, speed_tier, direction]),
            1,
        )
        return data[0] == 1

    def set_position_threshold(self, enable: bool, threshold: int) -> bool:
        """5.2.17 SetPositionThreshold (95H) -- "position completed" tolerance, bus mode."""
        data = bytes([1 if enable else 0]) + struct.pack(">H", threshold)
        resp = self._send(FunctionCode.SET_POSITION_THRESHOLD, data, 1)
        return resp[0] == 1

    def set_limit_port_remap(self, enable: bool) -> bool:
        """7.3.1 SetLimitPortRemap (9EH) -- remaps limit switches onto En/Dir pins, bus mode."""
        data = self._send(FunctionCode.SET_LIMIT_PORT_REMAP, bytes([1 if enable else 0]), 1)
        return data[0] == 1

    # ---- Closed-loop tuning ----
    #
    # 5.3: factory-tuned, manual warns "proceed with extreme caution to
    # avoid damaging the motor" if changed. Kp is what governs position
    # stiffness (restoring torque per unit of angle error) -- raise it to
    # make the "force needed to push it off target" curve steeper. Kd damps
    # the resulting overshoot/ringing; raising Kp alone without also raising
    # Kd tends to make the axis oscillate or growl at the target instead of
    # holding cleanly. Kv is undocumented beyond "range 0-1024" in the
    # manual. All four are uint16, range 0-1024 (values above that are
    # rejected/clamped by the driver, not validated here).

    def set_pid_vfoc(self, kp: int, ki: int, kd: int, kv: int) -> bool:
        """5.3.1 SetPidVFoc (96H) -- PID gains used in PulseVFoc/BusVFoc work modes.

        Factory defaults: Kp=0xDC Ki=0x64 Kd=0x10E Kv=0x140.
        """
        data = struct.pack(">HHHH", kp, ki, kd, kv)
        resp = self._send(FunctionCode.SET_PID_VFOC, data, 1)
        return resp[0] == 1

    def set_pid_close(self, kp: int, ki: int, kd: int, kv: int) -> bool:
        """5.3.2 SetPidClose (97H) -- PID gains used in PulseClose/BusClose work modes.

        Factory defaults: Kp=0xC8 Ki=0x50 Kd=0xFA Kv=0x12C.
        """
        data = struct.pack(">HHHH", kp, ki, kd, kv)
        resp = self._send(FunctionCode.SET_PID_CLOSE, data, 1)
        return resp[0] == 1

    # ---- Bus / multi-motor configuration ----

    def set_respond_and_active(self, respond: bool, active: bool) -> bool:
        """5.2.12 SetRespondAndActive (8CH)."""
        data = self._send(
            FunctionCode.SET_RESPOND_AND_ACTIVE,
            bytes([1 if respond else 0, 1 if active else 0]),
            1,
        )
        return data[0] == 1

    def set_group_address(self, group_address: int) -> bool:
        """5.2.15 SetGroupAddress (8DH)."""
        data = self._send(FunctionCode.SET_GROUP_ADDRESS, bytes([group_address]), 1)
        return data[0] == 1

    def set_modbus_rtu(self, enable: bool) -> bool:
        """5.2.13 SetModbusRtu (8EH). WARNING: enabling switches the driver to Modbus-RTU
        -- it stops responding to this protocol until Modbus-RTU is disabled again."""
        data = self._send(FunctionCode.SET_MODBUS_RTU, bytes([1 if enable else 0]), 1)
        return data[0] == 1

    def set_key_lock(self, locked: bool) -> bool:
        """5.2.14 SetKeyLock (8FH)."""
        data = self._send(FunctionCode.SET_KEY_LOCK, bytes([1 if locked else 0]), 1)
        return data[0] == 1

    def set_auto_run_on_power_on(self, action: int) -> int:
        """10.3 SetAutoRunOnPowerOn (FFH). action: 0xC8=Enable 0xCA=Disable. 0=fail 1=starting 2=complete."""
        data = self._send(FunctionCode.AUTO_RUN_ON_POWER_ON, bytes([action]), 1)
        return data[0]

    # ---- Bulk parameter dump (advanced) ----

    def read_all_config_params(self) -> bytes:
        """5.7.2 ReadAllConfigParams (47H) -- 34-byte blob, raw/unparsed."""
        return self._send(FunctionCode.READ_ALL_CONFIG_PARAMS, b"", 34)

    def write_all_config_params(self, params_34_bytes: bytes) -> bool:
        """5.7.1 WriteAllConfigParams (46H). No safety net -- malformed input is possible."""
        if len(params_34_bytes) != 34:
            raise ValueError("expected exactly 34 bytes (manual 5.7.1/5.7.2)")
        data = self._send(FunctionCode.WRITE_ALL_CONFIG_PARAMS, params_34_bytes, 1)
        return data[0] == 1

    def read_all_status_params(self) -> bytes:
        """5.7.3 ReadAllStatusParams (48H) -- 28-byte blob, raw/unparsed."""
        return self._send(FunctionCode.READ_ALL_STATUS_PARAMS, b"", 28)

    def read_config_params(self) -> dict:
        """Decoded form of read_all_config_params() -- offsets taken from the
        manual's page-54 "Return all motor parameters (47)" table (verified
        against the rendered table image, not just its text extraction,
        which merges columns unreliably).

        Fields not present anywhere in this 34-byte blob have no read-back
        at all in the RS485 protocol -- there's no other command that
        returns them either: PID gains (5.3), position threshold (5.2.17),
        position protect (5.2.14), heartbeat protect time (5.2.16), and
        auto-run-on-power-on (5.2.20). The driver only remembers what you
        last wrote for those; it can't be queried.
        """
        data = self.read_all_config_params()
        microstep = data[4] or 256  # wire byte 0 means 256, see set_microstep
        # Keep _rpm_to_wire_speed's calibration current -- see __init__.
        # This is the one path that can pick up a microstep set some other
        # way (onboard menu, another client) that this driver never saw.
        self._microstep = microstep
        return {
            "work_mode": data[0],
            "working_current_ma": struct.unpack(">H", data[1:3])[0],
            "holding_current_ratio": data[3],
            "microstep": microstep,
            "enable_pin_level": data[5],
            "direction": data[6],
            "auto_screen_off": data[7] != 0,
            "overcurrent_protect": data[8] != 0,
            "microstep_interpolation": data[9] != 0,
            "baud_code": data[10],
            "slave_address": data[11],
            "group_address": data[12],
            "respond": data[13] != 0,
            "active": data[14] != 0,
            "modbus_rtu": data[15] != 0,
            "key_lock": data[16] != 0,
            "home_trigger_high": data[17] != 0,
            "home_direction": data[18],
            "home_speed_rpm": self._wire_speed_to_rpm(struct.unpack(">H", data[19:21])[0]),
            "end_limit_enable": data[21] != 0,
            "home_origin_offset": struct.unpack(">i", data[22:26])[0],
            "home_mode": data[26],
            "home_current_ma": struct.unpack(">H", data[27:29])[0],
            "limit_port_remap": data[29] != 0,
            "zero_mode": data[30],
            # data[31] ("setting 0 point" in the write table) reads back as
            # a fixed 0xFF reserved placeholder per the manual, not a live
            # value -- omitted.
            "zero_speed_tier": data[32],
            "zero_direction": data[33],
        }
