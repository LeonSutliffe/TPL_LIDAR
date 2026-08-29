"""MKS SERVO42/57D RS485 proprietary-protocol framing.

Ported from the already-verified D:\\Downloads\\MKS_Servo_Tester\\app\\
MksServoTester.Protocol\\MksFrame.cs / MksFunctionCode.cs (manual: "MKS
SERVO42&57D_RS485 User Manual"). Frame layout:
  [header: FA downlink / FB uplink][address][function code][data...][checksum]
checksum = low byte of the sum of every preceding byte, header included.

This is the protocol used when the driver's MB_RTU setting is OFF (see
5.2.13 SetModbusRtu in the manual) -- confirmed working end-to-end this
session over the Pico USB-CDC<->RS485 bridge.
"""

from __future__ import annotations

from dataclasses import dataclass

DOWNLINK_HEADER = 0xFA
UPLINK_HEADER = 0xFB
BROADCAST_ADDRESS = 0x00


def checksum(frame_without_checksum: bytes) -> int:
    return sum(frame_without_checksum) & 0xFF


def build_downlink(address: int, function_code: int, data: bytes = b"") -> bytes:
    frame = bytes([DOWNLINK_HEADER, address, function_code]) + data
    return frame + bytes([checksum(frame)])


class MksProtocolError(Exception):
    pass


@dataclass
class UplinkFrame:
    address: int
    function_code: int
    data: bytes


def parse_uplink(buf: bytes, expected_data_length: int) -> UplinkFrame:
    """Parses exactly len(buf) == 3 + expected_data_length + 1 bytes.

    Raises MksProtocolError on a short buffer, bad header, or checksum
    mismatch -- callers should drop a byte and resync on error, same as the
    C# TryParseUplink caller contract.
    """
    total_length = 3 + expected_data_length + 1
    if len(buf) != total_length:
        raise MksProtocolError(
            f"expected {total_length} bytes, got {len(buf)}"
        )

    if buf[0] != UPLINK_HEADER:
        raise MksProtocolError(
            f"expected uplink header 0x{UPLINK_HEADER:02X}, got 0x{buf[0]:02X}"
        )

    expected_checksum = checksum(buf[: total_length - 1])
    actual_checksum = buf[total_length - 1]
    if expected_checksum != actual_checksum:
        raise MksProtocolError(
            f"checksum mismatch: expected 0x{expected_checksum:02X}, "
            f"got 0x{actual_checksum:02X}"
        )

    return UplinkFrame(
        address=buf[1],
        function_code=buf[2],
        data=buf[3 : 3 + expected_data_length],
    )


class FunctionCode:
    """Function codes from the MKS SERVO42/57D RS485 user manual V1.0.9."""

    # 5.1 Read-only parameter instructions
    READ_ENCODER = 0x30
    READ_CUMULATIVE_ENCODER = 0x31
    READ_SPEED = 0x32
    READ_PULSES = 0x33
    READ_IO_STATUS = 0x34
    READ_RAW_ENCODER = 0x35
    WRITE_IO_PORT = 0x36
    READ_ANGLE_ERROR = 0x39
    READ_ENABLE_STATUS = 0x3A
    RELEASE_STALL = 0x3D
    READ_STALL_STATUS = 0x3E
    RESTORE_FACTORY_DEFAULTS = 0x3F
    READ_VERSION = 0x40
    RESET_AND_RESTART = 0x41
    READ_WRITE_USER_ID = 0x42
    READ_ALL_CONFIG_PARAMS = 0x47
    READ_ALL_STATUS_PARAMS = 0x48

    # 5.2 Write-only parameter commands
    WRITE_ALL_CONFIG_PARAMS = 0x46
    CALIBRATE_ENCODER = 0x80
    SET_WORK_MODE = 0x82
    SET_WORKING_CURRENT = 0x83
    SET_MICROSTEP = 0x84
    SET_ENABLE_PIN_LEVEL = 0x85
    SET_DIRECTION = 0x86
    SET_AUTO_SCREEN_OFF = 0x87
    SET_OVERCURRENT_PROTECT = 0x88
    SET_MICROSTEP_INTERPOLATION = 0x89
    SET_BAUD_RATE = 0x8A
    SET_SLAVE_ADDRESS = 0x8B
    SET_RESPOND_AND_ACTIVE = 0x8C
    SET_GROUP_ADDRESS = 0x8D
    SET_MODBUS_RTU = 0x8E
    SET_KEY_LOCK = 0x8F
    SET_HOME_PARAMS = 0x90
    GO_HOME = 0x91
    SET_ZERO_POINT = 0x92
    SET_HOME_TORQUE_OFFSET = 0x94
    SET_POSITION_THRESHOLD = 0x95
    SET_PID_VFOC = 0x96
    SET_PID_CLOSE = 0x97
    SET_HEARTBEAT_PROTECT_TIME = 0x98
    SET_IO_PULSE_DIV_OUTPUT = 0x99
    SET_SINGLE_TURN_ZERO_RETURN = 0x9A
    SET_HOLDING_CURRENT_PERCENT = 0x9B
    SET_POSITION_PROTECT = 0x9D
    SET_LIMIT_PORT_REMAP = 0x9E
    SET_IN1_PORT_MODE = 0x9F

    # Bus control mode (Part 9-11)
    READ_MOTOR_STATUS = 0xF1
    RELATIVE_MOVE_BY_AXIS = 0xF4
    ABSOLUTE_MOVE_BY_AXIS = 0xF5
    RUN_SPEED_MODE = 0xF6
    EMERGENCY_STOP = 0xF7
    SET_ENABLE = 0xF3
    RELATIVE_MOVE_BY_PULSES = 0xFD
    ABSOLUTE_MOVE_BY_PULSES = 0xFE
    AUTO_RUN_ON_POWER_ON = 0xFF
