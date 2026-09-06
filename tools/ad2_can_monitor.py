#!/usr/bin/env python3
"""Passively monitor a CAN bus with a Digilent Analog Discovery 2.

The default raw backend samples the selected digital input, transfers DWF
value/span compressed records, and decodes standard CAN data frames in Python.
It does not configure any CAN transmit pin and does not ACK frames. The older
WaveForms CAN decoder is retained as an optional fallback.

By default it listens for a 1 Mbit/s logic-level CAN RX signal on DIO 7.
The live display has one row per CAN address.  A row is added only when an
address is first seen; later frames from that address refresh its data,
timestamp, DLC, and frame count in place.

Frame collection is intentionally independent of screen refreshes.  This
keeps terminal I/O out of the receive path on a busy bus.

Use --filter one or more times to show/count only selected CAN IDs.  Filtering
is after capture, so it does not reduce the raw-sample bandwidth requirement.

Known standard data frames from the local robot firmware, SCV2, Faster_Supercap,
and RoboMaster motor contracts are decoded in the `Decoded` display column.
Extended, remote, wrong-DLC, and unknown frames remain visible as raw traffic.

Run the packaged application from PowerShell:
  AD2-CAN-Monitor.exe

Run from source:
  & "$env:LOCALAPPDATA/Programs/Python/Python312/python.exe" tools/ad2_can_monitor.py
"""

from __future__ import annotations

import argparse
import ctypes as ct
import math
import os
import shutil
import struct
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import TextIO

try:  # Support both `python tools/ad2_can_monitor.py` and module execution.
    from .ad2_can_raw_monitor import RawCanDecoder
except ImportError:
    from ad2_can_raw_monitor import RawCanDecoder


DWF_DLL_PATHS = (
    Path(r"C:\Program Files\Digilent\WaveForms3\dwf.dll"),
    Path(r"C:\Program Files\Digilent\WaveFormsSDK\lib\dwf.dll"),
)
APP_NAME = "AD2 CAN Monitor"
APP_VERSION = "1.0.0"

# DWF CAN receiver status values.  Unknown non-zero statuses are still shown.
CAN_STATUS = {
    2: "bit-stuffing error",
    3: "CRC error",
}
RECORD_MODE = 3
SAMPLE_FORMAT_BITS = 8
MAX_RAW_READ_SAMPLES = 16_384
MAX_COMPRESSED_VALUES = 16_384

# SCV2 Classic-CAN wire identifiers and payload sizes.  The packet definitions
# live in scv2/Core/Inc/can_protocol.h and scv2/CAN_TELEMETRY.md.
SCV2_COMMAND_CAN_ID = 0x067
SCV2_COMMAND_DLC = 5
SCV2_TELEMETRY_CAN_ID = 0x077
SCV2_TELEMETRY_DLC = 8

# Contracts recovered from robot_firmware's CAN callbacks/tasks across all local
# and origin branches.  IDs shared by several motor models intentionally retain
# generic family names: a passive single-bus capture cannot determine the model.
DEVC_CHASSIS_COMMAND_CAN_ID = 0x100
DEVC_STATUS_CAN_ID = 0x101
DEVC_ODOMETRY_CAN_ID = 0x102
DEVC_IMU_CAN_IDS = {
    0x103: ("attitude", 0.0001, "rad"),
    0x104: ("gyro", 0.001, "rad/s"),
    0x105: ("accel", 0.002, "m/s^2"),
}
DJI_GROUP_COMMANDS = {
    0x200: "DJI cmd IDs1-4",
    0x1FF: "DJI cmd IDs5-8 / GM6020 IDs1-4",
    0x2FF: "GM6020 cmd IDs5-7",
    0x3FE: "DM DJI-mode cmd IDs1-4",
    0x4FE: "DM DJI-mode cmd IDs5-8",
}
# 60 V / 15 A unidirectional wattmeter revisions. The supplied documentation
# names 0x212 (formerly 0x211); the connected unit is observed on 0x213 with
# the same payload layout.
WATTMETER_CAN_IDS = {0x211, 0x212, 0x213}
CM01_PMM_CAN_ID = 0x270
LK_RMD_OPCODES = {
    0x19: "zero motor",
    0x30: "read PID",
    0x31: "write PID RAM",
    0x32: "write PID ROM",
    0x33: "read acceleration",
    0x80: "close motor",
    0x81: "stop motor",
    0x88: "resume motor",
    0x90: "read encoder",
    0x91: "write encoder zero",
    0x92: "read multi-turn angle",
    0x94: "read single-turn angle",
    0x95: "clear motor angle",
    0x9A: "read status 1",
    0x9B: "clear errors",
    0x9C: "status 2",
    0x9D: "status 3",
    0xA1: "torque control",
    0xA2: "speed control",
    0xA3: "multi-turn position",
    0xA4: "multi-turn position",
    0xA5: "single-turn position",
    0xA6: "single-turn position",
}


class DwfError(RuntimeError):
    """An error returned by the WaveForms DWF library."""


def prepare_windows_console() -> None:
    """Enable ANSI dashboard rendering when launched directly on Windows."""
    if os.name != "nt" or not sys.stdout.isatty():
        return
    kernel32 = ct.windll.kernel32
    output_handle = kernel32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
    mode = ct.c_uint()
    if output_handle and kernel32.GetConsoleMode(output_handle, ct.byref(mode)):
        kernel32.SetConsoleMode(output_handle, mode.value | 0x0004)  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
    kernel32.SetConsoleTitleW(APP_NAME)


def load_dwf() -> ct.CDLL:
    """Load DWF from the standard Windows WaveForms install locations."""
    for path in DWF_DLL_PATHS:
        if path.is_file():
            return ct.CDLL(os.fspath(path))
    raise DwfError("Could not find dwf.dll. Install Digilent WaveForms, then try again.")


def configure_signatures(dwf: ct.CDLL) -> None:
    handle = ct.c_int
    int_ptr = ct.POINTER(ct.c_int)
    uint_ptr = ct.POINTER(ct.c_uint)
    double_ptr = ct.POINTER(ct.c_double)
    byte_ptr = ct.POINTER(ct.c_ubyte)

    dwf.FDwfDeviceConfigOpen.argtypes = [ct.c_int, ct.c_int, int_ptr]
    dwf.FDwfDeviceConfigOpen.restype = ct.c_int
    dwf.FDwfDeviceClose.argtypes = [handle]
    dwf.FDwfDeviceClose.restype = ct.c_int
    dwf.FDwfGetLastErrorMsg.argtypes = [ct.c_char_p]
    dwf.FDwfGetLastErrorMsg.restype = ct.c_int

    dwf.FDwfDigitalCanReset.argtypes = [handle]
    dwf.FDwfDigitalCanReset.restype = ct.c_int
    dwf.FDwfDigitalCanRateSet.argtypes = [handle, ct.c_double]
    dwf.FDwfDigitalCanRateSet.restype = ct.c_int
    dwf.FDwfDigitalCanPolaritySet.argtypes = [handle, ct.c_int]
    dwf.FDwfDigitalCanPolaritySet.restype = ct.c_int
    dwf.FDwfDigitalCanRxSet.argtypes = [handle, ct.c_int]
    dwf.FDwfDigitalCanRxSet.restype = ct.c_int
    dwf.FDwfDigitalCanRx.argtypes = [
        handle, int_ptr, int_ptr, int_ptr, int_ptr, ct.POINTER(ct.c_ubyte), ct.c_int, int_ptr
    ]
    dwf.FDwfDigitalCanRx.restype = ct.c_int

    dwf.FDwfDigitalInReset.argtypes = [handle]
    dwf.FDwfDigitalInReset.restype = ct.c_int
    dwf.FDwfDigitalInInternalClockInfo.argtypes = [handle, double_ptr]
    dwf.FDwfDigitalInInternalClockInfo.restype = ct.c_int
    dwf.FDwfDigitalInBufferSizeInfo.argtypes = [handle, int_ptr]
    dwf.FDwfDigitalInBufferSizeInfo.restype = ct.c_int
    dwf.FDwfDigitalInAcquisitionModeSet.argtypes = [handle, ct.c_int]
    dwf.FDwfDigitalInAcquisitionModeSet.restype = ct.c_int
    dwf.FDwfDigitalInDividerSet.argtypes = [handle, ct.c_uint]
    dwf.FDwfDigitalInDividerSet.restype = ct.c_int
    dwf.FDwfDigitalInSampleFormatSet.argtypes = [handle, ct.c_int]
    dwf.FDwfDigitalInSampleFormatSet.restype = ct.c_int
    dwf.FDwfDigitalInTriggerPositionSet.argtypes = [handle, ct.c_int]
    dwf.FDwfDigitalInTriggerPositionSet.restype = ct.c_int
    dwf.FDwfDigitalInConfigure.argtypes = [handle, ct.c_int, ct.c_int]
    dwf.FDwfDigitalInConfigure.restype = ct.c_int
    dwf.FDwfDigitalInStatus.argtypes = [handle, ct.c_int, byte_ptr]
    dwf.FDwfDigitalInStatus.restype = ct.c_int
    dwf.FDwfDigitalInStatusRecord.argtypes = [handle, int_ptr, int_ptr, int_ptr]
    dwf.FDwfDigitalInStatusRecord.restype = ct.c_int
    dwf.FDwfDigitalInStatusData.argtypes = [handle, ct.c_void_p, ct.c_int]
    dwf.FDwfDigitalInStatusData.restype = ct.c_int
    dwf.FDwfDigitalInSampleSensibleSet.argtypes = [handle, ct.c_uint]
    dwf.FDwfDigitalInSampleSensibleSet.restype = ct.c_int
    dwf.FDwfDigitalInStatusCompress.argtypes = [handle, int_ptr, int_ptr, int_ptr]
    dwf.FDwfDigitalInStatusCompress.restype = ct.c_int
    dwf.FDwfDigitalInStatusCompressed.argtypes = [handle, ct.c_void_p, ct.c_int]
    dwf.FDwfDigitalInStatusCompressed.restype = ct.c_int


def error_message(dwf: ct.CDLL) -> str:
    message = ct.create_string_buffer(512)
    dwf.FDwfGetLastErrorMsg(message)
    return message.value.decode(errors="replace") or "unknown DWF error"


def require(ok: int, dwf: ct.CDLL, operation: str) -> None:
    if not ok:
        raise DwfError(f"{operation} failed: {error_message(dwf)}")


@lru_cache(maxsize=8)
def compressed_run_tables(dio: int) -> tuple[tuple[bytes, ...], tuple[bytes, ...]]:
    """Return reusable byte strings for every eight-bit compressed span."""
    mask = 1 << dio
    return (
        tuple(bytes(length) for length in range(1, 257)),
        tuple(bytes((mask,)) * length for length in range(1, 257)),
    )


def expand_compressed_samples(encoded: bytes, dio: int) -> bytes:
    """Expand DWF eight-bit (value, stable-count-minus-one) pairs."""
    if len(encoded) % 2:
        raise DwfError(f"DWF returned an odd compressed-value count ({len(encoded)})")
    low_runs, high_runs = compressed_run_tables(dio)
    mask = 1 << dio
    return b"".join(
        (high_runs if encoded[index] & mask else low_runs)[encoded[index + 1]]
        for index in range(0, len(encoded), 2)
    )


def timestamp() -> str:
    now = time.time_ns()
    return time.strftime("%H:%M:%S", time.localtime(now // 1_000_000_000)) + f".{now // 1_000_000 % 1_000:03d}"


def _decode_dji_feedback(identifier: int, payload: bytes) -> str:
    angle, rpm, torque = struct.unpack(">Hhh", payload[:6])
    if 0x209 <= identifier <= 0x20B:
        family = "GM6020 fb"
        auxiliary = f"temp={payload[6]} C"
    else:
        family = "DJI motor fb"
        # C620 and GM6020 define byte 6 as temperature; the older C610
        # documentation leaves it unused, and their feedback IDs overlap.
        auxiliary = f"temp/aux={payload[6]}"
    reserved = f" reserved=0x{payload[7]:02X}" if payload[7] else ""
    return f"{family}: angle={angle}/8192 rpm={rpm} torque_raw={torque} {auxiliary}{reserved}"


def _decode_dm_mit_feedback(payload: bytes) -> str:
    motor_id = payload[0] & 0x0F
    state = payload[0] >> 4
    position_raw = (payload[1] << 8) | payload[2]
    velocity_raw = (payload[3] << 4) | (payload[4] >> 4)
    torque_raw = ((payload[4] & 0x0F) << 8) | payload[5]
    position = position_raw * (8.0 * math.pi) / 65535.0 - 4.0 * math.pi
    velocity = velocity_raw * 90.0 / 4095.0 - 45.0
    torque = torque_raw * 36.0 / 4095.0 - 18.0
    return (
        f"DM MIT fb: id={motor_id} state={state} pos={position:.3f} rad "
        f"vel={velocity:.2f} rad/s torque={torque:.2f} Nm Tmos={payload[6]} C Tcoil={payload[7]} C"
    )


def _decode_lk_rmd(payload: bytes) -> str:
    opcode = payload[0]
    operation = LK_RMD_OPCODES.get(opcode)
    if operation is None:
        return f"LK/RMD motor: opcode=0x{opcode:02X}"

    if opcode == 0x9C and any(payload[1:]):
        torque, speed_dps, angle = struct.unpack_from("<hhH", payload, 2)
        return (
            f"LK/RMD status2: temp={payload[1]} C torque_raw={torque} "
            f"speed={speed_dps} dps angle={angle}/65536"
        )
    if opcode == 0xA2 and payload[1:4] == bytes(3):
        speed_dps = struct.unpack_from("<i", payload, 4)[0] / 100.0
        return f"LK/RMD speed cmd: {speed_dps:.2f} dps"
    if opcode == 0xA1 and payload[1:4] == bytes(3) and payload[6:] == bytes(2):
        torque_raw = struct.unpack_from("<h", payload, 4)[0]
        return f"LK/RMD torque cmd: raw={torque_raw}"
    return f"LK/RMD motor: {operation} (0x{opcode:02X})"


def _dji_crc8(data: bytes, seed: int = 119) -> int:
    """Return the reflected DJI packet-header CRC-8 (poly 0x31)."""
    crc = seed
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def _cm01_crc16(data: bytes, seed: int = 0x1862) -> int:
    """Return the observed CM01 message CRC-16 (reflected poly 0x1021)."""
    crc = seed
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


class Cm01PmmReassembler:
    """Reassemble observed DJI CM01/PMM messages fragmented over CAN ID 0x270.

    The measurement message is 30 bytes split 8/8/8/6. Its first four bytes
    are a DJI-style header: 0x5A, a 13-byte payload length, protocol marker
    0x10, and the standard DJI header CRC-8. Its final two bytes are a
    message-level CRC-16 using observed seed 0x1862. Losing one CAN fragment
    abandons only that message; the next CRC-valid header starts cleanly.

    The three little-endian floats were recovered from live traffic. Voltage
    is confirmed independently against the capacitor-bank voltage. A live
    drain/charge test showed that the second and third floats are separate
    positive discharge- and charge-current magnitudes: only the second rises
    while capacitor voltage falls, and only the third rises while it rises.
    Signed net capacitor current is therefore charge minus discharge.
    """

    START = bytes((0x5A, 0x0D, 0x10))
    MESSAGE_LENGTH = 30

    def __init__(self) -> None:
        self._buffer = bytearray()

    @classmethod
    def _is_start(cls, payload: bytes) -> bool:
        return (
            len(payload) == 8
            and payload.startswith(cls.START)
            and payload[3] == _dji_crc8(payload[:3])
        )

    def feed(self, payload: bytes) -> str | None:
        if self._is_start(payload):
            self._buffer = bytearray(payload)
            return None
        if not self._buffer:
            return None

        self._buffer.extend(payload)
        if len(self._buffer) < self.MESSAGE_LENGTH:
            return None
        if len(self._buffer) != self.MESSAGE_LENGTH:
            self._buffer.clear()
            return None

        packet = bytes(self._buffer)
        self._buffer.clear()
        if (
            packet[13:15] != bytes((0x09, 0x1A))
            or _cm01_crc16(packet[:-2]) != int.from_bytes(packet[-2:], "little")
        ):
            return None

        voltage_v, discharge_current_a, charge_current_a = struct.unpack_from("<fff", packet, 15)
        if not all(math.isfinite(value) for value in (voltage_v, discharge_current_a, charge_current_a)):
            return None
        route = struct.unpack_from("<H", packet, 6)[0]
        sequence = struct.unpack_from("<H", packet, 10)[0]
        net_current_a = charge_current_a - discharge_current_a
        return (
            f"CM01/PMM type=09/1A: voltage={voltage_v:.3f} V "
            f"net_current={net_current_a:+.4f} A (+charge) "
            f"charge={charge_current_a:.4f} A discharge={discharge_current_a:.4f} A "
            f"status=0x{packet[27]:02X} route=0x{route:04X} seq={sequence}"
        )


def decode_can_frame(identifier: int, extended: bool, remote: bool, dlc: int, payload: bytes) -> str:
    """Return an evidence-backed summary for a known valid Classic-CAN frame."""
    if extended or remote or len(payload) != dlc:
        return ""

    if identifier == SCV2_COMMAND_CAN_ID and dlc == SCV2_COMMAND_DLC and len(payload) == SCV2_COMMAND_DLC:
        enable_module, reset, power_limit_w, energy_j = struct.unpack("<BBBH", payload)
        energy = "disabled (777 J)" if energy_j == 777 else f"{energy_j} J"
        return f"SCV2 cmd: enable={enable_module} reset=0x{reset:02X} power={power_limit_w} W energy={energy}"

    if identifier == SCV2_TELEMETRY_CAN_ID and dlc == SCV2_TELEMETRY_DLC and len(payload) == SCV2_TELEMETRY_DLC:
        load_power_dw, vcap_dv, converter_current_da = struct.unpack_from("<HHh", payload)
        status = payload[7]
        faults = []
        if status & 0x01:
            faults.append("Vbus OVP")
        if status & 0x02:
            faults.append("Vcap OVP")
        fault_text = ", ".join(faults) if faults else "none"
        reserved_status = status & ~0x03
        if reserved_status:
            fault_text += f"; reserved=0x{reserved_status:02X}"
        reserved_byte = f" reserved=0x{payload[6]:02X}" if payload[6] else ""
        return (
            f"SCV2 telemetry: load={load_power_dw / 10:.1f} W vcap={vcap_dv / 10:.1f} V "
            f"iconv={converter_current_da / 10:.1f} A faults={fault_text}{reserved_byte}"
        )

    # Faster_Supercap's legacy 0x077 contract is unambiguous by its DLC 6.
    if identifier == SCV2_TELEMETRY_CAN_ID and dlc == 6:
        chassis_power_w, error, energy_raw = struct.unpack("<fBB", payload)
        error_names = {
            0: "none",
            1: "cap peak below required",
            2: "SWEN low",
            3: "RVSOFF low",
        }
        error_text = error_names.get(error, f"unknown({error})")
        return (
            f"legacy supercap: power={chassis_power_w:.1f} W error={error_text} "
            f"energy={energy_raw}/255 ({energy_raw * 100.0 / 255.0:.1f}%)"
        )

    if identifier == DEVC_CHASSIS_COMMAND_CAN_ID and dlc == 8:
        forward, horizontal, yaw = struct.unpack_from("<hhh", payload)
        return (
            f"DevC chassis cmd: fwd={forward / 1000:.3f} strafe={horizontal / 1000:.3f} "
            f"yaw={yaw / 1000:.3f} enable={payload[6]} power_limit={payload[7]} W"
        )

    if identifier == DEVC_STATUS_CAN_ID and dlc == 8:
        suffix = "" if payload[1:] == bytes(7) else " bytes1-7=uncontracted"
        return f"DevC status: supercap_charge={payload[0]}{suffix}"

    if identifier == DEVC_ODOMETRY_CAN_ID and dlc == 8:
        front_right, front_left, back_left, back_right = struct.unpack(">hhhh", payload)
        return (
            f"DevC odom: FR={front_right} FL={front_left} BL={back_left} BR={back_right} rpm"
        )

    imu_contract = DEVC_IMU_CAN_IDS.get(identifier)
    if imu_contract is not None and dlc == 8:
        name, scale, unit = imu_contract
        x_raw, y_raw, z_raw = struct.unpack_from("<hhh", payload, 2)
        return (
            f"DevC IMU {name}: seq={payload[0]} valid=0x{payload[1]:02X} "
            f"xyz=({x_raw * scale:.4g},{y_raw * scale:.4g},{z_raw * scale:.4g}) {unit}"
        )

    group_name = DJI_GROUP_COMMANDS.get(identifier)
    if group_name is not None and dlc == 8:
        outputs = struct.unpack(">hhhh", payload)
        return f"{group_name}: raw={outputs}"

    if 0x201 <= identifier <= 0x20B and dlc == 8:
        return _decode_dji_feedback(identifier, payload)

    if 0x301 <= identifier <= 0x308 and dlc == 8:
        angle, rpm, torque = struct.unpack(">Hhh", payload[:6])
        reserved = f" reserved=0x{payload[7]:02X}" if payload[7] else ""
        return (
            f"DM DJI-mode fb: angle={angle}/8192 rpm={rpm} torque_raw={torque} "
            f"temp={payload[6]} C{reserved}"
        )

    # 0x091 is the configured MIT-mode feedback address in the searched robot
    # configurations. Commands use a separately configured motor address.
    if identifier == 0x091 and dlc == 8:
        return _decode_dm_mit_feedback(payload)

    if 0x141 <= identifier <= 0x160 and dlc == 8:
        return _decode_lk_rmd(payload)

    if identifier in WATTMETER_CAN_IDS and dlc == 8:
        voltage_cv, current_ca = struct.unpack_from("<HH", payload)
        voltage_v = voltage_cv / 100.0
        current_a = current_ca / 100.0
        reserved = f" reserved={payload[4:].hex(' ').upper()}" if any(payload[4:]) else ""
        return (
            f"60V15A wattmeter: voltage={voltage_v:.2f} V current={current_a:.2f} A "
            f"power={voltage_v * current_a:.2f} W{reserved}"
        )

    if identifier == CM01_PMM_CAN_ID:
        return "CM01/PMM segmented stream (waiting for a complete 8/8/8/6 message)"

    return ""


# Preserve the old public helper name for callers that imported it directly.
decode_scv2_frame = decode_can_frame


@dataclass
class CanFrame:
    identifier: int
    extended: bool
    remote: bool
    dlc: int
    payload: bytes
    last_seen: str
    count: int = 1
    rate_times: deque[float] = field(default_factory=deque, repr=False)
    rate_hz: float = 0.0
    stream_decoded: str = ""

    @property
    def address(self) -> str:
        return f"0x{self.identifier:0{8 if self.extended else 3}X}"

    @property
    def format(self) -> str:
        return "EXT" if self.extended else "STD"

    @property
    def frame_type(self) -> str:
        return "RTR" if self.remote else "DATA"

    @property
    def data(self) -> str:
        return "--" if self.remote or not self.payload else " ".join(f"{byte:02X}" for byte in self.payload)

    @property
    def decoded(self) -> str:
        return self.stream_decoded or decode_can_frame(
            self.identifier, self.extended, self.remote, self.dlc, self.payload
        )


class LiveCanDisplay:
    """Render a stable address table using ANSI cursor controls."""

    def __init__(self, stream: TextIO, bitrate: float, dio: int, filters: set[int]) -> None:
        self.stream = stream
        self.bitrate = bitrate
        self.dio = dio
        self.filters = filters
        self.frames: dict[tuple[bool, int], CanFrame] = {}
        self.candidate_rate_times: dict[int, deque[float]] = {}
        self.candidate_rates: dict[int, float] = {}
        self.errors: Counter[str] = Counter()
        self.last_error = "--"
        self.total_frames = 0
        self.capture_health = "--"
        self.cm01_pmm = Cm01PmmReassembler()
        self._interactive = stream.isatty()
        self.dirty = True

    def add_candidate_headers(self, counts: Counter[int]) -> None:
        """Update approximate on-wire rates from CRC-independent CAN headers."""
        now = time.monotonic()
        cutoff = now - 1.0
        for identifier, count in counts.items():
            if self.filters and identifier not in self.filters:
                continue
            rate_times = self.candidate_rate_times.setdefault(identifier, deque())
            rate_times.extend([now] * count)

        for identifier, rate_times in self.candidate_rate_times.items():
            while len(rate_times) > 1 and rate_times[0] < cutoff:
                rate_times.popleft()
            if len(rate_times) > 1:
                span = rate_times[-1] - rate_times[0]
                self.candidate_rates[identifier] = (len(rate_times) - 1) / span if span else 0.0
            else:
                self.candidate_rates[identifier] = 0.0
        if counts:
            self.dirty = True

    def start(self) -> None:
        if self._interactive:
            # Clear once, then rewrite the complete dashboard from its origin.
            self.stream.write("\x1b[2J\x1b[H\x1b[?25l")
        self.render()

    def close(self) -> None:
        if self._interactive:
            self.stream.write("\x1b[?25h\n")
            self.stream.flush()

    def add_frame(self, identifier: int, extended: int, remote: int, dlc: int, data: bytes) -> None:
        if self.filters and identifier not in self.filters:
            return
        key = (bool(extended), identifier)
        seen_monotonic = time.monotonic()
        self.total_frames += 1
        current = self.frames.get(key)
        if current is None:
            current = CanFrame(
                identifier, bool(extended), bool(remote), dlc, data, timestamp(),
                rate_times=deque([seen_monotonic]),
            )
            self.frames[key] = current
        else:
            current.remote = bool(remote)
            current.dlc = dlc
            current.payload = data
            current.last_seen = timestamp()
            current.count += 1
            current.rate_times.append(seen_monotonic)

        if identifier == CM01_PMM_CAN_ID and not extended and not remote:
            stream_decoded = self.cm01_pmm.feed(data)
            if stream_decoded is not None:
                current.stream_decoded = stream_decoded

        cutoff = seen_monotonic - 1.0
        while len(current.rate_times) > 1 and current.rate_times[0] < cutoff:
            current.rate_times.popleft()
        if len(current.rate_times) > 1:
            span = current.rate_times[-1] - current.rate_times[0]
            current.rate_hz = (len(current.rate_times) - 1) / span if span else 0.0
        else:
            current.rate_hz = 0.0
        self.dirty = True

    def set_capture_health(self, description: str) -> None:
        if description != self.capture_health:
            self.capture_health = description
            self.dirty = True

    def add_error(self, status: int) -> None:
        description = CAN_STATUS.get(status, f"decoder error (status={status})")
        self.errors[description] += 1
        self.last_error = f"{timestamp()}  {description}"
        self.dirty = True

    def render(self) -> None:
        health_lines = self.capture_health.splitlines() or ["--"]
        lines = [
            f"Listening: CAN {self.bitrate / 1_000_000:g} Mbit/s, RX=DIO {self.dio}. Press Ctrl+C to stop.",
            f"Filter: {', '.join(f'0x{identifier:03X}' for identifier in sorted(self.filters)) if self.filters else 'all CAN IDs'}",
            f"Capture: {health_lines[0]}",
            *(f"         {line}" for line in health_lines[1:]),
            "",
            f"CAN packets by address ({len(self.frames)} unique, {self.total_frames} total)",
            "Address      Format  Type  DLC  Data                     Last received   CRC Hz  Header Hz  Frames  Decoded",
            "-----------  ------  ----  ---  -----------------------  --------------  ------  ---------  ------  -------",
        ]
        for frame in self.frames.values():
            lines.append(
                f"{frame.address:<11}  {frame.format:<6}  {frame.frame_type:<4}  {frame.dlc:>3}  "
                f"{frame.data:<23.23}  {frame.last_seen:<14}  {frame.rate_hz:>6.1f}  "
                f"{self.candidate_rates.get(frame.identifier, 0.0):>9.1f}  {frame.count:>6}  {frame.decoded}"
            )
        if not self.frames:
            lines.append("(waiting for CAN traffic)")

        lines.extend(["", "Decoder errors"])
        if self.errors:
            lines.extend(f"{name}: {count}" for name, count in self.errors.items())
            lines.append(f"Last error: {self.last_error}")
        else:
            lines.append("None")

        output = "\n".join(lines)
        if self._interactive:
            # A wrapped line changes the physical cursor row and corrupts the
            # next cursor-home redraw. Clear and clip every dashboard line.
            width = max(20, shutil.get_terminal_size((160, 24)).columns - 1)
            output = "\n".join(f"\x1b[2K{line[:width]}" for line in lines)
            self.stream.write("\x1b[H" + output + "\x1b[J")
        else:
            # Preserve a useful log when output is redirected instead of a terminal.
            self.stream.write(output + "\n\n")
        self.stream.flush()
        self.dirty = False


def monitor_decoder(
    device: int,
    dio: int,
    bitrate: float,
    refresh_hz: float,
    filters: set[int],
    duration: float | None,
) -> tuple[int, int]:
    dwf = load_dwf()
    configure_signatures(dwf)

    handle = ct.c_int()
    # AD2 configuration 3 supplies the digital I/O resources used by DWF CAN.
    require(dwf.FDwfDeviceConfigOpen(ct.c_int(device), ct.c_int(3), ct.byref(handle)), dwf, "open Analog Discovery 2")
    if not handle.value:
        raise DwfError(f"open Analog Discovery 2 failed: {error_message(dwf)}")

    display = LiveCanDisplay(sys.stdout, bitrate, dio, filters)
    display.set_capture_health("WaveForms CAN decoder (single-frame API; may drop traffic on a busy bus)")
    try:
        require(dwf.FDwfDigitalCanReset(handle), dwf, "reset CAN decoder")
        require(dwf.FDwfDigitalCanRateSet(handle, ct.c_double(bitrate)), dwf, "set CAN bitrate")
        # Normal CAN logic polarity: recessive high, dominant low.
        require(dwf.FDwfDigitalCanPolaritySet(handle, ct.c_int(0)), dwf, "set CAN polarity")
        require(dwf.FDwfDigitalCanRxSet(handle, ct.c_int(dio)), dwf, "set CAN RX pin")

        # Initializing the receiver is read-only. Never call FDwfDigitalCanTx*.
        require(dwf.FDwfDigitalCanRx(handle, None, None, None, None, None, ct.c_int(0), None), dwf, "initialize CAN receiver")

        identifier = ct.c_int()
        extended = ct.c_int()
        remote = ct.c_int()
        dlc = ct.c_int()
        status = ct.c_int()
        payload = (ct.c_ubyte * 8)()
        display.start()
        refresh_period = 1.0 / refresh_hz
        next_refresh = time.monotonic() + refresh_period
        stop_at = time.monotonic() + duration if duration is not None else None

        while True:
            require(
                dwf.FDwfDigitalCanRx(
                    handle, ct.byref(identifier), ct.byref(extended), ct.byref(remote), ct.byref(dlc),
                    payload, ct.c_int(len(payload)), ct.byref(status),
                ),
                dwf,
                "read CAN receiver",
            )
            if status.value == 1:
                # Copy before the next API call overwrites the ctypes buffer.
                display.add_candidate_headers(Counter({identifier.value: 1}))
                display.add_frame(identifier.value, extended.value, remote.value, dlc.value, bytes(payload[:dlc.value]))
            elif status.value:
                display.add_error(status.value)

            now = time.monotonic()
            if stop_at is not None and now >= stop_at:
                return display.total_frames, sum(display.errors.values())
            if now >= next_refresh:
                if display.dirty:
                    display.render()
                # Do not try to catch up on skipped refreshes: receive calls take
                # priority, and one display update is enough to show current state.
                next_refresh = now + refresh_period

    except KeyboardInterrupt:
        return display.total_frames, sum(display.errors.values())
    finally:
        # Include any frames received since the last rate-limited refresh.
        if display.dirty:
            display.render()
        display.close()
        dwf.FDwfDeviceClose(handle)


def monitor_raw(
    device: int,
    dio: int,
    bitrate: float,
    sample_rate: int,
    refresh_hz: float,
    filters: set[int],
    compressed: bool,
    duration: float | None,
) -> tuple[int, int]:
    """Capture batched DigitalIn samples and feed the Python CAN decoder."""
    if dio > 7:
        raise DwfError("the raw backend supports DIO 0 through 7 only (eight-bit sample format)")
    if bitrate != int(bitrate):
        raise DwfError("the raw backend requires an integer CAN bitrate")
    bitrate_int = int(bitrate)
    dwf = load_dwf()
    configure_signatures(dwf)
    handle = ct.c_int()
    require(dwf.FDwfDeviceConfigOpen(ct.c_int(device), ct.c_int(3), ct.byref(handle)), dwf, "open Analog Discovery 2")
    if not handle.value:
        raise DwfError(f"open Analog Discovery 2 failed: {error_message(dwf)}")

    display = LiveCanDisplay(sys.stdout, bitrate, dio, filters)
    total_samples = total_transfer_values = total_lost = total_corrupt = 0
    decoder: RawCanDecoder | None = None
    try:
        base_rate = ct.c_double()
        max_buffer = ct.c_int()
        require(dwf.FDwfDigitalInReset(handle), dwf, "reset DigitalIn")
        require(dwf.FDwfDigitalInInternalClockInfo(handle, ct.byref(base_rate)), dwf, "read DigitalIn clock")
        require(dwf.FDwfDigitalInBufferSizeInfo(handle, ct.byref(max_buffer)), dwf, "read DigitalIn buffer size")
        divider = max(1, round(base_rate.value / sample_rate))
        actual_rate = int(base_rate.value / divider)
        if actual_rate < bitrate_int * 4:
            raise DwfError("raw sample rate must provide at least four samples per CAN bit")
        if actual_rate % bitrate_int:
            raise DwfError(
                f"requested sample rate resolves to {actual_rate} Hz, which is not a whole-number multiple of the bitrate"
            )
        samples_per_bit = actual_rate // bitrate_int

        require(dwf.FDwfDigitalInAcquisitionModeSet(handle, ct.c_int(RECORD_MODE)), dwf, "set DigitalIn record mode")
        require(dwf.FDwfDigitalInDividerSet(handle, ct.c_uint(divider)), dwf, "set DigitalIn sample rate")
        require(dwf.FDwfDigitalInSampleFormatSet(handle, ct.c_int(SAMPLE_FORMAT_BITS)), dwf, "set eight-bit sample format")
        if compressed:
            require(
                dwf.FDwfDigitalInSampleSensibleSet(handle, ct.c_uint(1 << dio)),
                dwf,
                f"enable DIO {dio} record compression",
            )
        require(dwf.FDwfDigitalInTriggerPositionSet(handle, ct.c_int(0)), dwf, "set continuous DigitalIn acquisition")
        require(dwf.FDwfDigitalInConfigure(handle, ct.c_int(1), ct.c_int(1)), dwf, "start DigitalIn")

        decoder = RawCanDecoder(dio, samples_per_bit)
        data = (ct.c_ubyte * MAX_RAW_READ_SAMPLES)()
        compressed_data = (ct.c_ubyte * MAX_COMPRESSED_VALUES)()
        state = ct.c_ubyte()
        available = ct.c_int()
        lost = ct.c_int()
        corrupt = ct.c_int()
        monitor_started = time.perf_counter()
        decode_seconds = 0.0
        refresh_period = 1.0 / refresh_hz
        next_refresh = time.monotonic() + refresh_period
        stop_at = time.monotonic() + duration if duration is not None else None
        display.set_capture_health(
            f"raw DigitalIn: {actual_rate / 1_000_000:g} MHz, {samples_per_bit} samples/bit, "
            f"{'compressed' if compressed else 'uncompressed'}, buffer {max_buffer.value} samples"
        )
        display.start()

        while True:
            require(dwf.FDwfDigitalInStatus(handle, ct.c_int(1), ct.byref(state)), dwf, "poll DigitalIn")
            if compressed:
                require(
                    dwf.FDwfDigitalInStatusCompress(
                        handle, ct.byref(available), ct.byref(lost), ct.byref(corrupt)
                    ),
                    dwf,
                    "read compressed DigitalIn status",
                )
            else:
                require(
                    dwf.FDwfDigitalInStatusRecord(
                        handle, ct.byref(available), ct.byref(lost), ct.byref(corrupt)
                    ),
                    dwf,
                    "read DigitalIn record status",
                )
            total_lost += lost.value
            total_corrupt += corrupt.value
            if lost.value or corrupt.value:
                # Never join the tail of one frame to samples from after a FIFO
                # discontinuity. That creates false candidates and expensive
                # four-phase searches.
                decoder.reset_stream()

            count = min(
                available.value,
                MAX_COMPRESSED_VALUES if compressed else MAX_RAW_READ_SAMPLES,
            )
            if compressed:
                count -= count % 2  # compressed data are value/span pairs
            if count:
                decode_started = time.perf_counter()
                if compressed:
                    require(
                        dwf.FDwfDigitalInStatusCompressed(handle, compressed_data, ct.c_int(count)),
                        dwf,
                        "read compressed DigitalIn samples",
                    )
                    total_transfer_values += count
                    samples = expand_compressed_samples(bytes(compressed_data[:count]), dio)
                else:
                    require(
                        dwf.FDwfDigitalInStatusData(handle, data, ct.c_int(count)),
                        dwf,
                        "read DigitalIn samples",
                    )
                    total_transfer_values += count
                    samples = bytes(data[:count])
                total_samples += len(samples)
                previous_candidate_ids = decoder.candidate_ids.copy()
                decoded_frames = decoder.feed(samples)
                display.add_candidate_headers(decoder.candidate_ids - previous_candidate_ids)
                for frame in decoded_frames:
                    display.add_frame(frame.identifier, 0, 0, frame.dlc, frame.payload)
                decode_seconds += time.perf_counter() - decode_started

            now = time.monotonic()
            if stop_at is not None and now >= stop_at:
                raw_errors = total_lost + total_corrupt + decoder.bad_crc + decoder.decode_errors
                return display.total_frames, raw_errors
            if now >= next_refresh:
                transfer = (
                    f"compressed values {total_transfer_values} "
                    f"({total_samples / max(1, total_transfer_values):.1f}x)"
                    if compressed
                    else f"transfer samples {total_transfer_values}"
                )
                decode_load = 100.0 * decode_seconds / max(1e-9, time.perf_counter() - monitor_started)
                valid_frames = sum(decoder.frames.values())
                rejected_frames = decoder.bad_crc + decoder.decode_errors + decoder.unsupported
                valid_yield = 100.0 * valid_frames / max(1, valid_frames + rejected_frames)
                fallback_hits = sum(decoder.phase_hits.values())
                display.set_capture_health(
                    f"raw {actual_rate / 1_000_000:g} MHz; expanded samples {total_samples}; {transfer}\n"
                    f"continuity: lost {total_lost}; corrupt {total_corrupt}; bad CRC {decoder.bad_crc}; "
                    f"decode errors {decoder.decode_errors}; candidate yield {valid_yield:.1f}%\n"
                    f"decoder: load {decode_load:.0f}%; resyncs {decoder.resyncs}; "
                    f"vote hits {decoder.majority_hits}; fallback hits {fallback_hits}"
                )
                if display.dirty:
                    display.render()
                next_refresh = now + refresh_period
    except KeyboardInterrupt:
        raw_errors = total_lost + total_corrupt
        if decoder is not None:
            raw_errors += decoder.bad_crc + decoder.decode_errors
        return display.total_frames, raw_errors
    finally:
        if display.dirty:
            display.render()
        display.close()
        dwf.FDwfDigitalInConfigure(handle, ct.c_int(1), ct.c_int(0))
        dwf.FDwfDeviceClose(handle)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--version", action="version", version=f"{APP_NAME} {APP_VERSION}")
    parser.add_argument("--device", type=int, default=-1, help="WaveForms device index (default: first device)")
    parser.add_argument("--dio", type=int, default=7, help="AD2 digital input number (default: 7)")
    parser.add_argument("--bitrate", type=float, default=1_000_000, help="CAN bitrate in bit/s (default: 1000000)")
    parser.add_argument(
        "--backend", choices=("raw", "decoder"), default="raw",
        help="raw batches and decodes DigitalIn samples; decoder uses the WaveForms CAN API (default: raw)",
    )
    parser.add_argument(
        "--sample-rate", type=int, default=4_000_000,
        help="raw-backend DigitalIn sample rate in Hz (default: 4000000)",
    )
    parser.add_argument(
        "--uncompressed", action="store_true",
        help="disable DWF value/span record compression for troubleshooting",
    )
    parser.add_argument(
        "--filter", dest="filters", type=lambda value: int(value, 0), action="append", default=[], metavar="CAN_ID",
        help="show and count this CAN ID only; repeat for multiple IDs (for example: --filter 0x077)",
    )
    parser.add_argument(
        "--refresh-hz", type=float, default=24.0,
        help="maximum live-display refresh rate in Hz (default: 24)",
    )
    parser.add_argument(
        "--duration", type=float, default=None, metavar="SECONDS",
        help="stop cleanly after this many seconds (default: run until Ctrl+C)",
    )
    args = parser.parse_args()
    if not 0 <= args.dio <= 15:
        parser.error("--dio must be between 0 and 15 for an AD2")
    if args.bitrate <= 0:
        parser.error("--bitrate must be positive")
    if args.sample_rate <= 0:
        parser.error("--sample-rate must be positive")
    if args.refresh_hz <= 0:
        parser.error("--refresh-hz must be positive")
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    if any(identifier < 0 or identifier > 0x1FFFFFFF for identifier in args.filters):
        parser.error("--filter must be a CAN ID from 0 through 0x1FFFFFFF")
    args.filters = set(args.filters)
    return args


if __name__ == "__main__":
    prepare_windows_console()
    arguments = parse_args()
    try:
        if arguments.backend == "raw":
            received, errors = monitor_raw(
                arguments.device, arguments.dio, arguments.bitrate, arguments.sample_rate,
                arguments.refresh_hz, arguments.filters, not arguments.uncompressed, arguments.duration,
            )
            print(f"Stopped. Matching frames received: {received}; raw capture issues: {errors}.")
        else:
            received, errors = monitor_decoder(
                arguments.device, arguments.dio, arguments.bitrate, arguments.refresh_hz,
                arguments.filters, arguments.duration,
            )
            print(f"Stopped. Matching frames received: {received}; decoder errors: {errors}.")
    except DwfError as error:
        print(f"error: {error}", file=sys.stderr)
        # A double-clicked console would otherwise disappear before the user
        # can read a missing-installation or device-busy error.
        if getattr(sys, "frozen", False) and sys.stdin.isatty():
            try:
                input("Press Enter to close...")
            except (EOFError, KeyboardInterrupt):
                pass
        sys.exit(1)
