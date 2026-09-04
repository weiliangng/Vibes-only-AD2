#!/usr/bin/env python3
"""Passively monitor a CAN bus with a Digilent Analog Discovery 2.

The default raw backend samples the selected digital input, batches the logic
samples, and decodes standard CAN data frames in Python.  It does not configure
any CAN transmit pin and does not ACK frames.  The older WaveForms CAN decoder
is retained as an optional fallback.

By default it listens for a 1 Mbit/s logic-level CAN RX signal on DIO 7.
The live display has one row per CAN address.  A row is added only when an
address is first seen; later frames from that address refresh its data,
timestamp, DLC, and frame count in place.

Frame collection is intentionally independent of screen refreshes.  This
keeps terminal I/O out of the receive path on a busy bus.

Use --filter one or more times to show/count only selected CAN IDs.  Filtering
is after capture, so it does not reduce the raw-sample bandwidth requirement.

Run from PowerShell:
  & "$env:LOCALAPPDATA/Programs/Python/Python312/python.exe" tools/ad2_can_monitor.py
"""

from __future__ import annotations

import argparse
import ctypes as ct
import os
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass, field
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

# DWF CAN receiver status values.  Unknown non-zero statuses are still shown.
CAN_STATUS = {
    2: "bit-stuffing error",
    3: "CRC error",
}
RECORD_MODE = 3
SAMPLE_FORMAT_BITS = 8
MAX_RAW_READ_SAMPLES = 16_384


class DwfError(RuntimeError):
    """An error returned by the WaveForms DWF library."""


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


def error_message(dwf: ct.CDLL) -> str:
    message = ct.create_string_buffer(512)
    dwf.FDwfGetLastErrorMsg(message)
    return message.value.decode(errors="replace") or "unknown DWF error"


def require(ok: int, dwf: ct.CDLL, operation: str) -> None:
    if not ok:
        raise DwfError(f"{operation} failed: {error_message(dwf)}")


def timestamp() -> str:
    now = time.time_ns()
    return time.strftime("%H:%M:%S", time.localtime(now // 1_000_000_000)) + f".{now // 1_000_000 % 1_000:03d}"


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


class LiveCanDisplay:
    """Render a stable address table using ANSI cursor controls."""

    def __init__(self, stream: TextIO, bitrate: float, dio: int, filters: set[int]) -> None:
        self.stream = stream
        self.bitrate = bitrate
        self.dio = dio
        self.filters = filters
        self.frames: dict[tuple[bool, int], CanFrame] = {}
        self.errors: Counter[str] = Counter()
        self.last_error = "--"
        self.total_frames = 0
        self.capture_health = "--"
        self._interactive = stream.isatty()
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
        lines = [
            f"Listening: CAN {self.bitrate / 1_000_000:g} Mbit/s, RX=DIO {self.dio}. Press Ctrl+C to stop.",
            f"Filter: {', '.join(f'0x{identifier:03X}' for identifier in sorted(self.filters)) if self.filters else 'all CAN IDs'}",
            f"Capture health: {self.capture_health}",
            "",
            f"CAN packets by address ({len(self.frames)} unique, {self.total_frames} total)",
            "Address      Format  Type  DLC  Data                     Last received   Rate (Hz)  Frames",
            "-----------  ------  ----  ---  -----------------------  --------------  ---------  ------",
        ]
        for frame in self.frames.values():
            lines.append(
                f"{frame.address:<11}  {frame.format:<6}  {frame.frame_type:<4}  {frame.dlc:>3}  "
                f"{frame.data:<23.23}  {frame.last_seen:<14}  {frame.rate_hz:>9.1f}  {frame.count:>6}"
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
            self.stream.write("\x1b[H" + output + "\x1b[J")
        else:
            # Preserve a useful log when output is redirected instead of a terminal.
            self.stream.write(output + "\n\n")
        self.stream.flush()
        self.dirty = False


def monitor_decoder(device: int, dio: int, bitrate: float, refresh_hz: float, filters: set[int]) -> tuple[int, int]:
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
                display.add_frame(identifier.value, extended.value, remote.value, dlc.value, bytes(payload[:dlc.value]))
            elif status.value:
                display.add_error(status.value)

            now = time.monotonic()
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
    device: int, dio: int, bitrate: float, sample_rate: int, refresh_hz: float, filters: set[int]
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
    total_samples = total_lost = total_corrupt = 0
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
        require(dwf.FDwfDigitalInTriggerPositionSet(handle, ct.c_int(0)), dwf, "set continuous DigitalIn acquisition")
        require(dwf.FDwfDigitalInConfigure(handle, ct.c_int(1), ct.c_int(1)), dwf, "start DigitalIn")

        decoder = RawCanDecoder(dio, samples_per_bit)
        data = (ct.c_ubyte * MAX_RAW_READ_SAMPLES)()
        state = ct.c_ubyte()
        available = ct.c_int()
        lost = ct.c_int()
        corrupt = ct.c_int()
        refresh_period = 1.0 / refresh_hz
        next_refresh = time.monotonic() + refresh_period
        display.set_capture_health(
            f"raw DigitalIn: {actual_rate / 1_000_000:g} MHz, {samples_per_bit} samples/bit, "
            f"buffer {max_buffer.value} samples"
        )
        display.start()

        while True:
            require(dwf.FDwfDigitalInStatus(handle, ct.c_int(1), ct.byref(state)), dwf, "poll DigitalIn")
            require(
                dwf.FDwfDigitalInStatusRecord(handle, ct.byref(available), ct.byref(lost), ct.byref(corrupt)),
                dwf,
                "read DigitalIn record status",
            )
            total_lost += lost.value
            total_corrupt += corrupt.value
            count = min(available.value, MAX_RAW_READ_SAMPLES)
            if count:
                require(dwf.FDwfDigitalInStatusData(handle, data, ct.c_int(count)), dwf, "read DigitalIn samples")
                total_samples += count
                for frame in decoder.feed(bytes(data[:count])):
                    display.add_frame(frame.identifier, 0, 0, frame.dlc, frame.payload)

            now = time.monotonic()
            if now >= next_refresh:
                display.set_capture_health(
                    f"raw {actual_rate / 1_000_000:g} MHz; samples {total_samples}; lost {total_lost}; "
                    f"corrupt {total_corrupt}; bad CRC {decoder.bad_crc}; resyncs {decoder.resyncs}"
                )
                if display.dirty:
                    display.render()
                next_refresh = now + refresh_period
    except KeyboardInterrupt:
        raw_errors = total_lost + total_corrupt
        if decoder is not None:
            raw_errors += decoder.bad_crc + decoder.resyncs
        return display.total_frames, raw_errors
    finally:
        if display.dirty:
            display.render()
        display.close()
        dwf.FDwfDigitalInConfigure(handle, ct.c_int(1), ct.c_int(0))
        dwf.FDwfDeviceClose(handle)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
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
        "--filter", dest="filters", type=lambda value: int(value, 0), action="append", default=[], metavar="CAN_ID",
        help="show and count this CAN ID only; repeat for multiple IDs (for example: --filter 0x077)",
    )
    parser.add_argument(
        "--refresh-hz", type=float, default=24.0,
        help="maximum live-display refresh rate in Hz (default: 24)",
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
    if any(identifier < 0 or identifier > 0x1FFFFFFF for identifier in args.filters):
        parser.error("--filter must be a CAN ID from 0 through 0x1FFFFFFF")
    args.filters = set(args.filters)
    return args


if __name__ == "__main__":
    arguments = parse_args()
    try:
        if arguments.backend == "raw":
            received, errors = monitor_raw(
                arguments.device, arguments.dio, arguments.bitrate, arguments.sample_rate,
                arguments.refresh_hz, arguments.filters,
            )
            print(f"Stopped. Matching frames received: {received}; raw capture issues: {errors}.")
        else:
            received, errors = monitor_decoder(
                arguments.device, arguments.dio, arguments.bitrate, arguments.refresh_hz, arguments.filters,
            )
            print(f"Stopped. Matching frames received: {received}; decoder errors: {errors}.")
    except DwfError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
