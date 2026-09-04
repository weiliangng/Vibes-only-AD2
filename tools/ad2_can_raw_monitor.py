#!/usr/bin/env python3
"""Raw, batched CAN monitor for an Analog Discovery 2 DigitalIn channel.

Unlike FDwfDigitalCanRx, this uses DigitalIn record mode to transfer blocks of
logic samples and decodes standard CAN data frames in Python.  It is an
experiment: the monitor explicitly reports lost/corrupt samples, which makes
its capture quality measurable.

The default 10 MHz sample rate gives ten samples per bit at 1 Mbit/s.  DIO 7
must be connected to a logic-level CAN-RX signal (not CAN_H/CAN_L directly).
"""

from __future__ import annotations

import argparse
import ctypes as ct
import os
import sys
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


DWF_DLL_PATHS = (
    Path(r"C:\Program Files\Digilent\WaveForms3\dwf.dll"),
    Path(r"C:\Program Files\Digilent\WaveFormsSDK\lib\dwf.dll"),
)
RECORD_MODE = 3
SAMPLE_FORMAT_BITS = 8
MAX_READ_SAMPLES = 16_384
CRC15_POLYNOMIAL = 0x4599


class DwfError(RuntimeError):
    """An error returned by the WaveForms DWF library."""


def load_dwf() -> ct.CDLL:
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

    dwf.FDwfDigitalInReset.argtypes = [handle]
    dwf.FDwfDigitalInReset.restype = ct.c_int
    dwf.FDwfDigitalInInternalClockInfo.argtypes = [handle, double_ptr]
    dwf.FDwfDigitalInInternalClockInfo.restype = ct.c_int
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
    # Kept explicit for documentation: the AD2 reports this size at runtime.
    dwf.FDwfDigitalInBufferSizeInfo.argtypes = [handle, int_ptr]
    dwf.FDwfDigitalInBufferSizeInfo.restype = ct.c_int
    dwf.FDwfDigitalInDividerInfo.argtypes = [handle, uint_ptr]
    dwf.FDwfDigitalInDividerInfo.restype = ct.c_int


def error_message(dwf: ct.CDLL) -> str:
    message = ct.create_string_buffer(512)
    dwf.FDwfGetLastErrorMsg(message)
    return message.value.decode(errors="replace") or "unknown DWF error"


def require(ok: int, dwf: ct.CDLL, operation: str) -> None:
    if not ok:
        raise DwfError(f"{operation} failed: {error_message(dwf)}")


def crc15(bits: list[int]) -> int:
    """Calculate CAN's CRC-15 over destuffed bits from SOF through data."""
    crc = 0
    for bit in bits:
        feedback = ((crc >> 14) & 1) ^ bit
        crc = (crc << 1) & 0x7FFF
        if feedback:
            crc ^= CRC15_POLYNOMIAL
    return crc


@dataclass(frozen=True)
class CanFrame:
    identifier: int
    dlc: int
    payload: bytes


class RawCanDecoder:
    """Incrementally recover standard CAN frames from uniformly sampled RX data."""

    def __init__(self, dio: int, samples_per_bit: int) -> None:
        self._dio_mask = 1 << dio
        self.samples_per_bit = samples_per_bit
        self._to_level = bytes(1 if value & self._dio_mask else 0 for value in range(256))
        self._samples = bytearray()
        self.frames: Counter[int] = Counter()
        self.bad_crc = 0
        self.unsupported = 0
        self.resyncs = 0

    def feed(self, samples: bytes) -> list[CanFrame]:
        """Add raw eight-bit samples and return all complete, valid CAN frames."""
        self._samples.extend(samples.translate(self._to_level))
        result: list[CanFrame] = []
        scan_from = 1

        while True:
            start = self._samples.find(0, scan_from)
            if start < 0:
                # Keep a small idle tail so a falling edge spanning two blocks
                # remains detectable without allowing an unbounded idle buffer.
                del self._samples[:-2]
                return result
            if self._samples[start - 1] != 1:
                scan_from = start + 1
                continue

            decoded = self._decode_at(start)
            if decoded is None:
                # The current block may end in the middle of this candidate.
                del self._samples[:start - 1]
                return result

            frame, end, reason = decoded
            if frame is not None:
                result.append(frame)
                self.frames[frame.identifier] += 1
                # Retain one recessive sample before the next candidate; a
                # back-to-back frame may start at the very next sample.
                del self._samples[:end - 1]
                scan_from = 1
            else:
                if reason == "crc":
                    self.bad_crc += 1
                elif reason == "extended":
                    self.unsupported += 1
                else:
                    self.resyncs += 1
                # Avoid retrying every low sample inside a failed candidate.
                scan_from = start + self.samples_per_bit

    def _decode_at(self, start: int) -> tuple[CanFrame | None, int, str] | None:
        """Decode at a falling edge; None means more samples are required."""
        raw_index = 0
        last_bit: int | None = None
        run_length = 0
        protected_bits: list[int] = []

        def raw_bit() -> int | None:
            nonlocal raw_index
            center = start + (raw_index * self.samples_per_bit) + self.samples_per_bit // 2
            if center >= len(self._samples):
                return None
            raw_index += 1
            return self._samples[center]

        def destuffed_bit() -> int | None:
            nonlocal last_bit, run_length
            bit = raw_bit()
            if bit is None:
                return None
            if bit == last_bit:
                run_length += 1
            else:
                last_bit, run_length = bit, 1
            if run_length == 5:
                stuff = raw_bit()
                if stuff is None:
                    return None
                if stuff == bit:
                    return -1
                last_bit, run_length = stuff, 1
            return bit

        def protected(count: int) -> list[int] | None:
            values: list[int] = []
            for _ in range(count):
                bit = destuffed_bit()
                if bit is None:
                    return None
                if bit < 0:
                    return []
                values.append(bit)
                protected_bits.append(bit)
            return values

        header = protected(19)  # SOF, 11-bit ID, RTR, IDE, r0, 4-bit DLC
        if header is None:
            return None
        if not header or header[0] != 0:
            return None, start + self.samples_per_bit, "resync"
        if header[13]:
            return None, start + self.samples_per_bit, "extended"

        identifier = 0
        for bit in header[1:12]:
            identifier = (identifier << 1) | bit
        dlc = sum(bit << (3 - index) for index, bit in enumerate(header[15:19]))
        if dlc > 8 or header[12]:  # RTR frames are intentionally not decoded in this prototype.
            return None, start + self.samples_per_bit, "resync"

        data_bits = protected(dlc * 8)
        crc_bits = protected(15)
        if data_bits is None or crc_bits is None:
            return None
        if not data_bits or not crc_bits:
            return None, start + self.samples_per_bit, "resync"
        received_crc = sum(bit << (14 - index) for index, bit in enumerate(crc_bits))
        if crc15(protected_bits[:-15]) != received_crc:
            return None, start + self.samples_per_bit, "crc"

        # CRC delimiter, ACK slot/delimiter, EOF, and intermission are not stuffed.
        trailer = [raw_bit() for _ in range(13)]
        if any(bit is None for bit in trailer):
            return None
        if trailer[0] != 1 or trailer[2] != 1 or trailer[3:10] != [1] * 7:
            return None, start + self.samples_per_bit, "resync"

        payload = bytes(
            sum(data_bits[byte * 8 + offset] << (7 - offset) for offset in range(8))
            for byte in range(dlc)
        )
        end = start + raw_index * self.samples_per_bit
        return CanFrame(identifier, dlc, payload), end, "ok"


def monitor(device: int, dio: int, bitrate: int, sample_rate: int, refresh_hz: float) -> None:
    dwf = load_dwf()
    configure_signatures(dwf)
    handle = ct.c_int()
    require(dwf.FDwfDeviceConfigOpen(device, 3, ct.byref(handle)), dwf, "open Analog Discovery 2")
    if not handle.value:
        raise DwfError(f"open Analog Discovery 2 failed: {error_message(dwf)}")

    try:
        base_rate = ct.c_double()
        max_buffer = ct.c_int()
        require(dwf.FDwfDigitalInReset(handle), dwf, "reset DigitalIn")
        require(dwf.FDwfDigitalInInternalClockInfo(handle, ct.byref(base_rate)), dwf, "read DigitalIn clock")
        require(dwf.FDwfDigitalInBufferSizeInfo(handle, ct.byref(max_buffer)), dwf, "read DigitalIn buffer size")
        divider = max(1, round(base_rate.value / sample_rate))
        actual_rate = int(base_rate.value / divider)
        if actual_rate < bitrate * 4:
            raise DwfError("sample rate must provide at least four samples per CAN bit")

        require(dwf.FDwfDigitalInAcquisitionModeSet(handle, RECORD_MODE), dwf, "set record mode")
        require(dwf.FDwfDigitalInDividerSet(handle, divider), dwf, "set DigitalIn sample rate")
        require(dwf.FDwfDigitalInSampleFormatSet(handle, SAMPLE_FORMAT_BITS), dwf, "set eight-bit sample format")
        require(dwf.FDwfDigitalInTriggerPositionSet(handle, 0), dwf, "set continuous acquisition")
        require(dwf.FDwfDigitalInConfigure(handle, 1, 1), dwf, "start DigitalIn")

        samples_per_bit = actual_rate // bitrate
        decoder = RawCanDecoder(dio, samples_per_bit)
        data = (ct.c_ubyte * MAX_READ_SAMPLES)()
        state = ct.c_ubyte()
        available = ct.c_int()
        lost = ct.c_int()
        corrupt = ct.c_int()
        total_samples = total_lost = total_corrupt = 0
        next_report = time.monotonic() + (1.0 / refresh_hz)
        print(
            f"Raw capture: DIO {dio}, {actual_rate / 1_000_000:g} MHz ({samples_per_bit} samples/bit), "
            f"DigitalIn buffer {max_buffer.value} samples. Ctrl+C to stop."
        )

        while True:
            require(dwf.FDwfDigitalInStatus(handle, 1, ct.byref(state)), dwf, "poll DigitalIn")
            require(
                dwf.FDwfDigitalInStatusRecord(handle, ct.byref(available), ct.byref(lost), ct.byref(corrupt)),
                dwf,
                "read DigitalIn record status",
            )
            total_lost += lost.value
            total_corrupt += corrupt.value
            count = min(available.value, MAX_READ_SAMPLES)
            if count:
                require(dwf.FDwfDigitalInStatusData(handle, data, count), dwf, "read DigitalIn samples")
                total_samples += count
                decoder.feed(bytes(data[:count]))

            now = time.monotonic()
            if now >= next_report:
                frame_summary = " ".join(f"0x{identifier:03X}={count}" for identifier, count in decoder.frames.items())
                print(
                    f"frames: {sum(decoder.frames.values())} ({frame_summary or 'none'}); "
                    f"samples: {total_samples}; lost: {total_lost}; corrupt: {total_corrupt}; "
                    f"bad CRC: {decoder.bad_crc}; resyncs: {decoder.resyncs}; extended: {decoder.unsupported}",
                    flush=True,
                )
                next_report = now + (1.0 / refresh_hz)
    except KeyboardInterrupt:
        pass
    finally:
        dwf.FDwfDigitalInConfigure(handle, 1, 0)
        dwf.FDwfDeviceClose(handle)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", type=int, default=-1, help="WaveForms device index (default: first device)")
    parser.add_argument("--dio", type=int, default=7, help="AD2 digital input number (default: 7)")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate in bit/s (default: 1000000)")
    parser.add_argument("--sample-rate", type=int, default=10_000_000, help="DigitalIn sample rate in Hz (default: 10000000)")
    parser.add_argument("--refresh-hz", type=float, default=1.0, help="report rate in Hz (default: 1)")
    args = parser.parse_args()
    if not 0 <= args.dio <= 7:
        parser.error("--dio must be between 0 and 7 in eight-bit sample mode")
    if args.bitrate <= 0 or args.sample_rate <= 0 or args.refresh_hz <= 0:
        parser.error("--bitrate, --sample-rate, and --refresh-hz must be positive")
    return args


if __name__ == "__main__":
    arguments = parse_args()
    try:
        monitor(arguments.device, arguments.dio, arguments.bitrate, arguments.sample_rate, arguments.refresh_hz)
    except DwfError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)
