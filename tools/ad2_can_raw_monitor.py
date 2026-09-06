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
        # A valid SOF is preceded by ACK delimiter + seven EOF bits + three
        # intermission bits: at least eleven contiguous recessive bits. Using
        # only intermission wrongly treats legal five-bit runs inside a damaged
        # frame as fresh SOFs. Allow one sample of edge quantization uncertainty.
        self._minimum_idle_samples = max(1, 11 * samples_per_bit - 1)
        self._to_level = bytes(1 if value & self._dio_mask else 0 for value in range(256))
        self._samples = bytearray()
        self.frames: Counter[int] = Counter()
        self.bad_crc = 0
        self.unsupported = 0
        self.decode_errors = 0
        self.resyncs = 0
        self.majority_hits = 0
        self.phase_hits: Counter[int] = Counter()
        self.sof_candidates = 0
        self.candidate_ids: Counter[int] = Counter()

        if samples_per_bit < 4:
            raise ValueError("CAN decoding requires at least four samples per bit")

        # Vote across the bit cell to reject an isolated threshold glitch.
        # An early-phase retry recovers edge-aligned cells cheaply. If either
        # attempt gets all the way to a CRC mismatch, one middle-phase retry
        # is worthwhile; malformed headers do not pay that extra CPU cost.
        preferred_phase = min(samples_per_bit - 1, max(0, round(samples_per_bit * 0.7 - 0.5)))
        self.preferred_phase = preferred_phase
        self._vote_phases = (0, preferred_phase, samples_per_bit - 1)
        self._sample_phases = [0, max(1, preferred_phase - 1)]

    def reset_stream(self) -> None:
        """Discard an incomplete frame after a capture discontinuity."""
        self._samples.clear()

    def feed(self, samples: bytes) -> list[CanFrame]:
        """Add raw eight-bit samples and return all complete, valid CAN frames."""
        self._samples.extend(samples.translate(self._to_level))
        result: list[CanFrame] = []
        scan_from = 1

        while True:
            start = self._samples.find(0, scan_from)
            if start < 0:
                # Retain enough idle history to recognize an SOF that arrives
                # in the next capture block.
                keep = self._minimum_idle_samples + 1
                if len(self._samples) > keep:
                    del self._samples[:-keep]
                return result
            if self._samples[start - 1] != 1:
                scan_from = start + 1
                continue
            if (
                start < self._minimum_idle_samples
                or self._samples.find(0, start - self._minimum_idle_samples, start) >= 0
            ):
                # Falling edges inside a frame are resynchronization edges,
                # not possible SOFs. Skipping them prevents four expensive
                # phase attempts for every corrupt frame fragment.
                scan_from = start + 1
                continue

            decoded = None
            incomplete = False
            failures: list[str] = []
            attempted_identifiers: list[int] = []
            candidate = self._decode_at(start, None)
            if candidate is None:
                incomplete = True
            elif candidate[0] is not None:
                decoded = candidate
                self.majority_hits += 1
            else:
                failures.append(candidate[2])
                if candidate[4] is not None:
                    attempted_identifiers.append(candidate[4])

            for phase_index, phase in enumerate(self._sample_phases):
                if decoded is not None:
                    break
                if phase_index and "crc" not in failures:
                    break
                candidate = self._decode_at(start, phase)
                if candidate is None:
                    incomplete = True
                    continue
                if candidate[0] is not None:
                    decoded = candidate
                    self.phase_hits[phase] += 1
                    break
                failures.append(candidate[2])
                if candidate[4] is not None:
                    attempted_identifiers.append(candidate[4])

            if decoded is None and incomplete:
                # The current block may end in the middle of this candidate.
                del self._samples[:start - self._minimum_idle_samples]
                return result

            self.sof_candidates += 1
            if decoded is not None and decoded[0] is not None:
                self.candidate_ids[decoded[0].identifier] += 1
            elif attempted_identifiers:
                identifier = Counter(attempted_identifiers).most_common(1)[0][0]
                self.candidate_ids[identifier] += 1

            if decoded is None:
                # Count a rejected edge once, not once per attempted phase.
                reason = "extended" if "extended" in failures else "crc" if "crc" in failures else "decode"
                if reason == "crc":
                    self.bad_crc += 1
                elif reason == "extended":
                    self.unsupported += 1
                else:
                    self.decode_errors += 1
                scan_from = start + self.samples_per_bit
                continue

            frame, end, reason, timing_resyncs, _identifier = decoded
            if frame is not None:
                result.append(frame)
                self.frames[frame.identifier] += 1
                self.resyncs += timing_resyncs
                # The decoded trailer includes intermission. Retain it so an
                # immediately following frame still has a recognizable SOF.
                del self._samples[:max(0, end - self._minimum_idle_samples)]
                scan_from = 1
            else:
                if reason == "crc":
                    self.bad_crc += 1
                elif reason == "extended":
                    self.unsupported += 1
                else:
                    self.decode_errors += 1
                # Avoid retrying every low sample inside a failed candidate.
                scan_from = start + self.samples_per_bit

    def _decode_at(
        self, start: int, sample_phase: int | None
    ) -> tuple[CanFrame | None, int, str, int, int | None] | None:
        """Decode at a falling edge; None means more samples are required."""
        samples = self._samples
        sample_count = len(samples)
        samples_per_bit = self.samples_per_bit
        vote_phase_0, vote_phase_1, vote_phase_2 = self._vote_phases
        max_adjustment = max(1, samples_per_bit // 4)
        raw_index = 0
        anchor_index = 0
        anchor_sample = start
        timing_resyncs = 0
        last_bit: int | None = None
        run_length = 0
        protected_bits: list[int] = []

        def raw_bit() -> int | None:
            nonlocal raw_index, anchor_index, anchor_sample, timing_resyncs
            boundary = anchor_sample + (raw_index - anchor_index) * samples_per_bit

            if raw_index:
                # CAN receivers resynchronize on recessive-to-dominant edges.
                # Bound each correction so noise cannot move the decoder
                # arbitrarily far through a frame.
                at_boundary = (
                    0 < boundary < sample_count
                    and samples[boundary - 1] == 1
                    and samples[boundary] == 0
                )
                if not at_boundary:
                    edge = None
                    for adjustment in range(1, max_adjustment + 1):
                        early = boundary - adjustment
                        if 0 < early < sample_count and samples[early - 1] == 1 and samples[early] == 0:
                            edge = early
                            break
                        late = boundary + adjustment
                        if 0 < late < sample_count and samples[late - 1] == 1 and samples[late] == 0:
                            edge = late
                            break
                    if edge is not None:
                        anchor_index = raw_index
                        anchor_sample = edge
                        boundary = edge
                        timing_resyncs += 1

            if sample_phase is None:
                if boundary + vote_phase_2 >= sample_count:
                    return None
                ones = (
                    samples[boundary + vote_phase_0]
                    + samples[boundary + vote_phase_1]
                    + samples[boundary + vote_phase_2]
                )
                value = int(ones >= 2)
            else:
                sample = boundary + sample_phase
                if sample >= sample_count:
                    return None
                value = samples[sample]
            raw_index += 1
            return value

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
            return None, start + self.samples_per_bit, "decode", timing_resyncs, None

        identifier = 0
        for bit in header[1:12]:
            identifier = (identifier << 1) | bit
        if header[13]:
            return None, start + self.samples_per_bit, "extended", timing_resyncs, identifier
        dlc = sum(bit << (3 - index) for index, bit in enumerate(header[15:19]))
        if dlc > 8 or header[12]:  # RTR frames are intentionally not decoded in this prototype.
            return None, start + self.samples_per_bit, "decode", timing_resyncs, identifier

        data_bits = protected(dlc * 8)
        crc_bits = protected(15)
        if data_bits is None or crc_bits is None:
            return None
        if not crc_bits:
            return None, start + self.samples_per_bit, "decode", timing_resyncs, identifier
        received_crc = sum(bit << (14 - index) for index, bit in enumerate(crc_bits))
        if crc15(protected_bits[:-15]) != received_crc:
            return None, start + self.samples_per_bit, "crc", timing_resyncs, identifier

        # CRC delimiter, ACK slot/delimiter, EOF, and intermission are not stuffed.
        trailer = [raw_bit() for _ in range(13)]
        if any(bit is None for bit in trailer):
            return None
        if trailer[0] != 1 or trailer[2] != 1 or trailer[3:10] != [1] * 7:
            return None, start + self.samples_per_bit, "decode", timing_resyncs, identifier

        payload = bytes(
            sum(data_bits[byte * 8 + offset] << (7 - offset) for offset in range(8))
            for byte in range(dlc)
        )
        end = anchor_sample + (raw_index - anchor_index) * samples_per_bit
        return CanFrame(identifier, dlc, payload), end, "ok", timing_resyncs, identifier


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
                phase_summary = "/".join(str(decoder.phase_hits[phase]) for phase in range(samples_per_bit))
                print(
                    f"frames: {sum(decoder.frames.values())} ({frame_summary or 'none'}); "
                    f"samples: {total_samples}; lost: {total_lost}; corrupt: {total_corrupt}; "
                    f"bad CRC: {decoder.bad_crc}; decode errors: {decoder.decode_errors}; "
                    f"timing resyncs: {decoder.resyncs}; phase hits [0..{samples_per_bit - 1}]: {phase_summary}; "
                    f"extended: {decoder.unsupported}",
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
