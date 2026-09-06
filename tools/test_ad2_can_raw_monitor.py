import math
import unittest

from ad2_can_raw_monitor import RawCanDecoder, crc15
from ad2_can_monitor import expand_compressed_samples


DIO = 7
DIO_MASK = 1 << DIO


def bits_of(value: int, width: int) -> list[int]:
    return [(value >> shift) & 1 for shift in range(width - 1, -1, -1)]


def stuffed_standard_frame(identifier: int, payload: bytes) -> list[int]:
    protected = (
        [0]
        + bits_of(identifier, 11)
        + [0, 0, 0]
        + bits_of(len(payload), 4)
        + [bit for byte in payload for bit in bits_of(byte, 8)]
    )
    protected += bits_of(crc15(protected), 15)

    stuffed: list[int] = []
    last = None
    run_length = 0
    for bit in protected:
        stuffed.append(bit)
        if bit == last:
            run_length += 1
        else:
            last = bit
            run_length = 1
        if run_length == 5:
            last = 1 - bit
            stuffed.append(last)
            run_length = 1

    # CRC delimiter, ACK slot, ACK delimiter, EOF, and intermission.
    return stuffed + [1, 0, 1] + [1] * 7 + [1] * 3


def integer_samples(bits: list[int], samples_per_bit: int = 4) -> bytearray:
    levels = [1] * 48
    for bit in bits:
        levels.extend([bit] * samples_per_bit)
    levels.extend([1] * 16)
    return bytearray(DIO_MASK if level else 0 for level in levels)


def fractional_samples(bits: list[int], samples_per_bit: float) -> bytearray:
    edge_time = 48.25
    sample_count = math.ceil(edge_time + len(bits) * samples_per_bit) + 20
    levels: list[int] = []
    for sample in range(sample_count):
        if sample < edge_time:
            levels.append(1)
            continue
        bit_index = int((sample - edge_time) // samples_per_bit)
        levels.append(bits[bit_index] if bit_index < len(bits) else 1)
    return bytearray(DIO_MASK if level else 0 for level in levels)


class RawCanDecoderTests(unittest.TestCase):
    def test_decodes_crc_valid_standard_frame(self) -> None:
        payload = bytes.fromhex("02 00 06 01 22 00 00 00")
        decoder = RawCanDecoder(DIO, 4)

        frames = decoder.feed(integer_samples(stuffed_standard_frame(0x077, payload)))

        self.assertEqual([(frame.identifier, frame.payload) for frame in frames], [(0x077, payload)])
        self.assertEqual(decoder.bad_crc, 0)

    def test_decodes_zero_length_frame(self) -> None:
        decoder = RawCanDecoder(DIO, 4)

        frames = decoder.feed(integer_samples(stuffed_standard_frame(0x000, b"")))

        self.assertEqual([(frame.identifier, frame.dlc, frame.payload) for frame in frames], [(0x000, 0, b"")])

    def test_expands_dwf_value_span_pairs(self) -> None:
        encoded = bytes((DIO_MASK, 3, 0, 1, DIO_MASK, 0))

        samples = expand_compressed_samples(encoded, DIO)

        self.assertEqual(samples, bytes((DIO_MASK,)) * 4 + bytes(2) + bytes((DIO_MASK,)))

    def test_preserves_candidates_across_capture_blocks(self) -> None:
        expected = [
            (0x067, bytes.fromhex("01 00 50 3C 00")),
            (0x213, bytes.fromhex("3D 09 04 00 00 00 00 00")),
        ]
        samples = bytearray()
        for identifier, payload in expected:
            samples.extend(integer_samples(stuffed_standard_frame(identifier, payload)))
        decoder = RawCanDecoder(DIO, 4)
        frames = []

        for offset in range(0, len(samples), 37):
            frames.extend(decoder.feed(samples[offset:offset + 37]))

        self.assertEqual([(frame.identifier, frame.payload) for frame in frames], expected)

    def test_decodes_frames_separated_only_by_intermission(self) -> None:
        expected = [
            (0x067, bytes.fromhex("01 00 50 3C 00")),
            (0x077, bytes.fromhex("02 00 06 01 22 00 00 00")),
        ]
        levels = [1] * 48
        for identifier, payload in expected:
            for bit in stuffed_standard_frame(identifier, payload):
                levels.extend([bit] * 4)
        levels.extend([1] * 16)
        samples = bytes(DIO_MASK if level else 0 for level in levels)
        decoder = RawCanDecoder(DIO, 4)

        frames = decoder.feed(samples)

        self.assertEqual([(frame.identifier, frame.payload) for frame in frames], expected)

    def test_discards_partial_frame_after_capture_discontinuity(self) -> None:
        payload = bytes.fromhex("3D 09 04 00 00 00 00 00")
        samples = integer_samples(stuffed_standard_frame(0x213, payload))
        decoder = RawCanDecoder(DIO, 4)

        self.assertEqual(decoder.feed(samples[:len(samples) // 2]), [])
        decoder.reset_stream()
        frames = decoder.feed(samples)

        self.assertEqual([(frame.identifier, frame.payload) for frame in frames], [(0x213, payload)])

    def test_votes_across_sample_phases(self) -> None:
        payload = bytes.fromhex("3D 09 04 00 00 00 00 00")
        samples = integer_samples(stuffed_standard_frame(0x213, payload))
        start = samples.index(0)
        disturbed_raw_bit = 25
        disturbed_sample = start + disturbed_raw_bit * 4 + 2
        samples[disturbed_sample] ^= DIO_MASK

        fixed_phase = RawCanDecoder(DIO, 4)
        fixed_phase._samples.extend(samples.translate(fixed_phase._to_level))
        candidate = fixed_phase._decode_at(start, 2)
        self.assertTrue(candidate is None or candidate[0] is None)

        decoder = RawCanDecoder(DIO, 4)
        frames = decoder.feed(samples)

        self.assertEqual([(frame.identifier, frame.payload) for frame in frames], [(0x213, payload)])
        self.assertEqual(decoder._vote_phases, (0, 2, 3))
        self.assertEqual(decoder._sample_phases, [0, 1])
        self.assertEqual(decoder.majority_hits, 1)

    def test_resynchronizes_to_fractional_bit_timing(self) -> None:
        payload = bytes.fromhex("3D 09 04 00 00 00 00 00")
        samples = fractional_samples(stuffed_standard_frame(0x213, payload), 4.08)
        decoder = RawCanDecoder(DIO, 4)

        frames = decoder.feed(samples)

        self.assertEqual([(frame.identifier, frame.payload) for frame in frames], [(0x213, payload)])
        self.assertGreater(decoder.resyncs, 0)

    def test_five_recessive_bits_inside_frame_are_not_an_sof(self) -> None:
        levels = [1] * 48 + [0] * 4 + [1] * 20 + [0] * 4 + [1] * 48
        samples = bytes(DIO_MASK if level else 0 for level in levels)
        decoder = RawCanDecoder(DIO, 4)
        attempted_starts: list[int] = []

        def reject(start: int, _phase: int | None):
            attempted_starts.append(start)
            return None, start + 4, "decode", 0, None

        decoder._decode_at = reject  # type: ignore[method-assign]
        decoder.feed(samples)

        self.assertEqual(set(attempted_starts), {48})
        self.assertEqual(decoder.decode_errors, 1)

    def test_identifies_frame_header_when_payload_fails_validation(self) -> None:
        identifier = 0x077
        samples = integer_samples(stuffed_standard_frame(identifier, bytes(8)))
        frame_start = samples.index(0)
        for phase in range(4):
            samples[frame_start + 30 * 4 + phase] ^= DIO_MASK
        decoder = RawCanDecoder(DIO, 4)

        frames = decoder.feed(samples)

        self.assertEqual(frames, [])
        self.assertEqual(decoder.candidate_ids[identifier], 1)


if __name__ == "__main__":
    unittest.main()
