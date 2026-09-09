import math
import struct
import unittest

from ad2_can_raw_monitor import RawCanDecoder, crc15
from ad2_can_monitor import Cm01PmmReassembler, decode_can_frame, enable_vplus_supply, expand_compressed_samples


DIO = 7
DIO_MASK = 1 << DIO


class RecordingAnalogIo:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []

    def FDwfAnalogIOEnableSet(self, _handle: object, enabled: object) -> int:
        self.calls.append(("master", enabled.value))
        return 1

    def FDwfAnalogIOChannelNodeSet(
        self, _handle: object, channel: object, node: object, value: object
    ) -> int:
        self.calls.append(("node", channel.value, node.value, value.value))
        return 1


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
    def test_vplus_supply_setup_keeps_vminus_disabled(self) -> None:
        dwf = RecordingAnalogIo()

        enable_vplus_supply(dwf, object())

        self.assertEqual(
            dwf.calls,
            [
                ("master", 0),
                ("node", 1, 0, 0.0),
                ("node", 0, 1, 5.0),
                ("node", 0, 0, 1.0),
                ("master", 1),
            ],
        )

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


class CanContractDecoderTests(unittest.TestCase):
    def decode(self, identifier: int, payload: bytes) -> str:
        return decode_can_frame(identifier, False, False, len(payload), payload)

    def test_decodes_both_supercap_telemetry_contracts_by_dlc(self) -> None:
        scv2 = self.decode(0x077, bytes.fromhex("02 00 06 01 22 00 00 00"))
        legacy = self.decode(0x077, struct.pack("<fBB", 123.5, 2, 128))

        self.assertIn("vcap=26.2 V", scv2)
        self.assertIn("legacy supercap: power=123.5 W", legacy)
        self.assertIn("error=SWEN low", legacy)

    def test_decodes_inter_devc_contracts(self) -> None:
        chassis = self.decode(0x100, struct.pack("<hhhBB", 500, -250, 1000, 1, 80))
        odometry = self.decode(0x102, struct.pack(">hhhh", 100, -200, 300, -400))
        imu = self.decode(0x104, bytes((7, 0x07)) + struct.pack("<hhh", 1000, -2000, 3000))

        self.assertIn("fwd=0.500 strafe=-0.250 yaw=1.000 enable=1 power_limit=80 W", chassis)
        self.assertIn("FR=100 FL=-200 BL=300 BR=-400 rpm", odometry)
        self.assertIn("seq=7 valid=0x07 xyz=(1,-2,3) rad/s", imu)

    def test_decodes_motor_group_commands_and_feedback(self) -> None:
        command = self.decode(0x200, struct.pack(">hhhh", 1000, -8192, 0, 16384))
        feedback = self.decode(0x205, struct.pack(">HhhBB", 4096, -120, 55, 42, 0))

        self.assertIn("DJI torque-current cmd IDs1-4", command)
        self.assertIn("C610=[1.00,-8.19,0.00,16.38] A", command)
        self.assertIn("C620=[1.22,-10.00,0.00,20.00] A", command)
        self.assertIn("rotor_angle=180.00 deg (4096/8192) rotor_speed=-120 rpm", feedback)
        self.assertIn("shaft~M2006:-3.33/M3508:-6.25 rpm", feedback)
        self.assertIn("C610~0.055 A/C620~0.067 A/GM6020~0.010 A", feedback)

    def test_decodes_gm6020_current_and_voltage_modes(self) -> None:
        current = self.decode(0x1FE, struct.pack(">hhhh", 16384, -8192, 0, 1))
        voltage = self.decode(0x2FF, struct.pack(">hhhh", 25000, -12500, 0, 7))
        feedback = self.decode(0x209, struct.pack(">HhhBB", 2048, 320, -8192, 51, 0))

        self.assertIn("GM6020 torque-current cmd IDs1-4", current)
        self.assertIn("current=[3.000,-1.500,0.000,0.000] A", current)
        self.assertIn("GM6020 voltage cmd IDs5-7: demand=[100.0,-50.0,0.0]%FS", voltage)
        self.assertIn("reserved_raw=7", voltage)
        self.assertIn("GM6020 fb ID5: angle=90.00 deg", feedback)
        self.assertIn("speed=320 rpm torque_current~-1.500 A", feedback)

    def test_decodes_shared_dji_and_dm_scaled_fields(self) -> None:
        shared = self.decode(0x1FF, struct.pack(">hhhh", 25000, 0, -12500, 0))
        dm_command = self.decode(0x3FE, struct.pack(">hhhh", 8192, -16384, 0, 4096))
        dm_feedback = self.decode(0x302, struct.pack(">HhhBB", 4096, -1234, 77, 40, 0))

        self.assertIn("C610/C620 IDs5-8 current=", shared)
        self.assertIn("GM6020 IDs1-4 voltage=[100.0,0.0,-50.0,0.0]%FS", shared)
        self.assertIn("demand=[50.0,-100.0,0.0,25.0]%FS", dm_command)
        self.assertIn("DM DJI-mode fb ID2: angle=180.00 deg", dm_feedback)
        self.assertIn("speed=-12.34 rpm torque/current_raw=77", dm_feedback)

    def test_decodes_dm_mit_feedback(self) -> None:
        decoded = self.decode(0x091, bytes.fromhex("21 80 00 80 08 00 30 31"))

        self.assertIn("DM MIT fb: id=1 state=2", decoded)
        self.assertIn("Tmos=48 C Tcoil=49 C", decoded)

    def test_decodes_60v15a_wattmeter(self) -> None:
        decoded = self.decode(0x213, bytes.fromhex("3F 09 05 00 00 00 00 00"))

        self.assertEqual(
            decoded,
            "60V15A wattmeter: voltage=23.67 V current=0.05 A power=1.18 W",
        )

        # The supplied manual also identifies current and older revisions.
        self.assertIn("voltage=23.67 V", self.decode(0x212, bytes.fromhex("3F 09 05 00 00 00 00 00")))
        self.assertIn("voltage=23.67 V", self.decode(0x211, bytes.fromhex("3F 09 05 00 00 00 00 00")))

    def test_reassembles_cm01_pmm_measurement(self) -> None:
        packet = bytes.fromhex(
            "5A 0D 10 90 02 65 01 00 10 00 82 43 00 09 1A "
            "16 03 D2 41 5A 18 DB 3C 32 9D 99 3B 00 DF 73"
        )
        reassembler = Cm01PmmReassembler()

        results = [
            reassembler.feed(packet[0:8]),
            reassembler.feed(packet[8:16]),
            reassembler.feed(packet[16:24]),
            reassembler.feed(packet[24:30]),
        ]

        self.assertEqual(results[:3], [None, None, None])
        self.assertIn("voltage=26.252 V", results[3] or "")
        self.assertIn("net_current=-0.0221 A (+charge)", results[3] or "")
        self.assertIn("charge=0.0047 A discharge=0.0267 A", results[3] or "")
        self.assertIn("route=0x0001 seq=17282", results[3] or "")

    def test_cm01_reassembler_recovers_after_a_missing_fragment(self) -> None:
        packet = bytes.fromhex(
            "5A 0D 10 90 02 65 80 00 10 00 83 43 00 09 1A "
            "16 03 D2 41 5A 18 DB 3C 32 9D 99 3B 00 D3 39"
        )
        reassembler = Cm01PmmReassembler()

        reassembler.feed(packet[:8])
        reassembler.feed(packet[8:16])  # Deliberately omit the third and fourth fragments.
        self.assertIsNone(reassembler.feed(packet[:8]))
        self.assertIsNone(reassembler.feed(packet[8:16]))
        self.assertIsNone(reassembler.feed(packet[16:24]))
        decoded = reassembler.feed(packet[24:])

        self.assertIn("voltage=26.252 V", decoded or "")
        self.assertIn("route=0x0080 seq=17283", decoded or "")

    def test_cm01_reassembler_rejects_bad_message_crc(self) -> None:
        packet = bytearray.fromhex(
            "5A 0D 10 90 02 65 01 00 10 00 82 43 00 09 1A "
            "16 03 D2 41 5A 18 DB 3C 32 9D 99 3B 00 DF 73"
        )
        packet[19] ^= 0x01
        reassembler = Cm01PmmReassembler()

        self.assertIsNone(reassembler.feed(packet[:8]))
        self.assertIsNone(reassembler.feed(packet[8:16]))
        self.assertIsNone(reassembler.feed(packet[16:24]))
        self.assertIsNone(reassembler.feed(packet[24:]))

    def test_does_not_guess_wrong_dlc_or_extended_frames(self) -> None:
        self.assertEqual(decode_can_frame(0x100, False, False, 7, bytes(7)), "")
        self.assertEqual(decode_can_frame(0x200, True, False, 8, bytes(8)), "")


if __name__ == "__main__":
    unittest.main()
