import unittest

from protocol.protocol_constants import MessageId, SafetyFlag, SystemState
from jetson.amr_core.packet import (
    DriveCommand,
    Packet,
    PacketError,
    RobotStatus,
    WheelCommand,
    decode_drive_command,
    decode_packet,
    decode_robot_status,
    encode_drive_command,
    encode_packet,
    encode_robot_status,
    encode_wheel_command,
    decode_wheel_command,
    extract_packets,
)


class PacketTests(unittest.TestCase):
    def test_packet_round_trip(self) -> None:
        original = Packet(MessageId.HEARTBEAT, 7, b"\x01\x02\x03")
        self.assertEqual(decode_packet(encode_packet(original)), original)

    def test_crc_corruption_is_rejected(self) -> None:
        frame = bytearray(encode_packet(Packet(MessageId.HEARTBEAT, 1, b"ok")))
        frame[-1] ^= 0xFF
        with self.assertRaisesRegex(PacketError, "CRC"):
            decode_packet(bytes(frame))

    def test_drive_command_round_trip(self) -> None:
        original = DriveCommand(
            command_id=42,
            linear_velocity_mm_s=350,
            angular_velocity_mrad_s=-120,
            speed_limit_mm_s=500,
            control_flags=1,
        )
        packet = decode_packet(encode_drive_command(original, sequence=9))
        self.assertEqual(packet.message_id, MessageId.DRIVE_COMMAND)
        self.assertEqual(decode_drive_command(packet), original)

    def test_robot_status_round_trip(self) -> None:
        original = RobotStatus(
            system_state=SystemState.SLOW,
            safety_flags=SafetyFlag.UNDERVOLTAGE,
            left_velocity_mm_s=180,
            right_velocity_mm_s=175,
            battery_voltage_mv=23800,
            motor_error=0,
            last_command_id=91,
            rx_error_count=2,
            uptime_ms=120_000,
            ultrasonic_front_mm=620,
            sharp_left_mm=210,
            sharp_right_mm=230,
            push_switch_pressed=1,
            requested_base_rpm=100,
            left_velocity_rpm=98,
            right_velocity_rpm=101,
        )
        packet = decode_packet(encode_robot_status(original, sequence=10))
        self.assertEqual(decode_robot_status(packet), original)

    def test_wheel_command_round_trip(self) -> None:
        original = WheelCommand(51, 102, 100, 1, 0)
        packet = decode_packet(encode_wheel_command(original, sequence=11))
        self.assertEqual(packet.message_id, MessageId.WHEEL_COMMAND)
        self.assertEqual(decode_wheel_command(packet), original)

    def test_stream_parser_recovers_after_noise_and_bad_frame(self) -> None:
        bad = bytearray(encode_packet(Packet(MessageId.HEARTBEAT, 1, b"bad")))
        bad[-1] ^= 1
        good = encode_packet(Packet(MessageId.HEARTBEAT, 2, b"good"))
        packets, remainder = extract_packets(bytearray(b"noise") + bad + good)
        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].sequence, 2)
        self.assertEqual(remainder, bytearray())

    def test_incomplete_frame_is_retained(self) -> None:
        frame = encode_packet(Packet(MessageId.HEARTBEAT, 3, b"partial"))
        packets, remainder = extract_packets(bytearray(frame[:-2]))
        self.assertEqual(packets, [])
        self.assertEqual(remainder, bytearray(frame[:-2]))


if __name__ == "__main__":
    unittest.main()
