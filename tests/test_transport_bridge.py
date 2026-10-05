import unittest

from protocol.protocol_constants import MessageId, SafetyFlag, SystemState
from jetson.amr_core.packet import DriveCommand, RobotStatus, WheelCommand, decode_packet, encode_robot_status
from jetson.amr_core.serial_bridge import SerialBridge
from jetson.amr_core.transport import memory_transport_pair


class MemoryTransportTests(unittest.TestCase):
    def test_pair_is_full_duplex(self) -> None:
        first, second = memory_transport_pair()
        first.write(b"one")
        second.write(b"two")
        self.assertEqual(second.read(), b"one")
        self.assertEqual(first.read(), b"two")


class SerialBridgeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.jetson, self.stm32 = memory_transport_pair()
        self.bridge = SerialBridge(self.jetson, status_timeout_s=0.3)

    def test_drive_command_is_transmitted(self) -> None:
        command = DriveCommand(1, 300, 0, 500, 1)
        sequence = self.bridge.send_drive_command(command)
        raw = self.stm32.read()
        self.assertTrue(raw)
        self.assertEqual(sequence, 0)

    def test_wheel_command_is_transmitted(self) -> None:
        command = WheelCommand(2, 102, 100, 1, 0)
        sequence = self.bridge.send_wheel_command(command)
        packet = decode_packet(self.stm32.read())
        self.assertEqual(sequence, 0)
        self.assertEqual(packet.message_id, MessageId.WHEEL_COMMAND)

    def test_status_is_received_and_timeout_is_reported(self) -> None:
        status = RobotStatus(
            SystemState.RUN,
            SafetyFlag.NONE,
            300,
            300,
            24000,
            0,
            5,
            0,
            1000,
        )
        self.stm32.write(encode_robot_status(status, sequence=7))
        received = self.bridge.poll(now_s=1.0)
        self.assertEqual(received, [status])
        self.assertFalse(self.bridge.diagnostics(now_s=1.2).status_timed_out)
        self.assertTrue(self.bridge.diagnostics(now_s=1.31).status_timed_out)


if __name__ == "__main__":
    unittest.main()
