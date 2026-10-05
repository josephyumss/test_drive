import unittest

from jetson.amr_core.ascii_serial_bridge import (
    AsciiMcuStatus,
    AsciiSerialBridge,
    decode_ascii_status,
    encode_ascii_command,
)
from jetson.amr_core.transport import memory_transport_pair


class AsciiProtocolTests(unittest.TestCase):
    def test_command_matches_stm32_format(self) -> None:
        self.assertEqual(encode_ascii_command(3, 2, False), b"$CMD,3,2,0\r\n")
        self.assertEqual(encode_ascii_command(0, 0, True), b"$CMD,0,0,1\r\n")

    def test_status_matches_current_main_c_format(self) -> None:
        status = decode_ascii_status("$STATUS,3,1200,42,3,2,3,2,110,105,0\r\n")
        self.assertEqual(
            status,
            AsciiMcuStatus(3, 1200, 42, 3, 2, 3, 2, 110, 105, False),
        )

    def test_partial_lines_are_buffered(self) -> None:
        jetson, stm32 = memory_transport_pair()
        bridge = AsciiSerialBridge(jetson, status_timeout_s=0.35)
        stm32.write(b"$STATUS,2,1000,50,2")
        self.assertEqual(bridge.poll(now_s=1.0), [])
        stm32.write(b",2,2,2,80,81,0\r\n")
        statuses = bridge.poll(now_s=1.1)
        self.assertEqual(len(statuses), 1)
        self.assertFalse(bridge.diagnostics(now_s=1.4).status_timed_out)
        self.assertTrue(bridge.diagnostics(now_s=1.46).status_timed_out)

    def test_invalid_line_is_ignored(self) -> None:
        jetson, stm32 = memory_transport_pair()
        bridge = AsciiSerialBridge(jetson)
        stm32.write(b"debug text\r\n")
        self.assertEqual(bridge.poll(now_s=1.0), [])
        self.assertEqual(bridge.diagnostics(now_s=1.0).ignored_packets, 1)


if __name__ == "__main__":
    unittest.main()
