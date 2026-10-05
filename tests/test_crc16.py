import unittest

from jetson.amr_core.crc16 import crc16_ccitt_false


class Crc16Tests(unittest.TestCase):
    def test_standard_check_value(self) -> None:
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_empty_payload_uses_initial_value(self) -> None:
        self.assertEqual(crc16_ccitt_false(b""), 0xFFFF)


if __name__ == "__main__":
    unittest.main()
