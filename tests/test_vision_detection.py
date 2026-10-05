import json
import unittest

from jetson.amr_core.vision_detection import (
    decode_detection_packet,
    select_centered_person,
)


class VisionDetectionTests(unittest.TestCase):
    def packet(self, detections):
        return {
            "version": 1,
            "timestamp": 1.0,
            "width": 640,
            "height": 384,
            "detections": detections,
        }

    def test_packet_decode(self):
        packet = self.packet([])
        self.assertEqual(
            decode_detection_packet(json.dumps(packet).encode("utf-8")), packet
        )

    def test_centered_person_is_selected(self):
        packet = self.packet(
            [
                {"class": "chair", "confidence": 0.9, "xyxy": [200, 50, 400, 350]},
                {"class": "person", "confidence": 0.8, "xyxy": [220, 60, 420, 360]},
            ]
        )
        selected = select_centered_person(packet)
        self.assertIsNotNone(selected)
        self.assertEqual(selected["class"], "person")

    def test_edge_person_is_not_selected(self):
        packet = self.packet(
            [{"class": "person", "confidence": 0.9, "xyxy": [0, 50, 80, 350]}]
        )
        self.assertIsNone(select_centered_person(packet))

    def test_tiny_or_low_confidence_person_is_not_selected(self):
        tiny = self.packet(
            [{"class": "person", "confidence": 0.9, "xyxy": [300, 20, 340, 40]}]
        )
        weak = self.packet(
            [{"class": "person", "confidence": 0.2, "xyxy": [220, 20, 420, 360]}]
        )
        self.assertIsNone(select_centered_person(tiny))
        self.assertIsNone(select_centered_person(weak))


if __name__ == "__main__":
    unittest.main()
