"""Validation and selection helpers for external YOLO detections."""

from __future__ import annotations

import json
import math


def decode_detection_packet(raw: bytes) -> dict:
    packet = json.loads(raw.decode("utf-8"))
    if packet.get("version") != 1:
        raise ValueError("unsupported detection packet version")
    width = int(packet["width"])
    height = int(packet["height"])
    detections = packet["detections"]
    if width <= 0 or height <= 0 or not isinstance(detections, list):
        raise ValueError("invalid detection packet")
    return packet


def select_centered_person(
    packet: dict,
    *,
    minimum_confidence: float = 0.45,
    center_min_ratio: float = 0.20,
    center_max_ratio: float = 0.80,
    minimum_height_ratio: float = 0.12,
) -> dict | None:
    width = float(packet["width"])
    height = float(packet["height"])
    candidates = []
    for detection in packet["detections"]:
        if detection.get("class") != "person":
            continue
        confidence = float(detection.get("confidence", 0.0))
        coordinates = detection.get("xyxy", [])
        if confidence < minimum_confidence or len(coordinates) != 4:
            continue
        x1, y1, x2, y2 = (float(value) for value in coordinates)
        if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
            continue
        center_ratio = ((x1 + x2) * 0.5) / width
        height_ratio = max(0.0, y2 - y1) / height
        if not center_min_ratio <= center_ratio <= center_max_ratio:
            continue
        if height_ratio < minimum_height_ratio:
            continue
        candidates.append((confidence, height_ratio, detection))
    return max(candidates, default=(0.0, 0.0, None))[-1]
