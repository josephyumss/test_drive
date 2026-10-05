"""Run OAK-D RGB + Ultralytics in the JetPack container and emit UDP JSON."""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import socket
import threading
import time

import cv2
import depthai as dai
from ultralytics import YOLO


class PreviewState:
    def __init__(self) -> None:
        self.jpeg: bytes | None = None
        self.lock = threading.Lock()


def start_preview_server(state: PreviewState, port: int) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path == "/":
                page = b"<html><body><h2>OAK-D S2 + YOLO</h2><img src='/stream.mjpg'></body></html>"
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
                return
            if self.path != "/stream.mjpg":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()
            try:
                while True:
                    with state.lock:
                        jpeg = state.jpeg
                    if jpeg is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                return

        def log_message(self, format: str, *args) -> None:
            return

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def build_pipeline(width: int, height: int) -> dai.Pipeline:
    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.ColorCamera)
    camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camera.setPreviewSize(width, height)
    camera.setInterleaved(False)
    camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camera.setFps(30)
    output = pipeline.create(dai.node.XLinkOut)
    output.setStreamName("rgb")
    camera.preview.link(output.input)
    return pipeline


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--rate", type=float, default=15.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=384)
    parser.add_argument("--web-port", type=int, default=8081)
    args = parser.parse_args()

    model = YOLO(args.model)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (args.host, args.port)
    minimum_period = 1.0 / max(1.0, args.rate)
    last_inference = 0.0
    preview = PreviewState()
    preview_server = start_preview_server(preview, args.web_port)

    with dai.Device(build_pipeline(args.width, args.height)) as device:
        queue = device.getOutputQueue("rgb", maxSize=1, blocking=False)
        print(f"OAK-D + YOLO started; UDP destination={args.host}:{args.port}")
        while True:
            packet = queue.get()
            now = time.monotonic()
            if now - last_inference < minimum_period:
                continue
            last_inference = now
            frame = packet.getCvFrame()
            result = model.predict(
                frame, conf=args.confidence, device="0", verbose=False
            )[0]
            detections = []
            if result.boxes is not None:
                for box in result.boxes:
                    class_id = int(box.cls[0])
                    detections.append(
                        {
                            "class": str(result.names[class_id]),
                            "confidence": round(float(box.conf[0]), 4),
                            "xyxy": [round(float(value), 1) for value in box.xyxy[0]],
                        }
                    )
            payload = {
                "version": 1,
                "timestamp": time.time(),
                "width": int(frame.shape[1]),
                "height": int(frame.shape[0]),
                "detections": detections,
            }
            udp.sendto(json.dumps(payload, ensure_ascii=False).encode("utf-8"), destination)
            annotated = result.plot()
            encoded, jpeg = cv2.imencode(".jpg", annotated)
            if encoded:
                with preview.lock:
                    preview.jpeg = jpeg.tobytes()


if __name__ == "__main__":
    main()
