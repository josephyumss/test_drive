from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from amr_interfaces.msg import ObstacleInfo, SafetyState


@dataclass(order=True)
class SpeechItem:
    sort_priority: int
    sequence: int
    key: str = field(compare=False)
    text: str = field(compare=False)


class SpeechWorker:
    def __init__(self, command: str, voice: str, speed_wpm: int, output_device: str) -> None:
        self.command = command
        self.voice = voice
        self.speed_wpm = speed_wpm
        self.output_device = output_device
        self._queue: list[SpeechItem] = []
        self._condition = threading.Condition()
        self._sequence = 0
        self._stopping = False
        self._process: subprocess.Popen | None = None
        self._synth_process: subprocess.Popen | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def enqueue(self, key: str, text: str, priority: int, interrupt: bool = False) -> None:
        with self._condition:
            self._sequence += 1
            heapq.heappush(self._queue, SpeechItem(-priority, self._sequence, key, text))
            if interrupt and self._process is not None and self._process.poll() is None:
                self._process.terminate()
                if self._synth_process is not None and self._synth_process.poll() is None:
                    self._synth_process.terminate()
            self._condition.notify()

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._queue and not self._stopping:
                    self._condition.wait()
                if self._stopping:
                    return
                item = heapq.heappop(self._queue)
            try:
                command = [self.command, "-v", self.voice, "-s", str(self.speed_wpm)]
                if self.output_device:
                    self._synth_process = subprocess.Popen(
                        command + ["--stdout", item.text],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.DEVNULL,
                    )
                    self._process = subprocess.Popen(
                        ["aplay", "-q", "-D", self.output_device],
                        stdin=self._synth_process.stdout,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    if self._synth_process.stdout is not None:
                        self._synth_process.stdout.close()
                else:
                    self._process = subprocess.Popen(
                        command + [item.text],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                self._process.wait()
                if self._synth_process is not None:
                    self._synth_process.wait()
            except OSError:
                # Audio failure must never block or alter vehicle safety control.
                time.sleep(0.1)
            finally:
                self._process = None
                self._synth_process = None

    def close(self) -> None:
        with self._condition:
            self._stopping = True
            if self._process is not None and self._process.poll() is None:
                self._process.terminate()
            if self._synth_process is not None and self._synth_process.poll() is None:
                self._synth_process.terminate()
            self._condition.notify_all()
        self._thread.join(timeout=1.0)


class AudioNode(Node):
    def __init__(self) -> None:
        super().__init__("audio_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("tts_command", "espeak-ng")
        self.declare_parameter("voice", "ko")
        self.declare_parameter("output_device", "plughw:CARD=UACDemoV10,DEV=0")
        self.declare_parameter("speed_wpm", 145)
        self.declare_parameter("repeat_interval_s", 4.0)
        self.declare_parameter("obstacle_announce_distance_m", 2.0)
        self.worker = SpeechWorker(
            str(self.get_parameter("tts_command").value),
            str(self.get_parameter("voice").value),
            int(self.get_parameter("speed_wpm").value),
            str(self.get_parameter("output_device").value),
        )
        self.last_announced: dict[str, float] = {}
        self.previous_state: int | None = None
        self.create_subscription(SafetyState, "/safety/state", self._on_safety, 10)
        self.create_subscription(ObstacleInfo, "/obstacle/info", self._on_obstacle, 10)

    def _announce(self, key: str, text: str, priority: int, force: bool = False) -> None:
        if not bool(self.get_parameter("enabled").value):
            return
        now = time.monotonic()
        interval = float(self.get_parameter("repeat_interval_s").value)
        if not force and now - self.last_announced.get(key, -interval) < interval:
            return
        self.last_announced[key] = now
        self.worker.enqueue(key, text, priority, interrupt=priority >= 90)

    def _on_safety(self, message: SafetyState) -> None:
        state = int(message.state)
        if state == self.previous_state:
            return
        self.previous_state = state
        phrases = {
            3: ("slow", "전방 위험으로 감속합니다", 50),
            4: ("stop", "전방 위험으로 정지합니다", 80),
            5: ("emergency", "위험합니다. 비상 정지했습니다", 100),
            6: ("fault", "장치 이상이 발생했습니다", 95),
        }
        if state in phrases:
            key, text, priority = phrases[state]
            self._announce(key, text, priority, force=True)

    def _on_obstacle(self, message: ObstacleInfo) -> None:
        limit = float(self.get_parameter("obstacle_announce_distance_m").value)
        if not message.valid or not message.detected or message.distance_m > limit:
            return
        label = message.object_class or "장애물"
        distance = max(0.0, float(message.distance_m))
        self._announce(
            "obstacle:" + label,
            f"전방 {distance:.1f} 미터에 {label}이 있습니다",
            30,
        )

    def destroy_node(self):
        self.worker.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
