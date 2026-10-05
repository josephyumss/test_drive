#!/usr/bin/env python3
"""Bench test for YOLO + STL-27L + SHARP + dual HC-SR04 avoidance."""

from __future__ import annotations

import json
import math
import time

import Jetson.GPIO as GPIO
import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


PORT = "/dev/ttyTHS1"
BAUDRATE = 115200

LEFT_TRIG, LEFT_ECHO = 33, 31
RIGHT_TRIG, RIGHT_ECHO = 7, 15

LEFT_RPM, RIGHT_RPM = 20, 20
TURN_INNER_RPM, TURN_OUTER_RPM = 3, 22

SHARP_MIN_CM, SHARP_MAX_CM = 10, 80
SHARP_STOP_CM, SHARP_CLEAR_CM, SHARP_EMERGENCY_CM = 55, 70, 20

SIDE_CLEAR_CM, SIDE_STOP_CM = 35, 15
ULTRASONIC_MAX_CM = 300
ULTRASONIC_INTERVAL_S = 0.06
ULTRASONIC_ECHO_TIMEOUT_S = 0.03

LIDAR_FRONT_STOP_M = 1.2
LIDAR_FRONT_CLEAR_M = 1.6
LIDAR_SIDE_CLEAR_M = 0.8
LIDAR_SIDE_STOP_M = 0.35
LIDAR_FRONT_HALF_DEG = 15.0
LIDAR_SIDE_MIN_DEG = 20.0
LIDAR_SIDE_MAX_DEG = 75.0

IMAGE_WIDTH = 640.0
MIN_CONFIDENCE = 0.45
SENSOR_TIMEOUT_S = 1.0
REQUIRED_CLOSE_SAMPLES = 2
REQUIRED_CLEAR_SAMPLES = 5
STOP_BEFORE_TURN_S = 0.3
TURN_DURATION_S = 1.3
AVOIDANCE_COOLDOWN_S = 0.8
SOUND_SPEED_CM_S = 34300.0


class IntegratedAvoidance(Node):
    DRIVE = "직진"
    STOPPING = "회전 전 정지"
    WAIT_CLEAR = "공간 대기"
    TURN_LEFT = "좌회전"
    TURN_RIGHT = "우회전"

    def __init__(self) -> None:
        super().__init__("integrated_avoidance_test")
        self.serial = serial.Serial(PORT, BAUDRATE, timeout=0.01)
        self.serial.reset_input_buffer()
        self.rx_buffer = bytearray()

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LEFT_TRIG, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(LEFT_ECHO, GPIO.IN)
        GPIO.setup(RIGHT_TRIG, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(RIGHT_ECHO, GPIO.IN)
        time.sleep(0.1)

        self.stamps: dict[str, float | None] = {
            "camera": None,
            "stm32": None,
            "lidar": None,
            "left_us": None,
            "right_us": None,
        }
        self.detections: list[dict] = []
        self.labels: list[str] = []
        self.sharp_cm: int | None = None
        self.sharp_adc: int | None = None
        self.left_us_cm: float | None = None
        self.right_us_cm: float | None = None
        self.lidar_front_m: float | None = None
        self.lidar_left_m: float | None = None
        self.lidar_right_m: float | None = None

        self.state = self.DRIVE
        self.state_started_s = time.monotonic()
        self.cooldown_until_s = 0.0
        self.close_count = 0
        self.clear_count = 0
        self.last_center_turn = self.TURN_LEFT
        self.next_ultrasonic_s = time.monotonic()
        self.measure_left_next = True
        self.last_report: tuple | None = None

        self.create_subscription(String, "/yolo/detections", self.on_yolo, 10)
        self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )

    def on_yolo(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, list):
                raise ValueError
            self.detections = [
                item
                for item in payload
                if isinstance(item, dict)
                and float(item.get("confidence", 0.0)) >= MIN_CONFIDENCE
                and isinstance(item.get("xyxy"), list)
                and len(item["xyxy"]) == 4
            ]
            self.labels = [str(item.get("class", "object")) for item in self.detections]
            self.stamps["camera"] = time.monotonic()
        except (json.JSONDecodeError, TypeError, ValueError):
            self.detections = []
            self.labels = []
            self.stamps["camera"] = None

    @staticmethod
    def sector_minimum(message: LaserScan, low_deg: float, high_deg: float) -> float | None:
        low, high = math.radians(low_deg), math.radians(high_deg)
        values = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            if (
                low <= angle <= high
                and math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            ):
                values.append(float(distance))
        if values:
            return min(values)
        # LaserScan commonly uses +inf when no return exists in an open sector.
        # Treat that as clear up to the driver's declared maximum range.
        return float(message.range_max) if math.isfinite(message.range_max) else None

    def on_scan(self, message: LaserScan) -> None:
        self.lidar_front_m = self.sector_minimum(
            message, -LIDAR_FRONT_HALF_DEG, LIDAR_FRONT_HALF_DEG
        )
        self.lidar_left_m = self.sector_minimum(
            message, LIDAR_SIDE_MIN_DEG, LIDAR_SIDE_MAX_DEG
        )
        self.lidar_right_m = self.sector_minimum(
            message, -LIDAR_SIDE_MAX_DEG, -LIDAR_SIDE_MIN_DEG
        )
        if all(
            value is not None
            for value in (self.lidar_front_m, self.lidar_left_m, self.lidar_right_m)
        ):
            self.stamps["lidar"] = time.monotonic()

    def read_stm32(self) -> None:
        waiting = self.serial.in_waiting
        if waiting:
            self.rx_buffer.extend(self.serial.read(waiting))
        while b"\n" in self.rx_buffer:
            raw, _, remainder = self.rx_buffer.partition(b"\n")
            self.rx_buffer = bytearray(remainder)
            fields = raw.rstrip(b"\r").decode("ascii", errors="replace").split(",")
            if len(fields) < 4 or fields[0] != "$STATUS":
                continue
            try:
                adc, distance = int(fields[2]), int(fields[3])
            except ValueError:
                continue
            if SHARP_MIN_CM <= distance <= SHARP_MAX_CM:
                self.sharp_adc = adc
                self.sharp_cm = distance
                self.stamps["stm32"] = time.monotonic()

    @staticmethod
    def measure_ultrasonic(trigger: int, echo: int) -> float | None:
        GPIO.output(trigger, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(trigger, GPIO.HIGH)
        time.sleep(0.000010)
        GPIO.output(trigger, GPIO.LOW)
        started = time.monotonic()
        while GPIO.input(echo) == GPIO.LOW:
            if time.monotonic() - started > ULTRASONIC_ECHO_TIMEOUT_S:
                return None
        pulse_started = time.monotonic()
        while GPIO.input(echo) == GPIO.HIGH:
            if time.monotonic() - pulse_started > ULTRASONIC_ECHO_TIMEOUT_S:
                return None
        distance = (time.monotonic() - pulse_started) * SOUND_SPEED_CM_S / 2.0
        return distance if 2.0 <= distance <= ULTRASONIC_MAX_CM else None

    def update_ultrasonic(self) -> None:
        now = time.monotonic()
        if now < self.next_ultrasonic_s:
            return
        if self.measure_left_next:
            self.left_us_cm = self.measure_ultrasonic(LEFT_TRIG, LEFT_ECHO)
            self.stamps["left_us"] = now if self.left_us_cm is not None else None
        else:
            self.right_us_cm = self.measure_ultrasonic(RIGHT_TRIG, RIGHT_ECHO)
            self.stamps["right_us"] = now if self.right_us_cm is not None else None
        self.measure_left_next = not self.measure_left_next
        self.next_ultrasonic_s = time.monotonic() + ULTRASONIC_INTERVAL_S

    def fresh(self, name: str, now: float) -> bool:
        stamp = self.stamps[name]
        return stamp is not None and now - stamp <= SENSOR_TIMEOUT_S

    def all_sensors_fresh(self, now: float) -> bool:
        return all(self.fresh(name, now) for name in self.stamps)

    def camera_preference(self) -> str | None:
        if not self.detections:
            return None
        target = max(
            self.detections,
            key=lambda item: max(0.0, item["xyxy"][2] - item["xyxy"][0])
            * max(0.0, item["xyxy"][3] - item["xyxy"][1]),
        )
        center = (float(target["xyxy"][0]) + float(target["xyxy"][2])) / 2.0
        if center < IMAGE_WIDTH * 0.45:
            return self.TURN_RIGHT
        if center > IMAGE_WIDTH * 0.55:
            return self.TURN_LEFT
        selected = (
            self.TURN_RIGHT if self.last_center_turn == self.TURN_LEFT else self.TURN_LEFT
        )
        self.last_center_turn = selected
        return selected

    def left_clear(self) -> bool:
        return bool(
            self.left_us_cm is not None
            and self.left_us_cm >= SIDE_CLEAR_CM
            and self.lidar_left_m is not None
            and self.lidar_left_m >= LIDAR_SIDE_CLEAR_M
        )

    def right_clear(self) -> bool:
        return bool(
            self.right_us_cm is not None
            and self.right_us_cm >= SIDE_CLEAR_CM
            and self.lidar_right_m is not None
            and self.lidar_right_m >= LIDAR_SIDE_CLEAR_M
        )

    def choose_turn(self) -> str | None:
        preference = self.camera_preference()
        left, right = self.left_clear(), self.right_clear()
        if preference == self.TURN_LEFT and left:
            return self.TURN_LEFT
        if preference == self.TURN_RIGHT and right:
            return self.TURN_RIGHT
        if left and not right:
            return self.TURN_LEFT
        if right and not left:
            return self.TURN_RIGHT
        if left and right:
            return preference or (
                self.TURN_LEFT
                if float(self.lidar_left_m) >= float(self.lidar_right_m)
                else self.TURN_RIGHT
            )
        return None

    def transition(self, state: str, now: float) -> None:
        self.state = state
        self.state_started_s = now
        self.clear_count = 0
        print(f"\n상태 전환: {state}")

    def update_state(self, now: float) -> None:
        lidar_close = (
            self.lidar_front_m is not None and self.lidar_front_m <= LIDAR_FRONT_STOP_M
        )
        sharp_close = self.sharp_cm is not None and self.sharp_cm <= SHARP_STOP_CM
        close = lidar_close or (bool(self.detections) and sharp_close)
        clear = bool(
            self.lidar_front_m is not None
            and self.lidar_front_m >= LIDAR_FRONT_CLEAR_M
            and (self.sharp_cm is None or self.sharp_cm >= SHARP_CLEAR_CM)
        )

        if self.state == self.DRIVE:
            if now < self.cooldown_until_s:
                self.close_count = 0
            elif close:
                self.close_count += 1
                if self.close_count >= REQUIRED_CLOSE_SAMPLES:
                    self.close_count = 0
                    self.transition(self.STOPPING, now)
            else:
                self.close_count = 0
        elif self.state == self.STOPPING:
            if clear:
                self.clear_count += 1
                if self.clear_count >= REQUIRED_CLEAR_SAMPLES:
                    self.cooldown_until_s = now + AVOIDANCE_COOLDOWN_S
                    self.transition(self.DRIVE, now)
            elif now - self.state_started_s >= STOP_BEFORE_TURN_S:
                self.transition(self.choose_turn() or self.WAIT_CLEAR, now)
        elif self.state == self.WAIT_CLEAR:
            turn = self.choose_turn()
            if clear:
                self.clear_count += 1
                if self.clear_count >= REQUIRED_CLEAR_SAMPLES:
                    self.transition(self.DRIVE, now)
            elif turn is not None:
                self.transition(turn, now)
        elif self.state in (self.TURN_LEFT, self.TURN_RIGHT):
            side_blocked = (
                self.state == self.TURN_LEFT
                and (
                    self.left_us_cm is None
                    or self.left_us_cm <= SIDE_STOP_CM
                    or self.lidar_left_m is None
                    or self.lidar_left_m <= LIDAR_SIDE_STOP_M
                )
            ) or (
                self.state == self.TURN_RIGHT
                and (
                    self.right_us_cm is None
                    or self.right_us_cm <= SIDE_STOP_CM
                    or self.lidar_right_m is None
                    or self.lidar_right_m <= LIDAR_SIDE_STOP_M
                )
            )
            if side_blocked:
                self.transition(self.WAIT_CLEAR, now)
            elif now - self.state_started_s >= TURN_DURATION_S:
                self.cooldown_until_s = now + AVOIDANCE_COOLDOWN_S
                self.transition(self.DRIVE, now)

    def motor_command(self, now: float) -> tuple[int, int]:
        if not self.all_sensors_fresh(now):
            return 0, 0
        if self.sharp_cm is not None and self.sharp_cm <= SHARP_EMERGENCY_CM:
            return 0, 0
        if self.lidar_front_m is not None and self.lidar_front_m <= 0.25:
            return 0, 0
        if self.state == self.DRIVE:
            return LEFT_RPM, RIGHT_RPM
        if self.state == self.TURN_LEFT:
            return TURN_INNER_RPM, TURN_OUTER_RPM
        if self.state == self.TURN_RIGHT:
            return TURN_OUTER_RPM, TURN_INNER_RPM
        return 0, 0

    @staticmethod
    def fmt(value: float | int | None, precision: int = 1) -> str:
        return "None" if value is None else f"{value:.{precision}f}"

    def tick(self) -> None:
        now = time.monotonic()
        self.read_stm32()
        self.update_ultrasonic()
        self.update_state(now)
        left, right = self.motor_command(now)
        self.serial.write(f"$CMD,{left},{right},0\r\n".encode("ascii"))
        self.serial.flush()
        report = (
            self.state,
            left,
            right,
            self.sharp_cm,
            round(self.lidar_front_m or -1.0, 2),
            round(self.lidar_left_m or -1.0, 2),
            round(self.lidar_right_m or -1.0, 2),
            round(self.left_us_cm or -1.0, 1),
            round(self.right_us_cm or -1.0, 1),
            tuple(self.labels),
        )
        if report != self.last_report:
            print(
                f"\n{self.state} MOTOR=({left},{right}) SHARP={self.sharp_cm}cm "
                f"LIDAR(F/L/R)=({self.fmt(self.lidar_front_m, 2)}/"
                f"{self.fmt(self.lidar_left_m, 2)}/{self.fmt(self.lidar_right_m, 2)})m "
                f"US(L/R)=({self.fmt(self.left_us_cm)}/{self.fmt(self.right_us_cm)})cm "
                f"YOLO={','.join(self.labels) if self.labels else '없음'}"
            )
            self.last_report = report

    def shutdown(self) -> None:
        for _ in range(10):
            try:
                self.serial.write(b"$CMD,0,0,0\r\n")
                self.serial.flush()
            except Exception:
                pass
            time.sleep(0.05)
        self.serial.close()
        GPIO.output(LEFT_TRIG, GPIO.LOW)
        GPIO.output(RIGHT_TRIG, GPIO.LOW)
        GPIO.cleanup()


def main() -> None:
    rclpy.init()
    node = IntegratedAvoidance()
    print("YOLO + LiDAR + SHARP + 좌우 초음파 자동회피 시험 (종료: Ctrl+C)")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.tick()
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n시험 종료")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print("모터 및 GPIO 정리 완료")


if __name__ == "__main__":
    main()

