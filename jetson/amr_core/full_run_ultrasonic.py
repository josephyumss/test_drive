"""One GPIO owner, edge-timed echoes; never block the motor heartbeat thread."""
import queue
import threading
import time

from .full_run import SideReading


class UltrasonicWorker:
    def __init__(self, gpio, config, emit):
        self.gpio, self.c, self.emit = gpio, config, emit
        self.results = queue.Queue(maxsize=64)
        self.stop_event = threading.Event()
        self.echo_done = threading.Event()
        self.edge_lock = threading.Lock()
        self.active_echo = self.rise_ns = self.fall_ns = None
        self.inside = None
        self.pins = {"left": (config.left_trig, config.left_echo),
                     "right": (config.right_trig, config.right_echo)}
        self.initialized = []
        self.callbacks = []
        self.thread = None
        try:
            gpio.setmode(gpio.BOARD)
            for trig, echo in self.pins.values():
                gpio.setup(trig, gpio.OUT, initial=gpio.LOW)
                self.initialized.append(trig)
                gpio.setup(echo, gpio.IN)
                self.initialized.append(echo)
                gpio.add_event_detect(echo, gpio.BOTH, callback=self.on_edge)
                self.callbacks.append(echo)
            self.thread = threading.Thread(target=self.run, name="side-ultrasonic", daemon=True)
            self.thread.start()
        except Exception:
            self.close()
            raise

    def on_edge(self, channel):
        stamp = time.monotonic_ns()
        with self.edge_lock:
            if channel != self.active_echo:
                return
            high = self.gpio.input(channel) == self.gpio.HIGH
            if high and self.rise_ns is None:
                self.rise_ns = stamp
            elif not high and self.rise_ns is not None:
                self.fall_ns = stamp
                self.echo_done.set()

    def measure(self, side):
        trig, echo = self.pins[side]
        stamp = time.monotonic()
        if self.gpio.input(echo) == self.gpio.HIGH:
            return SideReading(side, stamp, "FAULT", detail="echo_stuck_high_before_trigger")
        self.echo_done.clear()
        with self.edge_lock:
            self.active_echo, self.rise_ns, self.fall_ns = echo, None, None
        self.gpio.output(trig, self.gpio.HIGH)
        time.sleep(0.000010)
        self.gpio.output(trig, self.gpio.LOW)
        completed = self.echo_done.wait(self.c.ultrasonic_timeout_s)
        with self.edge_lock:
            rise, fall = self.rise_ns, self.fall_ns
            self.active_echo = None
        if not completed:
            if rise is not None or self.gpio.input(echo) == self.gpio.HIGH:
                return SideReading(side, stamp, "FAULT", detail="echo_did_not_fall_within_timeout")
            return SideReading(side, stamp, "NO_ECHO", detail="no_rising_edge_ambiguous_no_object_or_disconnected")
        pulse_s = (fall - rise) / 1e9
        distance = pulse_s * 343.0 / 2
        if distance < 0.02:
            return SideReading(side, stamp, "FAULT", distance, pulse_s, "pulse_below_2cm_or_glitch")
        if distance > self.c.ultrasonic_max_m:
            return SideReading(side, stamp, "NO_ECHO", distance, pulse_s, "echo_outside_configured_range")
        return SideReading(side, stamp, "VALID", distance, pulse_s)

    def publish(self, reading):
        self.emit("ultrasonic", reading=reading)
        try:
            self.results.put_nowait(reading)
        except queue.Full:
            self.results.get_nowait()
            self.results.put_nowait(reading)
            self.emit("ultrasonic_queue_overflow")

    def run(self):
        index = 0
        try:
            while not self.stop_event.is_set():
                # Three inside samples then one outside sample during passing.
                side = (self.inside if index % 4 != 3 else
                        ("left" if self.inside == "right" else "right")) if self.inside else ("left" if index % 2 == 0 else "right")
                started = time.monotonic()
                self.publish(self.measure(side))
                index += 1
                self.stop_event.wait(max(0, self.c.ultrasonic_interval_s - (time.monotonic() - started)))
        except Exception as exc:
            self.publish(SideReading(side, time.monotonic(), "FAULT", detail=repr(exc)))

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1)
        for echo in self.callbacks:
            self.gpio.remove_event_detect(echo)
        if self.initialized:
            self.gpio.cleanup(self.initialized)
        self.initialized, self.callbacks = [], []
