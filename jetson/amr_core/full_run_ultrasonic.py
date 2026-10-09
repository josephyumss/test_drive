"""One GPIO owner, edge-timed echoes; never block the motor heartbeat thread."""
import queue
import threading
import time
import traceback

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
        self.stats = {side: {"triggers": 0, "edge_callbacks": 0, "rising_edges": 0, "falling_edges": 0,
                             "ignored_edges": 0, "VALID": 0, "NO_ECHO": 0, "FAULT": 0,
                             "last_trigger_high_s": None, "last_reading": None} for side in self.pins}
        self.edge_error = None
        setup_stage = "setmode_BOARD"
        try:
            gpio.setmode(gpio.BOARD)
            for side, (trig, echo) in self.pins.items():
                setup_stage = f"{side}_TRIG_setup_{trig}"
                gpio.setup(trig, gpio.OUT, initial=gpio.LOW)
                self.initialized.append(trig)
                setup_stage = f"{side}_ECHO_setup_{echo}"
                gpio.setup(echo, gpio.IN)
                self.initialized.append(echo)
                setup_stage = f"{side}_ECHO_edge_detection_{echo}"
                gpio.add_event_detect(echo, gpio.BOTH, callback=self.on_edge)
                self.callbacks.append(echo)
                emit("gpio_setup", side=side, trig_BOARD=trig, echo_BOARD=echo,
                     initial_echo_high=gpio.input(echo) == gpio.HIGH)
            self.thread = threading.Thread(target=self.run, name="side-ultrasonic", daemon=True)
            self.thread.start()
        except Exception:
            emit("ultrasonic_exception", stage=setup_stage, traceback=traceback.format_exc())
            try:
                self.close()
            except Exception:
                emit("ultrasonic_exception", stage="cleanup_after_setup_failure", traceback=traceback.format_exc())
            raise

    def on_edge(self, channel):
        stamp = time.monotonic_ns()
        try:
            with self.edge_lock:
                side = next((s for s, pins in self.pins.items() if pins[1] == channel), None)
                stats = self.stats.get(side)
                if stats is not None:
                    stats["edge_callbacks"] += 1
                if channel != self.active_echo:
                    if stats is not None:
                        stats["ignored_edges"] += 1
                    return
                high = self.gpio.input(channel) == self.gpio.HIGH
                if high and self.rise_ns is None:
                    self.rise_ns = stamp
                    if stats is not None:
                        stats["rising_edges"] += 1
                elif not high and self.rise_ns is not None:
                    self.fall_ns = stamp
                    if stats is not None:
                        stats["falling_edges"] += 1
                    self.echo_done.set()
        except Exception:
            self.edge_error = traceback.format_exc()
            self.echo_done.set()
            self.emit("ultrasonic_exception", stage="GPIO_edge_callback", channel=channel,
                      traceback=self.edge_error)

    def measure(self, side):
        trig, echo = self.pins[side]
        stamp = time.monotonic()
        if self.gpio.input(echo) == self.gpio.HIGH:
            return SideReading(side, stamp, "FAULT", detail="echo_stuck_high_before_trigger")
        self.echo_done.clear()
        with self.edge_lock:
            self.active_echo, self.rise_ns, self.fall_ns = echo, None, None
            self.edge_error = None
            self.stats[side]["triggers"] += 1
        high_started = time.monotonic()
        self.gpio.output(trig, self.gpio.HIGH)
        time.sleep(0.000010)
        self.gpio.output(trig, self.gpio.LOW)
        self.stats[side]["last_trigger_high_s"] = time.monotonic() - high_started
        completed = self.echo_done.wait(self.c.ultrasonic_timeout_s)
        with self.edge_lock:
            rise, fall = self.rise_ns, self.fall_ns
            self.active_echo = None
        if self.edge_error:
            return SideReading(side, stamp, "FAULT", detail="GPIO_edge_callback_exception")
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
        with self.edge_lock:
            stats = self.stats[reading.side]
            stats[reading.status] += 1
            stats["last_reading"] = {"status": reading.status, "distance_m": reading.distance_m,
                                     "pulse_s": reading.pulse_s, "detail": reading.detail, "stamp": reading.stamp}
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
            self.emit("ultrasonic_exception", stage="measurement_worker", side=side,
                      traceback=traceback.format_exc())
            self.publish(SideReading(side, time.monotonic(), "FAULT", detail=repr(exc)))

    def snapshot(self):
        with self.edge_lock:
            sides = {side: dict(values) for side, values in self.stats.items()}
        return {"pins_BOARD": self.pins, "thread_alive": bool(self.thread and self.thread.is_alive()),
                "inside_priority": self.inside, "queue_size": self.results.qsize(),
                "sides": sides, "ever_valid_echo_observed": {side: bool(v["VALID"]) for side, v in sides.items()},
                "NO_ECHO_is_ambiguous": True}

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1)
            if self.thread.is_alive():
                self.emit("gpio_cleanup", error="worker_join_timeout_no_GPIO_cleanup_while_thread_alive")
                raise RuntimeError("Ultrasonic worker did not stop before GPIO cleanup")
        errors = []
        for echo in self.callbacks:
            try:
                self.gpio.remove_event_detect(echo)
            except Exception:
                errors.append(f"remove_event_detect({echo})")
                self.emit("ultrasonic_exception", stage="remove_event_detect", pin=echo,
                          traceback=traceback.format_exc())
        if self.initialized:
            try:
                self.gpio.cleanup(self.initialized)
            except Exception:
                errors.append("GPIO_cleanup")
                self.emit("ultrasonic_exception", stage="GPIO_cleanup", pins=self.initialized,
                          traceback=traceback.format_exc())
        self.emit("gpio_cleanup", pins=self.initialized, callbacks=self.callbacks, errors=errors)
        self.initialized, self.callbacks = [], []
        if errors:
            raise RuntimeError("GPIO cleanup failures: " + ",".join(errors))
