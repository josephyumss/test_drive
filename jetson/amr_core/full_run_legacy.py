"""Use the already-deployed $CMD/$STATUS firmware, without $FULL/$CTRL.

Button events are INFERRED from changes in the firmware's 0..65 base RPM.
Selected speed is local to this Jetson process and starts at zero. No button
hold duration, MCU session/uptime, fault cause or cumulative encoder exists in
legacy STATUS; never pretend that those quantities came from the MCU.
"""
from dataclasses import asdict

from .ascii_serial_bridge import decode_ascii_status
from .full_run import ControlStatus


class LegacyControlAdapter:
    def __init__(self, session, maximum_rpm=20, emit=None):
        self.session = session  # Local identifier only, NOT acknowledged by MCU.
        self.maximum_rpm = maximum_rpm
        self.emit = emit or (lambda *_args, **_kwargs: None)
        self.raw = None
        self.selected_rpm = 0
        self.up_count = self.down_count = self.stop_count = 0
        self.software_stop = False
        self.await_zero_command = False
        self.first_stamp = None
        self.previous_emergency = False
        self.last_limit_warning = None

    def request_instant_stop(self, reason="terminal"):
        if not self.software_stop:
            self.stop_count += 1
        self.software_stop = True
        self.await_zero_command = True
        self.emit("legacy_instant_stop", reason=reason, saved_rpm=self.selected_rpm,
                  stop_count=self.stop_count)

    def decode(self, line, now, *, allow_controls=True):
        raw = decode_ascii_status(line)
        if (not 0 <= raw.base_rpm <= 65 or raw.base_rpm % 5
                or not 0 <= raw.sharp_adc <= 65535
                or any(not 0 <= value <= 65 for value in (raw.left_target_rpm, raw.right_target_rpm))
                or max(abs(raw.left_rpm), abs(raw.right_rpm)) > 1000
                or max(raw.left_pwm, raw.right_pwm) > 65535):
            raise ValueError("Invalid legacy STATUS ranges or non-5-RPM base step")
        previous = self.raw
        self.raw = raw
        if self.first_stamp is None:
            self.first_stamp = now
            self.emit("legacy_initial_speed_ignored", raw_base_rpm=raw.base_rpm,
                      selected_rpm=0, reason="fresh Jetson run waits for a new observable UP")
        delta = raw.base_rpm - previous.base_rpm if previous else 0
        stop_barrier = self.await_zero_command
        if stop_barrier and raw.left_target_rpm == 0 and raw.right_target_rpm == 0:
            self.await_zero_command = False
            self.emit("legacy_stop_acknowledged", raw_base_rpm=raw.base_rpm,
                      instruction="Zero wheel targets observed. A later UP can resume.")
        if delta > 0:
            steps = delta // 5
            self.up_count += steps
            if not allow_controls:
                self.emit("legacy_startup_button_consumed", direction="UP", base_delta=delta)
            elif stop_barrier:
                # The first zero-target acknowledgement sets a new baseline.
                # An UP queued before the stop must not undo that same stop.
                self.emit("legacy_stop_UP_consumed", base_delta=delta,
                          reason="waiting for/establishing zero-command acknowledgement")
            elif self.software_stop:
                if not raw.emergency:
                    self.software_stop = False
                    if self.selected_rpm == 0:
                        self.selected_rpm = 5
                # The first UP resumes saved speed, rather than increasing it.
            else:
                self.selected_rpm = min(self.maximum_rpm, self.selected_rpm + steps * 5)
        elif delta < 0:
            steps = -delta // 5
            self.down_count += steps
            if allow_controls:
                self.selected_rpm = max(0, self.selected_rpm - steps * 5)
            else:
                self.emit("legacy_startup_button_consumed", direction="DOWN", base_delta=delta)
        if raw.emergency and not self.previous_emergency:
            if allow_controls:
                self.request_instant_stop("observed_legacy_ESTOP_cause_not_reported")
            else:
                # Previous shutdown sends E=1. Zero E=0 during startup clears
                # it; do not latch that old flag and deadlock the READY check.
                self.emit("legacy_startup_ESTOP", action="wait for zero-command STATUS to clear it")
        self.previous_emergency = raw.emergency
        if raw.base_rpm == 65 and (self.last_limit_warning is None or now - self.last_limit_warning >= 5):
            self.last_limit_warning = now
            self.emit("legacy_button_limit", raw_base_rpm=65,
                      instruction="UP at firmware limit is invisible. Press DOWN once, then UP to restart.")
        self.emit("legacy_status_normalized", raw=asdict(raw), selected_rpm=self.selected_rpm,
                  base_delta=delta, inferred_up_count=self.up_count, inferred_down_count=self.down_count,
                  software_stop=self.software_stop, odometry="reported_RPM_integral",
                  await_zero_command=self.await_zero_command,
                  clock="Jetson_receive_time", session="local_only", fault_cause="not_reported")
        # Old main.c stops calculating RPM when ESTOP=1; the printed RPM can be
        # stale. Suppress that value, not the fresh RPM during normal zero CMD.
        left, right = (0, 0) if raw.emergency else (raw.left_rpm, raw.right_rpm)
        return ControlStatus(self.session, self.selected_rpm, self.up_count, self.down_count,
                             self.stop_count, 2 if self.software_stop or raw.emergency else 0,
                             0, left, right, int((now - self.first_stamp) * 1000) & 0xffffffff)

    def snapshot(self):
        return {"raw": asdict(self.raw) if self.raw else None, "selected_rpm": self.selected_rpm,
                "software_stop": self.software_stop, "button_events": "inferred_from_base_RPM",
                "await_zero_command": self.await_zero_command,
                "clock": "Jetson_receive_time", "session": "local_only",
                "odometry": "reported_RPM_integral", "MCU_fault_cause": "unavailable"}
