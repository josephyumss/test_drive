"""Use the already-deployed $CMD/$STATUS firmware, without $FULL/$CTRL.

Button events are INFERRED from changes in the firmware's byte-valued base RPM.
Selected speed is local to this Jetson process and starts at zero. No button
hold duration, MCU session/uptime, fault cause or cumulative encoder exists in
legacy STATUS; never pretend that those quantities came from the MCU.
"""
from dataclasses import asdict, dataclass

from .ascii_serial_bridge import decode_ascii_status
from .full_run import ControlStatus


# The deployed compact firmware has been observed reporting 80. This value is
# only a button-event counter/baseline on the Jetson: actual outgoing motor
# commands remain capped independently by FullRunConfig.maximum_rpm.
LEGACY_REPORTED_RPM_LIMIT = 255


class UnsupportedLegacyStatus(ValueError):
    def __init__(self, received_fields):
        self.received_fields = received_fields
        super().__init__(f"Unsupported legacy STATUS layout: received {received_fields} fields, "
                         "expected 7 or 11 including $STATUS. Need the deployed firmware's field definition; "
                         "do not substitute missing RPM/ESTOP/target feedback with zero.")


class LegacyStatusLayoutChanged(ValueError):
    pass


@dataclass(frozen=True)
class LegacyStatus:
    field_count: int
    base_rpm: int
    left_rpm: int
    right_rpm: int
    sharp_distance_cm: int
    # None means absent from the deployed message, NEVER a fabricated zero.
    sharp_adc: int | None = None
    left_target_rpm: int | None = None
    right_target_rpm: int | None = None
    left_pwm: int | None = None
    right_pwm: int | None = None
    emergency: bool | None = None
    left_us_cm: int | None = None
    right_us_cm: int | None = None


def validate_legacy_status(line):
    """Validate without changing button state (also used during RX discovery)."""
    fields = line.strip().split(",")
    if fields[0] == "$STATUS" and len(fields) not in (7, 11):
        raise UnsupportedLegacyStatus(len(fields))
    if fields[0] == "$STATUS" and len(fields) == 7:
        # Confirmed by the user's actual STM32 snprintf (2026-10-10).
        # BASE, LEFT_RPM, RIGHT_RPM, LEFT_US_CM, RIGHT_US_CM, SHARP_CM.
        base, left, right, left_us, right_us, sharp = (int(v) for v in fields[1:])
        if (not 0 <= base <= LEGACY_REPORTED_RPM_LIMIT or base % 5
                or max(abs(left), abs(right)) > 1000
                or any(not -1 <= value <= 65535 for value in (left_us, right_us, sharp))):
            raise ValueError("Invalid compact STATUS RPM/range fields")
        return LegacyStatus(7, base, left, right, sharp, left_us_cm=left_us, right_us_cm=right_us)
    raw = decode_ascii_status(line)
    if (not 0 <= raw.base_rpm <= LEGACY_REPORTED_RPM_LIMIT or raw.base_rpm % 5
            or not 0 <= raw.sharp_adc <= 65535
            or any(not 0 <= value <= LEGACY_REPORTED_RPM_LIMIT
                   for value in (raw.left_target_rpm, raw.right_target_rpm))
            or max(abs(raw.left_rpm), abs(raw.right_rpm)) > 1000
            or max(raw.left_pwm, raw.right_pwm) > 65535):
        raise ValueError("Invalid legacy STATUS ranges or non-5-RPM base step")
    return LegacyStatus(field_count=11, **asdict(raw))


class LegacyControlAdapter:
    def __init__(self, session, maximum_rpm=25, emit=None):
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
        self.zero_command_written_at = None
        self.zero_rpm_samples = 0
        self.last_zero_sample = None
        self.last_written_command = None

    def request_instant_stop(self, reason="terminal"):
        if not self.software_stop:
            self.stop_count += 1
        self.software_stop = True
        self.await_zero_command = True
        self.zero_command_written_at = None
        self.zero_rpm_samples = 0
        self.last_zero_sample = None
        self.emit("legacy_instant_stop", reason=reason, saved_rpm=self.selected_rpm,
                  stop_count=self.stop_count)

    def note_command_written(self, left, right, emergency, now):
        """Called only AFTER a complete host UART write; not an MCU ACK."""
        self.last_written_command = {"left_rpm": left, "right_rpm": right,
                                     "emergency": emergency, "host_written_at": now}
        if not self.await_zero_command:
            return
        if left or right:
            self.zero_command_written_at = None
            self.zero_rpm_samples = 0
            self.last_zero_sample = None
        elif self.zero_command_written_at is None:
            self.zero_command_written_at = now
            self.emit("legacy_stop_zero_sent", host_written_at=now,
                      acknowledgement="no_sequence_numbered_MCU_command_ACK_in_legacy_STATUS")

    def decode(self, line, now, *, allow_controls=True):
        raw = validate_legacy_status(line)
        previous = self.raw
        if previous is not None and previous.field_count != raw.field_count:
            raise LegacyStatusLayoutChanged("STATUS layout changed within the same run; restart required")
        self.raw = raw
        if self.first_stamp is None:
            self.first_stamp = now
            self.emit("legacy_status_format", field_count=raw.field_count,
                      columns=(["$STATUS", "base_rpm", "left_rpm", "right_rpm", "left_us_cm", "right_us_cm", "sharp_cm"]
                               if raw.field_count == 7 else ["$STATUS", "base_rpm", "ADC", "sharp_cm", "left_target_rpm",
                                                            "right_target_rpm", "left_rpm", "right_rpm", "left_PWM", "right_PWM", "ESTOP"]),
                      RPM_feedback="reported_left_and_right_RPM", MCU_ESTOP_reported=raw.emergency is not None,
                      wheel_targets_reported=raw.left_target_rpm is not None,
                      stop_confirmation=("zero_target_fields" if raw.field_count == 11
                                         else "3_time_separated_zero_RPM_samples_after_host_zero_write_NOT_MCU_ACK"),
                      MCU_side_cm_reported=raw.field_count == 7)
            self.emit("legacy_initial_speed_ignored", raw_base_rpm=raw.base_rpm,
                      selected_rpm=0, reason="fresh Jetson run waits for a new observable UP")
        delta = raw.base_rpm - previous.base_rpm if previous else 0
        stop_barrier = self.await_zero_command
        stop_confirmed = False
        if stop_barrier and raw.field_count == 11:
            stop_confirmed = raw.left_target_rpm == 0 and raw.right_target_rpm == 0
        elif stop_barrier and self.zero_command_written_at is not None and now > self.zero_command_written_at:
            if raw.left_rpm == 0 and raw.right_rpm == 0:
                # Buffered duplicate frames in one read cannot count as three
                # independent standstill observations. No zero targets/ESTOP
                # ACK exists in this deployed message.
                if self.last_zero_sample is None or now - self.last_zero_sample >= 0.05:
                    self.zero_rpm_samples += 1
                    self.last_zero_sample = now
                stop_confirmed = self.zero_rpm_samples >= 3
            else:
                self.zero_rpm_samples = 0
                self.last_zero_sample = None
        if stop_barrier and stop_confirmed:
            self.await_zero_command = False
            event = "legacy_stop_acknowledged" if raw.field_count == 11 else "legacy_stop_standstill_observed"
            self.emit(event, raw_base_rpm=raw.base_rpm, field_count=raw.field_count,
                      zero_RPM_samples=self.zero_rpm_samples,
                      instruction="Stop barrier established. A later UP can resume.",
                      wheel_target_feedback=raw.field_count == 11, MCU_command_ACK=False)
        if delta > 0:
            steps = delta // 5
            self.up_count += steps
            if not allow_controls:
                self.emit("legacy_startup_button_consumed", direction="UP", base_delta=delta)
            elif stop_barrier:
                # The confirming zero-target/standstill sample sets a baseline.
                # An UP queued before the stop must not undo that same stop.
                self.emit("legacy_stop_UP_consumed", base_delta=delta,
                          reason="waiting for/establishing zero-target_or_standstill_barrier")
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
        if raw.emergency is True and not self.previous_emergency:
            if allow_controls:
                self.request_instant_stop("observed_legacy_ESTOP_cause_not_reported")
            else:
                # Previous shutdown sends E=1. Zero E=0 during startup clears
                # it; do not latch that old flag and deadlock the READY check.
                self.emit("legacy_startup_ESTOP", action="wait for zero-command STATUS to clear it")
        self.previous_emergency = bool(raw.emergency)
        if raw.base_rpm >= 80 and (self.last_limit_warning is None or now - self.last_limit_warning >= 5):
            self.last_limit_warning = now
            self.emit("legacy_button_limit", raw_base_rpm=raw.base_rpm,
                      instruction="If UP produces no STATUS change, press DOWN once, then UP to restart.")
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
                "field_count": self.raw.field_count if self.raw else None,
                "MCU_ESTOP": self.raw.emergency if self.raw else None,
                "wheel_target_feedback_available": bool(self.raw and self.raw.field_count == 11),
                "MCU_command_ACK_available": False,
                "zero_RPM_samples": self.zero_rpm_samples,
                "zero_command_host_written_at": self.zero_command_written_at,
                "last_host_command": self.last_written_command,
                "clock": "Jetson_receive_time", "session": "local_only",
                "odometry": "reported_RPM_integral", "MCU_fault_cause": "unavailable"}
