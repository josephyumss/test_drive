"""Run an end-to-end safety scenario without robot hardware."""

from __future__ import annotations

from dataclasses import dataclass

from jetson.amr_core.controller import ControllerInputs, IntegratedController
from jetson.amr_core.serial_bridge import SerialBridge
from jetson.amr_core.transport import memory_transport_pair
from jetson.amr_core.velocity import VelocityCommand

from .fake_stm32 import FakeSTM32


@dataclass(frozen=True)
class ScenarioRecord:
    time_s: float
    phase: str
    jetson_state: str
    stm32_state: str
    safe_speed_mps: float
    actual_speed_mps: float
    safety_flags: int


def run_demo_scenario(dt_s: float = 0.05) -> list[ScenarioRecord]:
    jetson_link, stm32_link = memory_transport_pair()
    controller = IntegratedController()
    bridge = SerialBridge(jetson_link)
    stm32 = FakeSTM32(stm32_link)
    records: list[ScenarioRecord] = []
    now_s = 0.0

    phases = [
        ("normal", 1.0, 3.0, 0.2, False, False),
        ("slow", 1.0, 1.6, 0.45, False, False),
        ("obstacle_stop", 0.6, 1.0, 0.20, False, False),
        ("manual_reset", 0.2, 3.0, 0.0, True, False),
        ("restart", 0.6, 3.0, 0.2, False, False),
        ("cliff", 0.5, 3.0, 0.2, False, True),
    ]

    for phase, duration_s, distance_m, closing_speed, reset, cliff in phases:
        stm32.hazards.cliff_left = cliff
        steps = round(duration_s / dt_s)
        for index in range(steps):
            inputs = ControllerInputs(
                requested_velocity=VelocityCommand(0.45, 0.0),
                obstacle_distance_m=distance_m,
                obstacle_closing_speed_mps=closing_speed,
                drive_enable=phase not in {"manual_reset"},
                reset_requested=reset and index == 0,
                cliff_left=cliff,
            )
            output = controller.update(inputs)
            bridge.send_drive_command(output.drive_command)
            status = stm32.step(now_s, dt_s)
            bridge.poll(now_s=now_s)
            actual = (status.left_velocity_mm_s + status.right_velocity_mm_s) / 2000.0
            records.append(
                ScenarioRecord(
                    time_s=round(now_s, 3),
                    phase=phase,
                    jetson_state=output.safety.state.name,
                    stm32_state=status.system_state.name,
                    safe_speed_mps=output.safe_velocity.linear_mps,
                    actual_speed_mps=actual,
                    safety_flags=int(status.safety_flags),
                )
            )
            now_s += dt_s
    return records


def main() -> None:
    records = run_demo_scenario()
    previous_phase = None
    for record in records:
        if record.phase == previous_phase:
            continue
        previous_phase = record.phase
        print(
            f"{record.time_s:5.2f}s  {record.phase:14} "
            f"Jetson={record.jetson_state:16} "
            f"STM32={record.stm32_state:16} "
            f"safe={record.safe_speed_mps:.2f}m/s "
            f"actual={record.actual_speed_mps:.2f}m/s"
        )


if __name__ == "__main__":
    main()
