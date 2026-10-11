"""Timestamped, bounded JSONL flight recorder shared by controller threads."""
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
import json
import logging
from logging.handlers import RotatingFileHandler
import math
from pathlib import Path
import time


PHASE_MESSAGES = {
    "DRIVE": "일반 직진 주행",
    "STOPPING": "장애물 감지, 회피 진입 전 감속·정지",
    "WAIT_DIRECTION": "좌우 회피 가능 방향 확인",
    "ENTRY": "장애물 옆 통과 경로로 진입",
    "BASELINE": "측면 정렬 완료, 측면센서 기준값 측정",
    "PASS": "장애물 측면 통과",
    "REAR_CLEARANCE": "장애물 후단 통과 후 복귀 여유 확보",
    "RETURN": "원래 직진 경로로 복귀",
}

REASON_MESSAGES = {
    "front_clearance": "전방 안전거리 부족",
    "left_clearance": "왼쪽 측면 안전거리 부족",
    "right_clearance": "오른쪽 측면 안전거리 부족",
    "no_entry_direction": "좌우 모두 회피 진입 공간 부족",
    "insufficient_entry_forward_room": "회피 곡선을 만들 전방 거리 부족",
    "entry_path_too_tight": "회피 진입 곡선이 너무 급함",
    "new_obstacle_on_saved_path": "저장된 회피 경로에 새 장애물 감지",
    "user_stop_held_or_speed_zero": "정지 버튼이 눌렸거나 설정 속도가 0 RPM",
    "user_speed_zero": "DOWN 조작으로 설정 속도가 0 RPM",
    "user_instant_stop": "물리 비상정지 입력",
    "user_instant_stop_terminal": "터미널 비상정지 요청",
    "avoidance_heading_limit": "회피 중 허용 회전각 초과",
    "path_time_limit": "회피 경로 수행시간 초과",
    "pass_distance_limit": "장애물 측면 통과거리 한도 초과",
    "wheel_feedback_no_progress": "구동 명령 중 바퀴 진행 피드백 없음",
    "side_obstacle_never_seen_check_sensor_source_wiring_orientation": "회피 중 측면 장애물을 확인하지 못함",
    "control_loop_gap_over_250ms": "제어 주기가 250ms 이상 지연됨",
    "camera_missing_or_stale": "카메라 검출 데이터 대기 또는 지연",
    "lidar_missing_or_stale": "LiDAR 데이터 대기 또는 지연",
    "STM32_STATUS_missing_or_stale": "STM32 상태 데이터 대기 또는 지연",
    "STM32_CTRL_missing_or_stale": "STM32 제어 상태 데이터 대기 또는 지연",
    "startup_nonzero_speed": "시작 시 설정 속도가 0 RPM이 아님",
    "startup_stop_flag_active": "시작 시 정지 입력이 활성 상태",
}


def human_reason(reason):
    reason = str(reason or "원인 미상")
    if reason in REASON_MESSAGES:
        return f"{REASON_MESSAGES[reason]} ({reason})"
    if reason.startswith("sensor_health:"):
        return f"센서 데이터 누락 또는 지연 ({reason.removeprefix('sensor_health:')})"
    if reason.startswith("startup_timeout:"):
        return f"READY 준비시간 초과 ({reason.removeprefix('startup_timeout:')})"
    return reason


def json_safe(value):
    if is_dataclass(value):
        return json_safe(asdict(value))
    if isinstance(value, float) and not math.isfinite(value):
        return "NaN" if math.isnan(value) else ("+Inf" if value > 0 else "-Inf")
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


class RequiredFileHandler(RotatingFileHandler):
    def handleError(self, record):
        # A full disk must stop motion rather than silently lose evidence.
        raise IOError("Flight recorder write/rotation failed")


class FlightRecorder:
    def __init__(self, folder, max_bytes=16 * 1024 * 1024, backups=24):
        Path(folder).mkdir(parents=True, exist_ok=True)
        self.logger = logging.Logger(f"full_run.{id(self)}", logging.INFO)
        self.handler = RequiredFileHandler(Path(folder) / "events.jsonl", maxBytes=max_bytes,
                                           backupCount=backups, encoding="utf-8")
        self.handler.setFormatter(logging.Formatter("%(message)s"))
        self.logger.addHandler(self.handler)
        self.origin = time.monotonic()

    def emit(self, event, **data):
        record = {"utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                  "monotonic_s": time.monotonic(), "elapsed_s": time.monotonic() - self.origin,
                  "event": event, **data}
        self.logger.info(json.dumps(json_safe(record), ensure_ascii=False, allow_nan=False,
                                    separators=(",", ":")))
        if event == "startup_step":
            print(f"[STARTUP] {data.get('stage', 'unknown')}", flush=True)
            return
        if event == "state":
            before = data.get("before", "unknown")
            after = data.get("after", "unknown")
            reason = data.get("reason", "")
            print(f"[STATE] {before} -> {after}: {reason}", flush=True)
            if after == "READY":
                print("[READY] 준비 완료. 로봇은 정지 상태입니다. 물리 UP 버튼을 누르면 출발합니다.", flush=True)
            elif after == "RUNNING":
                print(f"[RUN] 출발/재출발 허용: {human_reason(reason)}", flush=True)
            elif after == "PAUSED":
                print(f"[PAUSED] {human_reason(reason)}. 원인 해소 후 UP을 다시 누르세요.", flush=True)
            elif after == "FAULT_STOP":
                print(f"[FAULT STOP] {human_reason(reason)}. 원인 확인 후 프로세스를 재시작하세요.", flush=True)
            return
        if event == "ready":
            print(f"[READY] {data.get('message', 'Readiness checks passed.')}", flush=True)
            return
        if event == "phase":
            phase = data.get("after", "unknown")
            print(f"[AVOID] {phase}: {PHASE_MESSAGES.get(phase, '회피 단계 전환')}", flush=True)
            return
        if event in ("up", "down"):
            print(f"[BUTTON] {event.upper()} 감지: 설정 속도 {data.get('base_rpm', '?')} RPM", flush=True)
            return
        if event == "legacy_startup_button_consumed":
            print(f"[BUTTON] {data.get('direction', '?')} 감지했지만 READY 전 입력이라 무시했습니다.", flush=True)
            return
        if event == "legacy_stop_UP_consumed":
            print("[BUTTON] UP 감지했지만 정지 확인 중이라 무시했습니다. 정지 완료 후 UP을 다시 누르세요.", flush=True)
            return
        if event == "legacy_button_limit":
            print(f"[BUTTON WARNING] {data.get('instruction', '버튼 입력 한계 상태를 확인하세요.')}", flush=True)
            return
        if event == "resume_denied":
            print(f"[START BLOCKED] 출발/재출발 불가: {human_reason(data.get('reason'))}", flush=True)
            return
        if event == "readiness_blocked":
            reasons = ", ".join(human_reason(reason) for reason in data.get("reasons", []))
            print(f"[WAIT READY] 준비되지 않은 원인: {reasons or '입력 대기 중'}", flush=True)
            return
        if event == "frozen_obstacle":
            target = data.get("target", {})
            print(f"[AVOID] 장애물 확정: {target.get('label', 'object')}, "
                  f"전방 {target.get('forward_distance_m', '?')}m", flush=True)
            return
        if event == "dynamic_avoidance_plan":
            print(f"[AVOID PLAN] 동적 회피: 폭 {data.get('obstacle_width_m', '?')}m, "
                  f"{data.get('side', '?')} 방향, 측면이동 {data.get('lateral_m', '?')}m, "
                  f"진입 전진거리 {data.get('actual_entry_forward_m', '?')}m, "
                  f"최대 예상각 {data.get('actual_peak_heading_deg', '?')}도", flush=True)
            return
        if event == "dynamic_pass_started":
            print(f"[AVOID] 정렬 완료: 차체 길이 {data.get('minimum_pass_m', '?')}m 전진 후 "
                  f"측면 {data.get('clear_threshold_m', '?')}m 이상을 "
                  f"{data.get('clear_samples', '?')}회 확인합니다.", flush=True)
            return
        if event == "side_object_seen":
            print(f"[AVOID] {data.get('side', '?')} 측면센서로 장애물 확인", flush=True)
            return
        if event == "tail_confirmed":
            print(f"[AVOID] 장애물 후단 확인, 추가 {data.get('extra_m', '?')}m 통과 예정", flush=True)
            return
        if event == "exception":
            print(f"[ERROR] {data.get('reason') or data.get('context') or data.get('error') or '예외 발생'}", flush=True)
            return
        if event == "MCU_schema_mismatch":
            print(f"[ERROR] STM32 STATUS 형식 불일치: {data.get('received_fields', '?')} fields", flush=True)
            return

    def close(self):
        self.handler.flush()
        self.handler.close()
        self.logger.removeHandler(self.handler)
