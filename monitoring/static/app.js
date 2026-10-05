"use strict";

const $ = (id) => document.getElementById(id);
const MAX_SPEED_MPS = 0.5;

const stateLabels = {
  INIT: "초기화",
  READY: "주행 준비",
  RUN: "정상 주행",
  SLOW: "감속 주행",
  CONTROLLED_STOP: "일반 정지",
  EMERGENCY_STOP: "긴급 정지",
  FAULT: "시스템 고장",
};

function numeric(value, digits = 2) {
  return Number.isFinite(value) ? Number(value).toFixed(digits) : "—";
}

function formatDuration(ms) {
  if (!Number.isFinite(ms)) return "—";
  const seconds = Math.floor(ms / 1000);
  const hours = String(Math.floor(seconds / 3600)).padStart(2, "0");
  const minutes = String(Math.floor((seconds % 3600) / 60)).padStart(2, "0");
  const remainder = String(seconds % 60).padStart(2, "0");
  return `${hours}:${minutes}:${remainder}`;
}

function formatTime(iso) {
  if (!iso) return "—";
  const date = new Date(iso);
  if (Number.isNaN(date.getTime())) return "—";
  return new Intl.DateTimeFormat("ko-KR", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hour12: false,
  }).format(date);
}

function setText(id, value) {
  $(id).textContent = value;
}

function updateConnection(connection) {
  Object.entries(connection).forEach(([device, connected]) => {
    const element = document.querySelector(`[data-device="${device}"]`);
    if (!element) return;
    element.classList.toggle("is-connected", connected);
    element.querySelector("strong").textContent = connected ? "연결" : "끊김";
  });
}

function updateState(status) {
  const hero = $("state-hero");
  hero.className = `state-hero state-${status.system_state.toLowerCase().replaceAll("_", "-")}`;
  setText("system-state", stateLabels[status.system_state] || status.system_state);
  setText("state-reason", status.state_reason);
}

function updateVelocity(velocity) {
  setText("actual-speed", numeric(velocity.actual_linear_mps));
  setText("metric-actual-speed", numeric(velocity.actual_linear_mps));
  setText("target-speed", numeric(velocity.target_linear_mps));
  setText("angular-speed", `${numeric(velocity.angular_rad_s)} rad/s`);
  setText("left-speed", numeric(velocity.left_mps));
  setText("right-speed", numeric(velocity.right_mps));

  const leftPercent = Math.min(100, Math.abs(velocity.left_mps) / MAX_SPEED_MPS * 100);
  const rightPercent = Math.min(100, Math.abs(velocity.right_mps) / MAX_SPEED_MPS * 100);
  $("left-speed-bar").style.width = `${leftPercent}%`;
  $("right-speed-bar").style.width = `${rightPercent}%`;
}

function updateObstacle(obstacle) {
  setText("obstacle-status", obstacle.detected ? "감지" : "미감지");
  setText("obstacle-distance", numeric(obstacle.distance_m));
  setText("obstacle-ttc", numeric(obstacle.ttc_s, 1));
  setText("object-class", obstacle.object_class || "없음");
  setText("object-direction", obstacle.direction || "—");
  $("obstacle-status").classList.toggle("is-danger-text", obstacle.detected);
}

function updateCliff(cliff) {
  const danger = cliff.left || cliff.right || cliff.tof_danger;
  setText("cliff-summary", danger ? "위험" : "정상");
  setText("cliff-left", cliff.left ? "위험" : "정상");
  setText("cliff-right", cliff.right ? "위험" : "정상");
  setText(
    "cliff-left-distance",
    cliff.left_distance_m == null ? "거리 —" : `${numeric(cliff.left_distance_m)} m`,
  );
  setText(
    "cliff-right-distance",
    cliff.right_distance_m == null ? "거리 —" : `${numeric(cliff.right_distance_m)} m`,
  );
  setText("tof-state", cliff.tof_danger ? "위험" : "정상");

  $("cliff-summary").classList.toggle("is-danger-text", danger);
  $("tof-state").classList.toggle("is-danger-text", cliff.tof_danger);
  $("cliff-left-cell").classList.toggle("is-danger", cliff.left);
  $("cliff-right-cell").classList.toggle("is-danger", cliff.right);
}

function updateBattery(battery) {
  const percent = Number.isFinite(battery.percent) ? battery.percent : 0;
  setText("battery-percent", Number.isFinite(battery.percent) ? Math.round(percent) : "—");
  setText(
    "battery-voltage",
    Number.isFinite(battery.voltage_v) ? `${numeric(battery.voltage_v, 1)} V` : "— V",
  );
  $("battery-fill").style.width = `${Math.min(100, Math.max(0, percent))}%`;
  $("battery-fill").style.background =
    battery.warning || percent < 20
      ? "var(--red)"
      : percent < 40
        ? "var(--yellow)"
        : "linear-gradient(90deg, var(--green), var(--cyan))";
  setText(
    "battery-message",
    battery.warning
      ? "배터리 경고 · 안전한 위치에서 충전 필요"
      : Number.isFinite(battery.percent)
        ? "배터리 상태 정상"
        : "배터리 데이터 대기 중",
  );
  $("battery-message").classList.toggle("is-danger-text", battery.warning);
}

function updateDiagnostics(status) {
  const diagnostics = status.diagnostics;
  setText("protocol-version", `v${diagnostics.protocol_version}`);
  setText("motor-error", String(diagnostics.motor_error));
  setText("rx-errors", String(diagnostics.rx_error_count));
  setText("last-command", String(diagnostics.last_command_id));
  setText("uptime", formatDuration(diagnostics.uptime_ms));
  setText("stop-reason", status.last_stop_reason || "없음");

  const area = $("safety-flags");
  area.replaceChildren();
  if (status.safety_flags.length === 0) {
    const clear = document.createElement("span");
    clear.className = "flag flag-clear";
    clear.textContent = "없음";
    area.append(clear);
  } else {
    status.safety_flags.forEach((flag) => {
      const element = document.createElement("span");
      element.className = "flag";
      element.textContent = flag;
      area.append(element);
    });
  }
}

function updateRangeSensors(sensors) {
  const distance = (value) => Number.isFinite(value) ? `${numeric(value)} m` : "—";
  setText("ultrasonic-front", distance(sensors.ultrasonic_front_m));
  setText("sharp-left", distance(sensors.sharp_left_m));
  setText("sharp-right", distance(sensors.sharp_right_m));
}

function updateEvents(events) {
  const list = $("event-list");
  list.replaceChildren();
  setText("event-count", `${events.length}건`);

  if (events.length === 0) {
    const empty = document.createElement("li");
    empty.className = "event-empty";
    empty.textContent = "수신된 이벤트가 없습니다.";
    list.append(empty);
    return;
  }

  [...events].reverse().forEach((event) => {
    const item = document.createElement("li");
    item.className = event.level;
    const time = document.createElement("time");
    time.textContent = formatTime(event.time);
    const message = document.createElement("p");
    message.textContent = event.message;
    item.append(time, message);
    list.append(item);
  });
}

function updateMonitoring(monitoring) {
  const indicator = $("live-indicator");
  indicator.classList.toggle("is-live", !monitoring.stale);
  indicator.classList.toggle("is-stale", monitoring.stale);
  setText("live-label", monitoring.stale ? "데이터 지연" : "실시간 연결");
  setText(
    "last-update",
    monitoring.age_ms == null ? "수신 전" : `${monitoring.age_ms}ms 전`,
  );
}

function render(payload) {
  const status = payload.status;
  updateState(status);
  updateConnection(status.connection);
  updateVelocity(status.velocity);
  updateObstacle(status.obstacle);
  updateCliff(status.cliff);
  updateRangeSensors(status.range_sensors);
  updateBattery(status.battery);
  updateDiagnostics(status);
  updateEvents(payload.events);
  updateMonitoring(payload.monitoring);
}

async function refresh() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  } catch (_error) {
    const indicator = $("live-indicator");
    indicator.classList.remove("is-live");
    indicator.classList.add("is-stale");
    setText("live-label", "관제 서버 연결 끊김");
  }
}

refresh();
setInterval(refresh, 500);
