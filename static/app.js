const state = {
  latest: null,
  filter: "",
  events: null,
  locoStatusKey: null,
};

const els = {
  subtitle: document.getElementById("subtitle"),
  connection: document.getElementById("connection"),
  age: document.getElementById("age"),
  rate: document.getElementById("rate"),
  chillMotors: document.getElementById("chillMotors"),
  networkType: document.getElementById("networkType"),
  sidebarNetworkType: document.getElementById("sidebarNetworkType"),
  sidebarNetworkQuality: document.getElementById("sidebarNetworkQuality"),
  footerNetworkType: document.getElementById("footerNetworkType"),
  robotFields: document.getElementById("robotFields"),
  imuFields: document.getElementById("imuFields"),
  batteryFields: document.getElementById("batteryFields"),
  handFields: document.getElementById("handFields"),
  forceFields: document.getElementById("forceFields"),
  motorRows: document.getElementById("motorRows"),
  rawJson: document.getElementById("rawJson"),
  cameraStream: document.getElementById("cameraStream"),
  cameraPlaceholder: document.getElementById("cameraPlaceholder"),
  rosSummary: document.getElementById("rosSummary"),
  rosMap: document.getElementById("rosMap"),
  rosEdges: document.getElementById("rosEdges"),
  refreshRosGraph: document.getElementById("refreshRosGraph"),
  filter: document.getElementById("filter"),
  navItems: document.querySelectorAll(".nav-item"),
  wristState: document.getElementById("wristState"),
  wristStop: document.getElementById("wristStop"),
  wristCurrentQ: document.getElementById("wristCurrentQ"),
  wristCurrentDq: document.getElementById("wristCurrentDq"),
  wristTargetQReadout: document.getElementById("wristTargetQReadout"),
  wristMessage: document.getElementById("wristMessage"),
  wristLastCommand: document.getElementById("wristLastCommand"),
  wristArm: document.getElementById("wristArm"),
  wristRisk: document.getElementById("wristRisk"),
  wristLive: document.getElementById("wristLive"),
  wristLowcmd: document.getElementById("wristLowcmd"),
  wristAutoGains: document.getElementById("wristAutoGains"),
  wristTargetRelative: document.getElementById("wristTargetRelative"),
  wristTargetLabel: document.getElementById("wristTargetLabel"),
  wristTarget: document.getElementById("wristTarget"),
  wristDelta: document.getElementById("wristDelta"),
  wristKp: document.getElementById("wristKp"),
  wristKd: document.getElementById("wristKd"),
  wristDuration: document.getElementById("wristDuration"),
  wristPeriod: document.getElementById("wristPeriod"),
  wristRate: document.getElementById("wristRate"),
  wristTargetValue: document.getElementById("wristTargetValue"),
  wristDeltaValue: document.getElementById("wristDeltaValue"),
  wristKpValue: document.getElementById("wristKpValue"),
  wristKdValue: document.getElementById("wristKdValue"),
  wristDurationValue: document.getElementById("wristDurationValue"),
  wristPeriodValue: document.getElementById("wristPeriodValue"),
  wristRateValue: document.getElementById("wristRateValue"),
  wristSendAbsolute: document.getElementById("wristSendAbsolute"),
  wristOscillate: document.getElementById("wristOscillate"),
  locoState: document.getElementById("locoState"),
  locoStop: document.getElementById("locoStop"),
  locoModeMachine: document.getElementById("locoModeMachine"),
  locoModePr: document.getElementById("locoModePr"),
  locoMotionOwner: document.getElementById("locoMotionOwner"),
  locoMessage: document.getElementById("locoMessage"),
  locoLastCommand: document.getElementById("locoLastCommand"),
  locoArm: document.getElementById("locoArm"),
  locoRisk: document.getElementById("locoRisk"),
  locoReady: document.getElementById("locoReady"),
  locoBalanceStand: document.getElementById("locoBalanceStand"),
  locoStandUp: document.getElementById("locoStandUp"),
  locoStart: document.getElementById("locoStart"),
  locoDamp: document.getElementById("locoDamp"),
  locoZeroTorque: document.getElementById("locoZeroTorque"),
  locoHighStand: document.getElementById("locoHighStand"),
  locoLowStand: document.getElementById("locoLowStand"),
  locoGaitOn: document.getElementById("locoGaitOn"),
  locoGaitOff: document.getElementById("locoGaitOff"),
  locoNextFootLeft: document.getElementById("locoNextFootLeft"),
  locoNextFootRight: document.getElementById("locoNextFootRight"),
  locoWaveHand: document.getElementById("locoWaveHand"),
  locoShakeHand: document.getElementById("locoShakeHand"),
  locoShakeStart: document.getElementById("locoShakeStart"),
  locoShakeEnd: document.getElementById("locoShakeEnd"),
  locoEnableOdom: document.getElementById("locoEnableOdom"),
  locoDisableOdom: document.getElementById("locoDisableOdom"),
  locoVx: document.getElementById("locoVx"),
  locoVy: document.getElementById("locoVy"),
  locoVyaw: document.getElementById("locoVyaw"),
  locoDuration: document.getElementById("locoDuration"),
  locoStandHeight: document.getElementById("locoStandHeight"),
  locoSwingHeight: document.getElementById("locoSwingHeight"),
  locoContinuousMove: document.getElementById("locoContinuousMove"),
  locoTargetRelative: document.getElementById("locoTargetRelative"),
  locoTargetX: document.getElementById("locoTargetX"),
  locoTargetY: document.getElementById("locoTargetY"),
  locoTargetYaw: document.getElementById("locoTargetYaw"),
  locoVxValue: document.getElementById("locoVxValue"),
  locoVyValue: document.getElementById("locoVyValue"),
  locoVyawValue: document.getElementById("locoVyawValue"),
  locoDurationValue: document.getElementById("locoDurationValue"),
  locoStandHeightValue: document.getElementById("locoStandHeightValue"),
  locoSwingHeightValue: document.getElementById("locoSwingHeightValue"),
  locoTargetXValue: document.getElementById("locoTargetXValue"),
  locoTargetYValue: document.getElementById("locoTargetYValue"),
  locoTargetYawValue: document.getElementById("locoTargetYawValue"),
  locoSendVelocity: document.getElementById("locoSendVelocity"),
  locoMove: document.getElementById("locoMove"),
  locoSetHeight: document.getElementById("locoSetHeight"),
  locoSetSwingHeight: document.getElementById("locoSetSwingHeight"),
  locoSetTargetPosition: document.getElementById("locoSetTargetPosition"),
  locoGetOdom: document.getElementById("locoGetOdom"),
  locoGetFsmId: document.getElementById("locoGetFsmId"),
  locoGetFsmMode: document.getElementById("locoGetFsmMode"),
  locoGetBalanceMode: document.getElementById("locoGetBalanceMode"),
  locoGetSwingHeight: document.getElementById("locoGetSwingHeight"),
  locoGetStandHeight: document.getElementById("locoGetStandHeight"),
  locoGetPhase: document.getElementById("locoGetPhase"),
  locoHistory: document.getElementById("locoHistory"),
  locoPresets: document.querySelectorAll("[data-loco-preset]"),
};

function fmt(value, suffix = "") {
  if (value === undefined || value === null || value === "") return "--";
  if (Array.isArray(value)) return `[${value.map((item) => fmt(item)).join(", ")}]`;
  if (typeof value === "number") return `${Number.isInteger(value) ? value : value.toFixed(3)}${suffix}`;
  if (typeof value === "boolean") return value ? "true" : "false";
  return String(value);
}

function esc(value) {
  return String(value ?? "--")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

function setFields(node, data) {
  const entries = Object.entries(data || {});
  if (entries.length === 0) {
    node.innerHTML = "<dt>state</dt><dd>--</dd>";
    return;
  }
  node.innerHTML = entries
    .map(([key, value]) => `<dt>${key}</dt><dd>${fmt(value)}</dd>`)
    .join("");
}

function valueList(values, labels) {
  return (values || []).map((value, index) => ({
    label: labels?.[index] || String(index),
    value,
  }));
}

function renderMetricCards(node, cards, rows = []) {
  node.innerHTML = `
    <div class="metric-card-grid">
      ${cards
        .map(
          (card) => `
            <div class="status-metric ${card.tone || ""}">
              <span>${card.label}</span>
              <strong>${fmt(card.value, card.suffix || "")}</strong>
            </div>
          `,
        )
        .join("")}
    </div>
    ${
      rows.length
        ? `<dl class="status-list">${rows
            .map((row) => `<dt>${row.label}</dt><dd>${fmt(row.value)}</dd>`)
            .join("")}</dl>`
        : ""
    }
  `;
}

function renderStatusList(node, rows) {
  node.innerHTML = `<dl class="status-list full">${rows
    .filter((row) => row.value !== undefined)
    .map((row) => `<dt>${row.label}</dt><dd>${fmt(row.value)}</dd>`)
    .join("")}</dl>`;
}

function renderRobotStatus(snapshot) {
  const robot = snapshot.robot || {};
  const imu = snapshot.imu || {};
  const battery = snapshot.battery || {};
  const hands = snapshot.hands || {};
  const analysis = snapshot.analysis || {};
  const motorSummary = analysis.motors || {};
  const imuSummary = analysis.imu || {};
  const health = analysis.health || {};
  const hottest = motorSummary.hottest;
  const maxTau = motorSummary.max_abs_tau;
  const maxVelocity = motorSummary.max_abs_velocity;

  renderMetricCards(
    els.robotFields,
    [
      { label: "Mode", value: robot.mode_machine ?? "--", tone: "red" },
      { label: "Samples", value: snapshot.samples ?? 0 },
      { label: "Motors", value: snapshot.motor_count ?? 0 },
    ],
    [
      { label: "Health", value: health.state },
      { label: "Control", value: robot.mode_pr },
      { label: "Tick", value: robot.tick },
      { label: "CRC", value: robot.crc },
      { label: "Real motors", value: motorSummary.real_count },
      { label: "Reserved slots", value: motorSummary.reserved_count },
      { label: "Mode counts", value: motorSummary.mode_counts },
    ],
  );

  renderStatusList(
    els.imuFields,
    [
      { label: "Roll", value: imuSummary.roll_deg === undefined ? imu.rpy?.[0] : `${fmt(imuSummary.roll_deg)} deg` },
      { label: "Pitch", value: imuSummary.pitch_deg === undefined ? imu.rpy?.[1] : `${fmt(imuSummary.pitch_deg)} deg` },
      { label: "Yaw", value: imuSummary.yaw_deg === undefined ? imu.rpy?.[2] : `${fmt(imuSummary.yaw_deg)} deg` },
      { label: "Gyroscope", value: imu.gyroscope },
      { label: "Accelerometer", value: imu.accelerometer },
      { label: "Temperature", value: imu.temperature },
    ],
  );

  renderStatusList(
    els.batteryFields,
    [
      { label: "State", value: battery.state },
      { label: "SOC", value: battery.soc === undefined ? undefined : `${fmt(battery.soc)}%` },
      { label: "Current", value: battery.current },
      { label: "Cycle", value: battery.cycle },
      { label: "Temperature", value: battery.temperature },
      { label: "Checked", value: battery.checked_fields },
    ],
  );

  renderMetricCards(
    els.handFields,
    [
      { label: "State", value: hands.connected ? "Connected" : "Offline", tone: hands.connected ? "" : "red", wide: true },
      { label: "Joints", value: hands.joint_count ?? 0 },
    ],
    [
      { label: "Samples", value: hands.samples ?? 0 },
      ...(hands.joints || []).slice(0, 4).map((joint) => ({ label: joint.name, value: joint.q })),
    ],
  );

  renderStatusList(
    els.forceFields,
    [
      { label: "Hottest motor", value: hottest ? `${hottest.name} ${fmt(hottest.value)} C` : "--" },
      { label: "Max torque", value: maxTau ? `${maxTau.name} ${fmt(maxTau.value)}` : "--" },
      { label: "Max velocity", value: maxVelocity ? `${maxVelocity.name} ${fmt(maxVelocity.value)}` : "--" },
      { label: "Health flags", value: (health.flags || []).map((flag) => flag.message) },
      { label: "Foot force", value: snapshot.foot_force?.length ? snapshot.foot_force : "--" },
      { label: "Estimated", value: snapshot.foot_force_est?.length ? snapshot.foot_force_est : "--" },
      ...valueList(snapshot.foot_force, ["FL", "FR", "RL", "RR"]),
      ...valueList(snapshot.foot_force_est, ["Est FL", "Est FR", "Est RL", "Est RR"]),
    ],
  );
}

function motorTableRows(snapshot) {
  const bodyRows = (snapshot.motors || []).map((motor) => ({ ...motor, source: "Body" }));
  const handRows = ((snapshot.hands || {}).joints || []).map((joint) => ({
    ...joint,
    index: `F${joint.index}`,
    source: "Finger",
  }));
  return [...bodyRows, ...handRows];
}

function renderMotors(motors) {
  const query = state.filter.trim().toLowerCase();
  const rows = (motors || []).filter((motor) => {
    if (!query) return true;
    return (
      String(motor.index).toLowerCase().includes(query) ||
      String(motor.name).toLowerCase().includes(query) ||
      String(motor.source || "").toLowerCase().includes(query)
    );
  });

  els.motorRows.innerHTML = rows
    .map((motor) => {
      const isExtra = String(motor.name).startsWith("ReservedMotorSlot");
      return `
        <tr>
          <td>${fmt(motor.index)}</td>
          <td class="${isExtra ? "extra" : "joint"}">${fmt(motor.name)}${motor.source ? ` <small>${esc(motor.source)}</small>` : ""}</td>
          <td>${fmt(motor.mode)}</td>
          <td>${fmt(motor.q)}</td>
          <td>${fmt(motor.dq)}</td>
          <td>${fmt(motor.tau_est)}</td>
          <td>${fmt(motor.temperature)}</td>
          <td>${fmt(motor.vol)}</td>
        </tr>
      `;
    })
    .join("");
}

function networkLabel(network) {
  const type = network?.type || "Network";
  const iface = network?.interface && network.interface !== "unknown" ? ` (${network.interface})` : "";
  return `${type}${iface}`;
}

function renderNetwork(network) {
  const robot = network?.robot || {};
  const host = network?.host || {};
  const robotTarget = robot.target ? ` to ${robot.target}` : "";
  const robotConnected = (robot.quality || "Disconnected") === "Connected";

  els.networkType.textContent = `${networkLabel(robot)}${robotTarget}`;
  els.connection.textContent = robotConnected ? "Live" : "Offline";
  els.connection.className = `pill ${robotConnected ? "good" : "bad"}`;
  els.sidebarNetworkType.textContent = networkLabel(host);
  els.sidebarNetworkQuality.textContent = host.quality || "Connected";
  els.footerNetworkType.textContent = `${networkLabel(robot)} Robot Link`;
}

function render(snapshot) {
  state.latest = snapshot;
  const connected = Boolean(snapshot.connected);
  const age = snapshot.timestamp ? Math.max(0, Date.now() / 1000 - snapshot.timestamp) : null;

  els.age.textContent = `age ${age === null ? "--" : age.toFixed(1)}s`;
  els.rate.textContent = `${fmt(snapshot.sample_rate_hz)} Hz`;
  els.subtitle.textContent = connected
    ? `${fmt(snapshot.motor_count)} motors, ${fmt(snapshot.samples)} samples`
    : snapshot.error || "Waiting for rt/lowstate";
  renderNetwork(snapshot.network);

  renderRobotStatus(snapshot);
  renderMotors(motorTableRows(snapshot));
  renderWristTelemetry(snapshot);
  renderLocoTelemetry(snapshot);
  if (snapshot.loco) renderLocoStatus(snapshot.loco);
  els.rawJson.textContent = JSON.stringify(snapshot, null, 2);
  window.dispatchEvent(new CustomEvent("telemetry-state", { detail: { snapshot } }));
}

function locoSafetyReady() {
  return Boolean(els.locoArm?.checked && els.locoRisk?.checked);
}

function updateLocoSliderLabels() {
  if (!els.locoVx) return;
  els.locoVxValue.textContent = `${Number(els.locoVx.value).toFixed(2)} m/s`;
  els.locoVyValue.textContent = `${Number(els.locoVy.value).toFixed(2)} m/s`;
  els.locoVyawValue.textContent = `${Number(els.locoVyaw.value).toFixed(2)} rad/s`;
  els.locoDurationValue.textContent = `${Number(els.locoDuration.value).toFixed(2)} s`;
  els.locoStandHeightValue.textContent = Number(els.locoStandHeight.value).toFixed(2);
  els.locoSwingHeightValue.textContent = Number(els.locoSwingHeight.value).toFixed(3);
  els.locoTargetXValue.textContent = `${Number(els.locoTargetX.value).toFixed(2)} m`;
  els.locoTargetYValue.textContent = `${Number(els.locoTargetY.value).toFixed(2)} m`;
  els.locoTargetYawValue.textContent = `${Number(els.locoTargetYaw.value).toFixed(2)} rad`;
}

function setLocoButtons() {
  if (!els.locoState) return;
  const ready = locoSafetyReady();
  [
    els.locoReady,
    els.locoBalanceStand,
    els.locoStandUp,
    els.locoStart,
    els.locoDamp,
    els.locoZeroTorque,
    els.locoHighStand,
    els.locoLowStand,
    els.locoGaitOn,
    els.locoGaitOff,
    els.locoNextFootLeft,
    els.locoNextFootRight,
    els.locoWaveHand,
    els.locoShakeHand,
    els.locoShakeStart,
    els.locoShakeEnd,
    els.locoEnableOdom,
    els.locoDisableOdom,
    els.locoSendVelocity,
    els.locoMove,
    els.locoSetHeight,
    els.locoSetSwingHeight,
    els.locoSetTargetPosition,
    els.locoGetOdom,
    els.locoGetFsmId,
    els.locoGetFsmMode,
    els.locoGetBalanceMode,
    els.locoGetSwingHeight,
    els.locoGetStandHeight,
    els.locoGetPhase,
  ].forEach((button) => {
    if (button) button.disabled = !ready;
  });
  els.locoPresets.forEach((button) => {
    button.disabled = !ready;
  });
  if (els.locoStop) els.locoStop.disabled = !ready;
  if (!ready && els.locoMessage) {
    stopLocoHold(false);
    els.locoMessage.textContent = "Enable both safety checkboxes before sending a loco command.";
  }
}

function renderLocoTelemetry(snapshot) {
  if (!els.locoModeMachine) return;
  const robot = snapshot.robot || {};
  els.locoModeMachine.textContent = fmt(robot.mode_machine);
  els.locoModePr.textContent = fmt(robot.mode_pr);
}

function motionOwnerLabel(mode) {
  if (!mode) return "--";
  if (mode.error) return `error: ${mode.error}`;
  if (typeof mode === "string") return mode;
  return mode.name || mode.alias || mode.form || JSON.stringify(mode);
}

function locoStatusKey(status) {
  return JSON.stringify([
    status.available,
    status.active,
    status.message,
    status.updated_at,
    status.motion_mode,
    status.motion_check_code,
    status.last_command,
    status.history,
  ]);
}

function renderLocoStatus(status, force = false) {
  if (!els.locoState) return;
  const key = locoStatusKey(status);
  if (!force && key === state.locoStatusKey) {
    if (status.robot) {
      els.locoModeMachine.textContent = fmt(status.robot.mode_machine);
      els.locoModePr.textContent = fmt(status.robot.mode_pr);
    }
    return;
  }
  state.locoStatusKey = key;
  const active = Boolean(status.active);
  const available = Boolean(status.available);
  els.locoState.textContent = active ? "Sending" : available ? "Ready" : "Unavailable";
  els.locoState.className = `pill ${available ? "good" : "bad"}`;
  els.locoMessage.textContent = status.message || "--";
  els.locoMotionOwner.textContent = motionOwnerLabel(status.motion_mode);
  if (status.robot) {
    els.locoModeMachine.textContent = fmt(status.robot.mode_machine);
    els.locoModePr.textContent = fmt(status.robot.mode_pr);
  }
  const last = status.last_command;
  els.locoLastCommand.textContent = last
    ? `${last.action} vx ${fmt(last.vx)} vy ${fmt(last.vy)} yaw ${fmt(last.vyaw)}`
    : "No loco command sent";
  renderLocoHistory(status.history || []);
}

function renderLocoHistory(history) {
  if (!els.locoHistory) return;
  if (!history.length) {
    els.locoHistory.innerHTML = `<div class="ros-empty">No loco commands sent.</div>`;
    return;
  }
  els.locoHistory.innerHTML = history
    .map((item) => {
      const timeLabel = item.time ? new Date(item.time * 1000).toLocaleTimeString() : "--";
      const code = item.call_code === undefined || item.call_code === null ? "ok" : item.call_code;
      return `
        <div class="loco-history-item">
          <strong>${esc(item.action)}</strong>
          <span>${esc(timeLabel)} code ${esc(code)}</span>
          <small>vx ${fmt(item.vx)} · vy ${fmt(item.vy)} · yaw ${fmt(item.vyaw)} · ${fmt(item.duration)}s</small>
          ${item.result === undefined || item.result === null ? "" : `<small>result ${esc(JSON.stringify(item.result))}</small>`}
        </div>
      `;
    })
    .join("");
}

function loadLocoStatus() {
  if (!els.locoState) return;
  fetch("/api/loco/status")
    .then((response) => response.json())
    .then((status) => renderLocoStatus(status, true))
    .catch((error) => {
      els.locoState.textContent = "Unavailable";
      els.locoState.className = "pill bad";
      els.locoMessage.textContent = error.message;
    });
}

let locoHold = null;

function locoPayload(action, overrides = {}) {
  return {
    action,
    armed: Boolean(els.locoArm?.checked),
    i_understand_risk: Boolean(els.locoRisk?.checked),
    vx: Number(els.locoVx?.value || 0),
    vy: Number(els.locoVy?.value || 0),
    vyaw: Number(els.locoVyaw?.value || 0),
    duration: Number(els.locoDuration?.value || 1),
    stand_height: Number(els.locoStandHeight?.value || 0.5),
    swing_height: Number(els.locoSwingHeight?.value || 0.05),
    continuous_move: Boolean(els.locoContinuousMove?.checked),
    target_relative: Boolean(els.locoTargetRelative?.checked),
    target_x: Number(els.locoTargetX?.value || 0),
    target_y: Number(els.locoTargetY?.value || 0),
    target_yaw: Number(els.locoTargetYaw?.value || 0),
    ...overrides,
    action,
  };
}

function sendLocoCommand(action, overrides = {}) {
  if (!els.locoState) return;
  if (!locoSafetyReady()) {
    els.locoMessage.textContent = "Command blocked: enable both safety checkboxes first.";
    els.locoState.textContent = "Blocked";
    els.locoState.className = "pill bad";
    setLocoButtons();
    return;
  }
  els.locoMessage.textContent = `Sending ${action}`;
  els.locoState.textContent = "Sending";
  els.locoState.className = "pill good";
  fetch("/api/loco/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(locoPayload(action, overrides)),
  })
    .then((response) =>
      response.json().then((data) => {
        if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
        return data;
      }),
    )
    .then((data) => renderLocoStatus(data.status || data, true))
    .catch((error) => {
      els.locoMessage.textContent = error.message;
      els.locoState.textContent = "Blocked";
      els.locoState.className = "pill bad";
    });
}

function applyLocoPreset(name) {
  const presets = {
    forward: [0.5, 0, 0],
    back: [-0.5, 0, 0],
    left: [0, 0.5, 0],
    right: [0, -0.5, 0],
    "turn-left": [0, 0, 0.5],
    "turn-right": [0, 0, -0.5],
  };
  const values = presets[name];
  if (!values || !els.locoVx) return;
  [els.locoVx.value, els.locoVy.value, els.locoVyaw.value] = values.map(String);
  updateLocoSliderLabels();
}

function sendLocoHoldVelocity(name) {
  applyLocoPreset(name);
  sendLocoCommand("move", { continuous_move: true });
}

function stopLocoHold(sendStop = true) {
  if (!locoHold) return;
  window.clearInterval(locoHold.timer);
  locoHold.button?.classList.remove("is-held");
  locoHold = null;
  if (sendStop) sendLocoCommand("stop_move");
}

function startLocoPresetHold(event, button) {
  if (!button.dataset.locoPreset || button.disabled || event.button > 0) return;
  event.preventDefault();
  stopLocoHold();
  button.setPointerCapture?.(event.pointerId);
  button.classList.add("is-held");
  sendLocoHoldVelocity(button.dataset.locoPreset);
  locoHold = {
    button,
  };
}

function setupLocoControls() {
  if (!els.locoState) return;
  [
    els.locoVx,
    els.locoVy,
    els.locoVyaw,
    els.locoDuration,
    els.locoStandHeight,
    els.locoSwingHeight,
    els.locoTargetX,
    els.locoTargetY,
    els.locoTargetYaw,
  ].forEach((slider) => {
    slider?.addEventListener("input", updateLocoSliderLabels);
  });
  els.locoArm?.addEventListener("change", setLocoButtons);
  els.locoRisk?.addEventListener("change", setLocoButtons);
  els.locoReady?.addEventListener("click", () => sendLocoCommand("ready"));
  els.locoBalanceStand?.addEventListener("click", () => sendLocoCommand("balance_stand"));
  els.locoStandUp?.addEventListener("click", () => sendLocoCommand("stand_up"));
  els.locoStart?.addEventListener("click", () => sendLocoCommand("start"));
  els.locoStop?.addEventListener("click", () => sendLocoCommand("stop_move"));
  els.locoDamp?.addEventListener("click", () => sendLocoCommand("damp"));
  els.locoZeroTorque?.addEventListener("click", () => sendLocoCommand("zero_torque"));
  els.locoHighStand?.addEventListener("click", () => sendLocoCommand("high_stand"));
  els.locoLowStand?.addEventListener("click", () => sendLocoCommand("low_stand"));
  els.locoGaitOn?.addEventListener("click", () => sendLocoCommand("continuous_gait_on"));
  els.locoGaitOff?.addEventListener("click", () => sendLocoCommand("continuous_gait_off"));
  els.locoNextFootLeft?.addEventListener("click", () => sendLocoCommand("next_foot_left"));
  els.locoNextFootRight?.addEventListener("click", () => sendLocoCommand("next_foot_right"));
  els.locoWaveHand?.addEventListener("click", () => sendLocoCommand("wave_hand"));
  els.locoShakeHand?.addEventListener("click", () => sendLocoCommand("shake_hand"));
  els.locoShakeStart?.addEventListener("click", () => sendLocoCommand("shake_hand_start"));
  els.locoShakeEnd?.addEventListener("click", () => sendLocoCommand("shake_hand_end"));
  els.locoEnableOdom?.addEventListener("click", () => sendLocoCommand("enable_odom"));
  els.locoDisableOdom?.addEventListener("click", () => sendLocoCommand("disable_odom"));
  els.locoSendVelocity?.addEventListener("click", () => sendLocoCommand("velocity"));
  els.locoMove?.addEventListener("click", () => sendLocoCommand("move"));
  els.locoSetHeight?.addEventListener("click", () => sendLocoCommand("set_height"));
  els.locoSetSwingHeight?.addEventListener("click", () => sendLocoCommand("set_swing_height"));
  els.locoSetTargetPosition?.addEventListener("click", () => sendLocoCommand("set_target_position"));
  els.locoGetOdom?.addEventListener("click", () => sendLocoCommand("get_odom"));
  els.locoGetFsmId?.addEventListener("click", () => sendLocoCommand("get_fsm_id"));
  els.locoGetFsmMode?.addEventListener("click", () => sendLocoCommand("get_fsm_mode"));
  els.locoGetBalanceMode?.addEventListener("click", () => sendLocoCommand("get_balance_mode"));
  els.locoGetSwingHeight?.addEventListener("click", () => sendLocoCommand("get_swing_height"));
  els.locoGetStandHeight?.addEventListener("click", () => sendLocoCommand("get_stand_height"));
  els.locoGetPhase?.addEventListener("click", () => sendLocoCommand("get_phase"));
  els.locoPresets.forEach((button) => {
    button.addEventListener("pointerdown", (event) => startLocoPresetHold(event, button));
    button.addEventListener("pointerup", stopLocoHold);
    button.addEventListener("pointercancel", stopLocoHold);
    button.addEventListener("lostpointercapture", stopLocoHold);
    button.addEventListener("click", (event) => {
      event.preventDefault();
    });
  });
  window.addEventListener("blur", stopLocoHold);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stopLocoHold();
  });
  updateLocoSliderLabels();
  setLocoButtons();
  loadLocoStatus();
  setInterval(() => {
    if (!document.hidden) loadLocoStatus();
  }, 15000);
}

function wristParams(mode = "absolute") {
  const commandMode = targetCommandMode(mode);
  applyAutoWristGains(commandMode);
  const targetValue = Number(els.wristTarget?.value || 0);
  return {
    mode: commandMode,
    armed: Boolean(els.wristArm?.checked),
    i_understand_risk: Boolean(els.wristRisk?.checked),
    target_q: targetValue,
    delta_q: commandMode === "relative" && mode === "absolute" ? targetValue : Number(els.wristDelta?.value || 0),
    kp: Number(els.wristKp?.value || 0),
    kd: Number(els.wristKd?.value || 0),
    duration: Number(els.wristDuration?.value || 0.35),
    period: Number(els.wristPeriod?.value || 2),
    rate: Number(els.wristRate?.value || 80),
    control_path: els.wristLowcmd?.checked ? "lowcmd" : "arm_sdk",
    auto_gains: Boolean(els.wristAutoGains?.checked),
  };
}

function targetCommandMode(mode) {
  if (mode !== "absolute") return mode;
  return els.wristTargetRelative?.checked ? "relative" : "absolute";
}

function wristSafetyReady() {
  return Boolean(els.wristArm?.checked && els.wristRisk?.checked);
}

function setWristButtons() {
  if (!els.wristSendAbsolute) return;
  const ready = wristSafetyReady();
  els.wristSendAbsolute.disabled = !ready;
  els.wristOscillate.disabled = !ready || !els.wristLowcmd?.checked;
  if (!ready && els.wristMessage) {
    els.wristMessage.textContent = "Enable both safety checkboxes before sending a wrist command.";
  }
  syncTargetMode();
}

function updateWristSliderLabels() {
  if (!els.wristTarget) return;
  const target = Number(els.wristTarget.value);
  const delta = Number(els.wristDelta.value);
  const kp = Number(els.wristKp.value);
  const kd = Number(els.wristKd.value);
  const duration = Number(els.wristDuration.value);
  const wristPeriod = Number(els.wristPeriod.value);
  const rate = Number(els.wristRate.value);
  els.wristTargetValue.textContent = target.toFixed(3);
  els.wristTargetQReadout.textContent = target.toFixed(3);
  els.wristDeltaValue.textContent = delta.toFixed(3);
  els.wristKpValue.textContent = kp.toFixed(2);
  els.wristKdValue.textContent = kd.toFixed(2);
  els.wristDurationValue.textContent = `${duration.toFixed(2)} s`;
  els.wristPeriodValue.textContent = `${wristPeriod.toFixed(2)} s`;
  els.wristRateValue.textContent = `${rate.toFixed(0)} Hz`;
}

function syncTargetMode() {
  if (!els.wristTarget || !els.wristTargetRelative) return;
  els.wristTargetRelative.disabled = false;
  const relative = Boolean(els.wristTargetRelative.checked);
  const current = Number(els.wristTarget.value || 0);
  els.wristTargetLabel.textContent = relative ? "Relative q" : "Target q";
  els.wristTarget.min = relative ? "-0.25" : "-1.2";
  els.wristTarget.max = relative ? "0.25" : "1.2";
  els.wristTarget.step = "0.005";
  if (relative && Math.abs(current) > 0.25) {
    els.wristTarget.value = "0.03";
    els.wristTarget.dataset.touched = "1";
  }
  if (relative && !els.wristTarget.dataset.touched) {
    els.wristTarget.value = "0.03";
  }
  updateWristSliderLabels();
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function computeAutoWristGains(mode = "absolute") {
  const currentQ = Number(els.wristCurrentQ?.textContent);
  const target = Number(els.wristTarget?.value || 0);
  const delta = Math.abs(Number(els.wristDelta?.value || 0));
  const period = Math.max(0.4, Number(els.wristPeriod?.value || 2));
  let kp;
  let kd;

  if (mode === "oscillate") {
    const maxTargetSpeed = (2.0 * Math.PI * delta) / period;
    kp = 6.0 + 80.0 * delta + 3.0 * maxTargetSpeed;
    kd = 0.4 + 0.08 * Math.sqrt(kp) + 0.8 * maxTargetSpeed;
  } else {
    const relativeTarget = Math.abs(Number(els.wristTarget?.value || 0));
    const error = mode === "relative" ? relativeTarget : Math.abs(target - (Number.isFinite(currentQ) ? currentQ : target));
    const x = clamp(error / 0.2, 0, 1);
    kp = 4.0 + (18.0 - 4.0) * x;
    kd = 0.28 * 2.0 * Math.sqrt(kp);
  }

  return {
    kp: clamp(kp, 4.0, 22.0),
    kd: clamp(kd, 0.35, 2.0),
  };
}

function applyAutoWristGains(mode = "absolute") {
  if (!els.wristAutoGains?.checked || !els.wristKp || !els.wristKd) return;
  const gains = computeAutoWristGains(mode);
  els.wristKp.value = String(gains.kp.toFixed(2));
  els.wristKd.value = String(gains.kd.toFixed(2));
  updateWristSliderLabels();
}

function renderWristTelemetry(snapshot) {
  if (!els.wristCurrentQ) return;
  const wrist = (snapshot.motors || []).find((motor) => motor.index === 26);
  els.wristCurrentQ.textContent = fmt(wrist?.q);
  els.wristCurrentDq.textContent = fmt(wrist?.dq);
  if (wrist && !els.wristTarget.dataset.touched) {
    const current = Math.max(-1.2, Math.min(1.2, Number(wrist.q || 0)));
    els.wristTarget.value = els.wristTargetRelative?.checked ? "0.03" : String(current);
    updateWristSliderLabels();
  }
  applyAutoWristGains(targetCommandMode("absolute"));
}

function renderWristStatus(status) {
  if (!els.wristState) return;
  const active = Boolean(status.active);
  const available = Boolean(status.available);
  els.wristState.textContent = active ? "Publishing" : available ? "Ready" : "Unavailable";
  els.wristState.className = `pill ${available ? "good" : "bad"}`;
  els.wristMessage.textContent = status.message || "--";
  const last = status.last_command;
  els.wristLastCommand.textContent = last
    ? `target ${fmt(last.target_q)} kp ${fmt(last.kp)} kd ${fmt(last.kd)}`
    : "No command sent";
}

function loadWristStatus() {
  if (!els.wristState) return;
  fetch("/api/wrist/status")
    .then((response) => response.json())
    .then(renderWristStatus)
    .catch((error) => {
      els.wristState.textContent = "Unavailable";
      els.wristState.className = "pill bad";
      els.wristMessage.textContent = error.message;
    });
}

function sendWristCommand(mode) {
  if (!els.wristState) return;
  if (!wristSafetyReady()) {
    els.wristMessage.textContent = "Command blocked: enable both safety checkboxes first.";
    els.wristState.textContent = "Blocked";
    els.wristState.className = "pill bad";
    setWristButtons();
    return;
  }
  const params = wristParams(mode);
  const currentQ = Number(els.wristCurrentQ.textContent);
  if (params.mode === "absolute" && Number.isFinite(currentQ) && Math.abs(params.target_q - currentQ) < 0.003) {
    els.wristMessage.textContent = "Target equals current q. Move the Target q slider or use Send Step.";
    els.wristState.textContent = "No delta";
    els.wristState.className = "pill good";
    return;
  }
  els.wristMessage.textContent =
    mode === "absolute"
      ? params.mode === "relative"
        ? `Sending relative q ${params.delta_q.toFixed(3)} rad`
        : `Sending target q ${params.target_q.toFixed(3)}`
      : mode === "oscillate"
        ? `Starting back/forth ${params.delta_q.toFixed(3)} rad`
        : `Sending step ${params.delta_q.toFixed(3)} rad`;
  els.wristState.textContent = "Sending";
  els.wristState.className = "pill good";
  fetch("/api/wrist/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(params),
  })
    .then((response) =>
      response.json().then((data) => {
        if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
        return data;
      }),
    )
    .then((data) => renderWristStatus(data.status || data))
    .catch((error) => {
      els.wristMessage.textContent = error.message;
      els.wristState.textContent = "Blocked";
      els.wristState.className = "pill bad";
    });
}

function stopWristCommand() {
  fetch("/api/wrist/stop", { method: "POST" })
    .then((response) => response.json())
    .then(renderWristStatus)
    .catch((error) => {
      els.wristMessage.textContent = error.message;
    });
}

function chillMotors() {
  if (!els.chillMotors) return;
  els.chillMotors.disabled = true;
  els.chillMotors.classList.add("pending");
  els.chillMotors.textContent = "Chilling";
  fetch("/api/robot/chill", { method: "POST" })
    .then((response) =>
      response.json().then((data) => {
        if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
        return data;
      }),
    )
    .then((data) => {
      els.subtitle.textContent = data.message || "Damp mode requested.";
      if (els.wristMessage) els.wristMessage.textContent = data.message || "Damp mode requested.";
    })
    .catch((error) => {
      els.subtitle.textContent = `Chill failed: ${error.message}`;
      if (els.wristMessage) els.wristMessage.textContent = error.message;
    })
    .finally(() => {
      els.chillMotors.disabled = false;
      els.chillMotors.classList.remove("pending");
      els.chillMotors.textContent = "Chill Motors";
      loadWristStatus();
    });
}

let wristLiveTimer = null;

function setupWristControls() {
  if (!els.wristTarget) return;
  const sliders = [els.wristTarget, els.wristDelta, els.wristKp, els.wristKd, els.wristDuration, els.wristRate];
  sliders.push(els.wristPeriod);
  sliders.forEach((slider) => {
    slider.addEventListener("input", () => {
      slider.dataset.touched = "1";
      if (slider === els.wristTarget) els.wristTarget.dataset.touched = "1";
      if (slider === els.wristKp || slider === els.wristKd) {
        els.wristAutoGains.checked = false;
      }
      syncTargetMode();
      applyAutoWristGains(targetCommandMode("absolute"));
      updateWristSliderLabels();
      if (els.wristLive.checked && els.wristArm.checked && els.wristRisk.checked) {
        clearTimeout(wristLiveTimer);
        wristLiveTimer = setTimeout(() => sendWristCommand("absolute"), 180);
      }
    });
  });
  els.wristSendAbsolute.addEventListener("click", () => sendWristCommand("absolute"));
  els.wristOscillate.addEventListener("click", () => sendWristCommand("oscillate"));
  els.wristStop.addEventListener("click", stopWristCommand);
  els.wristArm.addEventListener("change", setWristButtons);
  els.wristRisk.addEventListener("change", setWristButtons);
  els.wristLowcmd.addEventListener("change", setWristButtons);
  els.wristLowcmd.addEventListener("change", () => applyAutoWristGains(targetCommandMode("absolute")));
  els.wristTargetRelative.addEventListener("change", () => {
    els.wristTarget.dataset.touched = "1";
    syncTargetMode();
    applyAutoWristGains(targetCommandMode("absolute"));
  });
  els.wristAutoGains.addEventListener("change", () => applyAutoWristGains(targetCommandMode("absolute")));
  syncTargetMode();
  setWristButtons();
  loadWristStatus();
  setInterval(() => {
    if (!document.hidden) loadWristStatus();
  }, 2000);
}

function connectCameraPreview() {
  if (!els.cameraStream || !els.cameraPlaceholder) return;
  els.cameraStream.addEventListener("load", () => {
    els.cameraPlaceholder.classList.add("hidden");
  });
  els.cameraStream.addEventListener("error", () => {
    els.cameraPlaceholder.classList.remove("hidden");
  });
}

function renderRosGraph(graph) {
  if (!els.rosMap || !els.rosEdges || !els.rosSummary) return;
  const nodes = graph.nodes || [];
  const subscriptions = graph.subscriptions || [];
  const publishers = graph.publishers || [];
  const topicTypes = graph.topics || {};
  const topics = Object.keys(topicTypes).sort();
  const nodeNames = new Set(nodes.map((node) => node.name));
  const activeTopics = Array.from(
    new Set([...publishers.map((edge) => edge.topic), ...subscriptions.map((edge) => edge.topic)]),
  ).sort();
  const publisherNodes = new Set(publishers.map((edge) => edge.node));
  const subscriberNodes = new Set(subscriptions.map((edge) => edge.node));
  const graphNodes = Array.from(new Set([...publisherNodes, ...subscriberNodes, ...nodeNames])).sort();
  const hasEdges = publishers.length || subscriptions.length;
  const displayedTopics = hasEdges ? activeTopics : topics;
  const width = Math.max(980, 360 + displayedTopics.length * 24);
  const rowHeight = 78;
  const height = Math.max(520, Math.max(graphNodes.length, displayedTopics.length) * rowHeight + 80);
  const nodeXLeft = 170;
  const topicX = width / 2;
  const nodeXRight = width - 170;
  const nodeY = new Map(graphNodes.map((name, index) => [name, 60 + index * rowHeight]));
  const topicY = new Map(displayedTopics.map((name, index) => [name, 60 + index * rowHeight]));

  els.rosSummary.innerHTML = `
    <span>Nodes <strong>${fmt(nodes.length)}</strong></span>
    <span>Subs <strong>${fmt(subscriptions.length)}</strong></span>
    <span>Pubs <strong>${fmt(publishers.length)}</strong></span>
    <span>Topics <strong>${fmt(topics.length)}</strong></span>
  `;

  if (graph.error && nodes.length === 0) {
    els.rosMap.innerHTML = `<div class="ros-empty">ROS graph unavailable: ${esc(graph.error)}</div>`;
    els.rosEdges.innerHTML = "";
    return;
  }

  if (!hasEdges) {
    const topicCards = topics
      .map((name) => {
        const types = topicTypes[name] || [];
        return `
          <div class="ros-topic-card">
            <strong>${esc(name)}</strong>
            <span>${types.map(esc).join(", ") || "unknown type"}</span>
          </div>
        `;
      })
      .join("");

    els.rosMap.innerHTML = `
      <div class="ros-topic-browser">
        <div class="ros-topic-status">
          <strong>${fmt(topics.length)} topics discovered</strong>
          <span>Publisher and subscriber edges are not exposed by discovery.</span>
        </div>
        <div class="ros-topic-grid">${topicCards}</div>
      </div>
    `;
    els.rosEdges.innerHTML = topics.length
      ? topics
          .slice(0, 18)
          .map(
            (name) => `
              <div class="ros-edge">
                <strong>${esc(name)}</strong>
                <small>${esc((topicTypes[name] || []).join(", ") || "unknown type")}</small>
              </div>
            `,
          )
          .join("")
      : `<div class="ros-empty">No ROS topics discovered.</div>`;
    return;
  }

  const publisherLines = publishers
    .filter((edge) => nodeY.has(edge.node) && topicY.has(edge.topic))
    .map((edge) => {
      const y1 = nodeY.get(edge.node);
      const y2 = topicY.get(edge.topic);
      return `<path class="ros-link pub" d="M ${nodeXLeft + 126} ${y1} C ${nodeXLeft + 245} ${y1}, ${topicX - 245} ${y2}, ${topicX - 132} ${y2}"></path>`;
    })
    .join("");
  const subscriberLines = subscriptions
    .filter((edge) => nodeY.has(edge.node) && topicY.has(edge.topic))
    .map((edge) => {
      const y1 = topicY.get(edge.topic);
      const y2 = nodeY.get(edge.node);
      return `<path class="ros-link sub" d="M ${topicX + 132} ${y1} C ${topicX + 245} ${y1}, ${nodeXRight - 245} ${y2}, ${nodeXRight - 126} ${y2}"></path>`;
    })
    .join("");
  const publisherShapes = Array.from(publisherNodes)
    .sort()
    .map((name) => {
      const y = nodeY.get(name);
      return `
        <g class="ros-graph-node publisher" transform="translate(${nodeXLeft} ${y})">
          <rect x="-126" y="-24" width="252" height="48" rx="12"></rect>
          <text x="0" y="5">${esc(name)}</text>
        </g>
      `;
    })
    .join("");
  const subscriberShapes = Array.from(subscriberNodes)
    .sort()
    .map((name) => {
      const y = nodeY.get(name);
      return `
        <g class="ros-graph-node subscriber" transform="translate(${nodeXRight} ${y})">
          <rect x="-126" y="-24" width="252" height="48" rx="12"></rect>
          <text x="0" y="5">${esc(name)}</text>
        </g>
      `;
    })
    .join("");
  const topicShapes = displayedTopics
    .map((name) => {
      const y = topicY.get(name);
      return `
        <g class="ros-graph-topic" transform="translate(${topicX} ${y})">
          <rect x="-132" y="-20" width="264" height="40" rx="7"></rect>
          <text x="0" y="5">${esc(name)}</text>
        </g>
      `;
    })
    .join("");

  els.rosMap.innerHTML = `
    <svg class="ros-svg" viewBox="0 0 ${width} ${height}" role="img" aria-label="ROS graph">
      <defs>
        <marker id="rosArrowRed" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
          <path d="M 0 0 L 10 5 L 0 10 z"></path>
        </marker>
        <marker id="rosArrowWhite" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
          <path d="M 0 0 L 10 5 L 0 10 z"></path>
        </marker>
      </defs>
      <g class="ros-column-labels">
        <text x="${nodeXLeft}" y="24">Publishers</text>
        <text x="${topicX}" y="24">Topics</text>
        <text x="${nodeXRight}" y="24">Subscribers</text>
      </g>
      <g class="ros-links">${publisherLines}${subscriberLines}</g>
      <g>${topicShapes}${publisherShapes}${subscriberShapes}</g>
    </svg>
  `;

  const edgeList = [...publishers, ...subscriptions];
  els.rosEdges.innerHTML = edgeList.length
    ? edgeList
        .map(
          (edge) => `
            <div class="ros-edge">
              <strong>${esc(edge.node)}</strong>
              <span>${esc(edge.topic)}</span>
              <small>${esc(edge.type)}</small>
            </div>
          `,
        )
        .join("")
    : `<div class="ros-empty">No subscriptions discovered.</div>`;
}

function loadRosGraph() {
  if (!els.rosMap) return;
  els.rosMap.innerHTML = `<div class="ros-empty">Scanning ROS graph...</div>`;
  fetch("/api/ros-graph")
    .then((response) => response.json())
    .then(renderRosGraph)
    .catch((error) => {
      els.rosMap.innerHTML = `<div class="ros-empty">ROS graph request failed: ${esc(error.message)}</div>`;
    });
}

els.filter.addEventListener("input", () => {
  state.filter = els.filter.value;
  if (state.latest) renderMotors(motorTableRows(state.latest));
});

function syncActiveNav() {
  const activeHash = window.location.hash || "#dashboard";
  els.navItems.forEach((item) => {
    item.classList.toggle("active", item.getAttribute("href") === activeHash);
  });
}

function connectEvents() {
  if (state.events) return;
  const events = new EventSource("/events");
  state.events = events;
  events.onmessage = (event) => {
    try {
      render(JSON.parse(event.data));
    } catch (error) {
      console.error(error);
    }
  };
  events.onerror = () => {
    els.connection.textContent = "Stream lost";
    els.connection.className = "pill bad";
  };
}

function pauseEvents() {
  state.events?.close();
  state.events = null;
}

fetch("/api/state")
  .then((response) => response.json())
  .then(render)
  .catch(() => {});
window.addEventListener("hashchange", syncActiveNav);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    pauseEvents();
    stopLocoHold(false);
    return;
  }
  connectEvents();
  fetch("/api/state")
    .then((response) => response.json())
    .then(render)
    .catch(() => {});
  loadLocoStatus();
  loadWristStatus();
});
els.refreshRosGraph?.addEventListener("click", loadRosGraph);
els.chillMotors?.addEventListener("click", chillMotors);
syncActiveNav();
connectCameraPreview();
loadRosGraph();
setupLocoControls();
setupWristControls();
connectEvents();
