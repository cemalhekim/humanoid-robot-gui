const state = {
  latest: null,
  filter: "",
  events: null,
  locoStatusKey: null,
  editedPose: null,
  // True once the operator drags the arms/torso after the last file load —
  // the dragged pose then takes Move priority over the selected file.
  editedSinceLoad: false,
  recordingActive: false,
  sequenceBuilder: {
    active: false,
    points: [],
    deletedPoint: null,
    selectedIndex: null,
    playbackCurrentIndex: null,
    playbackNextIndex: null,
  },
  replay: {
    frames: [],
    index: 0,
    timer: null,
    mode: "trajectory",
    loadedFile: null,
    playing: false,
    previewComplete: false,
  },
};

const vrViewUrl = "https://10.2.100.142:8012/?ws=wss://10.2.100.142:8012";
const trajectorySampleRateHz = 60;
const trajectoryDenseMaxDt = 1 / 30;
const trajectoryMaxJointStep = 0.05;
// Doubled range: the old 100% (2.5) is the new 50% and the default; the new
// 100% (5.0) drives the PID at twice the old top aggressiveness (overdrive).
const replayResponseMax = 5.0;
const replayResponseDefault = 2.5;
const currentRobotPoseValue = "__current_robot_pose__";
const sequenceDraftStorageKey = "h1_sequence_builder_draft_v1";
const fallbackBodyJointNames = [
  "LeftHipYaw",
  "LeftHipPitch",
  "LeftHipRoll",
  "LeftKnee",
  "LeftAnklePitch",
  "LeftAnkleRoll",
  "RightHipYaw",
  "RightHipPitch",
  "RightHipRoll",
  "RightKnee",
  "RightAnklePitch",
  "RightAnkleRoll",
  "WaistYaw",
  "LeftShoulderPitch",
  "LeftShoulderRoll",
  "LeftShoulderYaw",
  "LeftElbow",
  "LeftWristRoll",
  "LeftWristPitch",
  "LeftWristYaw",
  "RightShoulderPitch",
  "RightShoulderRoll",
  "RightShoulderYaw",
  "RightElbow",
  "RightWristRoll",
  "RightWristPitch",
  "RightWristYaw",
];

const els = {
  subtitle: document.getElementById("subtitle"),
  connection: document.getElementById("connection"),
  age: document.getElementById("age"),
  rate: document.getElementById("rate"),
  chillMotors: document.getElementById("chillMotors"),
  straightRobot: document.getElementById("straightRobot"),
  homeRobot: document.getElementById("homeRobot"),
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
  recordingPage: document.getElementById("recordingPage"),
  recordingLayout: document.querySelector("#recordingPage .recording-layout"),
  sequenceBuilder: document.getElementById("sequenceBuilder"),
  sequenceBuilderClose: document.getElementById("sequenceBuilderClose"),
  sequenceBuilderStatus: document.getElementById("sequenceBuilderStatus"),
  sequenceUndoDelete: document.getElementById("sequenceUndoDelete"),
  sequencePointList: document.getElementById("sequencePointList"),
  endEffectorStatus: document.getElementById("endEffectorStatus"),
  collisionDebugToggle: document.getElementById("collisionDebugToggle"),
  elbowTargetsToggle: document.getElementById("elbowTargetsToggle"),
  recordingSequenceToggle: document.getElementById("recordingSequenceToggle"),
  recordingSaveSequence: document.getElementById("recordingSaveSequence"),
  recordingRobotMotionToggle: document.getElementById("recordingRobotMotionToggle"),
  recordingCapturePose: document.getElementById("recordingCapturePose"),
  recordingSamples: document.getElementById("recordingSamples"),
  recordingEvents: document.getElementById("recordingEvents"),
  recordingElapsed: document.getElementById("recordingElapsed"),
  recordingBytes: document.getElementById("recordingBytes"),
  recordingFile: document.getElementById("recordingFile"),
  recordingPath: document.getElementById("recordingPath"),
  recordingLastSample: document.getElementById("recordingLastSample"),
  recordingError: document.getElementById("recordingError"),
  recordingFileSelect: document.getElementById("recordingFileSelect"),
  recordingRename: document.getElementById("recordingRename"),
  recordingPlay: document.getElementById("recordingPlay"),
  recordingRobotPlay: document.getElementById("recordingRobotPlay"),
  recordingReplayResponse: document.getElementById("recordingReplayResponse"),
  recordingReplayResponseValue: document.getElementById("recordingReplayResponseValue"),
  recordingReplayPid: document.getElementById("recordingReplayPid"),
  recordingMirrorArms: document.getElementById("recordingMirrorArms"),
  recordingScrub: document.getElementById("recordingScrub"),
  recordingReplayFrame: document.getElementById("recordingReplayFrame"),
  recordingReplayTime: document.getElementById("recordingReplayTime"),
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
  locoBalanceModeStatic: document.getElementById("locoBalanceModeStatic"),
  locoBalanceModeGait: document.getElementById("locoBalanceModeGait"),
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
  teleoperationMethods: document.querySelectorAll("[data-teleoperation-method]"),
  chatLog: document.getElementById("chatLog"),
  chatForm: document.getElementById("chatForm"),
  chatInput: document.getElementById("chatInput"),
  chatSend: document.getElementById("chatSend"),
  chatStatus: document.getElementById("chatStatus"),
  chatHint: document.getElementById("chatHint"),
  mimicFileInput: document.getElementById("mimicFileInput"),
  mimicAttach: document.getElementById("mimicAttach"),
  mimicPreview: document.getElementById("mimicPreview"),
  mimicThumb: document.getElementById("mimicThumb"),
  mimicClear: document.getElementById("mimicClear"),
};

function fmt(value, suffix = "") {
  if (value === undefined || value === null || value === "") return "--";
  if (Array.isArray(value)) return `[${value.map((item) => fmt(item)).join(", ")}]`;
  if (typeof value === "number") return `${Number.isInteger(value) ? value : value.toFixed(3)}${suffix}`;
  if (typeof value === "boolean") return value ? "true" : "false";
  return String(value);
}

function fmtBytes(value) {
  const bytes = Number(value || 0);
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  if (bytes < 1024 * 1024 * 1024) return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
  return `${(bytes / (1024 * 1024 * 1024)).toFixed(2)} GB`;
}

function esc(value) {
  return String(value ?? "--")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");
}

function valueList(values, labels) {
  return (values || []).map((value, index) => ({
    label: labels?.[index] || String(index),
    value,
  }));
}

// Only touch the DOM when the built HTML actually differs from what the node
// already holds — the 5 Hz telemetry tick rebuilt these strings unconditionally,
// forcing a reparse + style recalc + layout even when the values were identical.
function _setHtmlIfChanged(node, html) {
  if (node._lastHtml === html) return;
  node._lastHtml = html;
  node.innerHTML = html;
}

function renderMetricCards(node, cards, rows = []) {
  _setHtmlIfChanged(node, `
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
  `);
}

function renderStatusList(node, rows) {
  _setHtmlIfChanged(node, `<dl class="status-list full">${rows
    .filter((row) => row.value !== undefined)
    .map((row) => `<dt>${row.label}</dt><dd>${fmt(row.value)}</dd>`)
    .join("")}</dl>`);
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

  _setHtmlIfChanged(els.motorRows, rows
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
    .join(""));
}

function csvCell(value) {
  if (value === undefined || value === null) return "";
  const text = Array.isArray(value) ? value.join("|") : String(value);
  return /[",\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

function snapshotToCsv(snapshot) {
  const headers = [
    "sample",
    "timestamp",
    "source",
    "index",
    "name",
    "mode",
    "q",
    "dq",
    "ddq",
    "tau",
    "tau_est",
    "temperature",
    "vol",
  ];
  const rows = [headers];
  for (const motor of snapshot.motors || []) {
    rows.push([
      snapshot.samples ?? snapshot.sample ?? "",
      snapshot.timestamp ?? "",
      "body",
      motor.index,
      motor.name,
      motor.mode,
      motor.q,
      motor.dq,
      motor.ddq,
      motor.tau,
      motor.tau_est,
      motor.temperature,
      motor.vol,
    ]);
  }
  for (const joint of snapshot.hands?.joints || []) {
    rows.push([
      snapshot.samples ?? snapshot.sample ?? "",
      snapshot.timestamp ?? "",
      "hand",
      joint.index,
      joint.name,
      joint.mode,
      joint.q,
      joint.dq,
      joint.ddq,
      joint.tau,
      joint.tau_est,
      joint.temperature,
      joint.vol,
    ]);
  }
  if (rows.length === 1) {
    rows.push([
      snapshot.samples ?? snapshot.sample ?? "",
      snapshot.timestamp ?? "",
      "status",
      "",
      "connected",
      "",
      snapshot.connected,
      "",
      "",
      "",
      "",
      "",
      "",
    ]);
    if (snapshot.error) {
      rows.push([
        snapshot.samples ?? snapshot.sample ?? "",
        snapshot.timestamp ?? "",
        "status",
        "",
        "error",
        "",
        snapshot.error,
        "",
        "",
        "",
        "",
        "",
        "",
      ]);
    }
    rows.push([
      snapshot.samples ?? snapshot.sample ?? "",
      snapshot.timestamp ?? "",
      "hand",
      "",
      "connected",
      "",
      snapshot.hands?.connected,
      "",
      "",
      "",
      "",
      "",
      "",
    ]);
  }
  return rows.map((row) => row.map(csvCell).join(",")).join("\n");
}

function networkLabel(network) {
  const type = network?.type || "Network";
  const iface = network?.interface && network.interface !== "unknown" ? ` (${network.interface})` : "";
  return `${type}${iface}`;
}

const _renderSig = {};

function renderNetwork(network) {
  // Network state changes on the order of seconds; skip the 6 textContent
  // writes when nothing changed (server also caches network_status for 5s).
  const sig = JSON.stringify(network || null);
  if (sig === _renderSig.network) return;
  _renderSig.network = sig;
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

  if (els.age) {
    els.age.textContent = `age ${age === null ? "--" : age.toFixed(1)}s`;
  }
  els.rate.textContent = `${fmt(snapshot.sample_rate_hz)} Hz`;
  els.subtitle.textContent = connected
    ? `${fmt(snapshot.motor_count)} motors, ${fmt(snapshot.samples)} samples`
    : snapshot.error || "Waiting for rt/lowstate";
  renderNetwork(snapshot.network);

  // Perf: only rebuild the DOM of the page the operator is actually looking
  // at. The motor table (35 rows of innerHTML) and the CSV dump used to be
  // rebuilt on EVERY 200 ms telemetry message even while hidden. Each page
  // re-renders from state.latest the moment it becomes active (see the
  // telemetry-tab-change listener).
  const activeHash = window.location.hash || "#dashboard";
  if (activeHash === "#dashboard" || activeHash === "") renderRobotStatus(snapshot);
  if (activeHash === "#motorPage") renderMotors(motorTableRows(snapshot));
  if (activeHash === "#wristPage") renderWristTelemetry(snapshot);
  if (activeHash === "#locoPage") {
    renderLocoTelemetry(snapshot);
    if (snapshot.loco) renderLocoStatus(snapshot.loco);
  }
  if (activeHash === "#rawView") els.rawJson.textContent = snapshotToCsv(snapshot);
  updateRobotMotionToggle();
  window.dispatchEvent(new CustomEvent("telemetry-state", { detail: { snapshot } }));
}

function updateRobotMotionToggle() {
  const button = els.recordingRobotMotionToggle;
  if (!button) return;
  const connected = Boolean(state.latest?.connected);
  const active = Boolean(state.recordingActive);
  button.innerHTML = active
    ? '<svg viewBox="0 0 24 24" aria-hidden="true"><circle cx="12" cy="12" r="8.5"/><rect x="8.6" y="8.6" width="6.8" height="6.8" rx="1" fill="currentColor" stroke="none"/></svg>'
    : '<svg viewBox="0 0 24 24" aria-hidden="true"><circle cx="12" cy="12" r="8.5"/><circle cx="12" cy="12" r="3.6" fill="currentColor" stroke="none"/></svg>';
  button.setAttribute("aria-label", active ? "Stop Saving Robot Motion" : "Save Real Robot Motion");
  button.disabled = !active && !connected;
  button.classList.remove("emergency-button");
  button.classList.toggle("ghost-button", !active);
  button.classList.toggle("chill-button", active);
  button.title = active
    ? "Stop recording live robot telemetry."
    : connected
      ? "Record live robot telemetry from the real robot."
      : "Locked until live robot telemetry is connected.";
}

function renderRecordingStatus(status) {
  const active = Boolean(status?.active);
  state.recordingActive = active;
  if (els.recordingSamples) els.recordingSamples.textContent = fmt(status?.samples ?? 0);
  if (els.recordingEvents) els.recordingEvents.textContent = fmt(status?.events ?? 0);
  if (els.recordingElapsed) els.recordingElapsed.textContent = `${Number(status?.elapsed_seconds || 0).toFixed(1)} s`;
  if (els.recordingBytes) els.recordingBytes.textContent = fmtBytes(status?.bytes_written);
  if (els.recordingFile) els.recordingFile.textContent = status?.filename || "--";
  if (els.recordingPath) els.recordingPath.textContent = status?.path || "--";
  if (els.recordingLastSample) {
    els.recordingLastSample.textContent = status?.last_sample_at
      ? new Date(status.last_sample_at * 1000).toLocaleTimeString()
      : "--";
  }
  if (els.recordingError) els.recordingError.textContent = status?.last_error || "--";
  updateRobotMotionToggle();
}

function renderRecordingFiles(files) {
  if (!els.recordingFileSelect) return;
  const current = els.recordingFileSelect.value;
  const fileOptions = (files || [])
    .map((file) => `<option value="${esc(file.name)}">${esc(file.name)} (${fmtBytes(file.size)})</option>`)
    .join("");
  els.recordingFileSelect.innerHTML = `<option value="${currentRobotPoseValue}">Current Robot Pose</option>${fileOptions}`;
  if (current && [...els.recordingFileSelect.options].some((option) => option.value === current)) {
    els.recordingFileSelect.value = current;
  }
  els.recordingFileSelect.disabled = false;
  updateRenameButton();
  return els.recordingFileSelect.value;
}

function updateRenameButton() {
  if (!els.recordingRename) return;
  els.recordingRename.disabled = !isRobotReplayFileName(els.recordingFileSelect?.value || "");
}

async function loadRecordingFiles({ loadSelected = false } = {}) {
  if (!els.recordingFileSelect) return;
  try {
    const response = await fetch("/api/recording/files");
    const payload = await response.json();
    // Perf: the list is polled every 5 s but rarely changes — skip the DOM
    // rebuild (and selection churn) when nothing did.
    const signature = (payload.files || []).map((f) => `${f.name}:${f.modified_at}`).join("|");
    if (!loadSelected && signature === state.recordingFilesSignature) return;
    state.recordingFilesSignature = signature;
    const selected = renderRecordingFiles(payload.files || []);
    if (loadSelected && selected && selected !== state.replay.loadedFile && !state.replay.playing) {
      await loadReplayRecording();
    }
  } catch (error) {
    if (els.recordingError) {
      els.recordingError.textContent = error instanceof Error ? error.message : "Could not list recordings.";
    }
  }
}

function recordingFrameToSnapshot(record) {
  const body = record.body || {};
  const hands = record.hands || {};
  const motors = body.motors || record.motors || [];
  return {
    connected: false,
    timestamp: record.timestamp,
    sample: record.sample,
    samples: record.sample,
    sample_rate_hz: 0,
    motor_count: record.motor_count ?? motors.length,
    motors,
    imu: body.imu || record.imu || {},
    robot: body.robot || record.robot || {},
    battery: body.battery || record.battery || {},
    foot_force: body.foot_force || record.foot_force || [],
    foot_force_est: body.foot_force_est || record.foot_force_est || [],
    hands: {
      ...hands,
      joint_count: hands.joint_count ?? hands.joints?.length ?? 0,
      joints: hands.joints || [],
    },
  };
}

function cloneSnapshot(snapshot) {
  return JSON.parse(JSON.stringify(snapshot || {}));
}

function fallbackCurrentPoseSnapshot() {
  const timestamp = Date.now() / 1000;
  const motors = fallbackBodyJointNames.map((name, index) => ({
    index,
    name,
    mode: 1,
    q: 0,
    dq: 0,
    ddq: 0,
    tau_est: 0,
    temperature: [],
    vol: null,
    sensor: [],
    reserve: 0,
  }));
  return {
    connected: false,
    synthetic: true,
    offline_fallback: true,
    source: "offline_current_robot_pose",
    timestamp,
    sample: 0,
    samples: 0,
    sample_rate_hz: 0,
    motor_count: motors.length,
    motors,
    imu: {},
    robot: {},
    battery: {},
    foot_force: [],
    foot_force_est: [],
    hands: {
      connected: false,
      joint_count: 0,
      joints: [],
    },
  };
}

function currentPoseSnapshot() {
  return hasBodyMotors(state.latest) ? cloneSnapshot(state.latest) : fallbackCurrentPoseSnapshot();
}

function interpolateValue(start, target, t) {
  const a = Number(start);
  const b = Number(target);
  if (!Number.isFinite(a)) return Number.isFinite(b) ? b : null;
  if (!Number.isFinite(b)) return a;
  return a + (b - a) * t;
}

function interpolateMotors(startMotors = [], targetMotors = [], t) {
  const startByKey = new Map(startMotors.map((motor) => [motorKey(motor), motor]));
  return targetMotors.map((target) => {
    const start = startByKey.get(motorKey(target)) || target;
    return {
      ...target,
      q: interpolateValue(start.q, target.q, t),
      dq: interpolateValue(start.dq, target.dq, t),
      tau_est: interpolateValue(start.tau_est, target.tau_est, t),
    };
  });
}

function motorKey(motor) {
  return motor?.index ?? motor?.name ?? "";
}

function maxMotorDelta(startMotors = [], targetMotors = []) {
  const startByKey = new Map(startMotors.map((motor) => [motorKey(motor), motor]));
  return targetMotors.reduce((maxDelta, target) => {
    const start = startByKey.get(motorKey(target));
    const a = Number(start?.q);
    const b = Number(target?.q);
    if (!Number.isFinite(a) || !Number.isFinite(b)) return maxDelta;
    return Math.max(maxDelta, Math.abs(b - a));
  }, 0);
}

function maxSnapshotDelta(start, target) {
  return Math.max(
    maxMotorDelta(start?.motors || [], target?.motors || []),
    maxMotorDelta(start?.hands?.joints || [], target?.hands?.joints || []),
  );
}

function hasBodyMotors(snapshot) {
  return Array.isArray(snapshot?.motors) && snapshot.motors.length > 0;
}

function neutralizeMotor(motor) {
  return {
    ...motor,
    q: 0,
    dq: 0,
    tau_est: 0,
  };
}

function neutralStartFromTarget(target) {
  const targetHands = target.hands || {};
  return {
    ...target,
    sample: 0,
    samples: 0,
    motors: (target.motors || []).map(neutralizeMotor),
    hands: {
      ...targetHands,
      joints: (targetHands.joints || []).map(neutralizeMotor),
    },
  };
}

function buildApproachFrames(targetSnapshot) {
  const target = cloneSnapshot(recordingFrameToSnapshot(targetSnapshot));
  const start = cloneSnapshot(hasBodyMotors(state.latest) ? state.latest : neutralStartFromTarget(target));
  const frameCount = 120;
  const startTime = Date.now() / 1000;
  const startHands = start.hands || {};
  const targetHands = target.hands || {};
  return Array.from({ length: frameCount }, (_, index) => {
    const t = frameCount === 1 ? 1 : index / (frameCount - 1);
    return {
      ...target,
      type: "telemetry_sample",
      timestamp: startTime + t * 3,
      sample: index + 1,
      motor_count: target.motor_count ?? target.motors?.length ?? 0,
      motors: interpolateMotors(start.motors || [], target.motors || [], t),
      hands: {
        ...targetHands,
        joint_count: targetHands.joint_count ?? targetHands.joints?.length ?? 0,
        joints: interpolateMotors(startHands.joints || [], targetHands.joints || [], t),
      },
    };
  });
}

function retimeSnapshot(snapshot, timestamp, sample) {
  return {
    ...snapshot,
    timestamp,
    sample,
    samples: sample,
  };
}

function interpolateSnapshot(start, target, t, timestamp, sample) {
  const startHands = start.hands || {};
  const targetHands = target.hands || {};
  return {
    ...target,
    timestamp,
    sample,
    samples: sample,
    motors: interpolateMotors(start.motors || [], target.motors || [], t),
    hands: {
      ...targetHands,
      joint_count: targetHands.joint_count ?? targetHands.joints?.length ?? 0,
      joints: interpolateMotors(startHands.joints || [], targetHands.joints || [], t),
    },
  };
}

function segmentDuration(start, target) {
  const startTime = Number(start?.timestamp);
  const targetTime = Number(target?.timestamp);
  if (Number.isFinite(startTime) && Number.isFinite(targetTime) && targetTime > startTime) {
    return targetTime - startTime;
  }
  return 1 / trajectorySampleRateHz;
}

function adaptiveSequenceFrames(sequence, sequenceStart) {
  if (!sequence.length) return [];
  const frames = [
    {
      snapshot: retimeSnapshot(sequence[0], sequenceStart, 1),
      currentPointIndex: 0,
      nextPointIndex: sequence.length > 1 ? 1 : null,
    },
  ];
  let previousSource = sequence[0];
  let previousTimestamp = sequenceStart;

  for (let index = 1; index < sequence.length; index += 1) {
    const target = sequence[index];
    const duration = segmentDuration(previousSource, target);
    const maxDelta = maxSnapshotDelta(previousSource, target);
    const denseEnough = duration <= trajectoryDenseMaxDt && maxDelta <= trajectoryMaxJointStep;
    const steps = denseEnough
      ? 1
      : Math.max(
          2,
          Math.ceil(duration * trajectorySampleRateHz),
          Math.ceil(maxDelta / trajectoryMaxJointStep),
        );

    for (let step = 1; step <= steps; step += 1) {
      const t = step / steps;
      const timestamp = previousTimestamp + duration * t;
      const sample = frames.length + 1;
      const atTarget = step === steps;
      frames.push({
        snapshot: atTarget
          ? retimeSnapshot(target, timestamp, sample)
          : interpolateSnapshot(previousSource, target, t, timestamp, sample),
        currentPointIndex: atTarget ? index : index - 1,
        nextPointIndex: atTarget ? (index + 1 < sequence.length ? index + 1 : null) : index,
      });
    }

    previousSource = target;
    previousTimestamp += duration;
  }

  return frames;
}

function frameTimestamp(frame) {
  return frame?.trajectory?.timestamp ?? frame?.target?.timestamp ?? frame?.timestamp ?? 0;
}

function timelineFrame(trajectory, target, phase, currentPointIndex = null, nextPointIndex = null) {
  return {
    trajectory,
    target,
    phase,
    currentPointIndex,
    nextPointIndex,
    timestamp: trajectory.timestamp ?? target.timestamp,
  };
}

function buildTrajectoryTimeline(records) {
  const sequence = records.map(recordingFrameToSnapshot).filter((snapshot) => snapshot.motors.length > 0);
  if (!sequence.length) return [];

  const firstTarget = sequence[0];
  const approach = buildApproachFrames(firstTarget);
  const sequenceStart = (approach.at(-1)?.timestamp ?? Date.now() / 1000) + 1 / 60;
  const retimedSequence = adaptiveSequenceFrames(sequence, sequenceStart);

  return [
    ...approach.map((trajectory) => timelineFrame(trajectory, firstTarget, "approach", null, 0)),
    ...retimedSequence.map(({ snapshot, currentPointIndex, nextPointIndex }) =>
      timelineFrame(snapshot, snapshot, "sequence", currentPointIndex, nextPointIndex),
    ),
  ];
}

function parseRecordingSnapshots(name, text) {
  if (name.endsWith(".jsonl")) {
    return text
      .split(/\n+/)
      .filter(Boolean)
      .map((line) => JSON.parse(line))
      .filter((record) => record.type === "telemetry_sample");
  }

  const data = JSON.parse(text);
  if (Array.isArray(data)) return data;
  if (Array.isArray(data.points)) return data.points;
  if (Array.isArray(data.frames)) return data.frames;
  if (Array.isArray(data.snapshots)) return data.snapshots;
  if (Array.isArray(data.trajectory)) return data.trajectory;
  if (data.snapshot) return [data.snapshot];
  return [data];
}

function updateReplayUi() {
  const total = state.replay.frames.length;
  const builderPointCount = state.sequenceBuilder.active ? state.sequenceBuilder.points.length : 0;
  const frameNumber = total ? state.replay.index + 1 : 0;
  if (els.recordingScrub) {
    els.recordingScrub.max = String(Math.max(0, total - 1));
    els.recordingScrub.value = String(Math.min(state.replay.index, Math.max(0, total - 1)));
    els.recordingScrub.disabled = total === 0;
  }
  if (els.recordingReplayFrame) els.recordingReplayFrame.textContent = `${frameNumber} / ${total}`;
  const frame = state.replay.frames[state.replay.index];
  if (els.recordingReplayTime) {
    const timestamp = frameTimestamp(frame);
    els.recordingReplayTime.textContent = timestamp ? new Date(timestamp * 1000).toLocaleTimeString() : "--";
  }
  if (els.recordingPlay) {
    els.recordingPlay.disabled = (total === 0 && builderPointCount === 0) || state.replay.playing;
  }
  if (els.recordingRobotPlay) {
    const moveTarget = pendingMoveTarget();
    const canMoveArms = Boolean(state.latest?.connected) && moveTarget !== null;
    els.recordingRobotPlay.disabled = !canMoveArms;
    els.recordingRobotPlay.title = robotReplayLockReason({ canMoveArms, moveTarget });
  }
}

function isRobotReplayFileName(name) {
  return Boolean(name && (name.endsWith(".jsonl") || name.endsWith(".pose.json") || name.endsWith(".sequence.json")));
}

// Resolve what "Move" should send, in priority order. Saving is optional:
// a saved+loaded file wins, otherwise an unsaved sequence draft, otherwise the
// pose the operator dragged in the 3D editor. Returns null when nothing is movable.
function pendingMoveTarget() {
  if (state.sequenceBuilder.active && state.sequenceBuilder.points.length > 0) {
    return { kind: "sequence", points: state.sequenceBuilder.points };
  }
  // A drag since the last file load wins: Move goes to the pose the operator
  // just made in the editor (sent inline; the server replays it via a hidden
  // temp file it deletes right after — no Save needed, nothing accumulates).
  if (state.editedSinceLoad && hasBodyMotors(state.editedPose)) {
    return { kind: "pose", snapshot: state.editedPose };
  }
  const selectedReplayFile = els.recordingFileSelect?.value || "";
  if (isRobotReplayFileName(selectedReplayFile) && state.replay.loadedFile === selectedReplayFile) {
    return { kind: "file", filename: selectedReplayFile };
  }
  if (hasBodyMotors(state.editedPose)) {
    return { kind: "pose", snapshot: state.editedPose };
  }
  return null;
}

function robotReplayLockReason({ canMoveArms, moveTarget }) {
  if (canMoveArms) {
    return moveTarget?.kind === "file"
      ? "Publish the validated arm/waist trajectory through arm_sdk."
      : "Move the robot to the pose/sequence you edited (saving is optional).";
  }
  if (!state.latest?.connected) return "Locked until live robot telemetry is connected.";
  return "Drag the arms or torso in the 3D editor, or load a saved file, before moving the robot.";
}

function replayResponseValue() {
  const value = Number(els.recordingReplayResponse?.value ?? replayResponseDefault);
  if (!Number.isFinite(value)) return replayResponseDefault;
  return Math.max(0, Math.min(replayResponseMax, value));
}

function replayResponseLabel(value = replayResponseValue()) {
  if (value < 0.7) return "Damped";
  if (value > 4.0) return "Overdrive";
  if (value > 2.5) return "Responsive";
  return "Balanced";
}

function updateReplayResponseLabel() {
  if (!els.recordingReplayResponseValue) return;
  const value = replayResponseValue();
  const percent = Math.round((value / replayResponseMax) * 100);
  els.recordingReplayResponseValue.textContent = `${replayResponseLabel(value)} ${percent}%`;
}

function replayClosedLoopEnabled() {
  return els.recordingReplayPid?.checked !== false;
}

function showReplayFrame(index) {
  if (!state.replay.frames.length) {
    updateReplayUi();
    return;
  }
  state.replay.index = Math.max(0, Math.min(index, state.replay.frames.length - 1));
  const frame = state.replay.frames[state.replay.index];
  updateSequencePlaybackHighlight(frame);
  window.dispatchEvent(new CustomEvent("recording-replay-target", { detail: { snapshot: frame.target } }));
  window.dispatchEvent(new CustomEvent("recording-trajectory-frame", { detail: { snapshot: frame.trajectory } }));
  updateReplayUi();
}

function updateSequencePlaybackHighlight(frame) {
  if (!state.sequenceBuilder.active || state.replay.loadedFile !== "__sequence_builder__") return;
  const currentIndex = Number.isInteger(frame?.currentPointIndex) ? frame.currentPointIndex : null;
  const nextIndex = Number.isInteger(frame?.nextPointIndex) ? frame.nextPointIndex : null;
  if (
    state.sequenceBuilder.playbackCurrentIndex === currentIndex &&
    state.sequenceBuilder.playbackNextIndex === nextIndex
  ) {
    return;
  }
  state.sequenceBuilder.playbackCurrentIndex = currentIndex;
  state.sequenceBuilder.playbackNextIndex = nextIndex;
  renderSequenceBuilder();
}

function refreshReplayTarget() {
  if (!state.replay.frames.length) return;
  const frame = state.replay.frames[state.replay.index] || state.replay.frames[0];
  if (!frame?.target) return;
  window.dispatchEvent(new CustomEvent("recording-replay-target", { detail: { snapshot: frame.target } }));
  if (frame.trajectory) {
    window.dispatchEvent(new CustomEvent("recording-trajectory-frame", { detail: { snapshot: frame.trajectory } }));
  }
}

function pauseReplay() {
  if (state.replay.timer) window.clearTimeout(state.replay.timer);
  state.replay.timer = null;
  state.replay.playing = false;
  // The green ghost is a simulation-only aid — hide it whenever playback stops.
  window.dispatchEvent(new CustomEvent("recording-trajectory-visibility", { detail: { visible: false } }));
  updateReplayUi();
}

function markReplayComplete() {
  state.replay.playing = false;
  state.replay.previewComplete = state.replay.frames.length > 0;
  window.dispatchEvent(new CustomEvent("recording-trajectory-visibility", { detail: { visible: false } }));
  updateReplayUi();
}

function scheduleReplayNext() {
  if (!state.replay.playing) return;
  if (state.replay.index >= state.replay.frames.length - 1) {
    markReplayComplete();
    return;
  }
  const current = state.replay.frames[state.replay.index];
  const next = state.replay.frames[state.replay.index + 1];
  const deltaMs = Math.max(8, Math.min(1000, (frameTimestamp(next) - frameTimestamp(current)) * 1000));
  state.replay.timer = window.setTimeout(() => {
    showReplayFrame(state.replay.index + 1);
    scheduleReplayNext();
  }, Number.isFinite(deltaMs) ? deltaMs : 33);
}

function playReplay() {
  if (state.sequenceBuilder.active && state.sequenceBuilder.points.length) {
    state.sequenceBuilder.selectedIndex = null;
    state.replay.frames = buildTrajectoryTimeline(state.sequenceBuilder.points);
    state.replay.index = 0;
    state.replay.loadedFile = "__sequence_builder__";
    state.replay.previewComplete = false;
  }
  if (!state.replay.frames.length) return;
  if (state.replay.index >= state.replay.frames.length - 1) state.replay.index = 0;
  state.replay.previewComplete = false;
  state.replay.playing = true;
  window.dispatchEvent(new CustomEvent("recording-trajectory-visibility", { detail: { visible: true } }));
  showReplayFrame(state.replay.index);
  scheduleReplayNext();
}

async function requestRobotReplay() {
  const moveTarget = pendingMoveTarget();
  if (!moveTarget) {
    if (els.recordingError) {
      els.recordingError.textContent =
        "Drag the arms or torso in the 3D editor, or load a saved file, before moving the robot.";
    }
    updateReplayUi();
    return;
  }
  const body = {
    execute_arm_sdk: true,
    command_scope: "arms",
    closed_loop: replayClosedLoopEnabled(),
    hold_after_convergence: true,
    position_tolerance_rad: 0.01,
    replay_response: replayResponseValue(),
  };
  if (moveTarget.kind === "file") {
    // Saved recording/pose/sequence loaded from disk.
    body.filename = moveTarget.filename;
  } else if (moveTarget.kind === "sequence") {
    // Unsaved sequence draft — sent inline, replayed via an ephemeral file server-side.
    body.points = moveTarget.points.map(compactSequencePoint);
  } else {
    // Unsaved single pose dragged in the 3D editor — sent inline.
    body.snapshot = moveTarget.snapshot;
  }
  try {
    const response = await fetch("/api/recording/replay/robot", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) throw new Error(payload.error || "Robot replay request was rejected.");
    if (els.recordingError) els.recordingError.textContent = payload.message || "Robot replay accepted.";
  } catch (error) {
    if (els.recordingError) {
      els.recordingError.textContent = error instanceof Error ? error.message : "Robot replay request failed.";
    }
  }
}

async function loadReplayRecording() {
  if (!els.recordingFileSelect?.value) return;
  updateRenameButton();
  pauseReplay();
  try {
    const name = els.recordingFileSelect.value;
    const snapshots =
      name === currentRobotPoseValue
        ? [currentPoseSnapshot()]
        : await loadRecordingSnapshots(name);
    state.replay.mode = "trajectory";
    state.replay.frames = buildTrajectoryTimeline(snapshots);
    state.replay.index = 0;
    state.replay.loadedFile = name;
    state.replay.previewComplete = false;
    // Loading a file makes it the Move target again — until the next drag.
    state.editedSinceLoad = false;
    const target = state.replay.frames[0]?.target;
    if (target) {
      refreshReplayTarget();
      window.dispatchEvent(new CustomEvent("recording-trajectory-visibility", { detail: { visible: false } }));
    }
    updateReplayUi();
    if (els.recordingError) {
      els.recordingError.textContent = state.replay.frames.length
        ? "--"
        : "Recording loaded, but it does not contain trajectory points.";
    }
  } catch (error) {
    state.replay.frames = [];
    state.replay.index = 0;
    state.replay.mode = "trajectory";
    state.replay.loadedFile = null;
    state.replay.previewComplete = false;
    window.dispatchEvent(new CustomEvent("recording-trajectory-visibility", { detail: { visible: false } }));
    updateReplayUi();
    if (els.recordingError) {
      els.recordingError.textContent = error instanceof Error ? error.message : "Replay load failed.";
    }
  }
}

async function loadRecordingSnapshots(name) {
  const response = await fetch(`/api/recording/files/${encodeURIComponent(name)}`);
  if (!response.ok) throw new Error(`Could not load ${name}`);
  const text = await response.text();
  return parseRecordingSnapshots(name, text);
}

async function loadRecordingStatus() {
  try {
    const response = await fetch("/api/recording/status");
    renderRecordingStatus(await response.json());
  } catch (error) {
    if (els.recordingError) {
      els.recordingError.textContent = error instanceof Error ? error.message : "Status request failed.";
    }
  }
}

async function startRecording() {
  if (!els.recordingRobotMotionToggle || !state.latest?.connected) return;
  els.recordingRobotMotionToggle.disabled = true;
  try {
    const response = await fetch("/api/recording/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ label: "h1_2_full_body_hands" }),
    });
    const payload = await response.json();
    renderRecordingStatus(payload.status || payload);
  } catch (error) {
    if (els.recordingError) els.recordingError.textContent = error instanceof Error ? error.message : "Start failed.";
  } finally {
    loadRecordingStatus();
    loadRecordingFiles();
  }
}

function sequencePointSnapshot() {
  return cloneSnapshot(state.editedPose || currentPoseSnapshot());
}

function saveSequenceDraft() {
  window.localStorage?.setItem(sequenceDraftStorageKey, JSON.stringify(state.sequenceBuilder.points));
}

function loadSequenceDraft() {
  try {
    const points = JSON.parse(window.localStorage?.getItem(sequenceDraftStorageKey) || "[]");
    state.sequenceBuilder.points = Array.isArray(points) ? points.filter(hasBodyMotors) : [];
  } catch {
    state.sequenceBuilder.points = [];
  }
  state.sequenceBuilder.selectedIndex = null;
  state.sequenceBuilder.playbackCurrentIndex = null;
  state.sequenceBuilder.playbackNextIndex = null;
}

function clearSequenceDraft() {
  state.sequenceBuilder.points = [];
  state.sequenceBuilder.deletedPoint = null;
  state.sequenceBuilder.selectedIndex = null;
  state.sequenceBuilder.playbackCurrentIndex = null;
  state.sequenceBuilder.playbackNextIndex = null;
  window.localStorage?.removeItem(sequenceDraftStorageKey);
}

function sequencePointDuration(point, index) {
  if (index === 0) return "base point";
  const previous = state.sequenceBuilder.points[index - 1];
  const currentTime = Number(point.timestamp);
  const previousTime = Number(previous?.timestamp);
  if (!Number.isFinite(currentTime) || !Number.isFinite(previousTime) || currentTime <= previousTime) {
    return "auto duration";
  }
  return `+${(currentTime - previousTime).toFixed(2)} s`;
}

function renderSequenceBuilder() {
  if (state.sequenceBuilder.active && state.replay.loadedFile === "__sequence_builder__" && !state.replay.playing) {
    state.replay.frames = state.sequenceBuilder.points.length
      ? buildTrajectoryTimeline(state.sequenceBuilder.points)
      : [];
    state.replay.index = 0;
    state.replay.previewComplete = false;
  }
  els.recordingPage?.classList.toggle("sequence-mode", state.sequenceBuilder.active);
  els.recordingLayout?.classList.toggle("sequence-active", state.sequenceBuilder.active);
  els.sequenceBuilder?.classList.toggle("is-hidden", !state.sequenceBuilder.active);
  els.recordingSequenceToggle?.classList.toggle("active", state.sequenceBuilder.active);
  if (els.recordingSequenceToggle) {
    const label = state.sequenceBuilder.active ? "Close Sequence" : "Create Sequence";
    els.recordingSequenceToggle.innerHTML = state.sequenceBuilder.active
      ? '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 6l12 12M18 6L6 18"/></svg>'
      : '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M4 6h12M4 12h12M4 18h7"/><path d="M18 14v6M15 17h6"/></svg>';
    els.recordingSequenceToggle.title = label;
    els.recordingSequenceToggle.setAttribute("aria-label", label);
  }
  if (els.recordingSaveSequence) {
    els.recordingSaveSequence.disabled = state.sequenceBuilder.points.length === 0;
  }
  if (els.recordingCapturePose) {
    const label = state.sequenceBuilder.active ? "Add Point" : "Save Pose";
    els.recordingCapturePose.innerHTML = state.sequenceBuilder.active
      ? '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M12 5v14M5 12h14"/></svg>'
      : '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 3h12v18l-6-4.2L6 21z"/></svg>';
    els.recordingCapturePose.title = label;
    els.recordingCapturePose.setAttribute("aria-label", label);
  }
  if (!state.sequenceBuilder.active) {
    els.endEffectorStatus?.classList.add("is-hidden");
  }
  if (els.sequenceBuilderStatus) {
    els.sequenceBuilderStatus.textContent = state.sequenceBuilder.points.length
      ? `Unsaved sequence · ${state.sequenceBuilder.points.length} point${state.sequenceBuilder.points.length === 1 ? "" : "s"}`
      : "Unsaved sequence · no points";
  }
  els.sequenceUndoDelete?.classList.toggle("is-hidden", !state.sequenceBuilder.deletedPoint);
  if (els.sequencePointList) {
    els.sequencePointList.innerHTML = state.sequenceBuilder.points.length
      ? state.sequenceBuilder.points
          .map((point, index) => {
            const motors = point.motors?.length ?? 0;
            const time = point.timestamp ? new Date(point.timestamp * 1000).toLocaleTimeString() : "--";
            const duration = sequencePointDuration(point, index);
            const selected = state.sequenceBuilder.selectedIndex === index;
            const current =
              Number.isInteger(state.sequenceBuilder.playbackCurrentIndex) &&
              index <= state.sequenceBuilder.playbackCurrentIndex;
            const next = state.sequenceBuilder.playbackNextIndex === index;
            const classes = ["sequence-point"];
            if (selected) classes.push("selected");
            if (current) classes.push("current");
            if (next) classes.push("next");
            return `
              <article class="${classes.join(" ")}" data-sequence-select="${index}" tabindex="0" role="button" aria-pressed="${selected ? "true" : "false"}">
                <span>
                  <strong>Point ${index + 1}</strong>
                  <small>${motors} motors · ${duration} · ${time}</small>
                </span>
                <button type="button" data-sequence-delete="${index}">Delete</button>
              </article>
            `;
          })
          .join("")
      : `<div class="sequence-point"><span><strong>No points yet</strong><small>Move either hand marker, then Save Pose.</small></span></div>`;
  }
  updateReplayUi();
}

function renderIkStatus(status) {
  if (!els.endEffectorStatus) return;
  const errorCm = Number.isFinite(status?.error) ? status.error * 100 : null;
  els.endEffectorStatus.classList.remove("is-hidden", "good", "warn", "bad");
  if (status?.blocked || status?.collision?.colliding) {
    els.endEffectorStatus.classList.add("bad");
    els.endEffectorStatus.textContent = `Self collision blocked · ${status.collision?.arm || "arm"} vs ${status.collision?.body || "body"}`;
  } else if (status?.reachable) {
    els.endEffectorStatus.classList.add("good");
    els.endEffectorStatus.textContent = `IK solved · ${errorCm.toFixed(1)} cm`;
  } else if (status?.limited) {
    els.endEffectorStatus.classList.add("warn");
    els.endEffectorStatus.textContent = `IK near limit · ${errorCm?.toFixed(1) ?? "--"} cm`;
  } else {
    els.endEffectorStatus.classList.add("bad");
    els.endEffectorStatus.textContent = `IK unreachable · ${errorCm?.toFixed(1) ?? "--"} cm`;
  }
}

async function setSequenceMode(active) {
  if (!active && state.sequenceBuilder.points.length) {
    const discard = window.confirm("Discard unsaved sequence points?");
    if (!discard) return;
    clearSequenceDraft();
  }
  state.sequenceBuilder.active = active;
  if (active) loadSequenceDraft();
  if (active && els.recordingFileSelect) {
    els.recordingFileSelect.value = currentRobotPoseValue;
    updateRenameButton();
    await loadReplayRecording();
  }
  renderSequenceBuilder();
}

function addSequencePoint() {
  const snapshot = sequencePointSnapshot();
  if (!hasBodyMotors(snapshot)) throw new Error("No edited or current robot pose is available.");
  snapshot.timestamp = Date.now() / 1000;
  snapshot.type = "telemetry_sample";
  snapshot.source = "sequence_builder";
  state.sequenceBuilder.points.push(snapshot);
  state.sequenceBuilder.deletedPoint = null;
  state.sequenceBuilder.selectedIndex = state.sequenceBuilder.points.length - 1;
  saveSequenceDraft();
  renderSequenceBuilder();
  showSequencePoint(state.sequenceBuilder.selectedIndex);
}

function showSequencePoint(index) {
  const point = state.sequenceBuilder.points[index];
  if (!point) return;
  pauseReplay();
  const snapshot = recordingFrameToSnapshot(point);
  state.sequenceBuilder.selectedIndex = index;
  state.sequenceBuilder.playbackCurrentIndex = null;
  state.sequenceBuilder.playbackNextIndex = null;
  state.replay.frames = [timelineFrame(snapshot, snapshot, "sequence_point")];
  state.replay.index = 0;
  state.replay.loadedFile = "__sequence_builder_point__";
  state.replay.previewComplete = false;
  window.dispatchEvent(new CustomEvent("recording-trajectory-visibility", { detail: { visible: false } }));
  refreshReplayTarget();
  updateReplayUi();
  renderSequenceBuilder();
}

function compactTrajectoryJoint(joint) {
  const compact = {
    index: joint.index,
    name: joint.name,
    q: Number(joint.q) || 0,
  };
  if (Number.isFinite(Number(joint.dq))) compact.dq = Number(joint.dq);
  if (Number.isFinite(Number(joint.tau_est))) compact.tau_est = Number(joint.tau_est);
  return compact;
}

function compactSequencePoint(point) {
  const hands = point.hands || {};
  const handJoints = Array.isArray(hands.joints) ? hands.joints.map(compactTrajectoryJoint) : [];
  return {
    type: "telemetry_sample",
    source: point.source || "sequence_builder",
    timestamp: point.timestamp,
    sample: point.sample ?? 0,
    samples: point.samples ?? point.sample ?? 0,
    motor_count: point.motor_count ?? point.motors?.length ?? 0,
    motors: (point.motors || []).map(compactTrajectoryJoint),
    hands: {
      joint_count: hands.joint_count ?? handJoints.length,
      joints: handJoints,
    },
  };
}

async function saveSequence() {
  if (!state.sequenceBuilder.points.length) return;
  if (els.recordingSaveSequence) els.recordingSaveSequence.disabled = true;
  try {
    const response = await fetch("/api/recording/sequence", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        label: "h1_2_edited_sequence",
        points: state.sequenceBuilder.points.map(compactSequencePoint),
      }),
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) throw new Error(payload.error || "Sequence save failed.");
    clearSequenceDraft();
    await loadRecordingFiles();
    if (payload.file?.name && els.recordingFileSelect) {
      await setSequenceMode(false);
      els.recordingFileSelect.value = payload.file.name;
      await loadReplayRecording();
    }
  } finally {
    renderSequenceBuilder();
  }
}

async function capturePosePoint() {
  if (!els.recordingCapturePose) return;
  if (state.sequenceBuilder.active) {
    try {
      addSequencePoint();
    } catch (error) {
      if (els.recordingError) els.recordingError.textContent = error instanceof Error ? error.message : "Could not add point.";
    }
    return;
  }
  els.recordingCapturePose.disabled = true;
  try {
    const response = await fetch("/api/recording/pose", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        label: state.editedPose ? "h1_2_edited_pose_point" : "h1_2_pose_point",
        snapshot: state.editedPose || currentPoseSnapshot(),
      }),
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) throw new Error(payload.error || "Pose capture failed.");
    if (els.recordingError) els.recordingError.textContent = `Captured ${payload.file?.name || "pose point"}`;
    await loadRecordingFiles();
    if (payload.file?.name && els.recordingFileSelect) {
      els.recordingFileSelect.value = payload.file.name;
      await loadReplayRecording();
    }
  } catch (error) {
    if (els.recordingError) {
      els.recordingError.textContent = error instanceof Error ? error.message : "Pose capture failed.";
    }
  } finally {
    els.recordingCapturePose.disabled = false;
  }
}

async function renameRecordingFile() {
  const name = els.recordingFileSelect?.value || "";
  if (!isRobotReplayFileName(name)) {
    if (els.recordingError) els.recordingError.textContent = "Select a saved pose, sequence, or recording to rename.";
    return;
  }
  const suggested = name.replace(/^\d{8}-\d{6}-/, "").replace(/(\.pose\.json|\.sequence\.json|\.jsonl)$/, "");
  const label = window.prompt("New name (letters, digits, - and _):", suggested);
  if (label === null || !label.trim()) return;
  if (els.recordingRename) els.recordingRename.disabled = true;
  try {
    const response = await fetch("/api/recording/rename", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name, label: label.trim() }),
    });
    const payload = await response.json();
    if (!response.ok || !payload.ok) throw new Error(payload.error || "Rename failed.");
    const newName = payload.file?.name;
    if (newName && state.replay.loadedFile === name) state.replay.loadedFile = newName;
    await loadRecordingFiles();
    if (newName && els.recordingFileSelect) els.recordingFileSelect.value = newName;
    if (els.recordingError) els.recordingError.textContent = newName ? `Renamed to ${newName}` : "Renamed.";
  } catch (error) {
    if (els.recordingError) els.recordingError.textContent = error instanceof Error ? error.message : "Rename failed.";
  } finally {
    updateRenameButton();
  }
}

async function stopRecording() {
  if (!els.recordingRobotMotionToggle) return;
  els.recordingRobotMotionToggle.disabled = true;
  try {
    const response = await fetch("/api/recording/stop", { method: "POST" });
    const payload = await response.json();
    renderRecordingStatus(payload.status || payload);
  } catch (error) {
    if (els.recordingError) els.recordingError.textContent = error instanceof Error ? error.message : "Stop failed.";
  } finally {
    loadRecordingStatus();
    loadRecordingFiles();
  }
}

function toggleRobotMotionRecording() {
  if (state.recordingActive) {
    stopRecording();
    return;
  }
  startRecording();
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
    els.locoBalanceModeStatic,
    els.locoBalanceModeGait,
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
    if (button) button.disabled = false;
  });
  els.locoPresets.forEach((button) => {
    button.disabled = false;
  });
  if (els.locoStop) els.locoStop.disabled = false;
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
    armed: true,
    i_understand_risk: true,
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
  els.locoBalanceModeStatic?.addEventListener("click", () => sendLocoCommand("set_balance_mode", { balance_mode: 0 }));
  els.locoBalanceModeGait?.addEventListener("click", () => sendLocoCommand("set_balance_mode", { balance_mode: 1 }));
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
  fetch("/api/robot/chill", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      armed: true,
      i_understand_risk: true,
    }),
  })
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
      els.subtitle.textContent = `Release failed: ${error.message}`;
      if (els.wristMessage) els.wristMessage.textContent = error.message;
    })
    .finally(() => {
      els.chillMotors.disabled = false;
      els.chillMotors.classList.remove("pending");
      loadWristStatus();
    });
}

function sendRobotHome() {
  if (!els.homeRobot) return;
  els.homeRobot.disabled = true;
  els.homeRobot.classList.add("pending");
  fetch("/api/robot/home", { method: "POST" })
    .then((response) =>
      response.json().then((data) => {
        if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
        return data;
      }),
    )
    .then((data) => {
      els.subtitle.textContent = data.message || "Home command sent.";
      if (els.wristMessage) els.wristMessage.textContent = data.message || "Home command sent.";
      if (els.locoMessage) els.locoMessage.textContent = data.message || "Home command sent.";
    })
    .catch((error) => {
      els.subtitle.textContent = `Home failed: ${error.message}`;
      if (els.wristMessage) els.wristMessage.textContent = error.message;
    })
    .finally(() => {
      els.homeRobot.disabled = false;
      els.homeRobot.classList.remove("pending");
      loadWristStatus();
      loadLocoStatus();
    });
}

function sendRobotStraight() {
  // "Stand Up": recovery after Release — the RC left+up (lock stand) combo,
  // done in software through the LocoClient instead of the physical remote.
  if (!els.straightRobot) return;
  els.straightRobot.disabled = true;
  els.straightRobot.classList.add("pending");
  fetch("/api/loco/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "stand_up" }),
  })
    .then((response) =>
      response.json().then((data) => {
        if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
        return data;
      }),
    )
    .then((data) => {
      const message = data.message || "Stand up (lock stand) requested.";
      els.subtitle.textContent = message;
      if (els.wristMessage) els.wristMessage.textContent = message;
      if (els.locoMessage) els.locoMessage.textContent = message;
    })
    .catch((error) => {
      els.subtitle.textContent = `Stand up failed: ${error.message}`;
      if (els.wristMessage) els.wristMessage.textContent = error.message;
    })
    .finally(() => {
      els.straightRobot.disabled = false;
      els.straightRobot.classList.remove("pending");
      loadWristStatus();
      loadLocoStatus();
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
  document.documentElement.scrollLeft = 0;
  document.body.scrollLeft = 0;
  let activeHash = window.location.hash || "#dashboard";
  // A bookmarked/stale hash for a page that no longer exists (e.g. the removed
  // Camera Stream page) would land on a blank view — send it to the dashboard.
  const knownPage = Array.from(els.navItems).some((item) => item.getAttribute("href") === activeHash);
  if (!knownPage && activeHash !== "#rawView") {
    activeHash = "#dashboard";
    window.location.hash = "#dashboard";
  }
  document.getElementById("dashboard")?.classList.toggle("page-rawView", activeHash === "#rawView");
  els.navItems.forEach((item) => {
    item.classList.toggle("active", item.getAttribute("href") === activeHash);
  });
  window.dispatchEvent(new CustomEvent("telemetry-tab-change", { detail: { hash: activeHash } }));
}

document.addEventListener("visibilitychange", () => {
  if (!document.hidden && state.latest) render(state.latest);
});

function connectEvents() {
  if (state.events) return;
  const events = new EventSource("/events");
  state.events = events;
  events.onmessage = (event) => {
    try {
      const snapshot = JSON.parse(event.data);
      // Perf: browser tab in the background — keep state fresh but skip all
      // DOM/3D work; one render catches up when the tab becomes visible.
      if (document.hidden) {
        state.latest = snapshot;
        return;
      }
      render(snapshot);
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
window.addEventListener("recording-edited-pose", (event) => {
  state.editedPose = event.detail?.snapshot || null;
  // A real drag (not a sync from a loaded file) makes the dragged pose the
  // pending Move target, overriding the previously selected file.
  if (event.detail?.origin !== "sync" && state.editedPose) state.editedSinceLoad = true;
  // Re-evaluate "Move" — an unsaved edited pose is now movable directly.
  updateReplayUi();
});
window.addEventListener("recording-ik-status", (event) => renderIkStatus(event.detail || {}));
window.addEventListener("recording-viewer-ready", () => {
  window.dispatchEvent(
    new CustomEvent("recording-collision-debug", { detail: { visible: Boolean(els.collisionDebugToggle?.checked) } }),
  );
  setTimeout(refreshReplayTarget, 0);
  setTimeout(refreshReplayTarget, 250);
});
window.addEventListener("telemetry-tab-change", () => {
  // Freshly shown page: render it from the latest snapshot right away instead
  // of waiting for the next SSE message.
  if (state.latest) render(state.latest);
}, { once: false });
window.addEventListener("telemetry-tab-change", () => {
  if (window.location.hash === "#recordingPage") {
    setTimeout(refreshReplayTarget, 0);
    setTimeout(refreshReplayTarget, 250);
  }
});
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
  loadRecordingStatus();
});
els.refreshRosGraph?.addEventListener("click", loadRosGraph);
els.chillMotors?.addEventListener("click", chillMotors);
els.straightRobot?.addEventListener("click", sendRobotStraight);
els.homeRobot?.addEventListener("click", sendRobotHome);
els.recordingRobotMotionToggle?.addEventListener("click", toggleRobotMotionRecording);
els.recordingCapturePose?.addEventListener("click", capturePosePoint);
els.recordingSequenceToggle?.addEventListener("click", () => setSequenceMode(!state.sequenceBuilder.active));
els.collisionDebugToggle?.addEventListener("change", () => {
  window.dispatchEvent(
    new CustomEvent("recording-collision-debug", { detail: { visible: Boolean(els.collisionDebugToggle?.checked) } }),
  );
});
els.elbowTargetsToggle?.addEventListener("change", () => {
  window.dispatchEvent(
    new CustomEvent("recording-elbow-targets", { detail: { visible: Boolean(els.elbowTargetsToggle?.checked) } }),
  );
});
els.sequenceBuilderClose?.addEventListener("click", () => setSequenceMode(false));
els.recordingSaveSequence?.addEventListener("click", () => {
  saveSequence().catch((error) => {
    if (els.recordingError) els.recordingError.textContent = error instanceof Error ? error.message : "Sequence save failed.";
    renderSequenceBuilder();
  });
});
els.sequenceUndoDelete?.addEventListener("click", () => {
  if (!state.sequenceBuilder.deletedPoint) return;
  state.sequenceBuilder.points.splice(state.sequenceBuilder.deletedPoint.index, 0, state.sequenceBuilder.deletedPoint.point);
  state.sequenceBuilder.deletedPoint = null;
  saveSequenceDraft();
  renderSequenceBuilder();
});
els.sequencePointList?.addEventListener("click", (event) => {
  const button = event.target.closest("[data-sequence-delete]");
  if (button) {
    const index = Number(button.dataset.sequenceDelete);
    const [point] = state.sequenceBuilder.points.splice(index, 1);
    state.sequenceBuilder.deletedPoint = point ? { index, point } : null;
    if (state.sequenceBuilder.selectedIndex === index) {
      state.sequenceBuilder.selectedIndex = null;
    } else if (state.sequenceBuilder.selectedIndex > index) {
      state.sequenceBuilder.selectedIndex -= 1;
    }
    saveSequenceDraft();
    renderSequenceBuilder();
    return;
  }
  const item = event.target.closest("[data-sequence-select]");
  if (!item) return;
  showSequencePoint(Number(item.dataset.sequenceSelect));
});
els.sequencePointList?.addEventListener("keydown", (event) => {
  if (event.key !== "Enter" && event.key !== " ") return;
  const item = event.target.closest("[data-sequence-select]");
  if (!item) return;
  event.preventDefault();
  showSequencePoint(Number(item.dataset.sequenceSelect));
});
els.recordingRename?.addEventListener("click", renameRecordingFile);
els.recordingFileSelect?.addEventListener("change", () => {
  updateRenameButton();
  loadReplayRecording();
});
els.recordingFileSelect?.addEventListener("click", () => {
  if (els.recordingFileSelect.value === currentRobotPoseValue) loadReplayRecording();
});
els.recordingPlay?.addEventListener("click", playReplay);
els.recordingRobotPlay?.addEventListener("click", requestRobotReplay);
els.recordingReplayResponse?.addEventListener("input", updateReplayResponseLabel);
els.recordingMirrorArms?.addEventListener("change", () => {
  window.dispatchEvent(
    new CustomEvent("recording-arm-mirror-toggle", { detail: { enabled: Boolean(els.recordingMirrorArms.checked) } }),
  );
});
els.recordingScrub?.addEventListener("input", () => {
  pauseReplay();
  state.replay.previewComplete = false;
  showReplayFrame(Number(els.recordingScrub.value || 0));
});
els.teleoperationMethods.forEach((method) => {
  method.addEventListener("click", async () => {
    method.classList.remove("click-glow");
    window.requestAnimationFrame(() => {
      method.classList.add("click-glow");
      window.setTimeout(() => method.classList.remove("click-glow"), 520);
    });
    const mode = method.dataset.teleoperationMode;
    if (!mode) return;
    const allMethods = Array.from(els.teleoperationMethods);
    allMethods.forEach((item) => {
      if (item instanceof HTMLButtonElement) item.disabled = true;
    });
    try {
      const response = await fetch("/api/xr/mode", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode }),
      });
      const payload = await response.json();
      if (!response.ok || !payload.ok) throw new Error(payload.error || "XR mode switch failed.");
      window.open(vrViewUrl, "_blank", "noreferrer");
    } catch (error) {
      if (els.locoMessage) {
        els.locoMessage.textContent = error instanceof Error ? error.message : "XR mode switch failed.";
      }
    } finally {
      allMethods.forEach((item) => {
        if (item instanceof HTMLButtonElement) item.disabled = false;
      });
    }
  });
});
document.addEventListener("selectstart", (event) => {
  if (event.target instanceof HTMLInputElement || event.target instanceof HTMLTextAreaElement) return;
  event.preventDefault();
});
document.addEventListener("dragstart", (event) => {
  event.preventDefault();
});
// --- Pose-proposal feedback (thumbs up/down learning data) ------------------
// Verdicts are independent of execution: the operator can dislike a pose and
// still say "okay", or like it and ask for changes. Every verdict lands in the
// server-side CSV and feeds qwen's learned examples.
function poseFeedbackEnabled() {
  return window.localStorage?.getItem("h1_pose_feedback") !== "0";
}

// --- Chat backend selection (qwen default vs Claude bridge) -----------------
// The server sends the SAME system prompt, tools and guards to either engine;
// this only picks which one answers.
function chatBackend() {
  return window.localStorage?.getItem("h1_chat_backend") === "claude" ? "claude" : "default";
}

function setupClaudeBackendToggle() {
  const toggle = document.getElementById("claudeBackendToggle");
  if (!toggle) return;
  const render = () => {
    const claude = chatBackend() === "claude";
    toggle.setAttribute("aria-pressed", claude ? "true" : "false");
    toggle.classList.toggle("active", claude);
  };
  toggle.addEventListener("click", () => {
    window.localStorage?.setItem("h1_chat_backend", chatBackend() === "claude" ? "default" : "claude");
    render();
  });
  render();
}

function setupPoseFeedbackToggle() {
  const toggle = document.getElementById("poseFeedbackToggle");
  if (!toggle) return;
  const render = () => {
    const on = poseFeedbackEnabled();
    toggle.setAttribute("aria-pressed", on ? "true" : "false");
    toggle.classList.toggle("active", on);
  };
  toggle.addEventListener("click", () => {
    window.localStorage?.setItem("h1_pose_feedback", poseFeedbackEnabled() ? "0" : "1");
    render();
  });
  render();
}

function buildPoseFeedbackCard(proposalId, requestText, hooks = {}) {
  const wrap = document.createElement("div");
  wrap.className = "pose-feedback";
  const label = document.createElement("span");
  label.className = "pose-feedback-label";
  label.textContent = "👍 executes · 👎 + note retries:";
  const comment = document.createElement("input");
  comment.type = "text";
  comment.maxLength = 500;
  comment.placeholder = "note for 👎 retry (optional)…";
  comment.className = "pose-feedback-comment";
  const status = document.createElement("span");
  status.className = "pose-feedback-status";
  // One verdict per card: a thumbs-up EXECUTES the pose, so a double-click (or a
  // later contradictory thumbs-down) must not fire a second move / retry.
  let settled = false;
  const send = async (verdict, button) => {
    if (settled) return;
    settled = true;
    const buttons = wrap.querySelectorAll("button");
    buttons.forEach((b) => (b.disabled = true));
    const note = comment.value.trim();
    status.textContent = verdict === "liked" ? "recording + executing…" : "recording…";
    try {
      const body = { proposal_id: proposalId, verdict, comment: note };
      // The thumbs-up click IS the operator's approval (same consent as the
      // Move button), so the server executes the staged pose in the same call.
      if (verdict === "liked") body.execute = true;
      const response = await fetch("/api/pose/feedback", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const payload = await response.json();
      if (!response.ok || !payload.ok) throw new Error(payload.error || "Feedback failed.");
      button.classList.add("chosen");
      if (verdict === "liked") {
        if (payload.move && payload.move.ok) {
          status.textContent = "recorded 👍 — moving ✓";
          hooks.onExecuted?.();
        } else {
          status.textContent = "recorded 👍, move failed: " + (payload.move?.error || "unknown");
        }
      } else if (note && hooks.onRetry) {
        const dispatched = hooks.onRetry(
          `Önerine 👎 verdim: "${note}". "${requestText}" isteğimi bu notu dikkate alarak düzeltilmiş açılarla yeniden öner.`,
        );
        status.textContent = dispatched
          ? "recorded 👎 — retrying with your note…"
          : "recorded 👎 (note saved; ask again to retry)";
      } else {
        status.textContent = "recorded 👎";
      }
    } catch (error) {
      // Let the operator try the other verdict if the POST itself failed.
      settled = false;
      buttons.forEach((b) => (b.disabled = false));
      status.textContent = error instanceof Error ? error.message : "Feedback failed.";
    }
  };
  const makeThumb = (verdict, glyph, title) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = `pose-feedback-thumb ${verdict}`;
    button.title = title;
    button.textContent = glyph;
    button.addEventListener("click", () => send(verdict, button));
    return button;
  };
  wrap.append(
    label,
    makeThumb("liked", "👍", "Good proposal (learning data)"),
    makeThumb("disliked", "👎", "Bad proposal (learning data)"),
    comment,
    status,
  );
  return wrap;
}

function setupChat() {
  if (!els.chatForm || !els.chatInput || !els.chatLog) return;
  setupPoseFeedbackToggle();
  setupClaudeBackendToggle();
  const history = [];
  let busy = false;
  // A pose-reference photo the operator attached ("copy this pose"), as a
  // downscaled JPEG data URL. Sent once with the next message, then cleared.
  let pendingMimicImage = null;
  // Set by a thumbs-down retry: the proposal id the next message corrects,
  // so the server chains the new proposal to its parent in the labeled data.
  let pendingRetryOf = null;
  // Closed-loop visual self-check: after a proposal is staged, the viewer render
  // (blue live + green ghost) is auto-sent back so the vision model verifies its
  // own proposal — bounded rounds per operator request so it cannot ping-pong.
  let twinCheckRounds = 0;
  const TWIN_CHECK_MAX_ROUNDS = 2;
  // Every pose the self-check loop photographed this request: {id, shot}.
  // When the loop settles, alternatives are offered as a clickable gallery.
  let twinCandidates = [];
  let lastStagedId = null;

  function scheduleTwinCheck() {
    // Give the viewer a beat to render the fresh green ghost (5 Hz state poll).
    window.setTimeout(() => {
      if (busy || els.chatInput.value.trim()) return; // never preempt the operator
      sendTwinCheck();
    }, 1200);
  }

  async function sendTwinCheck() {
    const evidence =
      typeof window.captureDigitalTwinEvidence === "function"
        ? window.captureDigitalTwinEvidence()
        : null;
    if (!evidence || !evidence.screenshot || busy) return;
    // Remember what this render shows: the currently staged candidate.
    if (lastStagedId && !twinCandidates.some((c) => c.id === lastStagedId)) {
      twinCandidates.push({ id: lastStagedId, shot: evidence.screenshot });
    }
    busy = true;
    els.chatSend.disabled = true;
    const pending = addMessage("assistant", "🔍 Checking my own preview against your request…", { pending: true });
    const text = "(automatic twin check — verify that the staged green pose matches my previous request)";
    history.push({ role: "user", content: text });
    try {
      const window_ = history.slice(-20);
      while (window_.length && window_[0].role !== "user") window_.shift();
      const response = await fetch("/api/chat", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          messages: window_, twin_evidence: evidence,
          backend: chatBackend(), twin_check: true,
        }),
      });
      const payload = await response.json();
      if (!response.ok || !payload.ok) throw new Error(payload.error || `Request failed (${response.status}).`);
      pending.card.classList.remove("pending");
      pending.p.textContent = payload.reply;
      const used = Array.isArray(payload.tools_used)
        ? payload.tools_used.map((tool) => ({ name: tool.name, arguments: tool.arguments, ok: tool.ok }))
        : [];
      history.push(
        used.length
          ? { role: "assistant", content: payload.reply, tools_used: used }
          : { role: "assistant", content: payload.reply },
      );
      if (payload.proposal && poseFeedbackEnabled()) {
        pending.card.append(
          buildPoseFeedbackCard(payload.proposal.id, "(visual self-check correction)", {
            onExecuted: () => {
              history.push({
                role: "assistant",
                content: "(Operator approved via thumbs-up; the staged pose was executed.)",
                tools_used: [{ name: "move", arguments: { position: "proposed", confirm: true }, ok: true }],
              });
            },
            onRetry: (message) => {
              if (busy || els.chatInput.value.trim()) return false;
              pendingRetryOf = payload.proposal.id;
              els.chatInput.value = message;
              els.chatForm.requestSubmit();
              return true;
            },
          }),
        );
      }
      // The model corrected itself → verify the NEW ghost too, within budget.
      const corrected = payload.proposal && used.some((t) => t.name === "propose_arm_pose");
      if (corrected) lastStagedId = payload.proposal.id;
      if (corrected && twinCheckRounds < TWIN_CHECK_MAX_ROUNDS) {
        twinCheckRounds++;
        scheduleTwinCheck();
      } else {
        finishTwinLoop(lastStagedId);
      }
    } catch (error) {
      // Self-checks are best-effort: withdraw quietly, never surface an error.
      pending.card.remove();
      history.pop();
    } finally {
      busy = false;
      els.chatSend.disabled = false;
      els.chatLog.scrollTop = els.chatLog.scrollHeight;
    }
  }

  function finishTwinLoop(finalId) {
    // Offer every pose the loop photographed as a clickable alternative, so the
    // operator can prefer an earlier try over the final proposal.
    const alternatives = twinCandidates.filter((c) => c.id && c.id !== finalId);
    if (!alternatives.length) {
      twinCandidates = [];
      return;
    }
    let finalShot = (twinCandidates.find((c) => c.id === finalId) || {}).shot || null;
    if (!finalShot && typeof window.captureDigitalTwinEvidence === "function") {
      const evidence = window.captureDigitalTwinEvidence();
      finalShot = (evidence && evidence.screenshot) || null;
    }
    const entries = [...alternatives.map((c) => ({ ...c, current: false }))];
    if (finalId) entries.push({ id: finalId, shot: finalShot, current: true });
    const card = document.createElement("article");
    card.className = "chat-card assistant-card";
    const span = document.createElement("span");
    span.textContent = "Pose candidates";
    const p = document.createElement("p");
    p.textContent = "The self-check tried these poses. Click one if you like it better — the green preview switches to it (👍 still decides).";
    const gallery = document.createElement("div");
    gallery.className = "pose-gallery";
    const status = document.createElement("p");
    status.className = "chat-tools-note";
    const feedbackHolder = document.createElement("div");
    entries.forEach((entry, index) => {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "pose-candidate" + (entry.current ? " current" : "");
      if (entry.shot) {
        const img = document.createElement("img");
        img.src = entry.shot;
        img.alt = `candidate pose ${index + 1}`;
        btn.append(img);
      }
      const label = document.createElement("span");
      label.textContent = entry.current ? "current ✓" : `try ${index + 1}`;
      btn.append(label);
      btn.addEventListener("click", async () => {
        if (busy) return;
        try {
          const response = await fetch("/api/pose/proposal/restage", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ proposal_id: entry.id }),
          });
          const result = await response.json();
          if (!response.ok || !result.ok) throw new Error(result.error || "restage failed");
          gallery.querySelectorAll(".pose-candidate").forEach((b) => b.classList.remove("current"));
          btn.classList.add("current");
          status.textContent = "Green preview switched to this pose — approve it with 👍.";
          feedbackHolder.innerHTML = "";
          if (poseFeedbackEnabled()) {
            feedbackHolder.append(
              buildPoseFeedbackCard(entry.id, "(alternative selected from candidates)", {
                onExecuted: () => {
                  history.push({
                    role: "assistant",
                    content: "(Operator approved via thumbs-up; the staged pose was executed.)",
                    tools_used: [{ name: "move", arguments: { position: "proposed", confirm: true }, ok: true }],
                  });
                },
                onRetry: (message) => {
                  if (busy || els.chatInput.value.trim()) return false;
                  pendingRetryOf = entry.id;
                  els.chatInput.value = message;
                  els.chatForm.requestSubmit();
                  return true;
                },
              }),
            );
          }
          history.push({ role: "assistant", content: "(Operator switched the staged preview to an earlier candidate pose.)" });
        } catch (error) {
          status.textContent = "Could not restage: " + ((error && error.message) || "unknown error");
        }
      });
      gallery.append(btn);
    });
    card.append(span, p, gallery, status, feedbackHolder);
    els.chatLog.append(card);
    els.chatLog.scrollTop = els.chatLog.scrollHeight;
    twinCandidates = [];
  }

  const MIMIC_MAX_EDGE = 1024; // longest side after downscale — keeps upload small
  const MIMIC_JPEG_QUALITY = 0.85;

  function setMimic(dataUrl) {
    pendingMimicImage = dataUrl;
    if (els.mimicThumb) els.mimicThumb.src = dataUrl;
    if (els.mimicPreview) els.mimicPreview.hidden = false;
    els.mimicAttach?.classList.add("active");
  }

  function clearMimic() {
    pendingMimicImage = null;
    if (els.mimicPreview) els.mimicPreview.hidden = true;
    if (els.mimicThumb) els.mimicThumb.removeAttribute("src");
    if (els.mimicFileInput) els.mimicFileInput.value = "";
    els.mimicAttach?.classList.remove("active");
  }

  // Downscale + re-encode to JPEG in the browser so a phone photo doesn't blow
  // past the server's size cap. Runs entirely client-side (no upload of the raw).
  function loadMimicFile(file) {
    if (!file || !file.type?.startsWith("image/")) return;
    const reader = new FileReader();
    reader.onload = () => {
      const img = new Image();
      img.onload = () => {
        const scale = Math.min(1, MIMIC_MAX_EDGE / Math.max(img.width, img.height));
        const w = Math.max(1, Math.round(img.width * scale));
        const h = Math.max(1, Math.round(img.height * scale));
        const canvas = document.createElement("canvas");
        canvas.width = w;
        canvas.height = h;
        canvas.getContext("2d").drawImage(img, 0, 0, w, h);
        try {
          setMimic(canvas.toDataURL("image/jpeg", MIMIC_JPEG_QUALITY));
        } catch {
          setMimic(reader.result); // fallback: original data URL
        }
      };
      img.onerror = () => setHint("Could not read that image.");
      img.src = reader.result;
    };
    reader.onerror = () => setHint("Could not read that file.");
    reader.readAsDataURL(file);
  }

  function addMessage(role, text, { pending = false, error = false, image = null } = {}) {
    const card = document.createElement("article");
    const cls =
      role === "user" ? "user-card" : error ? "error-card" : "assistant-card";
    card.className = `chat-card ${cls}${pending ? " pending" : ""}`;
    const label = role === "user" ? "Operator" : error ? "Error" : "AI Assistant";
    const span = document.createElement("span");
    span.textContent = label;
    const p = document.createElement("p");
    p.textContent = text;
    card.append(span, p);
    if (image) {
      // Show the attached image inside the sent message bubble.
      const img = document.createElement("img");
      img.className = "chat-image";
      img.src = image;
      img.alt = "attached image";
      card.append(img);
    }
    els.chatLog.append(card);
    els.chatLog.scrollTop = els.chatLog.scrollHeight;
    return { card, p };
  }

  function autosize() {
    // Grow the box upward with the content instead of showing a scrollbar.
    els.chatInput.style.height = "auto";
    els.chatInput.style.height = `${els.chatInput.scrollHeight}px`;
  }

  async function send() {
    // A photo with no typed message still sends: default to a mimic request.
    const text = els.chatInput.value.trim() || (pendingMimicImage ? "What is in this image?" : "");
    if (!text || busy) return;
    // Detach the photo up front so a resend doesn't ship it twice; restored on error.
    const mimicImage = pendingMimicImage;
    clearMimic();
    const retryOf = pendingRetryOf;
    pendingRetryOf = null;
    twinCheckRounds = 0; // fresh self-check budget per operator request
    twinCandidates = [];
    lastStagedId = null;
    const viaVoice = sendViaVoice;
    sendViaVoice = false;
    busy = true;
    els.chatSend.disabled = true;
    els.chatInput.value = "";
    autosize();
    addMessage("user", text, { image: mimicImage });
    history.push({ role: "user", content: text });
    const pending = addMessage("assistant", "Thinking…", { pending: true });

    try {
      const twinEvidence =
        typeof window.captureDigitalTwinEvidence === "function"
          ? window.captureDigitalTwinEvidence()
          : null;
      // The server rejects payloads over LLM_MAX_MESSAGES (24). The on-screen
      // log keeps everything, but only a recent window is sent — trimmed to
      // start on a user turn so the model never sees a dangling reply.
      const window_ = history.slice(-20);
      while (window_.length && window_[0].role !== "user") window_.shift();
      const response = await fetch("/api/chat", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          messages: window_,
          twin_evidence: twinEvidence,
          backend: chatBackend(),
          ...(mimicImage ? { image: mimicImage } : {}),
          ...(retryOf ? { retry_of: retryOf } : {}),
        }),
      });
      const payload = await response.json();
      if (!response.ok || !payload.ok) {
        throw new Error(payload.error || `Request failed (${response.status}).`);
      }
      pending.card.classList.remove("pending");
      pending.p.textContent = payload.reply;
      if (payload.backend === "claude") {
        const badge = document.createElement("span");
        badge.className = "chat-backend-badge";
        badge.textContent = `✳ ${payload.model || "Claude"}`;
        badge.title = "Answered by Claude via the operator-machine bridge";
        pending.card.querySelector("span")?.append(badge);
      }
      if (Array.isArray(payload.tools_used) && payload.tools_used.length) {
        const note = document.createElement("p");
        note.className = "chat-tools-note";
        note.textContent =
          "🔧 " +
          payload.tools_used
            .map((tool) => tool.name + (tool.ok ? "" : " (failed)"))
            .join(", ");
        pending.card.append(note);
      }
      if (payload.proposal && poseFeedbackEnabled()) {
        pending.card.append(
          buildPoseFeedbackCard(payload.proposal.id, text, {
            // Replayed to qwen as a real move call so later turns know the
            // pose was executed via the thumbs-up, not just talked about.
            onExecuted: () => {
              history.push({
                role: "assistant",
                content: "(Operator approved via thumbs-up; the staged pose was executed.)",
                tools_used: [{ name: "move", arguments: { position: "proposed", confirm: true }, ok: true }],
              });
            },
            // Returns true only if it actually dispatched the retry. Never
            // clobber a half-typed draft or fire while a request is in flight —
            // the card reports "note saved; ask again" in that case.
            onRetry: (message) => {
              if (busy || els.chatInput.value.trim()) return false;
              // Re-attach the original reference photo so the corrected
              // proposal is judged against (and labeled with) the same image.
              if (mimicImage) setMimic(mimicImage);
              // Chain the correction to the proposal it fixes.
              pendingRetryOf = payload.proposal.id;
              els.chatInput.value = message;
              els.chatForm.requestSubmit();
              return true;
            },
          }),
        );
        lastStagedId = payload.proposal.id;
        // Closed loop: let the vision model SEE its own green ghost and correct
        // itself before the operator judges. Only when a vision backend is in
        // play (Claude toggle on, or this turn carried an image → routed there).
        if ((chatBackend() === "claude" || mimicImage) && twinCheckRounds < TWIN_CHECK_MAX_ROUNDS) {
          twinCheckRounds++;
          scheduleTwinCheck();
        }
      }
      // Keep which tools ran with this reply: the server replays them to the
      // LLM as real tool calls so later motion commands aren't answered with
      // imitation prose (text in history looks like it "worked" without one).
      const used = Array.isArray(payload.tools_used)
        ? payload.tools_used.map((tool) => ({ name: tool.name, arguments: tool.arguments, ok: tool.ok }))
        : [];
      history.push(
        used.length
          ? { role: "assistant", content: payload.reply, tools_used: used }
          : { role: "assistant", content: payload.reply },
      );
      if (viaVoice) speak(payload.reply);
    } catch (error) {
      pending.card.classList.remove("pending");
      pending.card.classList.remove("assistant-card");
      pending.card.classList.add("error-card");
      pending.card.querySelector("span").textContent = "Error";
      const raw = error instanceof Error ? error.message : "Chat request failed.";
      // The AI server being momentarily down (Ollama restart / network blip)
      // surfaces as a raw "Cannot reach LLM … Connection refused"; explain it
      // and tell the operator to just resend — nothing was executed.
      const unreachable = /cannot reach llm|connection refused|errno 111|timed out|502|503|504/i.test(raw);
      pending.p.textContent = unreachable
        ? "AI server şu an ulaşılamıyor (geçici) — mesajını tekrar gönder. / AI server temporarily unreachable — resend your message."
        : raw;
      // Drop the failed user turn so it isn't resent with the next message.
      history.pop();
      // Put the pose photo (and retry link) back so a plain resend keeps them.
      if (mimicImage) setMimic(mimicImage);
      if (retryOf) pendingRetryOf = retryOf;
    } finally {
      busy = false;
      els.chatSend.disabled = false;
      els.chatLog.scrollTop = els.chatLog.scrollHeight;
      // A voice message returns the button to mic mode; a typed one keeps the
      // keyboard ready for the next message.
      if (viaVoice) {
        els.chatInput.blur();
        updateMode();
      } else {
        els.chatInput.focus();
      }
    }
  }

  // --- Voice: the action button morphs between Send (while typing) and Mic
  // (push-to-talk) depending on whether the cursor is active in the box. ---
  let voiceInput = false;
  let voiceOutput = false;
  let recording = false;
  let mediaRecorder = null;
  let mediaStream = null;
  let chunks = [];
  let sendViaVoice = false;
  const player = new Audio();

  function setHint(text) {
    if (els.chatHint) els.chatHint.textContent = text || "";
  }

  function micModeActive() {
    // Mic when the operator is not actively editing an empty message box.
    return document.activeElement !== els.chatInput && els.chatInput.value.trim() === "";
  }

  function updateMode() {
    if (recording) return;
    const mic = micModeActive();
    els.chatSend.classList.toggle("mic-mode", mic);
    els.chatSend.type = mic ? "button" : "submit";
    els.chatSend.title = mic ? "Hold to talk" : "Send message";
    els.chatSend.setAttribute("aria-label", mic ? "Hold to record a voice message" : "Send message");
    if (!recording) setHint(mic ? (voiceInput ? "Hold to talk" : "") : "");
  }

  async function startRecording() {
    if (recording || busy) return;
    if (!voiceInput) {
      setHint("Voice input isn't set up yet.");
      return;
    }
    if (!navigator.mediaDevices?.getUserMedia || typeof MediaRecorder === "undefined") {
      setHint("This browser can't record audio.");
      return;
    }
    try {
      mediaStream = await navigator.mediaDevices.getUserMedia({ audio: true });
    } catch (error) {
      setHint("Microphone permission denied.");
      return;
    }
    chunks = [];
    const mime = MediaRecorder.isTypeSupported("audio/webm") ? "audio/webm" : "";
    mediaRecorder = new MediaRecorder(mediaStream, mime ? { mimeType: mime } : undefined);
    mediaRecorder.addEventListener("dataavailable", (event) => {
      if (event.data && event.data.size) chunks.push(event.data);
    });
    mediaRecorder.addEventListener("stop", handleRecordingStop);
    mediaRecorder.start();
    recording = true;
    els.chatSend.classList.add("recording");
    els.chatSend.classList.add("mic-mode");
    setHint("Recording… release to send 🎙️");
  }

  function stopRecording() {
    if (!recording) return;
    recording = false;
    els.chatSend.classList.remove("recording");
    try {
      if (mediaRecorder && mediaRecorder.state !== "inactive") mediaRecorder.stop();
    } catch (error) {
      /* ignore */
    }
    if (mediaStream) mediaStream.getTracks().forEach((track) => track.stop());
    mediaStream = null;
  }

  async function handleRecordingStop() {
    const type = (mediaRecorder && mediaRecorder.mimeType) || "audio/webm";
    const blob = new Blob(chunks, { type });
    chunks = [];
    if (blob.size < 1200) {
      setHint("Too short — hold the mic to talk.");
      updateMode();
      return;
    }
    setHint("Transcribing…");
    try {
      const response = await fetch("/api/stt", {
        method: "POST",
        headers: { "Content-Type": type },
        body: blob,
      });
      const data = await response.json();
      if (!response.ok || !data.ok) throw new Error(data.error || "Transcription failed.");
      if (!data.text) {
        setHint("Didn't catch that — try again.");
        updateMode();
        return;
      }
      setHint("");
      sendViaVoice = true;
      els.chatInput.value = data.text;
      send();
    } catch (error) {
      setHint(error instanceof Error ? error.message : "Transcription failed.");
      updateMode();
    }
  }

  async function speak(text) {
    if (!voiceOutput || !text) return;
    try {
      const response = await fetch("/api/tts", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ text }),
      });
      if (!response.ok) return;
      const blob = await response.blob();
      const url = URL.createObjectURL(blob);
      player.src = url;
      player.onended = () => URL.revokeObjectURL(url);
      player.play().catch(() => URL.revokeObjectURL(url));
    } catch (error) {
      /* speech playback is best-effort */
    }
  }

  els.chatForm.addEventListener("submit", (event) => {
    event.preventDefault();
    send();
  });
  // Attach-a-photo → file picker → downscale → preview. The button is a plain
  // proxy for the hidden <input type=file>.
  els.mimicAttach?.addEventListener("click", () => els.mimicFileInput?.click());
  els.mimicFileInput?.addEventListener("change", (event) => {
    const file = event.target.files && event.target.files[0];
    if (file) loadMimicFile(file);
  });
  els.mimicClear?.addEventListener("click", clearMimic);
  // Paste an image straight into the chat (Cmd/Ctrl+V) — clipboard screenshots too.
  els.chatInput?.addEventListener("paste", (event) => {
    const items = (event.clipboardData && event.clipboardData.items) || [];
    for (const item of items) {
      if (item.kind === "file" && item.type && item.type.startsWith("image/")) {
        const file = item.getAsFile();
        if (file) {
          event.preventDefault(); // don't also paste a blob path as text
          loadMimicFile(file);
          break;
        }
      }
    }
  });
  els.chatInput.addEventListener("input", () => {
    autosize();
    updateMode();
  });
  els.chatInput.addEventListener("focus", updateMode);
  els.chatInput.addEventListener("blur", () => window.setTimeout(updateMode, 0));
  els.chatInput.addEventListener("keydown", (event) => {
    if (event.key === "Enter" && !event.shiftKey) {
      event.preventDefault();
      send();
    }
  });

  // Push-to-talk: hold the mic button to record, release to transcribe + send.
  els.chatSend.addEventListener("pointerdown", (event) => {
    if (!els.chatSend.classList.contains("mic-mode")) return;
    event.preventDefault();
    try {
      els.chatSend.setPointerCapture(event.pointerId);
    } catch (error) {
      /* ignore */
    }
    startRecording();
  });
  const endHold = () => stopRecording();
  els.chatSend.addEventListener("pointerup", endHold);
  els.chatSend.addEventListener("pointercancel", endHold);
  els.chatSend.addEventListener("lostpointercapture", endHold);
  window.addEventListener("blur", endHold);

  fetch("/api/chat/status")
    .then((response) => response.json())
    .then((status) => {
      voiceInput = Boolean(status.voice_input);
      voiceOutput = Boolean(status.voice_output);
      if (els.chatStatus) {
        if (status.enabled) {
          // "Assistant online" is always true, so it's noise — keep the badge
          // hidden. The offline case below still surfaces (it disables input).
          els.chatStatus.hidden = true;
        } else {
          els.chatStatus.hidden = false;
          els.chatStatus.textContent = "Assistant offline";
          els.chatStatus.classList.add("offline");
          els.chatInput.disabled = true;
          els.chatSend.disabled = true;
          els.chatInput.placeholder = "Assistant is disabled";
        }
      }
      updateMode();
    })
    .catch(() => updateMode());

  updateMode();
}

syncActiveNav();
setupChat();
loadRosGraph();
loadRecordingStatus();
loadRecordingFiles({ loadSelected: true });
renderSequenceBuilder();
updateReplayResponseLabel();
updateReplayUi();
// Don't poll while the tab is backgrounded — nothing to update, pure network drain.
window.setInterval(() => { if (!document.hidden) loadRecordingStatus(); }, 2000);
window.setInterval(() => { if (!document.hidden) loadRecordingFiles(); }, 5000);
setupLocoControls();
setupWristControls();
connectEvents();

// ---- Fluent "reveal" highlight: cards glow around the cursor ----
// One delegated pointermove handler; writes the pointer position into CSS
// vars on the hovered card so .reveal-target::before can paint the glow.
(function setupRevealHighlight() {
  const media = window.matchMedia?.("(prefers-reduced-motion: reduce)");
  if (media?.matches) return;
  const selector = ".mini-panel, .camera-card, .loco-card, .wrist-card, .panel";
  document.querySelectorAll(selector).forEach((el) => el.classList.add("reveal-target"));
  // Perf: batch to one layout read per frame instead of per pointer event.
  let pending = null;
  document.addEventListener(
    "pointermove",
    (event) => {
      const card = event.target.closest?.(".reveal-target");
      if (!card) return;
      if (!pending) {
        window.requestAnimationFrame(() => {
          const { card: c, x, y } = pending;
          pending = null;
          const rect = c.getBoundingClientRect();
          c.style.setProperty("--reveal-x", `${x - rect.left}px`);
          c.style.setProperty("--reveal-y", `${y - rect.top}px`);
        });
      }
      pending = { card, x: event.clientX, y: event.clientY };
    },
    { passive: true },
  );
})();

// ---- RH56BFX hands toggle: gray the card + remove hands from the digital ----
// twin (live dashboard viewer AND the High Level Controller page's models).
(function setupHandsToggle() {
  const HANDS_KEY = "h1_hands_enabled";
  const toggle = document.getElementById("handsToggle");
  if (!toggle) return;
  const label = toggle.parentElement;
  const panel = toggle.closest(".mini-panel");
  const apply = (enabled) => {
    panel?.classList.toggle("hands-off", !enabled);
    if (label) label.childNodes[label.childNodes.length - 1].textContent = enabled ? " On" : " Off";
    window.dispatchEvent(new CustomEvent("hands-visibility", { detail: { enabled } }));
  };
  const enabled = window.localStorage?.getItem(HANDS_KEY) !== "0";
  toggle.checked = enabled;
  apply(enabled);
  toggle.addEventListener("change", () => {
    window.localStorage?.setItem(HANDS_KEY, toggle.checked ? "1" : "0");
    apply(toggle.checked);
  });
})();

// ---- floating robot camera: icon bubble -> draggable, corner-resizable view ----
(function setupFloatCam() {
  const OPEN_KEY = "h1_float_cam_open";
  const BOX_KEY = "h1_float_cam_box";
  const icon = document.getElementById("floatCamIcon");
  const panel = document.getElementById("floatCam");
  const header = document.getElementById("floatCamHeader");
  const minimize = document.getElementById("floatCamMinimize");
  const img = document.getElementById("floatCamStream");
  if (!icon || !panel || !header || !img) return;

  const webcam = document.getElementById("floatWebcamStream");
  let webcamRetry = null;
  const attachWebcam = () => {
    if (!webcam) return;
    window.clearTimeout(webcamRetry);
    webcam.src = `/webcam.mjpg?float=${Date.now()}`;
  };
  const detachWebcam = () => {
    if (!webcam) return;
    window.clearTimeout(webcamRetry);
    webcam.removeAttribute("src");
    webcam.classList.add("hidden");
  };
  const attach = () => { img.src = `/camera.mjpg?float=${Date.now()}`; attachWebcam(); };
  const detach = () => { img.removeAttribute("src"); detachWebcam(); };

  const clampBox = (box) => {
    const minW = 220, minH = 150;
    const maxW = window.innerWidth - 24, maxH = window.innerHeight - 24;
    box.w = Math.max(minW, Math.min(maxW, box.w));
    box.h = Math.max(minH, Math.min(maxH, box.h));
    box.x = Math.max(4, Math.min(window.innerWidth - box.w - 4, box.x));
    box.y = Math.max(4, Math.min(window.innerHeight - box.h - 4, box.y));
    return box;
  };
  const defaultBox = () => clampBox({
    x: window.innerWidth - 380, y: window.innerHeight - 264, w: 360, h: 240,
  });
  let box = (() => {
    try { return clampBox({ ...defaultBox(), ...JSON.parse(localStorage.getItem(BOX_KEY) || "{}") }); }
    catch { return defaultBox(); }
  })();
  const applyBox = () => {
    panel.style.left = `${box.x}px`;
    panel.style.top = `${box.y}px`;
    panel.style.right = "auto";
    panel.style.bottom = "auto";
    panel.style.width = `${box.w}px`;
    panel.style.height = `${box.h}px`;
  };
  const saveBox = () => { try { localStorage.setItem(BOX_KEY, JSON.stringify(box)); } catch {} };

  const render = () => {
    const open = localStorage.getItem(OPEN_KEY) === "1";
    icon.classList.toggle("hidden", open);
    panel.classList.toggle("hidden", !open);
    if (open) { applyBox(); if (!img.getAttribute("src")) attach(); }
    else detach();
  };

  icon.addEventListener("click", () => { localStorage.setItem(OPEN_KEY, "1"); render(); });
  minimize.addEventListener("click", (event) => {
    event.stopPropagation();
    localStorage.setItem(OPEN_KEY, "0");
    render();
  });

  // Drag to move (header).
  header.addEventListener("pointerdown", (event) => {
    if (event.target === minimize) return;
    event.preventDefault();
    header.setPointerCapture(event.pointerId);
    const startX = event.clientX, startY = event.clientY;
    const origX = box.x, origY = box.y;
    const move = (ev) => {
      box.x = origX + (ev.clientX - startX);
      box.y = origY + (ev.clientY - startY);
      clampBox(box); applyBox();
    };
    const up = () => {
      header.removeEventListener("pointermove", move);
      header.removeEventListener("pointerup", up);
      saveBox();
    };
    header.addEventListener("pointermove", move);
    header.addEventListener("pointerup", up);
  });

  // Resize from any corner.
  panel.querySelectorAll(".fc-resize").forEach((grip) => {
    grip.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      event.stopPropagation();
      grip.setPointerCapture(event.pointerId);
      const corner = grip.dataset.corner;
      const startX = event.clientX, startY = event.clientY;
      const orig = { ...box };
      const move = (ev) => {
        const dx = ev.clientX - startX, dy = ev.clientY - startY;
        if (corner.includes("r")) box.w = orig.w + dx;
        if (corner.includes("b")) box.h = orig.h + dy;
        if (corner.includes("l")) { box.w = orig.w - dx; box.x = orig.x + dx; }
        if (corner.includes("t")) { box.h = orig.h - dy; box.y = orig.y + dy; }
        clampBox(box); applyBox();
      };
      const up = () => {
        grip.removeEventListener("pointermove", move);
        grip.removeEventListener("pointerup", up);
        saveBox();
      };
      grip.addEventListener("pointermove", move);
      grip.addEventListener("pointerup", up);
    });
  });

  if (webcam) {
    // The webcam slot shows itself only while frames actually flow (üst üste:
    // head camera on top, webcam below) and keeps retrying quietly while the
    // panel is open, so it lights up as soon as a webcam is plugged in.
    webcam.addEventListener("load", () => webcam.classList.remove("hidden"));
    webcam.addEventListener("error", () => {
      webcam.classList.add("hidden");
      window.clearTimeout(webcamRetry);
      webcamRetry = window.setTimeout(() => {
        if (!panel.classList.contains("hidden")) attachWebcam();
      }, 5000);
    });
  }

  img.addEventListener("error", () => {
    panel.classList.add("offline");
    window.setTimeout(() => {
      if (!panel.classList.contains("hidden")) attach();
    }, 4000);
  });
  img.addEventListener("load", () => panel.classList.remove("offline"));

  window.addEventListener("hashchange", render);
  window.addEventListener("resize", () => { clampBox(box); applyBox(); });
  render();
})();

// ---- Bullseye Mode (formerly Sentry Mode): head-tracked glowing lock buttons
// on the webcam feed. Internal ids/endpoints keep the original `sentry` name. ----
(function setupBullseye() {
  const toggle = document.getElementById("sentryToggle");
  const panel = document.getElementById("floatCam");
  const counter = document.getElementById("floatCamSentry");
  const img = document.getElementById("floatWebcamStream");
  const layer = document.getElementById("floatWebcamTargets");
  const boxCanvas = document.getElementById("floatWebcamOverlay");
  const boxesToggle = document.getElementById("sentryBoxesToggle");
  if (!toggle || !panel || !img || !layer) return;

  const BOXES_KEY = "h1_sentry_boxes";
  const isBoxesOn = () => localStorage.getItem(BOXES_KEY) === "1";
  // Mimic Mode (separate IIFE below) publishes its server-confirmed state on
  // the body dataset; while it mirrors, this panel shows what mimic sees.
  const isMimicOn = () => document.body.dataset.mimicOn === "1";

  const MATCH_DIST = 0.18;    // fallback center-distance gate (id-less detections)
  const SMOOTH_ALPHA = 0.75;     // light data smoothing; the rAF spring does the rest
  const TRACK_TTL_MS = 1000;     // a box no person confirms within 1 s is removed
  const PENDING_LOCK_MS = 12000; // how long an off-screen lock waits to re-attach
  const POLL_MS = 250;           // gate check only — detections arrive via SSE push
  // Person-lock pointing disabled for now (2026-07-28) — keep in sync with
  // server.py PERSON_LOCK_ENABLED, which refuses "point" sessions while off.
  const PERSON_LOCK_ENABLED = false;

  let tracks = [];            // {id, serviceId, x1, y1, x2, y2, hx, hy, conf, lastSeen, btn}
  let nextTrackId = 1;
  let lockedId = null;
  // When the locked person walks out of frame, remember their service id for
  // a while: if BoT-SORT re-identifies them on re-entry, the lock re-attaches
  // and continues where it left off instead of demanding a new click.
  let pendingLock = null;     // {serviceId, until}
  let count = null;           // persons in last good detection, or null
  let lastError = null;
  let pointing = false;       // physical tracker confirmed active by server
  let trackingRequested = false;
  let trackRequestGeneration = 0;
  let trackError = null;
  let trackActionQueue = Promise.resolve();
  const queueTrackAction = (action) => {
    trackActionQueue = trackActionQueue.then(action, action);
    return trackActionQueue;
  };

  // The server owns the Bullseye arming flag. Bullseye ON enables prediction;
  // physical motion begins only after an explicit person-lock request.
  let serverOn = false;
  const isOn = () => serverOn;
  const renderToggle = () => {
    toggle.classList.toggle("on", isOn());
    toggle.setAttribute("aria-pressed", isOn() ? "true" : "false");
  };
  const applyServerFlag = (flag) => {
    if (typeof flag === "boolean" && flag !== serverOn) {
      serverOn = flag;
      renderToggle();
    }
  };
  const stopPointing = () => {
    trackRequestGeneration += 1;
    if (!pointing && !trackingRequested) return Promise.resolve();
    pointing = false;
    trackingRequested = false;
    return fetch("/api/track/stop", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: "{}",
    }).then((resp) => resp.json())
      .then((data) => {
        pointing = !!(data.tracking && data.tracking.active);
        if (!pointing) trackError = null;
        renderCounter();
      })
      .catch(() => {
        trackError = "Tracking stop request failed.";
        renderCounter();
      });
  };
  const startPointing = async (track) => {
    trackError = null;
    if (track.serviceId === null) {
      trackError = "Waiting for a stable person identity; tap the lock again.";
      renderCounter();
      return;
    }
    if (pointing || trackingRequested) await stopPointing();
    const generation = ++trackRequestGeneration;
    trackingRequested = true;
    const target = center(track);
    try {
      const resp = await fetch("/api/track/start", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          armed: true,
          i_understand_risk: true,
          source: "sentry-lock",
          camera: "webcam",
          permanent: true,
          closed_loop: true,
          target,
          target_id: track.serviceId,
        }),
      });
      const data = await resp.json();
      if (generation !== trackRequestGeneration) return;
      trackingRequested = false;
      pointing = !!(resp.ok && data.ok && data.tracking && data.tracking.active);
      if (!pointing) trackError = data.error || "Physical tracking did not start.";
    } catch {
      if (generation !== trackRequestGeneration) return;
      trackingRequested = false;
      pointing = false;
      trackError = "Physical tracking start request failed.";
    }
    renderCounter();
  };
  const pushMode = (on) => fetch("/api/sentry/mode", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ on }),
  }).then((resp) => resp.json())
    .then((data) => {
      applyServerFlag(data.sentry_mode);
      trackingRequested = false;
      pointing = !!(data.tracking && data.tracking.active);
      if (!data.sentry_mode) {
        lockedId = null;
        pendingLock = null;
      }
      renderCounter();
    })
    .catch(() => {});
  const syncMode = () => fetch("/api/track/status")
    .then((resp) => resp.json())
    .then((data) => {
      const status = data.tracking || {};
      applyServerFlag(status.sentry_mode);
      if (status.active) trackingRequested = false;
      pointing = !!status.active;
      renderCounter();
    })
    .catch(() => {});
  toggle.addEventListener("click", () => {
    const on = !isOn();
    if (!on) {
      lockedId = null;
      pendingLock = null;
      void queueTrackAction(stopPointing);
    }
    void pushMode(on);
  });
  renderToggle();
  syncMode();
  window.setInterval(() => { if (!document.hidden) syncMode(); }, 2000);

  const renderBoxesToggle = () => {
    if (!boxesToggle) return;
    boxesToggle.classList.toggle("on", isBoxesOn());
    boxesToggle.setAttribute("aria-pressed", isBoxesOn() ? "true" : "false");
  };
  if (boxesToggle) {
    // Stop the header's drag handler from claiming the press.
    boxesToggle.addEventListener("pointerdown", (event) => event.stopPropagation());
    boxesToggle.addEventListener("click", (event) => {
      event.stopPropagation();
      try { localStorage.setItem(BOXES_KEY, isBoxesOn() ? "0" : "1"); } catch {}
      renderBoxesToggle();
      renderButtons();
    });
    renderBoxesToggle();
  }

  // The stream image renders with object-fit: cover — map normalized
  // detection coords through the centered-crop transform.
  const coverTransform = () => {
    const ew = img.clientWidth, eh = img.clientHeight;
    const nw = img.naturalWidth, nh = img.naturalHeight;
    if (!ew || !eh || !nw || !nh) return null;
    const scale = Math.max(ew / nw, eh / nh);
    const dw = nw * scale, dh = nh * scale;
    return { ox: (ew - dw) / 2, oy: (eh - dh) / 2, dw, dh };
  };

  const removeTrack = (track) => {
    if (track.btn) track.btn.remove();
    if (lockedId === track.id) lockedId = null;
  };

  const clearBoxes = () => {
    if (!boxCanvas) return;
    const ctx = boxCanvas.getContext("2d");
    ctx.clearRect(0, 0, boxCanvas.width, boxCanvas.height);
    boxCanvas.classList.add("hidden");
  };

  const clearAllTracks = () => {
    if (pointing || trackingRequested) void queueTrackAction(stopPointing);
    tracks.forEach(removeTrack);
    tracks = [];
    lockedId = null;
    pendingLock = null;
    layer.classList.add("hidden");
    clearBoxes();
    count = null;
    lastError = null;
    if (counter) { counter.classList.add("hidden"); counter.textContent = ""; counter.title = ""; }
  };

  const center = (box) => ({ cx: (box.x1 + box.x2) / 2, cy: (box.y1 + box.y2) / 2 });

  // Identity primarily comes from the detection service's ByteTrack ids
  // (Kalman motion prediction — survives fast movement and brief occlusions).
  // The center-distance fallback only covers the first frames of a track,
  // before the service has confirmed it and its id is still null.
  const associate = (persons, now) => {
    const byServiceId = new Map();
    tracks.forEach((track) => {
      if (track.serviceId !== null) byServiceId.set(track.serviceId, track);
    });
    const unmatched = tracks.slice();
    const maybeReattachLock = (track) => {
      if (!pendingLock || lockedId !== null || track.serviceId === null) return;
      if (track.serviceId === pendingLock.serviceId && now <= pendingLock.until) {
        lockedId = track.id;
        pendingLock = null;
      }
    };
    const claim = (track, person) => {
      const idx = unmatched.indexOf(track);
      if (idx !== -1) unmatched.splice(idx, 1);
      ["x1", "y1", "x2", "y2"].forEach((key) => {
        track[key] += SMOOTH_ALPHA * (person[key] - track[key]);
      });
      if (person.head) {
        track.hx = track.hx === undefined ? person.head.x
          : track.hx + SMOOTH_ALPHA * (person.head.x - track.hx);
        track.hy = track.hy === undefined ? person.head.y
          : track.hy + SMOOTH_ALPHA * (person.head.y - track.hy);
      }
      // Pose keypoints (Mimic skeleton overlay), lightly smoothed like the
      // box; points the detector stops reporting are dropped immediately so
      // a stale limb never lingers on screen.
      if (person.keypoints) {
        track.kp = track.kp || {};
        Object.entries(person.keypoints).forEach(([name, p]) => {
          const prev = track.kp[name];
          track.kp[name] = prev
            ? { x: prev.x + SMOOTH_ALPHA * (p.x - prev.x),
                y: prev.y + SMOOTH_ALPHA * (p.y - prev.y) }
            : { x: p.x, y: p.y };
        });
        Object.keys(track.kp).forEach((name) => {
          if (!person.keypoints[name]) delete track.kp[name];
        });
      } else {
        track.kp = null;
      }
      track.conf = person.conf;
      track.lastSeen = now;
      maybeReattachLock(track);
    };
    persons.forEach((person) => {
      const sid = person.id === undefined || person.id === null ? null : person.id;
      if (sid !== null && byServiceId.has(sid)) {
        claim(byServiceId.get(sid), person);
        return;
      }
      const pc = center(person);
      let best = null;
      let bestDist = MATCH_DIST;
      unmatched.forEach((track) => {
        if (sid !== null && track.serviceId !== null && track.serviceId !== sid) return;
        const tc = center(track);
        const dist = Math.hypot(pc.cx - tc.cx, pc.cy - tc.cy);
        if (dist < bestDist) { best = track; bestDist = dist; }
      });
      if (best) {
        if (sid !== null) { best.serviceId = sid; byServiceId.set(sid, best); }
        claim(best, person);
      } else {
        const fresh = {
          x1: person.x1, y1: person.y1, x2: person.x2, y2: person.y2,
          hx: person.head ? person.head.x : undefined,
          hy: person.head ? person.head.y : undefined,
          conf: person.conf, id: nextTrackId++, serviceId: sid, lastSeen: now, btn: null,
        };
        tracks.push(fresh);
        maybeReattachLock(fresh);
      }
    });
    tracks = tracks.filter((track) => {
      if (now - track.lastSeen > TRACK_TTL_MS) {
        if (lockedId === track.id && track.serviceId !== null) {
          pendingLock = { serviceId: track.serviceId, until: now + PENDING_LOCK_MS };
        }
        removeTrack(track);
        return false;
      }
      return true;
    });
    if (pendingLock && now > pendingLock.until) {
      pendingLock = null;
      if (pointing || trackingRequested) void queueTrackAction(stopPointing);
    }
  };

  const buttonFor = (track) => {
    if (track.btn) return track.btn;
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "target-lock-btn";
    btn.textContent = "🔓";
    // pointerdown, not click: the button repositions every poll tick, so a
    // click's down+up pair can land on different spots and never fire.
    btn.addEventListener("pointerdown", (event) => {
      event.stopPropagation();
      event.preventDefault();
      if (lockedId === track.id) {
        lockedId = null;
        pendingLock = null;
        void queueTrackAction(stopPointing);
      } else {
        lockedId = track.id;
        pendingLock = null;
        void queueTrackAction(() => (
          lockedId === track.id ? startPointing(track) : Promise.resolve()
        ));
      }
      renderButtons();
      renderCounter();
    });
    // While the pointer hovers, freeze the button in place so it cannot
    // slide out from under the cursor between ticks.
    btn.addEventListener("pointerenter", () => { track.hover = true; });
    btn.addEventListener("pointerleave", () => { track.hover = false; });
    layer.appendChild(btn);
    track.btn = btn;
    return btn;
  };

  const renderButtons = () => {
    const t = coverTransform();
    layer.style.left = `${img.offsetLeft}px`;
    layer.style.top = `${img.offsetTop}px`;
    layer.style.width = `${img.clientWidth}px`;
    layer.style.height = `${img.clientHeight}px`;
    layer.classList.remove("hidden");
    tracks.forEach((track) => {
      const btn = buttonFor(track);
      // Person-lock buttons belong to Bullseye pointing; in a mimic-only
      // view a lock click would only produce a "Bullseye is off" error.
      // PERSON_LOCK_ENABLED mirrors the server flag of the same name: while
      // pointing is operator-disabled (2026-07-28) the buttons never render
      // (the server would 409 the click anyway), Bullseye stays view-only.
      if (!t || !isOn() || !PERSON_LOCK_ENABLED) { btn.classList.add("hidden"); return; }
      btn.classList.remove("hidden");
      if (!track.hover) {
        // Real head keypoint when the pose model provides it; box top otherwise.
        const nx = track.hx !== undefined ? track.hx : (track.x1 + track.x2) / 2;
        const ny = track.hy !== undefined ? track.hy : track.y1;
        track.tx = t.ox + nx * t.dw;
        track.ty = Math.max(0, t.oy + ny * t.dh - 6);
      }
      const locked = lockedId === track.id;
      btn.classList.toggle("locked", locked);
      btn.textContent = locked ? "🔒" : "🔓";
      btn.title = locked ? "Kilidi kaldır" : "Bu kişiye kitlen";
    });
    renderBoxes(t);
  };

  // Arm chains Mimic actually consumes, plus the shoulder line for context.
  const MIMIC_BONES = [
    ["l_shoulder", "r_shoulder"],
    ["l_shoulder", "l_elbow"], ["l_elbow", "l_wrist"],
    ["r_shoulder", "r_elbow"], ["r_elbow", "r_wrist"],
  ];
  const MIMIC_JOINTS = [
    "l_shoulder", "r_shoulder", "l_elbow", "r_elbow", "l_wrist", "r_wrist",
  ];

  const renderBoxes = (t) => {
    if (!boxCanvas) return;
    if ((!isBoxesOn() && !isMimicOn()) || !t || !tracks.length) { clearBoxes(); return; }
    boxCanvas.style.left = `${img.offsetLeft}px`;
    boxCanvas.style.top = `${img.offsetTop}px`;
    boxCanvas.width = img.clientWidth;
    boxCanvas.height = img.clientHeight;
    boxCanvas.classList.remove("hidden");
    const ctx = boxCanvas.getContext("2d");
    ctx.clearRect(0, 0, boxCanvas.width, boxCanvas.height);
    ctx.lineWidth = 2;
    ctx.font = "600 11px system-ui, sans-serif";
    tracks.forEach((track) => {
      const x = t.ox + track.x1 * t.dw;
      const y = t.oy + track.y1 * t.dh;
      const w = (track.x2 - track.x1) * t.dw;
      const h = (track.y2 - track.y1) * t.dh;
      const locked = lockedId === track.id;
      ctx.strokeStyle = locked ? "#e60000" : "#12b252";
      ctx.strokeRect(x, y, w, h);
      const label = `${Math.round((track.conf || 0) * 100)}%`;
      const labelWidth = ctx.measureText(label).width + 8;
      ctx.fillStyle = locked ? "#e60000" : "#12b252";
      ctx.fillRect(x, Math.max(0, y - 15), labelWidth, 15);
      ctx.fillStyle = "#fff";
      ctx.fillText(label, x + 4, Math.max(11, y - 4));

      // Mimic skeleton: the exact shoulder→elbow→wrist chains the retarget
      // math consumes, in the mimic accent blue. Missing keypoints simply
      // leave their bone undrawn — that is genuinely what mimic sees.
      if (isMimicOn() && track.kp) {
        const px = (p) => t.ox + p.x * t.dw;
        const py = (p) => t.oy + p.y * t.dh;
        ctx.strokeStyle = "#2f6fed";
        ctx.fillStyle = "#2f6fed";
        ctx.lineWidth = 3;
        MIMIC_BONES.forEach(([a, b]) => {
          const pa = track.kp[a];
          const pb = track.kp[b];
          if (!pa || !pb) return;
          ctx.beginPath();
          ctx.moveTo(px(pa), py(pa));
          ctx.lineTo(px(pb), py(pb));
          ctx.stroke();
        });
        MIMIC_JOINTS.forEach((name) => {
          const p = track.kp[name];
          if (!p) return;
          ctx.beginPath();
          ctx.arc(px(p), py(p), name.endsWith("wrist") ? 5 : 4, 0, Math.PI * 2);
          ctx.fill();
        });
        ctx.lineWidth = 2;
      }
    });
  };

  const renderCounter = () => {
    if (!counter) return;
    if (count === null && !lastError) {
      counter.classList.add("hidden");
      counter.textContent = "";
      counter.title = "";
      return;
    }
    counter.classList.remove("hidden");
    const label = isMimicOn() && !isOn() ? "Mimic" : "Bullseye";
    if (count !== null) {
      const lockState = pointing ? " • POINTING"
        : (lockedId !== null ? " • LOCKED" : (pendingLock ? " • RE-LOCK…" : ""));
      const mimicState = isMimicOn() ? " • MIRRORING" : "";
      counter.textContent = `${label}: ${count}${lockState}${mimicState}`;
      counter.title = trackError || lastError || "People detected on the webcam feed";
    } else {
      counter.textContent = `${label}: —`;
      counter.title = lastError;
    }
  };

  // Push architecture: the robot server runs one detect loop (~15 Hz) only
  // while this EventSource is open, and pushes results here as they land —
  // no request/response phase lag. Closing the stream stops all detection.
  let es = null;
  const handleResult = (data) => {
    const now = Date.now();
    if (data.ok) {
      lastError = null;
      const persons = data.persons || [];
      associate(persons, now);
      count = persons.length;
    } else {
      lastError = data.error || "Detection failed.";
      count = null;
      associate([], now); // age out tracks while errors persist
    }
    renderButtons();
    renderCounter();
  };
  const openStream = () => {
    if (es) return;
    es = new EventSource("/api/sentry/stream");
    es.onmessage = (event) => {
      try { handleResult(JSON.parse(event.data)); } catch {}
    };
    es.onerror = () => { lastError = "Bullseye stream reconnecting…"; renderCounter(); };
  };
  const closeStream = () => {
    if (es) { es.close(); es = null; }
  };

  window.setInterval(() => {
    const active = (isOn() || isMimicOn()) && !panel.classList.contains("hidden")
      && !img.classList.contains("hidden") && !!img.getAttribute("src");
    if (!active) { closeStream(); clearAllTracks(); return; }
    openStream();
  }, POLL_MS);

  // 60 fps spring interpolation: SSE updates set each button's target pixel
  // position; this loop glides the rendered position toward it every frame.
  const animate = () => {
    tracks.forEach((track) => {
      if (!track.btn || track.tx === undefined || track.hover) return;
      track.rx = track.rx === undefined ? track.tx : track.rx + (track.tx - track.rx) * 0.28;
      track.ry = track.ry === undefined ? track.ty : track.ry + (track.ty - track.ry) * 0.28;
      track.btn.style.left = `${track.rx}px`;
      track.btn.style.top = `${track.ry}px`;
    });
    window.requestAnimationFrame(animate);
  };
  window.requestAnimationFrame(animate);
})();

// ---- Mimic Mode: the robot mirrors the person's upper-body pose (both arms)
// from webcam pose keypoints. ON immediately starts motion, so the toggle
// itself carries the risk acknowledgement after an explicit confirm.
(function setupMimic() {
  const toggle = document.getElementById("mimicModeToggle");
  if (!toggle) return;

  let serverOn = false;
  let busy = false;
  const render = () => {
    toggle.classList.toggle("on", serverOn);
    toggle.setAttribute("aria-pressed", serverOn ? "true" : "false");
    toggle.title = serverOn
      ? "Mimic Mode is ON — the robot mirrors your arms. Click to stop."
      : "Mimic Mode — the robot mirrors your upper-body pose from the webcam";
    // The Bullseye panel reads this to draw the skeleton overlay and keep
    // the webcam detection stream open while mimic runs.
    document.body.dataset.mimicOn = serverOn ? "1" : "0";
  };
  const apply = (flag) => {
    if (typeof flag === "boolean" && flag !== serverOn) {
      serverOn = flag;
      render();
    }
  };
  const push = async (on) => {
    if (busy) return;
    busy = true;
    try {
      const body = on
        ? { on: true, armed: true, i_understand_risk: true }
        : { on: false };
      const resp = await fetch("/api/mimic/mode", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const data = await resp.json();
      apply(!!data.mimic_mode);
      if (on && !data.mimic_mode && data.error) {
        window.alert(`Mimic Mode did not start: ${data.error}`);
      }
    } catch {
      // Status poll below re-syncs on the next tick.
    } finally {
      busy = false;
    }
  };
  toggle.addEventListener("click", () => {
    // No confirm dialog (operator request 2026-07-28): the toggle click IS
    // the deliberate action; the server-side risk ack rides on the request
    // and every motion interlock (limits, rate limiter, staleness park)
    // stays active.
    void push(!serverOn);
  });
  const sync = () => fetch("/api/track/status")
    .then((resp) => resp.json())
    .then((data) => apply(!!(data.tracking || {}).mimic_mode))
    .catch(() => {});
  render();
  sync();
  window.setInterval(() => { if (!document.hidden) sync(); }, 2000);
})();
