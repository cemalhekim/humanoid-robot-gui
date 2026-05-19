const state = {
  latest: null,
  filter: "",
};

const els = {
  subtitle: document.getElementById("subtitle"),
  connection: document.getElementById("connection"),
  age: document.getElementById("age"),
  rate: document.getElementById("rate"),
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
  filter: document.getElementById("filter"),
  navItems: document.querySelectorAll(".nav-item"),
};

function fmt(value, suffix = "") {
  if (value === undefined || value === null || value === "") return "--";
  if (Array.isArray(value)) return `[${value.map((item) => fmt(item)).join(", ")}]`;
  if (typeof value === "number") return `${Number.isInteger(value) ? value : value.toFixed(3)}${suffix}`;
  if (typeof value === "boolean") return value ? "true" : "false";
  return String(value);
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

  renderMetricCards(
    els.robotFields,
    [
      { label: "Mode", value: robot.mode_machine ?? "--", tone: "red" },
      { label: "Samples", value: snapshot.samples ?? 0 },
      { label: "Motors", value: snapshot.motor_count ?? 0 },
    ],
    [
      { label: "Control", value: robot.mode_pr },
      { label: "Tick", value: robot.tick },
      { label: "CRC", value: robot.crc },
    ],
  );

  renderStatusList(
    els.imuFields,
    [
      { label: "Roll", value: imu.rpy?.[0] },
      { label: "Pitch", value: imu.rpy?.[1] },
      { label: "Yaw", value: imu.rpy?.[2] },
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
      { label: "Foot force", value: snapshot.foot_force?.length ? snapshot.foot_force : "--" },
      { label: "Estimated", value: snapshot.foot_force_est?.length ? snapshot.foot_force_est : "--" },
      ...valueList(snapshot.foot_force, ["FL", "FR", "RL", "RR"]),
      ...valueList(snapshot.foot_force_est, ["Est FL", "Est FR", "Est RL", "Est RR"]),
    ],
  );
}

function renderMotors(motors) {
  const query = state.filter.trim().toLowerCase();
  const rows = (motors || []).filter((motor) => {
    if (!query) return true;
    return String(motor.index).includes(query) || String(motor.name).toLowerCase().includes(query);
  });

  els.motorRows.innerHTML = rows
    .map((motor) => {
      const isExtra = String(motor.name).startsWith("ReservedMotorSlot");
      return `
        <tr>
          <td>${fmt(motor.index)}</td>
          <td class="${isExtra ? "extra" : "joint"}">${fmt(motor.name)}</td>
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
  renderMotors(snapshot.motors);
  els.rawJson.textContent = JSON.stringify(snapshot, null, 2);
  window.dispatchEvent(new CustomEvent("telemetry-state", { detail: { snapshot } }));
}

els.filter.addEventListener("input", () => {
  state.filter = els.filter.value;
  if (state.latest) renderMotors(state.latest.motors);
});

function syncActiveNav() {
  const activeHash = window.location.hash || "#dashboard";
  els.navItems.forEach((item) => {
    item.classList.toggle("active", item.getAttribute("href") === activeHash);
  });
}

function connectEvents() {
  const events = new EventSource("/events");
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

fetch("/api/state")
  .then((response) => response.json())
  .then(render)
  .catch(() => {});
window.addEventListener("hashchange", syncActiveNav);
syncActiveNav();
connectEvents();
