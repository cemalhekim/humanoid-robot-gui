const state = {
  latest: null,
  filter: "",
};

const els = {
  subtitle: document.getElementById("subtitle"),
  connection: document.getElementById("connection"),
  age: document.getElementById("age"),
  rate: document.getElementById("rate"),
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

function render(snapshot) {
  state.latest = snapshot;
  const connected = Boolean(snapshot.connected);
  const age = snapshot.timestamp ? Math.max(0, Date.now() / 1000 - snapshot.timestamp) : null;

  els.connection.textContent = connected ? "Connected" : "Disconnected";
  els.connection.className = `pill ${connected ? "good" : "bad"}`;
  els.age.textContent = `age ${age === null ? "--" : age.toFixed(1)}s`;
  els.rate.textContent = `${fmt(snapshot.sample_rate_hz)} Hz`;
  els.subtitle.textContent = connected
    ? `${fmt(snapshot.motor_count)} motors, ${fmt(snapshot.samples)} samples`
    : snapshot.error || "Waiting for rt/lowstate";

  setFields(els.robotFields, snapshot.robot);
  setFields(els.imuFields, snapshot.imu);
  setFields(els.batteryFields, snapshot.battery);
  const handSummary = snapshot.hands || {};
  const handFields = {
    connected: handSummary.connected,
    topic: handSummary.topic,
    samples: handSummary.samples,
    joint_count: handSummary.joint_count,
  };
  for (const joint of handSummary.joints || []) {
    handFields[joint.name] = joint.q;
  }
  if (handSummary.note) handFields.note = handSummary.note;
  setFields(els.handFields, handFields);
  setFields(els.forceFields, {
    foot_force: snapshot.foot_force,
    foot_force_est: snapshot.foot_force_est,
  });
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
