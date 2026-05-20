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
  cameraStream: document.getElementById("cameraStream"),
  cameraPlaceholder: document.getElementById("cameraPlaceholder"),
  rosSummary: document.getElementById("rosSummary"),
  rosMap: document.getElementById("rosMap"),
  rosEdges: document.getElementById("rosEdges"),
  refreshRosGraph: document.getElementById("refreshRosGraph"),
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

function connectCameraPreview() {
  if (!els.cameraStream || !els.cameraPlaceholder) return;
  const feed = `/camera.mjpg?ts=${Date.now()}`;
  els.cameraStream.src = feed;
  els.cameraStream.addEventListener("load", () => {
    els.cameraPlaceholder.classList.add("hidden");
  });
  els.cameraStream.addEventListener("error", () => {
    els.cameraPlaceholder.classList.remove("hidden");
  });
  fetch("/api/camera")
    .then((response) => response.json())
    .then((camera) => {
      if (camera.available) {
        els.cameraPlaceholder.classList.add("hidden");
      }
    })
    .catch(() => {});
}

function renderRosGraph(graph) {
  if (!els.rosMap || !els.rosEdges || !els.rosSummary) return;
  const nodes = graph.nodes || [];
  const subscriptions = graph.subscriptions || [];
  const publishers = graph.publishers || [];
  const topics = Object.keys(graph.topics || {});
  const nodeNames = new Set(nodes.map((node) => node.name));
  const activeTopics = Array.from(
    new Set([...publishers.map((edge) => edge.topic), ...subscriptions.map((edge) => edge.topic)]),
  ).sort();
  const publisherNodes = new Set(publishers.map((edge) => edge.node));
  const subscriberNodes = new Set(subscriptions.map((edge) => edge.node));
  const graphNodes = Array.from(new Set([...publisherNodes, ...subscriberNodes, ...nodeNames])).sort();
  const width = Math.max(980, 360 + activeTopics.length * 24);
  const rowHeight = 78;
  const height = Math.max(520, Math.max(graphNodes.length, activeTopics.length) * rowHeight + 80);
  const nodeXLeft = 170;
  const topicX = width / 2;
  const nodeXRight = width - 170;
  const nodeY = new Map(graphNodes.map((name, index) => [name, 60 + index * rowHeight]));
  const topicY = new Map(activeTopics.map((name, index) => [name, 60 + index * rowHeight]));

  els.rosSummary.innerHTML = `
    <span>Nodes <strong>${fmt(nodes.length)}</strong></span>
    <span>Subscriptions <strong>${fmt(subscriptions.length)}</strong></span>
    <span>Publishers <strong>${fmt(publishers.length)}</strong></span>
    <span>Topics <strong>${fmt(topics.length)}</strong></span>
  `;

  if (graph.error && nodes.length === 0) {
    els.rosMap.innerHTML = `<div class="ros-empty">ROS graph unavailable: ${esc(graph.error)}</div>`;
    els.rosEdges.innerHTML = "";
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
  const topicShapes = activeTopics
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

  els.rosEdges.innerHTML = subscriptions.length
    ? subscriptions
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
els.refreshRosGraph?.addEventListener("click", loadRosGraph);
syncActiveNav();
connectCameraPreview();
loadRosGraph();
connectEvents();
