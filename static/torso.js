// Torso ("belly") twist rotary control. Hold and drag the dial to rotate the
// upper body about WaistYaw via /api/torso/twist. Self-contained; no deps.
(function () {
  "use strict";

  const dial = document.getElementById("torsoDial");
  if (!dial) return;

  const knob = document.getElementById("torsoKnob");
  const pointerLine = document.getElementById("torsoPointer");
  const angleText = document.getElementById("torsoAngle");
  const liveText = document.getElementById("torsoLive");
  const limitText = document.getElementById("torsoLimits");
  const stateEl = document.getElementById("torsoState");
  const armToggle = document.getElementById("torsoArm");
  const centerBtn = document.getElementById("torsoCenter");
  const releaseBtn = document.getElementById("torsoRelease");

  const CENTER = 120;
  const R = 95;
  let limit = 1.0; // rad, refreshed from status
  let target = 0; // rad
  let dragging = false;
  let lastSend = 0;
  let lastActive = false;

  const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
  const deg = (rad) => (rad * 180) / Math.PI;

  function angleToXY(rad) {
    return { x: CENTER + R * Math.sin(rad), y: CENTER - R * Math.cos(rad) };
  }

  function setHandle(rad) {
    const p = angleToXY(rad);
    if (knob) {
      knob.setAttribute("cx", p.x.toFixed(2));
      knob.setAttribute("cy", p.y.toFixed(2));
    }
    if (pointerLine) {
      pointerLine.setAttribute("x2", p.x.toFixed(2));
      pointerLine.setAttribute("y2", p.y.toFixed(2));
    }
    if (angleText) angleText.textContent = `${deg(rad).toFixed(1)}°`;
  }

  function pointerToAngle(evt) {
    const rect = dial.getBoundingClientRect();
    const x = ((evt.clientX - rect.left) / rect.width) * 240;
    const y = ((evt.clientY - rect.top) / rect.height) * 240;
    const rad = Math.atan2(x - CENTER, -(y - CENTER)); // clockwise from top
    return clamp(rad, -limit, limit);
  }

  const armed = () => Boolean(armToggle && armToggle.checked);

  async function sendTwist(rad, force) {
    if (!armed()) return;
    const now = Date.now();
    if (!force && now - lastSend < 55) return; // throttle ~18 Hz
    lastSend = now;
    try {
      await fetch("/api/torso/twist", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ target_q: rad, armed: true, i_understand_risk: true }),
      });
    } catch (_) {
      /* ignore transient errors */
    }
  }

  function renderState(active) {
    if (!stateEl) return;
    if (!armed()) {
      stateEl.textContent = "Disarmed";
      stateEl.className = "pill bad";
    } else if (active) {
      stateEl.textContent = "Twisting";
      stateEl.className = "pill good";
    } else {
      stateEl.textContent = "Ready";
      stateEl.className = "pill warn";
    }
  }

  function onDown(e) {
    if (!armed()) {
      renderState(false);
      dial.classList.add("shake");
      setTimeout(() => dial.classList.remove("shake"), 300);
      return;
    }
    dragging = true;
    dial.classList.add("dragging");
    if (dial.setPointerCapture) dial.setPointerCapture(e.pointerId);
    target = pointerToAngle(e);
    setHandle(target);
    sendTwist(target, true);
  }

  function onMove(e) {
    if (!dragging) return;
    target = pointerToAngle(e);
    setHandle(target);
    sendTwist(target, false);
  }

  function onUp(e) {
    if (!dragging) return;
    dragging = false;
    dial.classList.remove("dragging");
    if (dial.releasePointerCapture) {
      try { dial.releasePointerCapture(e.pointerId); } catch (_) { /* ignore */ }
    }
    sendTwist(target, true); // hold at the final target
  }

  dial.addEventListener("pointerdown", onDown);
  dial.addEventListener("pointermove", onMove);
  dial.addEventListener("pointerup", onUp);
  dial.addEventListener("pointercancel", onUp);

  if (centerBtn) {
    centerBtn.addEventListener("click", () => {
      target = 0;
      setHandle(0);
      sendTwist(0, true);
    });
  }
  if (releaseBtn) {
    releaseBtn.addEventListener("click", async () => {
      try { await fetch("/api/torso/stop", { method: "POST" }); } catch (_) { /* ignore */ }
      if (armToggle) armToggle.checked = false;
      renderState(false);
    });
  }
  if (armToggle) armToggle.addEventListener("change", () => renderState(lastActive));

  async function poll() {
    try {
      const res = await fetch("/api/torso/status");
      const s = await res.json();
      const lim = s && s.joint && s.joint.limits;
      if (lim && Number.isFinite(lim.max)) {
        limit = lim.max;
        if (limitText) limitText.textContent = `±${Math.round(deg(limit))}°`;
      }
      const tel = s && s.joint && s.joint.telemetry;
      if (tel && liveText) liveText.textContent = `${deg(Number(tel.q || 0)).toFixed(1)}°`;
      lastActive = Boolean(s && s.active);
      renderState(lastActive);
    } catch (_) {
      /* ignore */
    }
  }

  setHandle(0);
  renderState(false);
  poll();
  setInterval(poll, 1000);
})();
