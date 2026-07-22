# Sentry Head-Lock Buttons (Phase 2 UI) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Sentry bounding boxes with glowing lock buttons that hover over each detected person's head in the webcam feed, follow the head smoothly, and lock onto a person when clicked (pure UI state, no robot motion).

**Architecture:** All frontend. The existing 4 Hz `/api/sentry/detect?feed=webcam` poll now feeds a small client-side tracker (greedy nearest-center association + exponential smoothing) instead of a canvas painter. Each track renders one absolutely-positioned HTML `<button>` in a new `#floatWebcamTargets` layer over the webcam `<img>`; clicking toggles/switches a single `lockedId`. No server changes.

**Tech Stack:** Vanilla JS/CSS/HTML, `unittest` contract test.

**Spec:** `docs/superpowers/specs/2026-07-22-sentry-head-lock-buttons-design.md`

## Global Constraints

- Repo rule: commit **and push**; commit message ends with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- No motion code path touched; no server.py changes at all in this plan.
- Full suite (`python3 -m unittest discover -s tests -p 'test_*.py'`) and `make production-gate` must pass before push.
- Work in `~/Workspace/humanoid-robot-gui` on `main`.
- Association threshold 0.18 (normalized center distance), smoothing alpha 0.45, track TTL 2000 ms, poll stays 250 ms — exact values.

---

### Task 1: Head-tracked lock buttons replacing boxes

**Files:**
- Modify: `static/index.html` (add targets layer in `.float-cam-body`; bump `app.js?v=` to `sentry-v3`)
- Modify: `static/app.js` (replace the ENTIRE `setupSentry` IIFE — from the comment line `// ---- Sentry Mode: passive person-detection boxes on the floating camera ----` through its closing `})();` — with the new module below)
- Modify: `static/styles.css` (append new styles)
- Test: `tests/test_sentry.py` (extend `SentryFrontendContractTests`)

**Interfaces:**
- Consumes: `GET /api/sentry/detect?feed=webcam` (`{ok, feed, persons:[{x1,y1,x2,y2,cx,cy,conf}], ts}` normalized 0..1 / `{ok:false,error}`); DOM ids `floatCam`, `floatWebcamStream`, `sentryToggle`, `floatCamSentry`; localStorage key `h1_sentry_mode`.
- Produces: DOM id `floatWebcamTargets`, CSS class `target-lock-btn` (+ `.locked`), single-lock state `lockedId` (Phase 3 will read this module's lock to feed the server). Existing overlay canvases stay in the DOM, untouched and undrawn.

- [ ] **Step 1: Write the failing contract test**

Append inside `SentryFrontendContractTests` in `tests/test_sentry.py`:

```python
    def test_lock_button_wiring_present(self):
        with open("static/index.html") as fh:
            html = fh.read()
        self.assertIn("floatWebcamTargets", html)
        with open("static/app.js") as fh:
            js = fh.read()
        for needle in ("target-lock-btn", "floatWebcamTargets", "LOCKED"):
            self.assertIn(needle, js)
        with open("static/styles.css") as fh:
            css = fh.read()
        self.assertIn("target-pulse", css)
```

- [ ] **Step 2: Run it to verify it fails**

Run: `python3 -m unittest tests.test_sentry.SentryFrontendContractTests -v`
Expected: FAIL — `floatWebcamTargets` not found.

- [ ] **Step 3: Add the markup**

In `static/index.html`, inside `.float-cam-body`, immediately after the two `<canvas>` elements add:

```html
        <div id="floatWebcamTargets" class="float-cam-targets hidden"></div>
```

Bump the cache-buster on the same page: `/app.js?v=sentry-v2` → `/app.js?v=sentry-v3`.

- [ ] **Step 4: Add the styles**

Append to `static/styles.css`:

```css
/* ---- Sentry Mode Phase 2: head-tracked lock buttons ---- */
.float-cam-targets {
  position: absolute;
  left: 0;
  top: 0;
  z-index: 3;
  pointer-events: none;
  overflow: hidden;
}
.float-cam-targets.hidden { display: none; }
.target-lock-btn {
  position: absolute;
  transform: translate(-50%, -100%);
  width: 30px;
  height: 30px;
  border-radius: 50%;
  border: 2px solid rgba(255, 255, 255, 0.85);
  background: rgba(230, 0, 0, 0.78);
  color: #fff;
  font-size: 14px;
  line-height: 1;
  padding: 0;
  cursor: pointer;
  pointer-events: auto;
  animation: target-pulse 1.6s ease-in-out infinite;
  transition: left 0.12s linear, top 0.12s linear;
}
.target-lock-btn.hidden { display: none; }
.target-lock-btn.locked {
  background: rgba(18, 178, 82, 0.9);
  animation: target-pulse-locked 1.6s ease-in-out infinite;
}
@keyframes target-pulse {
  0%, 100% { box-shadow: 0 0 6px 2px rgba(230, 0, 0, 0.55); }
  50% { box-shadow: 0 0 18px 7px rgba(230, 0, 0, 0.85); }
}
@keyframes target-pulse-locked {
  0%, 100% { box-shadow: 0 0 6px 2px rgba(18, 178, 82, 0.6); }
  50% { box-shadow: 0 0 18px 7px rgba(18, 178, 82, 0.9); }
}
```

- [ ] **Step 5: Replace the `setupSentry` IIFE in `static/app.js`**

Delete the entire existing block from `// ---- Sentry Mode: passive person-detection boxes on the floating camera ----` through its closing `})();` and put this in its place:

```js
// ---- Sentry Mode: head-tracked glowing lock buttons on the webcam feed ----
(function setupSentry() {
  const KEY = "h1_sentry_mode";
  const toggle = document.getElementById("sentryToggle");
  const panel = document.getElementById("floatCam");
  const counter = document.getElementById("floatCamSentry");
  const img = document.getElementById("floatWebcamStream");
  const layer = document.getElementById("floatWebcamTargets");
  if (!toggle || !panel || !img || !layer) return;

  const MATCH_DIST = 0.18;    // normalized center distance for association
  const SMOOTH_ALPHA = 0.45;  // exponential smoothing for box coords
  const TRACK_TTL_MS = 2000;  // drop tracks unseen this long

  let inFlight = false;
  let tracks = [];            // {id, x1, y1, x2, y2, lastSeen, btn}
  let nextTrackId = 1;
  let lockedId = null;
  let count = null;           // persons in last good detection, or null
  let lastError = null;

  const isOn = () => localStorage.getItem(KEY) === "1";
  const renderToggle = () => {
    toggle.classList.toggle("on", isOn());
    toggle.setAttribute("aria-pressed", isOn() ? "true" : "false");
  };
  toggle.addEventListener("click", () => {
    try { localStorage.setItem(KEY, isOn() ? "0" : "1"); } catch {}
    renderToggle();
  });
  renderToggle();

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

  const clearAllTracks = () => {
    tracks.forEach(removeTrack);
    tracks = [];
    lockedId = null;
    layer.classList.add("hidden");
    count = null;
    lastError = null;
    if (counter) { counter.classList.add("hidden"); counter.textContent = ""; counter.title = ""; }
  };

  const center = (box) => ({ cx: (box.x1 + box.x2) / 2, cy: (box.y1 + box.y2) / 2 });

  const associate = (persons, now) => {
    const unmatched = tracks.slice();
    persons.forEach((person) => {
      const pc = center(person);
      let best = null;
      let bestDist = MATCH_DIST;
      unmatched.forEach((track) => {
        const tc = center(track);
        const dist = Math.hypot(pc.cx - tc.cx, pc.cy - tc.cy);
        if (dist < bestDist) { best = track; bestDist = dist; }
      });
      if (best) {
        unmatched.splice(unmatched.indexOf(best), 1);
        ["x1", "y1", "x2", "y2"].forEach((key) => {
          best[key] += SMOOTH_ALPHA * (person[key] - best[key]);
        });
        best.lastSeen = now;
      } else {
        tracks.push({
          x1: person.x1, y1: person.y1, x2: person.x2, y2: person.y2,
          id: nextTrackId++, lastSeen: now, btn: null,
        });
      }
    });
    tracks = tracks.filter((track) => {
      if (now - track.lastSeen > TRACK_TTL_MS) { removeTrack(track); return false; }
      return true;
    });
  };

  const buttonFor = (track) => {
    if (track.btn) return track.btn;
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "target-lock-btn";
    btn.textContent = "🔒";
    btn.addEventListener("click", (event) => {
      event.stopPropagation();
      lockedId = lockedId === track.id ? null : track.id;
      renderButtons();
      renderCounter();
    });
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
      if (!t) { btn.classList.add("hidden"); return; }
      btn.classList.remove("hidden");
      const headX = t.ox + ((track.x1 + track.x2) / 2) * t.dw;
      const headY = t.oy + track.y1 * t.dh;
      btn.style.left = `${headX}px`;
      btn.style.top = `${Math.max(0, headY - 6)}px`;
      const locked = lockedId === track.id;
      btn.classList.toggle("locked", locked);
      btn.title = locked ? "Kilidi kaldır" : "Bu kişiye kitlen";
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
    if (count !== null) {
      counter.textContent = `Sentry: ${count}${lockedId !== null ? " • LOCKED" : ""}`;
      counter.title = lastError || "People detected on the webcam feed";
    } else {
      counter.textContent = "Sentry: —";
      counter.title = lastError;
    }
  };

  const poll = async () => {
    if (inFlight) return;
    if (img.classList.contains("hidden") || !img.getAttribute("src")) {
      clearAllTracks();
      return;
    }
    inFlight = true;
    try {
      const resp = await fetch("/api/sentry/detect?feed=webcam", { cache: "no-store" });
      const data = await resp.json();
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
    } catch {
      lastError = "Sentry request failed.";
      count = null;
    } finally {
      inFlight = false;
      renderCounter();
    }
  };

  window.setInterval(() => {
    const active = isOn() && !panel.classList.contains("hidden");
    if (!active) { clearAllTracks(); return; }
    poll();
  }, 250);
})();
```

- [ ] **Step 6: Run the contract tests to verify they pass**

Run: `python3 -m unittest tests.test_sentry -v`
Expected: all tests PASS (including the two frontend contract tests).

- [ ] **Step 7: Full suite + gate**

Run: `python3 -m unittest discover -s tests -p 'test_*.py'` then `make production-gate`
Expected: all pass; the gate's `node --check static/app.js` confirms the new JS parses.

- [ ] **Step 8: Commit and push**

```bash
git add static/index.html static/app.js static/styles.css tests/test_sentry.py
git commit -m "Sentry Mode: replace boxes with head-tracked glowing lock buttons (single-lock UI)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git push
```
