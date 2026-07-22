# Sentry Mode Phase 1 (Passive Person-Detection Overlay) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Draw live YOLO person-detection boxes on both floating-camera feeds (head camera + USB webcam) when a "Sentry Mode" toggle is on and the floating camera is open — with zero robot motion.

**Architecture:** The browser polls a new read-only `GET /api/sentry/detect?feed=head|webcam` endpoint (~4 Hz per visible feed). The endpoint forwards the latest cached JPEG for that feed to the AI-host YOLO service (`TRACKING_DETECT_URL`, `http://10.2.125.3:8188/detect`) and returns normalized person boxes. Transparent canvases positioned over each `<img>` draw the boxes, mapping normalized coords through the `object-fit: cover` crop transform. No server-side loop: when the UI stops polling, detection traffic stops.

**Tech Stack:** Python stdlib (`http.server`, `urllib`), vanilla JS/canvas, `unittest`.

**Spec:** `docs/superpowers/specs/2026-07-22-sentry-mode-person-detection-design.md`

## Global Constraints

- Repo rule: commit **and push** after every task; every commit message ends with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- This feature must not touch any motion code path (arm_sdk, loco, wrist, XR publishers). No risk-ack anywhere.
- Full test suite (`python3 -m unittest discover -s tests -p 'test_*.py'`) and `make production-gate` must pass before each push.
- All work happens in `~/Workspace/humanoid-robot-gui` on `main`.

---

### Task 1: `/api/sentry/detect` endpoint (server)

**Files:**
- Modify: `server.py` (new `TelemetryStore.sentry_detect()` next to `track_snapshot()` ~line 4243; new route in `do_GET` next to `"/api/track/status"` ~line 5711)
- Test: `tests/test_sentry.py` (new file)

**Interfaces:**
- Consumes: existing `TelemetryStore.get_camera_frame()`, `self.webcam_frame`/`self.webcam_lock`, `TRACKING_DETECT_URL`, `self._send_json()`.
- Produces: `TelemetryStore.sentry_detect(feed: str = "head") -> dict` returning `{"ok": True, "feed": feed, "persons": [...], "ts": float}` or `{"ok": False, "error": str}`; HTTP route `GET /api/sentry/detect?feed=head|webcam`. Task 2's poll loop relies on exactly these shapes.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_sentry.py`:

```python
import json
import unittest
from unittest import mock

import server


def fake_response(payload: dict) -> mock.MagicMock:
    resp = mock.MagicMock()
    resp.read.return_value = json.dumps(payload).encode()
    resp.__enter__.return_value = resp
    resp.__exit__.return_value = False
    return resp


class SentryDetectTests(unittest.TestCase):
    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_no_head_frame(self):
        result = self.make_store().sentry_detect("head")
        self.assertFalse(result["ok"])
        self.assertIn("No head frame", result["error"])

    def test_unknown_feed(self):
        result = self.make_store().sentry_detect("thermal")
        self.assertFalse(result["ok"])
        self.assertIn("Unknown feed", result["error"])

    def test_head_persons_passed_through(self):
        store = self.make_store()
        store.set_camera_frame(b"jpeg-bytes")
        persons = [{"x1": 0.1, "y1": 0.2, "x2": 0.3, "y2": 0.9,
                    "cx": 0.2, "cy": 0.55, "conf": 0.9}]
        with mock.patch.object(server.urllib.request, "urlopen",
                               return_value=fake_response({"persons": persons})):
            result = store.sentry_detect("head")
        self.assertTrue(result["ok"])
        self.assertEqual(result["feed"], "head")
        self.assertEqual(result["persons"], persons)
        self.assertIn("ts", result)

    def test_webcam_feed_uses_webcam_frame(self):
        store = self.make_store()
        store.set_webcam_frame(b"jpeg-bytes")
        with mock.patch.object(server.urllib.request, "urlopen",
                               return_value=fake_response({"persons": []})):
            result = store.sentry_detect("webcam")
        self.assertTrue(result["ok"])
        self.assertEqual(result["feed"], "webcam")
        self.assertEqual(result["persons"], [])

    def test_detect_service_down(self):
        store = self.make_store()
        store.set_camera_frame(b"jpeg-bytes")
        with mock.patch.object(server.urllib.request, "urlopen",
                               side_effect=OSError("boom")):
            result = store.sentry_detect("head")
        self.assertFalse(result["ok"])
        self.assertIn("unreachable", result["error"])


class SentryRouteTests(unittest.TestCase):
    def test_route_dispatched(self):
        with open("server.py") as fh:
            src = fh.read()
        self.assertIn('"/api/sentry/detect"', src)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m unittest tests.test_sentry -v`
Expected: FAIL — `AttributeError: 'TelemetryStore' object has no attribute 'sentry_detect'` (and the route assertion fails).

- [ ] **Step 3: Implement `sentry_detect` in `TelemetryStore`**

In `server.py`, directly above `def track_snapshot(self)` (~line 4243), add:

```python
    def sentry_detect(self, feed: str = "head") -> dict[str, Any]:
        """Sentry Mode (detection only): forward one cached frame to the YOLO
        service and return its person boxes. Never touches motion paths."""
        if feed == "head":
            frame = self.get_camera_frame()
        elif feed == "webcam":
            with self.webcam_lock:
                frame = self.webcam_frame
        else:
            return {"ok": False, "error": "Unknown feed."}
        if not frame:
            return {"ok": False, "error": f"No {feed} frame."}
        try:
            req = urllib.request.Request(
                TRACKING_DETECT_URL, data=frame,
                headers={"Content-Type": "image/jpeg"}, method="POST",
            )
            with urllib.request.urlopen(req, timeout=0.5) as resp:
                persons = json.loads(resp.read()).get("persons", [])
        except Exception:
            return {"ok": False, "error": "Detection service unreachable."}
        return {"ok": True, "feed": feed, "persons": persons, "ts": time.time()}
```

- [ ] **Step 4: Add the HTTP route**

In `do_GET`, after the `"/api/track/status"` elif (~line 5711), add:

```python
        elif request_path == "/api/sentry/detect":
            query = parse_qs(urlsplit(self.path).query)
            feed = (query.get("feed") or ["head"])[0]
            self._send_json(self.store.sentry_detect(feed))
```

Check the `from urllib.parse import ...` line at the top of `server.py`: if `parse_qs` is not already imported, add it to that import list.

- [ ] **Step 5: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_sentry -v`
Expected: all 6 tests PASS.

- [ ] **Step 6: Run the full suite + gate**

Run: `python3 -m unittest discover -s tests -p 'test_*.py'` then `make production-gate`
Expected: all pass, gate green.

- [ ] **Step 7: Commit and push**

```bash
git add tests/test_sentry.py server.py
git commit -m "Add /api/sentry/detect: detection-only person boxes for Sentry Mode

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git push
```

---

### Task 2: Sentry Mode toggle + overlay canvases (frontend)

**Files:**
- Modify: `static/index.html` (status strip ~line 78–84, float-cam markup ~line 663–677, `app.js?v=` cache-buster ~line 679)
- Modify: `static/app.js` (new IIFE after `setupFloatCam`, which ends ~line 3230)
- Modify: `static/styles.css` (append at end)
- Test: `tests/test_sentry.py` (add a frontend-contract test class)

**Interfaces:**
- Consumes: `GET /api/sentry/detect?feed=head|webcam` from Task 1 (`{ok, feed, persons:[{x1,y1,x2,y2,conf}], ts}` / `{ok:false, error}`); existing DOM ids `floatCam`, `floatCamStream`, `floatWebcamStream`; float-cam open state = `#floatCam` not having class `hidden`.
- Produces: DOM ids `sentryToggle`, `floatCamOverlay`, `floatWebcamOverlay`, `floatCamSentry`; localStorage key `h1_sentry_mode` (`"1"`/`"0"`). Phase 2 (target lock) will build on this module.

- [ ] **Step 1: Write the failing contract test**

Append to `tests/test_sentry.py`:

```python
class SentryFrontendContractTests(unittest.TestCase):
    def test_ui_wiring_present(self):
        with open("static/index.html") as fh:
            html = fh.read()
        for needle in ("sentryToggle", "floatCamOverlay",
                       "floatWebcamOverlay", "floatCamSentry"):
            self.assertIn(needle, html)
        with open("static/app.js") as fh:
            js = fh.read()
        self.assertIn("/api/sentry/detect", js)
        self.assertIn("h1_sentry_mode", js)
```

- [ ] **Step 2: Run it to verify it fails**

Run: `python3 -m unittest tests.test_sentry.SentryFrontendContractTests -v`
Expected: FAIL — `sentryToggle` not found in `static/index.html`.

- [ ] **Step 3: Add the markup**

In `static/index.html` status strip, insert between the closing `</span>` of the `robot-power` span and `<span id="networkType" ...>`:

```html
            <button id="sentryToggle" class="sentry-toggle" type="button" aria-pressed="false" title="Sentry Mode — draw person-detection boxes on the floating camera (no robot motion)">Sentry Mode</button>
```

In the float-cam header, after the text `Robot View` and before the minimize button:

```html
        <span id="floatCamSentry" class="float-cam-sentry hidden"></span>
```

In `.float-cam-body`, after the two `<img>` elements:

```html
        <canvas id="floatCamOverlay" class="float-cam-overlay hidden" aria-hidden="true"></canvas>
        <canvas id="floatWebcamOverlay" class="float-cam-overlay hidden" aria-hidden="true"></canvas>
```

Bump the script cache-buster on the same page: `/app.js?v=loco-console-v4` → `/app.js?v=sentry-v1`.

- [ ] **Step 4: Add the styles**

Append to `static/styles.css`:

```css
/* ---- Sentry Mode: passive person-detection overlay ---- */
.sentry-toggle {
  border: 1px solid rgba(230, 0, 0, 0.55);
  background: transparent;
  color: var(--muted);
  font: inherit;
  font-size: 12px;
  font-weight: 650;
  letter-spacing: 0.04em;
  padding: 6px 12px;
  border-radius: 999px;
  cursor: pointer;
}
.sentry-toggle.on {
  background: #e60000;
  border-color: #e60000;
  color: #fff;
  box-shadow: 0 0 14px rgba(230, 0, 0, 0.45);
}
.float-cam-body { position: relative; }
.float-cam-overlay {
  position: absolute;
  left: 0;
  top: 0;
  pointer-events: none;
  z-index: 2;
}
.float-cam-overlay.hidden { display: none; }
.float-cam-sentry { color: #ff8080; font-size: 11px; letter-spacing: 0.06em; }
.float-cam-sentry.hidden { display: none; }
```

- [ ] **Step 5: Add the JS module**

In `static/app.js`, immediately after the `setupFloatCam` IIFE closes, add:

```js
// ---- Sentry Mode: passive person-detection boxes on the floating camera ----
(function setupSentry() {
  const KEY = "h1_sentry_mode";
  const toggle = document.getElementById("sentryToggle");
  const panel = document.getElementById("floatCam");
  const counter = document.getElementById("floatCamSentry");
  const feeds = [
    { name: "head", img: document.getElementById("floatCamStream"),
      canvas: document.getElementById("floatCamOverlay"), inFlight: false, count: null },
    { name: "webcam", img: document.getElementById("floatWebcamStream"),
      canvas: document.getElementById("floatWebcamOverlay"), inFlight: false, count: null },
  ].filter((feed) => feed.img && feed.canvas);
  if (!toggle || !panel || !feeds.length) return;

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

  const clearFeed = (feed) => {
    const ctx = feed.canvas.getContext("2d");
    ctx.clearRect(0, 0, feed.canvas.width, feed.canvas.height);
    feed.canvas.classList.add("hidden");
    feed.count = null;
  };
  const clearAll = () => {
    feeds.forEach(clearFeed);
    if (counter) { counter.classList.add("hidden"); counter.textContent = ""; counter.title = ""; }
  };

  // The stream images render with object-fit: cover — map normalized
  // detection coords through the centered-crop transform.
  const coverTransform = (img) => {
    const ew = img.clientWidth, eh = img.clientHeight;
    const nw = img.naturalWidth, nh = img.naturalHeight;
    if (!ew || !eh || !nw || !nh) return null;
    const scale = Math.max(ew / nw, eh / nh);
    const dw = nw * scale, dh = nh * scale;
    return { ox: (ew - dw) / 2, oy: (eh - dh) / 2, dw, dh };
  };

  const drawFeed = (feed, persons) => {
    const { img, canvas } = feed;
    canvas.style.left = `${img.offsetLeft}px`;
    canvas.style.top = `${img.offsetTop}px`;
    canvas.width = img.clientWidth;
    canvas.height = img.clientHeight;
    canvas.classList.remove("hidden");
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const t = coverTransform(img);
    feed.count = persons.length;
    if (!t) return;
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#e60000";
    ctx.font = "600 11px system-ui, sans-serif";
    persons.forEach((p) => {
      const x = t.ox + p.x1 * t.dw;
      const y = t.oy + p.y1 * t.dh;
      const w = (p.x2 - p.x1) * t.dw;
      const h = (p.y2 - p.y1) * t.dh;
      ctx.strokeRect(x, y, w, h);
      const label = `${Math.round((p.conf || 0) * 100)}%`;
      const labelWidth = ctx.measureText(label).width + 8;
      ctx.fillStyle = "#e60000";
      ctx.fillRect(x, Math.max(0, y - 15), labelWidth, 15);
      ctx.fillStyle = "#fff";
      ctx.fillText(label, x + 4, Math.max(11, y - 4));
    });
  };

  const renderCounter = (error) => {
    if (!counter) return;
    counter.classList.remove("hidden");
    if (error) { counter.textContent = "Sentry: —"; counter.title = error; return; }
    const total = feeds.reduce((sum, feed) => sum + (feed.count || 0), 0);
    counter.textContent = `Sentry: ${total}`;
    counter.title = "People detected across the visible feeds";
  };

  const pollFeed = async (feed) => {
    if (feed.inFlight) return;
    if (feed.img.classList.contains("hidden") || !feed.img.getAttribute("src")) {
      clearFeed(feed);
      return;
    }
    feed.inFlight = true;
    try {
      const resp = await fetch(`/api/sentry/detect?feed=${feed.name}`, { cache: "no-store" });
      const data = await resp.json();
      if (data.ok) { drawFeed(feed, data.persons || []); renderCounter(null); }
      else { clearFeed(feed); renderCounter(data.error || "Detection failed."); }
    } catch {
      clearFeed(feed);
      renderCounter("Sentry request failed.");
    } finally {
      feed.inFlight = false;
    }
  };

  window.setInterval(() => {
    const active = isOn() && !panel.classList.contains("hidden");
    if (!active) { clearAll(); return; }
    feeds.forEach(pollFeed);
  }, 250);
})();
```

- [ ] **Step 6: Run the contract test to verify it passes**

Run: `python3 -m unittest tests.test_sentry -v`
Expected: all tests PASS (including `SentryFrontendContractTests`).

- [ ] **Step 7: Full suite + gate**

Run: `python3 -m unittest discover -s tests -p 'test_*.py'` then `make production-gate`
Expected: all pass, gate green.

- [ ] **Step 8: Manual smoke check (robot is live)**

Open `http://192.168.123.164:8080` (or the operator URL in use), then:
1. Sentry toggle OFF, float cam open → no boxes, DevTools Network shows **zero** `/api/sentry/detect` requests.
2. Toggle ON, float cam open, person in view → red boxes with confidence labels on the head feed and (when the webcam feed is visible) on the bottom feed; header shows `Sentry: N`.
3. Minimize the float cam → requests stop; reopen → they resume.
4. Reload the page → toggle state persists.

- [ ] **Step 9: Commit and push**

```bash
git add static/index.html static/app.js static/styles.css tests/test_sentry.py
git commit -m "Add Sentry Mode toggle + person-detection overlays on both floating-cam feeds

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git push
```
