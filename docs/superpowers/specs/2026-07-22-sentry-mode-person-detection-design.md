# Sentry Mode — passive person detection overlay (Phase 1)

**Date:** 2026-07-22
**Status:** Approved design, Phase 1 of 3

## Roadmap context

Person interaction is built in three phases; this spec covers only Phase 1.

1. **Phase 1 (this spec):** Sentry Mode — detect **all** humans in the head
   camera and draw bounding boxes on the floating camera view. **No robot
   motion of any kind.**
2. **Phase 2 (later):** target lock — pick and visually highlight one person
   (association/stickiness already exists in `tracking.py`).
3. **Phase 3 (later):** arm pointing at the locked person — the existing
   guarded `/api/track/*` session (risk-acked, arm_sdk) becomes the motion
   layer on top of Phases 1–2.

## Goal

While Sentry Mode is enabled and the floating (drop-down) camera is open, the
operator sees live bounding boxes around every person the YOLO service detects
in the head-camera feed. Hiding the floating camera or disabling the toggle
removes the boxes and stops all detection traffic.

## Non-goals (Phase 1)

- No arm/locomotion commands, no XR publisher suspension, no risk ack.
- No target selection or person identity — every person gets an identical box.
- No boxes on the USB webcam feed (YOLO pipeline is head-camera based).
- No new page (the earlier side-by-side CV page idea is superseded).

## Server

New read-only endpoint in `server.py`:

- `GET /api/sentry/detect` — synchronous, one detection per call:
  1. Take the latest cached head-camera JPEG (`store.get_camera_frame()`).
  2. POST it to the AI-host YOLO service (`TRACKING_DETECT_URL`, default
     `http://10.2.125.3:8188/detect`) with a 0.5 s timeout.
  3. Return `{"ok": true, "persons": [...], "ts": <epoch>}` where `persons`
     is the service's normalized list (`x1,y1,x2,y2,cx,cy,conf`, coordinates
     in 0..1).
  - No camera frame yet → `{"ok": false, "error": "No head-camera frame."}`
    (HTTP 200; the UI treats it as "waiting").
  - YOLO unreachable/timeout → `{"ok": false, "error": "Detection service
    unreachable."}`.

Design choice: **no server-side worker/loop and no start/stop state.** The
browser polls only while boxes are visible, so detection load on the AI host
is naturally gated by the UI. Stopping the poll stops everything; there is
nothing to leak. `ThreadingHTTPServer` makes the blocking upstream call safe.

## Frontend (`static/index.html`, `static/app.js`, `static/styles.css`)

- **Sentry Mode toggle** on the main dashboard: a labelled switch
  ("Sentry Mode"). State persists in `localStorage` key `h1_sentry_mode`
  (`"1"`/`"0"`, default off).
- **Overlay canvas** in the floating camera panel: a transparent `<canvas>`
  absolutely positioned over the head-camera `<img>` (`#floatCamStream`),
  kept in sync with the image's displayed size (ResizeObserver + window
  resize). The webcam image below is untouched.
- **Poll loop:** every ~250 ms (4 Hz) fetch `/api/sentry/detect` and redraw.
  The loop runs **only** when *both* are true:
  - Sentry Mode toggle is ON, and
  - the floating camera panel is open (not minimized).
  Any other combination: loop stopped, canvas cleared, zero requests.
- **Drawing:** one rectangle per person (normalized coords × canvas size),
  confidence label (e.g. `87%`) at the top-left corner of each box, and a
  small "Sentry: N" count in the float-cam header while active. Detection
  errors show as "Sentry: —" with the error as the header tooltip; the video
  stream itself is never blocked.
- Overlapping fetches are prevented (skip a tick if the previous request is
  still in flight).

## Testing

- Unit tests for `/api/sentry/detect` with the YOLO upstream mocked:
  frame-present + service-OK → persons passed through; no frame → ok=false;
  upstream timeout → ok=false error. Follow the existing `tests/` patterns.
- `make production-gate` must pass before push.
- No motion code paths are touched; safety interlocks are unaffected.
