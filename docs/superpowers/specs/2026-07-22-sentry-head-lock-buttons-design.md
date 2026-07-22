# Sentry Mode — head-tracked glowing lock buttons (Phase 2, UI-only)

**Date:** 2026-07-22
**Status:** Approved design (operator-directed), supersedes the bounding-box
drawing of the Phase 1 spec (`2026-07-22-sentry-mode-person-detection-design.md`).
Everything else from Phase 1 (endpoint, toggle, gating, webcam-only polling)
stays as-is.

## Goal

While Sentry Mode is on and the floating camera is open, **no bounding boxes
are drawn**. Instead, every person detected in the webcam feed gets a small
**glowing (pulsing) lock button hovering just above their head**, which
follows the head continuously and smoothly. Clicking the button **locks onto
that person**; the locked person's button switches to a distinct locked style.
Still zero robot motion — the lock is pure UI state that Phase 3 will feed
into the guarded arm-pointing session.

## Client-side person tracks

Detections arrive per-frame with no identity, so the browser keeps tracks:

- **Association:** greedy nearest-center match between incoming persons and
  existing tracks, normalized center distance threshold 0.18. Unmatched
  persons spawn new tracks (incrementing ids); tracks unseen for **2 s** are
  removed.
- **Smoothing:** exponential smoothing (alpha 0.45) on the box coordinates so
  the button glides with the head instead of jittering.
- **Head point:** top-center of the (smoothed) box — `x = (x1+x2)/2`,
  `y = y1` — mapped through the existing `object-fit: cover` transform, with
  the button anchored above that point (`translate(-50%, -100%)` minus a
  small gap).

## Lock semantics

- One lock at a time. Click an unlocked button → that track is locked.
  Click another person's button → the lock switches. Click the locked
  button → unlock.
- Locked button style: green glow, `title="Kilidi kaldır"`; unlocked style:
  red pulsing glow, `title="Bu kişiye kitlen"`. Both use a lock glyph.
- Auto-unlock when the locked track is removed (person gone > 2 s), when the
  floating cam closes, or when Sentry Mode is toggled off.
- The header counter keeps `Sentry: N` and appends ` • LOCKED` while a lock
  is active.

## Rendering

- New absolutely-positioned layer `#floatWebcamTargets` inside
  `.float-cam-body`, sized/positioned to the webcam `<img>` rectangle each
  frame (same sync approach the canvas used). Layer itself is
  `pointer-events: none`; only the buttons are `pointer-events: auto`.
- One `<button class="target-lock-btn">` per track, created/removed with the
  track, repositioned every poll tick; short CSS `left/top` transition covers
  the 250 ms between ticks.
- The Phase 1 canvases stay in the DOM but are no longer drawn on (reserved
  for future visuals).

## Non-goals

- No robot motion, no server changes (`/api/sentry/detect` untouched).
- No lock reporting to the server yet — Phase 3 wires the locked target into
  `/api/track/*`.
- No multi-person lock, no identity persistence across occlusions beyond the
  2 s track memory.

## Testing

- Extend the frontend contract test with the new needles
  (`floatWebcamTargets`, `target-lock-btn` in the right files).
- Full suite + `make production-gate` before push; manual on-robot check:
  buttons appear over heads, follow movement, lock/unlock/switch works,
  everything vanishes when the cam closes or the toggle goes off.
