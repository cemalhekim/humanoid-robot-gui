---
tags: [feature, 3d, urdf, three-js, viewer, ik, self-collision, rendering]
summary: The Three.js URDF viewer in static/viewer.js — how it loads the H1-2 model from URDF + STL, renders the live/reference/trajectory/replay/proposal models, drives arm IK + self-collision from draggable hand markers and the 6-DOF panel, and stays cheap with render-on-demand.
---

# 17 - 3D URDF Viewer

> [!abstract] Goal
> Render the Unitree H1-2 as an articulated 3D model in the browser, driven live
> by telemetry, and let the operator pose the arms/torso by dragging hand
> markers (backed by IK + self-collision). One `RobotViewer` class
> (`static/viewer.js`) serves both the **live dashboard viewer** and the
> **recording/replay editor**, sharing the same URDF, materials, IK and render
> loop.

Sources: `static/viewer.js` (`RobotViewer`, `buildRobot`, `loadRobot`,
`loadVisualMeshes`, `solveArmTo`, `armSelfCollision`, the 6-DOF panel methods,
`animate`), `static/index.html` (`#robotCanvas`, importmap, lazy `import`),
`static/app.js` (viewer wiring). The pose-editing surface is documented in
13 - Telemetry Recording & Pose Editor.

## Vendored Three.js + model assets

The viewer is an ES module imported lazily (`import("/viewer.js")`) only when
the page is **not** in lite mode (`?lite=1` / `?no3d=1`). Three.js is vendored
and wired through an importmap in `static/index.html` — no CDN:

| Import specifier | Served from |
| --- | --- |
| `three` | `/vendor/three/three.module.js` |
| `three/addons/controls/OrbitControls.js` | `/vendor/three/OrbitControls.js` |
| `three/addons/loaders/STLLoader.js` | `/vendor/three/STLLoader.js` |

The model comes from `MODEL_BASE = "/models/h1_2_description/"`:
`h1_2.urdf` is fetched and parsed with `DOMParser`; each `<link>`'s `<visual>`
`<mesh filename>` is resolved (`resolveMeshPath`, `package://` → last two path
segments) and loaded as an **STL** via `STLLoader`. `buildRobot` walks
`robot > joint`, builds a `THREE.Group` tree honoring each joint's `origin`,
`axis`, and `limit` (`lower`/`upper`), rotates the root `-π/2` about X (URDF
Z-up → scene Y-up) and drops it to `position.y = -0.55`.

## The models in the scene

`RobotViewer` is constructed with `{ live, compare }`. There are two instances:
a **live** viewer (`#robotCanvas`) and a **compare/replay** viewer (used by the
recorder). Which models exist depends on the mode (`loadRobot`,
`materialFromVisual`):

| Model (`tone`) | Color | Built when | Meaning |
| --- | --- | --- | --- |
| **default** | URDF material colors (fallback gray `0xaab4bd`) | live viewer | The **live robot** posed by telemetry |
| **reference** | faint **blue** `0x7fb8ff`, opacity 0.24 | compare viewer | Translucent reference silhouette (a loaded file frame) |
| **trajectory** | **green** `0x1dff75`, opacity 0.62 | compare viewer | Simulated-trajectory ghost — hidden unless *Simulate Trajectory* is running |
| **replay** | **red** `0xff3b30`, opacity 0.78 | compare viewer | The editable / replay **target** model (`robotRoot`, `jointGroups`) |
| **proposal** (tone `trajectory`, green) | **green** `0x1dff75` | live viewer, lazily | LLM arm-pose-proposal twin — see 20 - LLM Arm Pose Proposals & Mimic |

> [!note] The proposal ghost is built lazily
> To avoid keeping a second robot's meshes/GPU buffers resident for a feature
> that may never fire, the live viewer stashes the parsed URDF (`_urdfXml`) and
> builds the green proposal twin only on the first proposal
> (`ensureProposalGhost`). It mirrors the live body/hands, then overrides the
> proposed arm joints, and is shown/hidden as proposals come and go.

Each model has its own joint map (`jointGroups`, `referenceJointGroups`,
`trajectoryJointGroups`, `proposalJointGroups`); `applyTelemetry`,
`applyReference`, and `applyTrajectory` route body motors and Inspire-hand
joints (`BODY_JOINTS` / `HAND_JOINTS`) into the right map. Only the opaque
models (`default`, `replay`) cast/receive shadows — the translucent ghosts skip
the shadow pass to halve per-frame cost.

## Render-on-demand

The robot is idle most of the time, so the viewer does **not** redraw every
frame. `animate()` (driven by `requestAnimationFrame`) renders only when
something changed (`viewer.js`):

- `_needsRender` is set on any joint change (`setJointValueIn`, guarded by
  `JOINT_EPSILON` so a held joint repeating its value is a no-op), OrbitControls
  `change` (drag/zoom/pan), visibility toggles, and mesh loads.
- `controls.update()` returns true while the camera is still easing (damping /
  auto-rotate) — it keeps drawing until it settles.
- A `RENDER_BACKSTOP_MS = 400` ms backstop forces a redraw even if a trigger is
  ever missed, bounding worst-case staleness.
- `IntersectionObserver` sets `_hidden` to skip all per-frame work when the
  canvas is off-screen (buffering the newest telemetry and applying it on
  return); `ResizeObserver` flags `_sizeDirty` so layout is only read when the
  size actually changed.

Renderer: `WebGLRenderer({ antialias: true })`, `setPixelRatio(min(dpr, 1.5))`,
shadow map on, a `PerspectiveCamera(48°)`, hemisphere + directional key light, a
`GridHelper` at `FLOOR_Y = -1.52`. `OrbitControls` has damping enabled; view
buttons (`bindViewTools`) snap the camera to front/back/left/right/top/bottom
and toggle grid / auto-rotate.

## Hand-target editing → IK + self-collision

In the compare viewer, each hand has a draggable sphere marker
(`createEndEffectorMarker`) and the torso has a twist ring (`torso_joint`,
±1.2 rad). Dragging a hand marker retargets that arm via an **iterative
CCD-style IK solver** (`solveArmTo`):

- Chains: `RIGHT_ARM_IK_JOINTS` / `LEFT_ARM_IK_JOINTS` — shoulder pitch/roll/yaw,
  elbow, and wrist roll/pitch/yaw. Up to 14 iterations, per-step angle clamped
  to ±0.12 rad, respecting each joint's URDF `lower`/`upper` (out-of-range =
  `limited`).
- **Self-collision**: after solving, `armSelfCollision` builds axis-aligned
  boxes from arm spheres (hand base, wrist yaw/pitch, elbow) vs body spheres
  (torso, pelvis, camera, lidar) and checks `intersectsBox`. If the pose
  collides, the arm is **reverted to its pre-solve snapshot** (`restoreArmPose`)
  and the move is rejected.
- Status is emitted on the `recording-ik-status` event
  (`emitIkStatus`: `error`, `limited`, `collision`, `blocked`, `reachable`),
  which the recorder UI renders as *IK solved · N cm* / *near limit* /
  *unreachable* / **Blocked: self-collision** — see
  13 - Telemetry Recording & Pose Editor.

### Elbow targets (added 2026-07-28)

The High Level Controller toolbar has an **Elbow targets** toggle
(`#elbowTargetsToggle`, bent-arm icon next to Mirror arms). ON pops an
**orange draggable sphere on each elbow** (`createElbowMarkerForSide`,
smaller than the hand balls and ray-cast with priority over them). Dragging
an elbow sphere re-aims the **upper arm only**: the same `solveArmTo` CCD
runs with the elbow link as the effector over a 2-joint chain
(`RIGHT/LEFT_ELBOW_IK_JOINTS` = shoulder pitch + roll — an elbow position
is fully determined by those two; yaw only spins the arm in place). The
elbow angle, forearm and wrist joints are untouched, so the operator can
place the elbow first, then the hand — self-collision, mirror-arms sync
and `emitEditedPose` all behave exactly as with hand drags. Wiring:
checkbox → `recording-elbow-targets` CustomEvent (`app.js`) →
`setElbowTargetsVisible` on the compare viewer.

### 6-DOF hand-target panel

A **press without dragging is a click** (movement < 6 px): it dispatches
`end-effector-selected`, which opens the hovering 6-DOF panel (rendered by
`static/app.js`, positioned via `endEffectorScreenPosition`). Panel state comes
from `endEffectorPanelState`:

| Control | Backing method | Notes |
| --- | --- | --- |
| **X / Y / Z** (with Ground / Relative toggle) | `setEndEffectorAxis` | *Ground* = X/Y in robot-local frame (X forward, Y left), Z above the fixed floor (`FLOOR_Y`); *Relative* = offsets from the hand's last-synced baseline (`endEffectorBaselines`) |
| **Wrist roll / pitch / yaw** | `setWristJoint` | Bounded by the real URDF joint limits; excluded from the position IK chain (`armPositionJoints` drops `_wrist_` joints) so orientation stays as set |

Both paths run the **same IK + self-collision** as dragging. In lock mode the
untouched axes come from a caller-held locked target (not the drifting marker),
and rotating the wrist re-solves shoulder/elbow to hold X/Y/Z. An optional
**mirror** mode (`ARM_MIRROR_JOINTS`) copies one arm's pose to the other with
per-joint sign flips, also collision-checked.

> [!important] The viewer never commands the robot
> All edits change only the **preview pose**. `emitEditedPose` dispatches
> `recording-edited-pose` (a telemetry-shaped snapshot with the edited arm/torso
> joints) for the recorder/app to save or replay — no motor command is ever
> published from here. The arm only moves through the guarded closed-loop replay
> path in 16 - Arm Control & Command Surfaces and
> 14 - Recording Replay & Digital Twin. See 03 - Safety Interlocks.

## Live spatial evidence

The live viewer can produce a compact **spatial snapshot** of hand positions
(`liveSpatialEvidence`): each hand's robot-ground coordinates plus coarse labels
(forward/behind, left/right/center, high/mid/low). It optionally attaches a
JPEG screenshot of the canvas (`toDataURL`) — this feeds the chatbot's
digital-twin evidence and the shared-pose publish. See 05 - Chat & MCP Tools
and 20 - LLM Arm Pose Proposals & Mimic.

