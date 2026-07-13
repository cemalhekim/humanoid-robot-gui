import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const MODEL_BASE = "/models/h1_2_description/";
const URDF_PATH = `${MODEL_BASE}h1_2.urdf`;
const FLOOR_Y = -1.52;

const BODY_JOINTS = {
  LeftHipYaw: "left_hip_yaw_joint",
  LeftHipPitch: "left_hip_pitch_joint",
  LeftHipRoll: "left_hip_roll_joint",
  LeftKnee: "left_knee_joint",
  LeftAnklePitch: "left_ankle_pitch_joint",
  LeftAnkleRoll: "left_ankle_roll_joint",
  RightHipYaw: "right_hip_yaw_joint",
  RightHipPitch: "right_hip_pitch_joint",
  RightHipRoll: "right_hip_roll_joint",
  RightKnee: "right_knee_joint",
  RightAnklePitch: "right_ankle_pitch_joint",
  RightAnkleRoll: "right_ankle_roll_joint",
  WaistYaw: "torso_joint",
  LeftShoulderPitch: "left_shoulder_pitch_joint",
  LeftShoulderRoll: "left_shoulder_roll_joint",
  LeftShoulderYaw: "left_shoulder_yaw_joint",
  LeftElbow: "left_elbow_joint",
  LeftWristRoll: "left_wrist_roll_joint",
  LeftWristPitch: "left_wrist_pitch_joint",
  LeftWristYaw: "left_wrist_yaw_joint",
  RightShoulderPitch: "right_shoulder_pitch_joint",
  RightShoulderRoll: "right_shoulder_roll_joint",
  RightShoulderYaw: "right_shoulder_yaw_joint",
  RightElbow: "right_elbow_joint",
  RightWristRoll: "right_wrist_roll_joint",
  RightWristPitch: "right_wrist_pitch_joint",
  RightWristYaw: "right_wrist_yaw_joint",
};

const URDF_TO_BODY_JOINT = Object.fromEntries(
  Object.entries(BODY_JOINTS).map(([bodyName, urdfName]) => [urdfName, bodyName]),
);

// Torso ("belly") twist: a grab-ring around the waist rotates WaistYaw in the
// editor, exactly like the hand IK markers. Clamped to a conservative range.
const TORSO_TWIST_JOINT = "torso_joint";
const TORSO_TWIST_LIMIT = 1.2; // rad (±)

const RIGHT_ARM_IK_JOINTS = [
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint",
];

const LEFT_ARM_IK_JOINTS = [
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint",
  "left_elbow_joint",
  "left_wrist_roll_joint",
  "left_wrist_pitch_joint",
  "left_wrist_yaw_joint",
];

const ARM_MIRROR_JOINTS = [
  ["left_shoulder_pitch_joint", "right_shoulder_pitch_joint", 1],
  ["left_shoulder_roll_joint", "right_shoulder_roll_joint", -1],
  ["left_shoulder_yaw_joint", "right_shoulder_yaw_joint", -1],
  ["left_elbow_joint", "right_elbow_joint", 1],
  ["left_wrist_roll_joint", "right_wrist_roll_joint", -1],
  ["left_wrist_pitch_joint", "right_wrist_pitch_joint", 1],
  ["left_wrist_yaw_joint", "right_wrist_yaw_joint", -1],
];

const finger = (joint) => ({ joint, min: 0, max: 1.7 });
const thumbPitch = (joint, min, max, offset = 0) => ({ joint, min, max, offset });

const HAND_JOINTS = {
  RightPinky: [finger("R_pinky_proximal_joint"), finger("R_pinky_intermediate_joint")],
  RightRing: [finger("R_ring_proximal_joint"), finger("R_ring_intermediate_joint")],
  RightMiddle: [finger("R_middle_proximal_joint"), finger("R_middle_intermediate_joint")],
  RightIndex: [finger("R_index_proximal_joint"), finger("R_index_intermediate_joint")],
  RightThumbBend: [
    thumbPitch("R_thumb_proximal_pitch_joint", -0.1, 0.6),
    thumbPitch("R_thumb_intermediate_joint", 0, 0.8),
    thumbPitch("R_thumb_distal_joint", 0, 1.2),
  ],
  RightThumbRotation: [thumbPitch("R_thumb_proximal_yaw_joint", -0.1, 1.3, -Math.PI / 2)],
  LeftPinky: [finger("L_pinky_proximal_joint"), finger("L_pinky_intermediate_joint")],
  LeftRing: [finger("L_ring_proximal_joint"), finger("L_ring_intermediate_joint")],
  LeftMiddle: [finger("L_middle_proximal_joint"), finger("L_middle_intermediate_joint")],
  LeftIndex: [finger("L_index_proximal_joint"), finger("L_index_intermediate_joint")],
  LeftThumbBend: [
    thumbPitch("L_thumb_proximal_pitch_joint", -0.1, 0.6),
    thumbPitch("L_thumb_intermediate_joint", 0, 0.8),
    thumbPitch("L_thumb_distal_joint", 0, 1.2),
  ],
  LeftThumbRotation: [thumbPitch("L_thumb_proximal_yaw_joint", -0.1, 1.3)],
};

function handValueToUrdfAngle(value, config) {
  if (!Number.isFinite(value)) return null;
  const normalized = Math.max(0, Math.min(1, value));
  return config.min + (config.max - config.min) * (1 - normalized) + (config.offset || 0);
}

function parseVector(value, fallback = [0, 0, 0]) {
  if (!value) return fallback;
  const parts = value.trim().split(/\s+/).map(Number);
  return parts.length === 3 && parts.every(Number.isFinite) ? parts : fallback;
}

function applyOrigin(object, element) {
  const origin = element?.querySelector(":scope > origin");
  const xyz = parseVector(origin?.getAttribute("xyz"));
  const rpy = parseVector(origin?.getAttribute("rpy"));
  object.position.set(xyz[0], xyz[1], xyz[2]);
  object.rotation.set(rpy[0], rpy[1], rpy[2], "XYZ");
}

function materialFromVisual(visual, tone = "default") {
  if (tone === "reference") {
    return new THREE.MeshStandardMaterial({
      color: 0x7fb8ff,
      roughness: 0.82,
      metalness: 0.04,
      transparent: true,
      opacity: 0.24,
      depthWrite: false,
    });
  }
  if (tone === "replay") {
    return new THREE.MeshStandardMaterial({
      color: 0xff3b30,
      roughness: 0.68,
      metalness: 0.1,
      transparent: true,
      opacity: 0.78,
    });
  }
  if (tone === "trajectory") {
    return new THREE.MeshStandardMaterial({
      color: 0x1dff75,
      roughness: 0.7,
      metalness: 0.08,
      transparent: true,
      opacity: 0.62,
    });
  }
  const color = visual.querySelector(":scope > material > color")?.getAttribute("rgba");
  if (color) {
    const [r, g, b, a] = color.split(/\s+/).map(Number);
    return new THREE.MeshStandardMaterial({
      color: new THREE.Color(r, g, b),
      roughness: 0.72,
      metalness: 0.08,
      transparent: Number.isFinite(a) && a < 1,
      opacity: Number.isFinite(a) ? a : 1,
    });
  }
  return new THREE.MeshStandardMaterial({ color: 0xaab4bd, roughness: 0.78, metalness: 0.08 });
}

function resolveMeshPath(filename) {
  if (filename.startsWith("package://")) {
    const parts = filename.split("/");
    return `${MODEL_BASE}${parts.slice(-2).join("/")}`;
  }
  return `${MODEL_BASE}${filename}`;
}

class RobotViewer {
  constructor({ container, fields, live = false, compare = false }) {
    this.container = container;
    this.fields = fields;
    this.live = live;
    this.compare = compare;
    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;
    this.robotRoot = null;
    this.trajectoryRoot = null;
    this.linkGroups = null;
    this.referenceJointGroups = new Map();
    this.trajectoryJointGroups = new Map();
    this.gridHelper = null;
    this.jointGroups = new Map();
    this.latestState = null;
    this.loadedMeshes = 0;
    this.failedMeshes = 0;
    this.totalMeshes = 0;
    this.modelReady = false;
    this.autoRotate = false;
    this.started = false;
    this.endEffectorMarker = null;
    this.leftEndEffectorMarker = null;
    this.mirrorArmsEnabled = false;
    this.collisionDebugVisible = false;
    this.collisionDebugHelpers = [];
    this.draggingEndEffector = false;
    this.draggingEndEffectorSide = null;
    this.dragPlane = new THREE.Plane();
    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();
    this.torsoRing = null;
    this.draggingTorso = false;
    this.torsoDragPlane = new THREE.Plane();
    this.torsoPivot = new THREE.Vector3();
    this.torsoAxisWorld = new THREE.Vector3();
    this.torsoStartVec = new THREE.Vector3();
    this.torsoStartValue = 0;
    // Hand-target baselines: world position of each hand in the last pose that
    // came from a file/replay sync (not from editing). "Relative" panel
    // coordinates are offsets from these.
    this.endEffectorBaselines = { right: null, left: null };
    this.endEffectorPointerDown = null;
  }

  setFields(data) {
    if (!this.fields) return;
    this.fields.innerHTML = Object.entries(data)
      .map(([key, value]) => `<dt>${key}</dt><dd>${value}</dd>`)
      .join("");
  }

  setJointValue(jointName, value) {
    this.setJointValueIn(this.jointGroups, jointName, value);
  }

  setJointValueIn(groups, jointName, value) {
    const joint = groups.get(jointName);
    if (!joint || !Number.isFinite(value)) return;
    joint.value = Math.max(joint.lower, Math.min(joint.upper, value));
    joint.group.quaternion.copy(joint.baseQuaternion);
    joint.group.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(joint.axis, joint.value));
  }

  setCamera(position, target = [0, 0.15, 0]) {
    this.camera.position.set(position[0], position[1], position[2]);
    this.controls.target.set(target[0], target[1], target[2]);
    this.controls.update();
    this.resize();
  }

  bindViewTools() {
    if (!this.live) return;
    document.querySelectorAll("[data-view]").forEach((button) => {
      button.addEventListener("click", () => {
        const view = button.dataset.view;
        if (["front", "back", "left", "right", "top", "bottom"].includes(view)) {
          document.querySelectorAll(".cube-label.active").forEach((label) => label.classList.remove("active"));
          button.classList.add("active");
        }
        if (view === "front") this.setCamera([2.15, 0.55, 0], [0, -0.65, 0]);
        if (view === "back") this.setCamera([-2.15, 0.55, 0], [0, -0.65, 0]);
        if (view === "left") this.setCamera([0, 0.55, 2.15], [0, -0.65, 0]);
        if (view === "right") this.setCamera([0, 0.55, -2.15], [0, -0.65, 0]);
        if (view === "top") this.setCamera([0.01, 2.25, 0.01], [0, -0.55, 0]);
        if (view === "bottom") this.setCamera([0.01, -3.0, 0.01], [0, -0.55, 0]);
        if (view === "grid" && this.gridHelper) {
          this.gridHelper.visible = !this.gridHelper.visible;
          button.classList.toggle("active", this.gridHelper.visible);
        }
        if (view === "rotate") {
          this.autoRotate = !this.autoRotate;
          this.controls.autoRotate = this.autoRotate;
          this.controls.autoRotateSpeed = 1.2;
          button.classList.toggle("active", this.autoRotate);
        }
      });
    });

    document.querySelector('[data-view="grid"]')?.classList.add("active");
    document.querySelector('[data-view="front"]')?.classList.add("active");
  }

  applyTelemetry(snapshot, source = "live") {
    this.latestState = snapshot;
    if (!this.modelReady) return;

    for (const motor of snapshot.motors || []) {
      const urdfJoint = BODY_JOINTS[motor.name];
      if (urdfJoint) this.setJointValue(urdfJoint, motor.q);
    }

    for (const hand of snapshot.hands?.joints || []) {
      const urdfJoints = HAND_JOINTS[hand.name] || [];
      for (const jointConfig of urdfJoints) {
        this.setJointValue(jointConfig.joint, handValueToUrdfAngle(hand.q, jointConfig));
      }
    }

    this.setFields({
      model: "h1_2.urdf",
      status: source,
      body_motors: snapshot.motor_count ?? snapshot.motors?.length ?? 0,
      hand_joints: snapshot.hands?.joint_count ?? snapshot.hands?.joints?.length ?? 0,
      sample: snapshot.sample ?? snapshot.samples ?? "--",
      time: snapshot.timestamp ? new Date(snapshot.timestamp * 1000).toLocaleTimeString() : "--",
      meshes: `${this.loadedMeshes}/${this.totalMeshes}`,
      failed_meshes: this.failedMeshes,
    });
    if (this.compare) {
      this.robotRoot?.updateWorldMatrix(true, true);
      // A synced pose becomes the new "initial" reference for relative coords.
      this.endEffectorBaselines = { right: null, left: null };
      this.updateEndEffectorMarker();
      if (source === "target") this.emitEditedPose("sync");
    }
  }

  applyReference(snapshot) {
    if (!this.compare || !this.modelReady) return;
    for (const motor of snapshot.motors || []) {
      const urdfJoint = BODY_JOINTS[motor.name];
      if (urdfJoint) this.setJointValueIn(this.referenceJointGroups, urdfJoint, motor.q);
    }
    for (const hand of snapshot.hands?.joints || []) {
      const urdfJoints = HAND_JOINTS[hand.name] || [];
      for (const jointConfig of urdfJoints) {
        this.setJointValueIn(this.referenceJointGroups, jointConfig.joint, handValueToUrdfAngle(hand.q, jointConfig));
      }
    }
  }

  applyTrajectory(snapshot) {
    if (!this.compare || !this.modelReady) return;
    // Pose the ghost but do NOT force it visible: the green silhouette is a
    // simulation-only aid, shown solely via recording-trajectory-visibility
    // (dispatched when Simulate Trajectory starts, hidden when it ends).
    for (const motor of snapshot.motors || []) {
      const urdfJoint = BODY_JOINTS[motor.name];
      if (urdfJoint) this.setJointValueIn(this.trajectoryJointGroups, urdfJoint, motor.q);
    }
    for (const hand of snapshot.hands?.joints || []) {
      const urdfJoints = HAND_JOINTS[hand.name] || [];
      for (const jointConfig of urdfJoints) {
        this.setJointValueIn(this.trajectoryJointGroups, jointConfig.joint, handValueToUrdfAngle(hand.q, jointConfig));
      }
    }
    this.setFields({
      model: "h1_2.urdf",
      status: "simulated trajectory",
      body_motors: snapshot.motor_count ?? snapshot.motors?.length ?? 0,
      hand_joints: snapshot.hands?.joint_count ?? snapshot.hands?.joints?.length ?? 0,
      sample: snapshot.sample ?? snapshot.samples ?? "--",
      time: snapshot.timestamp ? new Date(snapshot.timestamp * 1000).toLocaleTimeString() : "--",
      meshes: `${this.loadedMeshes}/${this.totalMeshes}`,
      failed_meshes: this.failedMeshes,
    });
  }

  setTrajectoryVisible(visible) {
    if (this.trajectoryRoot) this.trajectoryRoot.visible = visible;
  }

  setHandsVisible(visible) {
    // Inspire-hand links/joints are all named L_* / R_* in the URDF (body
    // links use full words), so toggling those groups hides/shows the hands on
    // EVERY model in this scene (live, editable, reference, trajectory ghost).
    // Drag markers live at the scene root, so arm dragging keeps working.
    if (!this.scene) return;
    this.handsVisible = Boolean(visible);
    this.scene.traverse((object) => {
      if (object.isGroup && /^[LR]_/.test(object.name || "")) {
        object.visible = this.handsVisible;
      }
    });
  }

  getEndEffectorObject(side = "right") {
    if (side === "left") {
      return this.linkGroups?.get("L_hand_base_link") || this.linkGroups?.get("left_wrist_yaw_link") || null;
    }
    return this.linkGroups?.get("R_hand_base_link") || this.linkGroups?.get("right_wrist_yaw_link") || null;
  }

  getEndEffectorPosition(side = "right") {
    const effector = this.getEndEffectorObject(side);
    if (!effector) return null;
    effector.updateWorldMatrix(true, false);
    return effector.getWorldPosition(new THREE.Vector3());
  }

  markerForSide(side = "right") {
    return side === "left" ? this.leftEndEffectorMarker : this.endEffectorMarker;
  }

  sideForMarkerHit(object) {
    let node = object;
    while (node) {
      if (node.userData?.side) return node.userData.side;
      node = node.parent;
    }
    return "right";
  }

  createEndEffectorMarkerForSide(side) {
    const marker = new THREE.Group();
    marker.name = `${side}_end_effector_target`;
    marker.userData.side = side;
    const isLeft = side === "left";

    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry(0.035, 24, 16),
      new THREE.MeshStandardMaterial({
        color: isLeft ? 0x40a6ff : 0xff3030,
        emissive: isLeft ? 0x003b80 : 0x8c0000,
        roughness: 0.42,
        metalness: 0.15,
      }),
    );
    marker.add(sphere);

    const ringMaterial = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 0.58,
      side: THREE.DoubleSide,
    });
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.055, 0.003, 8, 36), ringMaterial);
    ring.rotation.x = Math.PI / 2;
    marker.add(ring);

    marker.visible = false;
    this.scene.add(marker);
    return marker;
  }

  createEndEffectorMarker() {
    if (!this.compare || this.endEffectorMarker) return;
    this.endEffectorMarker = this.createEndEffectorMarkerForSide("right");
    this.leftEndEffectorMarker = this.createEndEffectorMarkerForSide("left");
    this.createTorsoRing();
    this.bindEndEffectorDrag();
  }

  createTorsoRing() {
    if (!this.compare || this.torsoRing) return;
    const torso = this.jointGroups.get(TORSO_TWIST_JOINT);
    if (!torso || !torso.group) return;

    const group = new THREE.Group();
    group.name = "torso_twist_ring";
    group.userData.torsoRing = true;
    // Lie the ring in the plane perpendicular to the yaw axis, lifted to belly height.
    const axis = torso.axis.clone().normalize();
    group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), axis);
    group.position.copy(axis).multiplyScalar(0.16);

    const ringMat = new THREE.MeshBasicMaterial({
      color: 0x35e08a,
      transparent: true,
      opacity: 0.85,
      side: THREE.DoubleSide,
    });
    group.add(new THREE.Mesh(new THREE.TorusGeometry(0.2, 0.012, 12, 56), ringMat));

    const knob = new THREE.Mesh(
      new THREE.SphereGeometry(0.03, 20, 14),
      new THREE.MeshStandardMaterial({ color: 0x35e08a, emissive: 0x0b5a34, roughness: 0.4, metalness: 0.1 }),
    );
    // Knob at local +X = the robot's front (URDF frame: X forward, Y left, Z up).
    knob.position.set(0.2, 0, 0);
    group.add(knob);

    // Invisible thicker torus so the ring is easy to grab (raycast target only).
    const hit = new THREE.Mesh(
      new THREE.TorusGeometry(0.2, 0.05, 8, 40),
      new THREE.MeshBasicMaterial({ visible: false }),
    );
    hit.userData.torsoRing = true;
    group.add(hit);

    torso.group.add(group);
    this.torsoRing = group;
  }

  beginTorsoDrag() {
    const torso = this.jointGroups.get(TORSO_TWIST_JOINT);
    if (!torso || !torso.group) return false;
    this.draggingTorso = true;
    this.controls.enabled = false;
    torso.group.updateWorldMatrix(true, false);
    this.torsoPivot.copy(torso.group.getWorldPosition(new THREE.Vector3()));
    this.torsoAxisWorld
      .copy(torso.axis)
      .applyQuaternion(torso.group.getWorldQuaternion(new THREE.Quaternion()))
      .normalize();
    this.torsoDragPlane.setFromNormalAndCoplanarPoint(this.torsoAxisWorld, this.torsoPivot);
    const hit = new THREE.Vector3();
    if (this.raycaster.ray.intersectPlane(this.torsoDragPlane, hit)) {
      this.torsoStartVec.copy(hit.sub(this.torsoPivot)).normalize();
    } else {
      this.torsoStartVec.set(1, 0, 0);
    }
    this.torsoStartValue = torso.value || 0;
    return true;
  }

  updateEndEffectorMarker() {
    this.updateEndEffectorMarkerForSide("right");
    this.updateEndEffectorMarkerForSide("left");
  }

  updateEndEffectorMarkerForSide(side) {
    const marker = this.markerForSide(side);
    if (!marker || this.draggingEndEffectorSide === side) return;
    const position = this.getEndEffectorPosition(side);
    if (!position) return;
    marker.position.copy(position);
    marker.visible = true;
    if (!this.endEffectorBaselines[side]) this.endEffectorBaselines[side] = position.clone();
    this.emitEndEffectorMoved(side);
  }

  emitEndEffectorMoved(side) {
    if (!this.compare) return;
    window.dispatchEvent(new CustomEvent("end-effector-moved", { detail: { side } }));
  }

  setPointerFromEvent(event) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    this.pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -(((event.clientY - rect.top) / rect.height) * 2 - 1);
    this.raycaster.setFromCamera(this.pointer, this.camera);
  }

  bindEndEffectorDrag() {
    if (!this.renderer || !this.endEffectorMarker) return;
    const canvas = this.renderer.domElement;

    canvas.addEventListener("pointerdown", (event) => {
      this.setPointerFromEvent(event);
      // Torso grab-ring takes priority when hit.
      if (this.torsoRing && this.torsoRing.visible !== false) {
        const torsoHits = this.raycaster.intersectObject(this.torsoRing, true);
        if (torsoHits.length && this.beginTorsoDrag()) {
          event.preventDefault();
          canvas.setPointerCapture?.(event.pointerId);
          return;
        }
      }
      const markers = [this.endEffectorMarker, this.leftEndEffectorMarker].filter((marker) => marker?.visible);
      if (!markers.length) return;
      const hits = this.raycaster.intersectObjects(markers, true);
      if (!hits.length) return;
      event.preventDefault();
      canvas.setPointerCapture?.(event.pointerId);
      this.draggingEndEffector = true;
      this.draggingEndEffectorSide = this.sideForMarkerHit(hits[0].object);
      this.endEffectorPointerDown = { x: event.clientX, y: event.clientY };
      this.controls.enabled = false;
      const normal = this.camera.getWorldDirection(new THREE.Vector3()).normalize();
      this.dragPlane.setFromNormalAndCoplanarPoint(normal, this.markerForSide(this.draggingEndEffectorSide).position);
    });

    canvas.addEventListener("pointermove", (event) => {
      if (this.draggingTorso) {
        this.setPointerFromEvent(event);
        const hit = new THREE.Vector3();
        if (!this.raycaster.ray.intersectPlane(this.torsoDragPlane, hit)) return;
        const current = hit.sub(this.torsoPivot).normalize();
        const cross = new THREE.Vector3().crossVectors(this.torsoStartVec, current);
        const delta = Math.atan2(cross.dot(this.torsoAxisWorld), this.torsoStartVec.dot(current));
        let value = this.torsoStartValue + delta;
        value = Math.max(-TORSO_TWIST_LIMIT, Math.min(TORSO_TWIST_LIMIT, value));
        this.setJointValueIn(this.jointGroups, TORSO_TWIST_JOINT, value);
        this.robotRoot?.updateWorldMatrix(true, true);
        this.updateEndEffectorMarker();
        this.emitEditedPose();
        return;
      }
      if (!this.draggingEndEffector || !this.draggingEndEffectorSide) return;
      this.setPointerFromEvent(event);
      const target = new THREE.Vector3();
      if (!this.raycaster.ray.intersectPlane(this.dragPlane, target)) return;
      const solved = this.solveArmTo(this.draggingEndEffectorSide, target);
      this.updateEndEffectorMarkerFromIk(this.draggingEndEffectorSide);
      if (solved && !this.syncMirroredArmFrom(this.draggingEndEffectorSide)) this.emitEditedPose();
    });

    const finishDrag = (event) => {
      if (this.draggingTorso) {
        canvas.releasePointerCapture?.(event.pointerId);
        this.draggingTorso = false;
        this.controls.enabled = true;
        this.updateEndEffectorMarker();
        this.emitEditedPose();
        return;
      }
      if (!this.draggingEndEffector) return;
      canvas.releasePointerCapture?.(event.pointerId);
      const side = this.draggingEndEffectorSide;
      const down = this.endEffectorPointerDown;
      this.draggingEndEffector = false;
      this.draggingEndEffectorSide = null;
      this.endEffectorPointerDown = null;
      this.controls.enabled = true;
      if (side) this.updateEndEffectorMarkerForSide(side);
      if (!side || !this.syncMirroredArmFrom(side)) this.emitEditedPose();
      // A press without movement is a click: open the 6-DOF target panel.
      if (side && down && Math.hypot(event.clientX - down.x, event.clientY - down.y) < 6) {
        window.dispatchEvent(new CustomEvent("end-effector-selected", { detail: { side } }));
      }
    };
    canvas.addEventListener("pointerup", finishDrag);
    canvas.addEventListener("pointercancel", finishDrag);
  }

  updateEndEffectorMarkerFromIk(side = "right") {
    const marker = this.markerForSide(side);
    if (!marker) return;
    const position = this.getEndEffectorPosition(side);
    if (position) {
      marker.position.copy(position);
      this.emitEndEffectorMoved(side);
    }
  }

  armIkJoints(side = "right") {
    return side === "left" ? LEFT_ARM_IK_JOINTS : RIGHT_ARM_IK_JOINTS;
  }

  armPoseSnapshot(side = "right") {
    const snapshot = new Map();
    for (const jointName of this.armIkJoints(side)) {
      const joint = this.jointGroups.get(jointName);
      if (joint) snapshot.set(jointName, joint.value || 0);
    }
    return snapshot;
  }

  restoreArmPose(snapshot) {
    for (const [jointName, value] of snapshot) {
      this.setJointValueIn(this.jointGroups, jointName, value);
    }
    this.robotRoot?.updateWorldMatrix(true, true);
  }

  mirrorArmPose(source = "right", target = "left") {
    if (!this.compare || !this.modelReady || !this.robotRoot || source === target) return false;
    const previousPose = this.armPoseSnapshot(target);
    const sourceIndex = source === "left" ? 0 : 1;
    const targetIndex = target === "left" ? 0 : 1;

    for (const pair of ARM_MIRROR_JOINTS) {
      const sourceJoint = this.jointGroups.get(pair[sourceIndex]);
      const targetJointName = pair[targetIndex];
      if (!sourceJoint || !Number.isFinite(sourceJoint.value)) continue;
      this.setJointValueIn(this.jointGroups, targetJointName, sourceJoint.value * pair[2]);
    }

    this.robotRoot.updateWorldMatrix(true, true);
    const collision = this.armSelfCollision(target);
    if (collision.colliding) {
      this.restoreArmPose(previousPose);
      this.emitIkStatus(Number.POSITIVE_INFINITY, false, collision);
      return false;
    }
    this.updateEndEffectorMarkerForSide(target);
    this.emitIkStatus(0, false, collision);
    this.emitEditedPose();
    return true;
  }

  mirroredSide(side = "right") {
    return side === "left" ? "right" : "left";
  }

  syncMirroredArmFrom(source = "right") {
    if (!this.mirrorArmsEnabled) return false;
    return this.mirrorArmPose(source, this.mirroredSide(source));
  }

  setMirrorArmsEnabled(enabled) {
    this.mirrorArmsEnabled = Boolean(enabled);
    if (this.mirrorArmsEnabled) this.mirrorArmPose("right", "left");
  }

  // --- 6-DOF hand-target panel support -------------------------------------
  // Panel coordinates use the robot frame: X forward, Y left, Z up (meters).
  // "ground" mode measures X/Y from the pelvis axis and Z from the floor
  // (which is fixed); "relative" mode measures all three as offsets from the
  // hand's position in the last synced (pre-edit) pose.

  worldToRobotLocal(position) {
    if (!this.robotRoot) return null;
    this.robotRoot.updateWorldMatrix(true, false);
    const inverse = new THREE.Matrix4().copy(this.robotRoot.matrixWorld).invert();
    return position.clone().applyMatrix4(inverse);
  }

  robotLocalToWorld(local) {
    if (!this.robotRoot) return null;
    this.robotRoot.updateWorldMatrix(true, false);
    return local.clone().applyMatrix4(this.robotRoot.matrixWorld);
  }

  endEffectorBaseline(side) {
    if (!this.endEffectorBaselines[side]) {
      const position = this.getEndEffectorPosition(side);
      if (position) this.endEffectorBaselines[side] = position.clone();
    }
    return this.endEffectorBaselines[side];
  }

  wristJointNames(side = "right") {
    return {
      roll: `${side}_wrist_roll_joint`,
      pitch: `${side}_wrist_pitch_joint`,
      yaw: `${side}_wrist_yaw_joint`,
    };
  }

  endEffectorCoordsFromWorld(side, mode, world) {
    const local = this.worldToRobotLocal(world);
    const baseline = this.endEffectorBaseline(side);
    if (!local || !baseline) return null;
    if (mode === "relative") {
      const baseLocal = this.worldToRobotLocal(baseline);
      return { x: local.x - baseLocal.x, y: local.y - baseLocal.y, z: local.z - baseLocal.z };
    }
    return { x: local.x, y: local.y, z: world.y - FLOOR_Y };
  }

  endEffectorWorldFromCoords(side, mode, coords) {
    const baseline = this.endEffectorBaseline(side);
    if (!baseline) return null;
    if (mode === "relative") {
      const baseLocal = this.worldToRobotLocal(baseline);
      return this.robotLocalToWorld(
        new THREE.Vector3(baseLocal.x + coords.x, baseLocal.y + coords.y, baseLocal.z + coords.z),
      );
    }
    // Ground frame: X/Y are robot-local; Z sets the world height above the
    // floor directly (the root only rotates URDF Z-up to scene Y-up, so
    // local X/Y never affect world Y).
    const world = this.robotLocalToWorld(new THREE.Vector3(coords.x, coords.y, 0));
    world.y = FLOOR_Y + coords.z;
    return world;
  }

  endEffectorPanelState(side = "right") {
    const marker = this.markerForSide(side);
    if (!this.modelReady || !marker || !marker.visible) return null;
    const world = marker.position;
    const ground = this.endEffectorCoordsFromWorld(side, "ground", world);
    const relative = this.endEffectorCoordsFromWorld(side, "relative", world);
    if (!ground || !relative) return null;
    const wrist = {};
    for (const [key, jointName] of Object.entries(this.wristJointNames(side))) {
      const joint = this.jointGroups.get(jointName);
      if (joint) wrist[key] = { value: joint.value || 0, lower: joint.lower, upper: joint.upper };
    }
    return {
      side,
      ground,
      relative,
      wrist,
      screen: this.endEffectorScreenPosition(side),
    };
  }

  endEffectorScreenPosition(side = "right") {
    const marker = this.markerForSide(side);
    if (!marker || !this.renderer || !this.camera) return null;
    const rect = this.renderer.domElement.getBoundingClientRect();
    const projected = marker.position.clone().project(this.camera);
    return {
      x: ((projected.x + 1) / 2) * rect.width,
      y: ((1 - projected.y) / 2) * rect.height,
    };
  }

  setEndEffectorAxis(side, mode, axis, value, options = {}) {
    const marker = this.markerForSide(side);
    if (!this.modelReady || !marker || !Number.isFinite(value)) return { solved: false, world: null };
    // Lock mode: the untouched axes come from the caller-held locked target
    // (not the drifting marker), and the wrist is excluded from the IK chain
    // so orientation stays exactly as set.
    const baseWorld = options.baseWorld || marker.position;
    const coords = this.endEffectorCoordsFromWorld(side, mode, baseWorld);
    if (!coords) return { solved: false, world: null };
    coords[axis] = value;
    const target = this.endEffectorWorldFromCoords(side, mode, coords);
    if (!target) return { solved: false, world: null };
    const chain = options.excludeWrist ? this.armPositionJoints(side) : null;
    const solved = this.solveArmTo(side, target, chain);
    this.updateEndEffectorMarkerFromIk(side);
    if (solved && !this.syncMirroredArmFrom(side)) this.emitEditedPose();
    return { solved, world: target };
  }

  setWristJoint(side, key, value, restoreWorld = null) {
    const jointName = this.wristJointNames(side)[key];
    const joint = jointName ? this.jointGroups.get(jointName) : null;
    if (!this.modelReady || !joint || !Number.isFinite(value)) return false;
    const previousPose = this.armPoseSnapshot(side);
    this.setJointValueIn(this.jointGroups, jointName, value);
    this.robotRoot.updateWorldMatrix(true, true);
    const collision = this.armSelfCollision(side);
    if (collision.colliding) {
      this.restoreArmPose(previousPose);
      this.emitIkStatus(Number.POSITIVE_INFINITY, false, collision);
      return false;
    }
    if (restoreWorld) {
      // Lock mode: rotating the wrist shifts the hand slightly — re-solve the
      // shoulder/elbow chain so X/Y/Z stay at the locked target.
      this.solveArmTo(side, restoreWorld, this.armPositionJoints(side));
    } else {
      this.emitIkStatus(0, false, collision);
    }
    this.updateEndEffectorMarkerForSide(side);
    if (!this.syncMirroredArmFrom(side)) this.emitEditedPose();
    return true;
  }

  linkWorldPosition(name) {
    const link = this.linkGroups?.get(name);
    if (!link) return null;
    link.updateWorldMatrix(true, false);
    return link.getWorldPosition(new THREE.Vector3());
  }

  armCollisionSpheres(side = "right") {
    if (side === "left") {
      return [
        { name: "L_hand_base_link", radius: 0.085, role: "arm" },
        { name: "left_wrist_yaw_link", radius: 0.075, role: "arm" },
        { name: "left_wrist_pitch_link", radius: 0.075, role: "arm" },
        { name: "left_elbow_link", radius: 0.08, role: "arm" },
      ];
    }
    return [
      { name: "R_hand_base_link", radius: 0.085, role: "arm" },
      { name: "right_wrist_yaw_link", radius: 0.075, role: "arm" },
      { name: "right_wrist_pitch_link", radius: 0.075, role: "arm" },
      { name: "right_elbow_link", radius: 0.08, role: "arm" },
    ];
  }

  bodyCollisionSpheres() {
    return [
      { name: "torso_link", radius: 0.2, role: "body" },
      { name: "pelvis", radius: 0.18, role: "body" },
      { name: "camera_link", radius: 0.14, role: "body" },
      { name: "lidar_link", radius: 0.14, role: "body" },
    ];
  }

  collisionVisualTargets(name) {
    const link = this.linkGroups?.get(name);
    if (!link) return [];
    const visualTargets = link.children.filter((child) => child.userData?.visualGeometry);
    return visualTargets.length ? visualTargets : [link];
  }

  collisionBounds(definitions) {
    const bounds = [];
    for (const definition of definitions) {
      for (const target of this.collisionVisualTargets(definition.name)) {
        target.updateWorldMatrix(true, true);
        bounds.push({
          ...definition,
          target,
          box: new THREE.Box3().setFromObject(target),
        });
      }
    }
    return bounds;
  }

  createCollisionDebugHelpers() {
    if (!this.compare || !this.linkGroups || this.collisionDebugHelpers.length) return;
    for (const sphere of [...this.armCollisionSpheres("right"), ...this.armCollisionSpheres("left"), ...this.bodyCollisionSpheres()]) {
      for (const target of this.collisionVisualTargets(sphere.name)) {
        const helper = new THREE.BoxHelper(target, sphere.role === "arm" ? 0xffcf40 : 0x45a3ff);
        helper.name = `collision_bounds_${sphere.name}`;
        helper.visible = this.collisionDebugVisible;
        this.scene.add(helper);
        this.collisionDebugHelpers.push({ helper, link: target });
      }
    }
  }

  setCollisionDebugVisible(visible) {
    this.collisionDebugVisible = Boolean(visible);
    if (!this.compare) return;
    this.createCollisionDebugHelpers();
    for (const item of this.collisionDebugHelpers) item.helper.visible = this.collisionDebugVisible;
  }

  updateCollisionDebugHelpers() {
    if (!this.collisionDebugVisible) return;
    for (const item of this.collisionDebugHelpers) item.helper.update();
  }

  armSelfCollision(side = "right") {
    const armBounds = this.collisionBounds(this.armCollisionSpheres(side));
    const bodyBounds = this.collisionBounds(this.bodyCollisionSpheres());

    for (const arm of armBounds) {
      for (const body of bodyBounds) {
        if (arm.box.intersectsBox(body.box)) {
          return { colliding: true, arm: arm.name, body: body.name, method: "bounds" };
        }
      }
    }
    return { colliding: false };
  }

  armPositionJoints(side = "right") {
    // Shoulder + elbow only: positions the hand while leaving the wrist
    // orientation untouched (used by the panel's lock mode).
    return this.armIkJoints(side).filter((name) => !name.includes("_wrist_"));
  }

  solveArmTo(side, targetPosition, jointNames = null) {
    if (!this.modelReady || !this.robotRoot) return false;
    const effector = this.getEndEffectorObject(side);
    if (!effector) return false;
    const chain = jointNames || this.armIkJoints(side);
    const previousPose = this.armPoseSnapshot(side);
    let limited = false;

    const end = new THREE.Vector3();
    const jointPosition = new THREE.Vector3();
    const jointQuaternion = new THREE.Quaternion();
    const axisWorld = new THREE.Vector3();
    const toEnd = new THREE.Vector3();
    const toTarget = new THREE.Vector3();
    const cross = new THREE.Vector3();

    for (let iteration = 0; iteration < 14; iteration += 1) {
      effector.getWorldPosition(end);
      if (end.distanceTo(targetPosition) < 0.012) break;

      for (const jointName of [...chain].reverse()) {
        const joint = this.jointGroups.get(jointName);
        if (!joint || joint.type === "fixed") continue;

        joint.group.getWorldPosition(jointPosition);
        joint.group.getWorldQuaternion(jointQuaternion);
        axisWorld.copy(joint.axis).applyQuaternion(jointQuaternion).normalize();

        effector.getWorldPosition(end);
        toEnd.subVectors(end, jointPosition).normalize();
        toTarget.subVectors(targetPosition, jointPosition).normalize();
        if (toEnd.lengthSq() === 0 || toTarget.lengthSq() === 0) continue;

        cross.crossVectors(toEnd, toTarget);
        const signedAngle = Math.atan2(cross.dot(axisWorld), Math.max(-1, Math.min(1, toEnd.dot(toTarget))));
        if (!Number.isFinite(signedAngle) || Math.abs(signedAngle) < 0.0005) continue;

        const step = Math.max(-0.12, Math.min(0.12, signedAngle));
        const requested = (joint.value || 0) + step;
        const clamped = Math.max(joint.lower, Math.min(joint.upper, requested));
        if (Math.abs(clamped - requested) > 1e-6) limited = true;
        this.setJointValueIn(this.jointGroups, jointName, clamped);
        this.robotRoot.updateWorldMatrix(true, true);
      }
    }
    effector.getWorldPosition(end);
    const collision = this.armSelfCollision(side);
    if (collision.colliding) {
      this.restoreArmPose(previousPose);
      effector.getWorldPosition(end);
      this.emitIkStatus(end.distanceTo(targetPosition), limited, collision);
      return false;
    }
    this.emitIkStatus(end.distanceTo(targetPosition), limited, collision);
    return true;
  }

  emitIkStatus(error, limited, collision = null) {
    const blocked = Boolean(collision?.colliding);
    const reachable = Number.isFinite(error) && error < 0.04 && !limited && !blocked;
    window.dispatchEvent(
      new CustomEvent("recording-ik-status", {
        detail: {
          error,
          limited,
          collision,
          blocked,
          reachable,
        },
      }),
    );
  }

  currentEditedPoseSnapshot() {
    if (!this.latestState) return null;
    const snapshot = JSON.parse(JSON.stringify(this.latestState));
    snapshot.timestamp = Date.now() / 1000;
    snapshot.type = "telemetry_sample";
    snapshot.source = "end_effector_editor";
    for (const jointName of [...LEFT_ARM_IK_JOINTS, ...RIGHT_ARM_IK_JOINTS, TORSO_TWIST_JOINT]) {
      const bodyName = URDF_TO_BODY_JOINT[jointName];
      const joint = this.jointGroups.get(jointName);
      if (!bodyName || !joint) continue;
      const motor = (snapshot.motors || []).find((item) => item.name === bodyName);
      if (motor) motor.q = joint.value || 0;
    }
    return snapshot;
  }

  emitEditedPose(origin = "drag") {
    // origin "drag" = the operator moved the arms/torso in the editor;
    // origin "sync" = the pose was applied from a loaded file/replay frame.
    // The app uses this to let a fresh drag take Move priority over the
    // previously selected file.
    const snapshot = this.currentEditedPoseSnapshot();
    if (!snapshot) return;
    window.dispatchEvent(new CustomEvent("recording-edited-pose", { detail: { snapshot, origin } }));
  }

  loadVisualMeshes(linkElement, linkGroup, tone = "default") {
    const loader = new STLLoader();
    for (const visual of linkElement.querySelectorAll(":scope > visual")) {
      const meshElement = visual.querySelector(":scope > geometry > mesh");
      const filename = meshElement?.getAttribute("filename");
      if (!filename) continue;

      this.totalMeshes += 1;
      const visualGroup = new THREE.Group();
      visualGroup.name = `${linkGroup.name}_visual_${this.totalMeshes}`;
      visualGroup.userData.visualGeometry = true;
      applyOrigin(visualGroup, visual);
      linkGroup.add(visualGroup);

      loader.load(
        resolveMeshPath(filename),
        (geometry) => {
          geometry.computeVertexNormals();
          const mesh = new THREE.Mesh(geometry, materialFromVisual(visual, tone));
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          visualGroup.add(mesh);
          this.loadedMeshes += 1;
          if (this.latestState) this.applyTelemetry(this.latestState, this.live ? "live" : "replay");
        },
        undefined,
        () => {
          this.failedMeshes += 1;
          if (this.latestState) this.applyTelemetry(this.latestState, this.live ? "live" : "replay");
        },
      );
    }
  }

  buildRobot(xml, { name, tone = "default", targetGroups = null }) {
    const linkGroups = new Map();
    for (const link of xml.querySelectorAll("robot > link")) {
      const group = new THREE.Group();
      group.name = link.getAttribute("name");
      linkGroups.set(group.name, group);
      this.loadVisualMeshes(link, group, tone);
    }

    const root = new THREE.Group();
    root.name = name;
    this.scene.add(root);

    const childLinks = new Set();
    for (const joint of xml.querySelectorAll("robot > joint")) {
      const name = joint.getAttribute("name");
      const type = joint.getAttribute("type");
      const parent = joint.querySelector(":scope > parent")?.getAttribute("link");
      const child = joint.querySelector(":scope > child")?.getAttribute("link");
      const childGroup = linkGroups.get(child);
      if (!childGroup) continue;

      const jointGroup = new THREE.Group();
      jointGroup.name = name;
      applyOrigin(jointGroup, joint);
      const axis = new THREE.Vector3(...parseVector(joint.querySelector(":scope > axis")?.getAttribute("xyz"), [1, 0, 0]));
      axis.normalize();
      const limit = joint.querySelector(":scope > limit");
      const lower = Number(limit?.getAttribute("lower"));
      const upper = Number(limit?.getAttribute("upper"));

      if (targetGroups) {
        targetGroups.set(name, {
          group: jointGroup,
          axis,
          baseQuaternion: jointGroup.quaternion.clone(),
          type,
          lower: Number.isFinite(lower) ? lower : -Math.PI,
          upper: Number.isFinite(upper) ? upper : Math.PI,
          value: 0,
        });
      }

      const parentGroup = linkGroups.get(parent);
      if (parentGroup) {
        parentGroup.add(jointGroup);
      } else {
        root.add(jointGroup);
      }
      jointGroup.add(childGroup);
      childLinks.add(child);
    }

    for (const [name, group] of linkGroups) {
      if (!childLinks.has(name)) root.add(group);
    }

    root.rotation.x = -Math.PI / 2;
    root.position.y = -0.55;
    if (targetGroups === this.jointGroups) this.linkGroups = linkGroups;
    return root;
  }

  async loadRobot() {
    const response = await fetch(URDF_PATH);
    const text = await response.text();
    const xml = new DOMParser().parseFromString(text, "application/xml");

    if (this.compare) {
      this.buildRobot(xml, { name: "h1_2_reference", tone: "reference", targetGroups: this.referenceJointGroups });
      this.trajectoryRoot = this.buildRobot(xml, {
        name: "h1_2_trajectory",
        tone: "trajectory",
        targetGroups: this.trajectoryJointGroups,
      });
      this.trajectoryRoot.visible = false;
      this.robotRoot = this.buildRobot(xml, { name: "h1_2_replay", tone: "replay", targetGroups: this.jointGroups });
      this.createEndEffectorMarker();
      if (this.torsoRing) this.torsoRing.visible = true;
      this.setCollisionDebugVisible(this.collisionDebugVisible);
    } else {
      this.robotRoot = this.buildRobot(xml, { name: "h1_2", tone: "default", targetGroups: this.jointGroups });
    }
    this.modelReady = true;
    // Apply the operator's hands on/off preference to the freshly built models.
    this.setHandsVisible(window.localStorage?.getItem("h1_hands_enabled") !== "0");
    if (this.latestState) this.applyTelemetry(this.latestState, this.live ? "live" : "replay");
  }

  initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b0e11);

    this.camera = new THREE.PerspectiveCamera(48, 1, 0.01, 100);
    this.camera.position.set(2.15, 0.55, 0);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, -0.65, 0);
    this.controls.enableDamping = true;

    const hemi = new THREE.HemisphereLight(0xffffff, 0x24282d, 2.4);
    this.scene.add(hemi);
    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(3, 5, 3);
    key.castShadow = true;
    this.scene.add(key);

    this.gridHelper = new THREE.GridHelper(3, 30, 0x36424d, 0x242a30);
    this.gridHelper.position.y = FLOOR_Y;
    this.scene.add(this.gridHelper);
  }

  resize() {
    if (!this.renderer || !this.container) return;
    const rect = this.container.getBoundingClientRect();
    if (rect.width < 10 || rect.height < 10) return;
    this.renderer.setSize(rect.width, rect.height, false);
    this.camera.aspect = rect.width / rect.height;
    this.camera.updateProjectionMatrix();
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    this.controls?.update();
    this.resize();
    this.updateCollisionDebugHelpers();
    this.renderer?.render(this.scene, this.camera);
  }

  start() {
    if (!this.container || this.started) return;
    this.started = true;
    this.initScene();
    this.bindViewTools();
    this.setFields({ model: "loading", status: "loading URDF", meshes: "0/0" });
    this.loadRobot().catch((error) => {
      console.error(error);
      this.setFields({ model: "h1_2.urdf", status: "load failed", error: error.message });
    });
    this.animate();
  }
}

const params = new URLSearchParams(window.location.search);
const viewerDisabled = ["1", "true", "yes"].includes((params.get("lite") || params.get("no3d") || "").toLowerCase());

const liveViewer = new RobotViewer({
  container: document.getElementById("robotCanvas"),
  fields: document.getElementById("viewerFields"),
  live: true,
});

const replayViewer = new RobotViewer({
  container: document.getElementById("recordingReplayCanvas"),
  fields: document.getElementById("recordingReplayFields"),
  live: false,
  compare: true,
});

window.addEventListener("telemetry-state", (event) => {
  liveViewer.applyTelemetry(event.detail.snapshot, "live");
  replayViewer.applyReference(event.detail.snapshot);
});
window.addEventListener("recording-replay-frame", (event) => replayViewer.applyTelemetry(event.detail.snapshot, "replay"));
window.addEventListener("recording-replay-target", (event) => replayViewer.applyTelemetry(event.detail.snapshot, "target"));
window.addEventListener("recording-trajectory-frame", (event) => replayViewer.applyTrajectory(event.detail.snapshot));
window.addEventListener("recording-trajectory-visibility", (event) =>
  replayViewer.setTrajectoryVisible(Boolean(event.detail.visible)),
);
window.addEventListener("hands-visibility", (event) => {
  const enabled = Boolean(event.detail?.enabled);
  liveViewer.setHandsVisible(enabled);
  replayViewer.setHandsVisible(enabled);
});
window.addEventListener("recording-collision-debug", (event) =>
  replayViewer.setCollisionDebugVisible(Boolean(event.detail?.visible)),
);
window.addEventListener("recording-mirror-arm", (event) => {
  const source = event.detail?.source === "left" ? "left" : "right";
  const target = event.detail?.target === "right" ? "right" : "left";
  replayViewer.mirrorArmPose(source, target);
});
window.addEventListener("recording-arm-mirror-toggle", (event) =>
  replayViewer.setMirrorArmsEnabled(Boolean(event.detail?.enabled)),
);
window.addEventListener("telemetry-tab-change", () => {
  setTimeout(() => {
    liveViewer.resize();
    replayViewer.resize();
  }, 0);
});
window.addEventListener("resize", () => {
  liveViewer.resize();
  replayViewer.resize();
});

function startReplayViewer() {
  replayViewer.start();
  setTimeout(() => {
    replayViewer.resize();
    window.dispatchEvent(new CustomEvent("recording-viewer-ready"));
  }, 0);
}

function startVisibleViewers() {
  if (viewerDisabled) {
    liveViewer.setFields({ model: "disabled", status: "lite mode" });
    replayViewer.setFields({ model: "disabled", status: "lite mode" });
    return;
  }

  liveViewer.start();
  if (window.location.hash === "#recordingPage") startReplayViewer();
}

window.addEventListener("telemetry-tab-change", () => {
  if (!viewerDisabled && window.location.hash === "#recordingPage") startReplayViewer();
});

startVisibleViewers();

// --- 6-DOF hand-target panel ------------------------------------------------
// Clicking a hand ball (without dragging) opens a hovering panel with numeric
// control of the target: X/Y/Z position with a ground/relative frame toggle,
// and the three wrist joints (roll/pitch/yaw) as sliders bounded by their real
// URDF limits. Everything drives the same IK/collision path as dragging.
const RAD2DEG = 180 / Math.PI;

const EE_AXIS_RANGES = {
  ground: { x: [-1.2, 1.2], y: [-1.2, 1.2], z: [0, 2] },
  relative: { x: [-0.8, 0.8], y: [-0.8, 0.8], z: [-0.8, 0.8] },
};

const EE_AXIS_LABELS = { x: "X fwd", y: "Y left", z: "Z up" };

class EndEffectorPanel {
  constructor(viewer) {
    this.viewer = viewer;
    this.side = null;
    this.mode = "ground";
    this.inputs = new Map();
    this.root = null;
    // Lock mode: while editing one dimension, the other five hold their
    // values — position axes come from lockedWorld instead of the (slightly
    // drifting) marker, and the wrist is kept out of the position IK chain.
    this.lockOthers = false;
    this.lockedWorld = null;
    this.applying = false;
    this.build();
    this.bind();
  }

  applyAxis(axis, value) {
    this.applying = true;
    try {
      const locked = this.lockOthers && this.lockedWorld;
      const result = this.viewer.setEndEffectorAxis(
        this.side,
        this.mode,
        axis,
        value,
        locked ? { baseWorld: this.lockedWorld, excludeWrist: true } : {},
      );
      if (this.lockOthers && result.world) this.lockedWorld = result.world.clone();
    } finally {
      this.applying = false;
    }
  }

  applyWrist(key, value) {
    this.applying = true;
    try {
      this.viewer.setWristJoint(this.side, key, value, this.lockOthers ? this.lockedWorld : null);
    } finally {
      this.applying = false;
    }
  }

  captureLockTarget() {
    const marker = this.side ? this.viewer.markerForSide(this.side) : null;
    this.lockedWorld = marker && marker.visible ? marker.position.clone() : null;
  }

  build() {
    const root = document.createElement("div");
    root.className = "ee-panel";
    root.hidden = true;

    const head = document.createElement("div");
    head.className = "ee-panel-head";
    this.title = document.createElement("span");
    this.title.className = "ee-panel-title";
    head.appendChild(this.title);

    this.modeButtons = {};
    const toggle = document.createElement("div");
    toggle.className = "ee-frame-toggle";
    for (const [mode, label] of [["ground", "Ground"], ["relative", "Relative"]]) {
      const button = document.createElement("button");
      button.type = "button";
      button.textContent = label;
      button.title = mode === "ground"
        ? "X/Y from the pelvis axis, Z measured from the fixed floor"
        : "X/Y/Z offsets from the hand's initial (pre-edit) position";
      button.addEventListener("click", () => {
        this.mode = mode;
        this.refresh();
      });
      this.modeButtons[mode] = button;
      toggle.appendChild(button);
    }
    head.appendChild(toggle);

    const close = document.createElement("button");
    close.type = "button";
    close.className = "ee-panel-close";
    close.textContent = "×";
    close.addEventListener("click", () => this.close());
    head.appendChild(close);
    root.appendChild(head);
    this.bindPanelDrag(head);

    const lockRow = document.createElement("div");
    lockRow.className = "ee-panel-lock";
    this.lockButton = document.createElement("button");
    this.lockButton.type = "button";
    this.lockButton.title = "While changing one dimension, hold the other five exactly where they are";
    this.lockButton.addEventListener("click", () => {
      this.lockOthers = !this.lockOthers;
      if (this.lockOthers) this.captureLockTarget();
      else this.lockedWorld = null;
      this.refresh();
    });
    lockRow.appendChild(this.lockButton);
    root.appendChild(lockRow);

    for (const axis of ["x", "y", "z"]) {
      root.appendChild(this.buildRow(`pos-${axis}`, EE_AXIS_LABELS[axis], "m", 0.01, (value) => {
        this.applyAxis(axis, value);
      }));
    }
    for (const key of ["roll", "pitch", "yaw"]) {
      root.appendChild(this.buildRow(`wrist-${key}`, `W ${key}`, "°", 1, (value) => {
        this.applyWrist(key, value / RAD2DEG);
      }));
    }

    this.status = document.createElement("p");
    this.status.className = "ee-panel-status";
    root.appendChild(this.status);

    this.viewer.container.appendChild(root);
    this.root = root;
  }

  buildRow(key, label, unit, step, apply) {
    const row = document.createElement("div");
    row.className = "ee-panel-row";
    const caption = document.createElement("span");
    caption.textContent = label;
    const slider = document.createElement("input");
    slider.type = "range";
    slider.step = String(step);
    const number = document.createElement("input");
    number.type = "number";
    number.step = String(step);
    const suffix = document.createElement("span");
    suffix.className = "ee-panel-unit";
    suffix.textContent = unit;

    slider.addEventListener("input", () => {
      const value = Number(slider.value);
      if (Number.isFinite(value)) apply(value);
    });
    number.addEventListener("change", () => {
      const value = Number(number.value);
      if (Number.isFinite(value)) apply(value);
    });

    row.append(caption, slider, number, suffix);
    this.inputs.set(key, { slider, number });
    return row;
  }

  bindPanelDrag(handle) {
    // The header is a drag handle so the panel can be moved anywhere over the
    // viewer (it opens over the robot, which is often exactly what you want to
    // look at). Buttons inside the header keep working normally.
    let dragging = null;
    handle.addEventListener("pointerdown", (event) => {
      if (event.target.closest("button")) return;
      dragging = {
        pointerId: event.pointerId,
        offsetX: event.clientX - this.root.offsetLeft,
        offsetY: event.clientY - this.root.offsetTop,
      };
      handle.setPointerCapture?.(event.pointerId);
      event.preventDefault();
    });
    handle.addEventListener("pointermove", (event) => {
      if (!dragging || event.pointerId !== dragging.pointerId) return;
      const container = this.viewer.container.getBoundingClientRect();
      const x = Math.max(0, Math.min(container.width - this.root.offsetWidth, event.clientX - dragging.offsetX));
      const y = Math.max(0, Math.min(container.height - this.root.offsetHeight, event.clientY - dragging.offsetY));
      this.root.style.left = `${x}px`;
      this.root.style.top = `${y}px`;
    });
    const finish = (event) => {
      if (!dragging || event.pointerId !== dragging.pointerId) return;
      handle.releasePointerCapture?.(event.pointerId);
      dragging = null;
    };
    handle.addEventListener("pointerup", finish);
    handle.addEventListener("pointercancel", finish);
  }

  bind() {
    window.addEventListener("end-effector-selected", (event) => {
      const side = event.detail?.side === "left" ? "left" : "right";
      this.open(side);
    });
    window.addEventListener("end-effector-moved", (event) => {
      if (!this.side || this.root.hidden) return;
      if (event.detail?.side !== this.side && !this.viewer.mirrorArmsEnabled) return;
      // A move we didn't initiate (ball drag, replay sync) re-anchors the lock.
      if (this.lockOthers && !this.applying && event.detail?.side === this.side) this.captureLockTarget();
      this.refresh();
    });
    window.addEventListener("recording-ik-status", (event) => {
      if (this.root.hidden) return;
      const detail = event.detail || {};
      if (detail.blocked) {
        this.status.textContent = "Blocked: self-collision";
      } else if (detail.reachable === false) {
        this.status.textContent = "Target not fully reachable";
      } else {
        this.status.textContent = "IK solved";
      }
    });
    document.addEventListener("pointerdown", (event) => {
      if (this.root.hidden || this.root.contains(event.target)) return;
      this.close();
    });
    document.addEventListener("keydown", (event) => {
      if (event.key === "Escape") this.close();
    });
  }

  open(side) {
    this.side = side;
    this.root.hidden = false;
    this.status.textContent = "";
    if (this.lockOthers) this.captureLockTarget();
    this.refresh();
    this.position();
  }

  close() {
    this.root.hidden = true;
    this.side = null;
  }

  position() {
    // Always open along the right edge of the viewer (never over the robot),
    // vertically tracking the clicked ball. Draggable from there.
    const screen = this.viewer.endEffectorScreenPosition(this.side);
    const container = this.viewer.container.getBoundingClientRect();
    const width = this.root.offsetWidth || 250;
    const height = this.root.offsetHeight || 220;
    const x = Math.max(8, container.width - width - 14);
    const anchorY = screen ? screen.y - height / 2 : 14;
    const y = Math.max(8, Math.min(container.height - height - 8, anchorY));
    this.root.style.left = `${x}px`;
    this.root.style.top = `${y}px`;
  }

  setRow(key, value, min, max, digits) {
    const row = this.inputs.get(key);
    if (!row) return;
    row.slider.min = String(min);
    row.slider.max = String(max);
    row.number.min = String(min);
    row.number.max = String(max);
    const text = value.toFixed(digits);
    if (document.activeElement !== row.slider) row.slider.value = text;
    if (document.activeElement !== row.number) row.number.value = text;
  }

  refresh() {
    if (!this.side) return;
    const state = this.viewer.endEffectorPanelState(this.side);
    if (!state) {
      this.close();
      return;
    }
    this.title.textContent = `${this.side === "left" ? "Left" : "Right"} hand target`;
    for (const [mode, button] of Object.entries(this.modeButtons)) {
      button.classList.toggle("active", mode === this.mode);
    }
    this.lockButton.textContent = this.lockOthers ? "🔒 Lock others: on" : "🔓 Lock others: off";
    this.lockButton.classList.toggle("active", this.lockOthers);
    const ranges = EE_AXIS_RANGES[this.mode];
    const coords = state[this.mode === "relative" ? "relative" : "ground"];
    for (const axis of ["x", "y", "z"]) {
      this.setRow(`pos-${axis}`, coords[axis], ranges[axis][0], ranges[axis][1], 3);
    }
    for (const [key, joint] of Object.entries(state.wrist)) {
      this.setRow(
        `wrist-${key}`,
        joint.value * RAD2DEG,
        Math.ceil(joint.lower * RAD2DEG),
        Math.floor(joint.upper * RAD2DEG),
        0,
      );
    }
  }
}

if (!viewerDisabled) new EndEffectorPanel(replayViewer);
