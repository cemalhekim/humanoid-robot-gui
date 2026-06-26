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

const HAND_JOINTS = {
  RightPinky: ["R_pinky_proximal_joint", "R_pinky_intermediate_joint"],
  RightRing: ["R_ring_proximal_joint", "R_ring_intermediate_joint"],
  RightMiddle: ["R_middle_proximal_joint", "R_middle_intermediate_joint"],
  RightIndex: ["R_index_proximal_joint", "R_index_intermediate_joint"],
  RightThumbBend: ["R_thumb_proximal_pitch_joint", "R_thumb_intermediate_joint", "R_thumb_distal_joint"],
  RightThumbRotation: ["R_thumb_proximal_yaw_joint"],
  LeftPinky: ["L_pinky_proximal_joint", "L_pinky_intermediate_joint"],
  LeftRing: ["L_ring_proximal_joint", "L_ring_intermediate_joint"],
  LeftMiddle: ["L_middle_proximal_joint", "L_middle_intermediate_joint"],
  LeftIndex: ["L_index_proximal_joint", "L_index_intermediate_joint"],
  LeftThumbBend: ["L_thumb_proximal_pitch_joint", "L_thumb_intermediate_joint", "L_thumb_distal_joint"],
  LeftThumbRotation: ["L_thumb_proximal_yaw_joint"],
};

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

function materialFromVisual(visual) {
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
  constructor({ container, fields, live = false }) {
    this.container = container;
    this.fields = fields;
    this.live = live;
    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;
    this.robotRoot = null;
    this.gridHelper = null;
    this.jointGroups = new Map();
    this.latestState = null;
    this.loadedMeshes = 0;
    this.failedMeshes = 0;
    this.totalMeshes = 0;
    this.modelReady = false;
    this.autoRotate = false;
  }

  setFields(data) {
    if (!this.fields) return;
    this.fields.innerHTML = Object.entries(data)
      .map(([key, value]) => `<dt>${key}</dt><dd>${value}</dd>`)
      .join("");
  }

  setJointValue(jointName, value) {
    const joint = this.jointGroups.get(jointName);
    if (!joint || !Number.isFinite(value)) return;
    joint.group.quaternion.copy(joint.baseQuaternion);
    joint.group.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(joint.axis, value));
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
      for (const jointName of urdfJoints) this.setJointValue(jointName, hand.q);
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
  }

  loadVisualMeshes(linkElement, linkGroup) {
    const loader = new STLLoader();
    for (const visual of linkElement.querySelectorAll(":scope > visual")) {
      const meshElement = visual.querySelector(":scope > geometry > mesh");
      const filename = meshElement?.getAttribute("filename");
      if (!filename) continue;

      this.totalMeshes += 1;
      const visualGroup = new THREE.Group();
      applyOrigin(visualGroup, visual);
      linkGroup.add(visualGroup);

      loader.load(
        resolveMeshPath(filename),
        (geometry) => {
          geometry.computeVertexNormals();
          const mesh = new THREE.Mesh(geometry, materialFromVisual(visual));
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

  async loadRobot() {
    const response = await fetch(URDF_PATH);
    const text = await response.text();
    const xml = new DOMParser().parseFromString(text, "application/xml");

    const linkGroups = new Map();
    for (const link of xml.querySelectorAll("robot > link")) {
      const group = new THREE.Group();
      group.name = link.getAttribute("name");
      linkGroups.set(group.name, group);
      this.loadVisualMeshes(link, group);
    }

    this.robotRoot = new THREE.Group();
    this.robotRoot.name = "h1_2";
    this.scene.add(this.robotRoot);

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

      this.jointGroups.set(name, {
        group: jointGroup,
        axis,
        baseQuaternion: jointGroup.quaternion.clone(),
        type,
      });

      const parentGroup = linkGroups.get(parent);
      if (parentGroup) {
        parentGroup.add(jointGroup);
      } else {
        this.robotRoot.add(jointGroup);
      }
      jointGroup.add(childGroup);
      childLinks.add(child);
    }

    for (const [name, group] of linkGroups) {
      if (!childLinks.has(name)) this.robotRoot.add(group);
    }

    this.robotRoot.rotation.x = -Math.PI / 2;
    this.robotRoot.position.y = -0.55;
    this.modelReady = true;
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
    this.renderer?.render(this.scene, this.camera);
  }

  start() {
    if (!this.container) return;
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

const liveViewer = new RobotViewer({
  container: document.getElementById("robotCanvas"),
  fields: document.getElementById("viewerFields"),
  live: true,
});

const replayViewer = new RobotViewer({
  container: document.getElementById("recordingReplayCanvas"),
  fields: document.getElementById("recordingReplayFields"),
  live: false,
});

window.addEventListener("telemetry-state", (event) => liveViewer.applyTelemetry(event.detail.snapshot, "live"));
window.addEventListener("recording-replay-frame", (event) => replayViewer.applyTelemetry(event.detail.snapshot, "replay"));
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

liveViewer.start();
replayViewer.start();
