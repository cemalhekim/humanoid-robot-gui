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

const container = document.getElementById("robotCanvas");
const fields = document.getElementById("viewerFields");
const fallback = document.getElementById("viewerFallback");
let scene;
let camera;
let renderer;
let controls;
let robotRoot;
let quickRoot;
let gridHelper;
let jointGroups = new Map();
let latestState = null;
let loadedMeshes = 0;
let failedMeshes = 0;
let totalMeshes = 0;
let modelReady = false;
let autoRotate = false;
const placeholderMaterial = new THREE.MeshStandardMaterial({
  color: 0x4b6f8f,
  roughness: 0.8,
  metalness: 0.05,
  transparent: true,
  opacity: 0.34,
  wireframe: true,
});

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

function placeholderSize(linkName) {
  const name = String(linkName || "").toLowerCase();
  if (name.includes("pelvis")) return [0.22, 0.18, 0.2];
  if (name.includes("torso")) return [0.24, 0.18, 0.34];
  if (name.includes("hip") || name.includes("shoulder")) return [0.12, 0.12, 0.16];
  if (name.includes("knee") || name.includes("elbow")) return [0.1, 0.1, 0.24];
  if (name.includes("ankle") || name.includes("wrist")) return [0.09, 0.09, 0.14];
  if (name.includes("hand") || /link\d+_[lr]/.test(name)) return [0.045, 0.045, 0.09];
  return [0.08, 0.08, 0.12];
}

function addLinkPlaceholder(linkName, linkGroup) {
  const geometry = new THREE.BoxGeometry(...placeholderSize(linkName));
  const mesh = new THREE.Mesh(geometry, placeholderMaterial);
  mesh.name = `${linkName || "link"}_placeholder`;
  linkGroup.add(mesh);
  return mesh;
}

function quickPart(size, position, color = 0x9aa8b4) {
  const material = new THREE.MeshStandardMaterial({
    color,
    roughness: 0.75,
    metalness: 0.08,
    transparent: true,
    opacity: 0.68,
  });
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), material);
  mesh.position.set(...position);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function createQuickRobot() {
  const group = new THREE.Group();
  group.name = "quick_h1_placeholder";
  group.add(quickPart([0.28, 0.34, 0.18], [0, -0.35, 0], 0xc3ccd3));
  group.add(quickPart([0.22, 0.18, 0.2], [0, -0.72, 0], 0x7f8c96));
  group.add(quickPart([0.12, 0.46, 0.11], [0, -1.08, 0.13], 0x8fa1ad));
  group.add(quickPart([0.12, 0.46, 0.11], [0, -1.08, -0.13], 0x8fa1ad));
  group.add(quickPart([0.11, 0.36, 0.1], [0, -1.45, 0.13], 0x6e7e89));
  group.add(quickPart([0.11, 0.36, 0.1], [0, -1.45, -0.13], 0x6e7e89));
  group.add(quickPart([0.12, 0.52, 0.1], [0, -0.36, 0.34], 0x8fa1ad));
  group.add(quickPart([0.12, 0.52, 0.1], [0, -0.36, -0.34], 0x8fa1ad));
  group.add(quickPart([0.1, 0.36, 0.09], [0, -0.78, 0.34], 0x6e7e89));
  group.add(quickPart([0.1, 0.36, 0.09], [0, -0.78, -0.34], 0x6e7e89));
  group.add(quickPart([0.08, 0.08, 0.08], [0, -0.03, 0], 0xe60000));
  return group;
}

function resolveMeshPath(filename) {
  if (filename.startsWith("package://")) {
    const parts = filename.split("/");
    return `${MODEL_BASE}${parts.slice(-2).join("/")}`;
  }
  return `${MODEL_BASE}${filename}`;
}

function setFields(data) {
  if (!fields) return;
  fields.innerHTML = Object.entries(data)
    .map(([key, value]) => `<dt>${key}</dt><dd>${value}</dd>`)
    .join("");
}

function showFallback() {
  fallback?.classList.remove("is-hidden");
}

function hideFallback() {
  fallback?.classList.add("is-hidden");
}

function setJointValue(jointName, value) {
  const joint = jointGroups.get(jointName);
  if (!joint || !Number.isFinite(value)) return;
  joint.group.quaternion.copy(joint.baseQuaternion);
  joint.group.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(joint.axis, value));
}

function setCamera(position, target = [0, 0.15, 0]) {
  camera.position.set(position[0], position[1], position[2]);
  controls.target.set(target[0], target[1], target[2]);
  controls.update();
  resize();
}

function bindViewTools() {
  document.querySelectorAll("[data-view]").forEach((button) => {
    button.addEventListener("click", () => {
      const view = button.dataset.view;
      if (["front", "back", "left", "right", "top", "bottom"].includes(view)) {
        document.querySelectorAll(".cube-label.active").forEach((label) => label.classList.remove("active"));
        button.classList.add("active");
      }
      if (view === "front") setCamera([2.15, 0.55, 0], [0, -0.65, 0]);
      if (view === "back") setCamera([-2.15, 0.55, 0], [0, -0.65, 0]);
      if (view === "left") setCamera([0, 0.55, 2.15], [0, -0.65, 0]);
      if (view === "right") setCamera([0, 0.55, -2.15], [0, -0.65, 0]);
      if (view === "top") setCamera([0.01, 2.25, 0.01], [0, -0.55, 0]);
      if (view === "bottom") setCamera([0.01, -3.0, 0.01], [0, -0.55, 0]);
      if (view === "grid" && gridHelper) {
        gridHelper.visible = !gridHelper.visible;
        button.classList.toggle("active", gridHelper.visible);
      }
      if (view === "rotate") {
        autoRotate = !autoRotate;
        controls.autoRotate = autoRotate;
        controls.autoRotateSpeed = 1.2;
        button.classList.toggle("active", autoRotate);
      }
    });
  });

  const gridButton = document.querySelector('[data-view="grid"]');
  gridButton?.classList.add("active");
  document.querySelector('[data-view="front"]')?.classList.add("active");
}

function applyTelemetry(snapshot) {
  latestState = snapshot;
  if (!modelReady) return;

  for (const motor of snapshot.motors || []) {
    const urdfJoint = BODY_JOINTS[motor.name];
    if (urdfJoint) setJointValue(urdfJoint, motor.q);
  }

  for (const hand of snapshot.hands?.joints || []) {
    const urdfJoints = HAND_JOINTS[hand.name] || [];
    for (const jointName of urdfJoints) setJointValue(jointName, hand.q);
  }

  setFields({
    model: "h1_2.urdf",
    status: snapshot.connected ? "live" : "waiting",
    body_motors: snapshot.motor_count ?? 0,
    hand_joints: snapshot.hands?.joint_count ?? 0,
    sample_rate: `${snapshot.sample_rate_hz ?? 0} Hz`,
    meshes: `${loadedMeshes}/${totalMeshes}`,
    failed_meshes: failedMeshes,
  });
}

function isDeferredMesh(filename) {
  return /(?:L_|R_|hand|link\d+_[LR])/i.test(filename);
}

function loadVisualMeshes(linkElement, linkGroup) {
  const loader = new STLLoader();
  const placeholder = addLinkPlaceholder(linkElement.getAttribute("name"), linkGroup);
  for (const visual of linkElement.querySelectorAll(":scope > visual")) {
    const meshElement = visual.querySelector(":scope > geometry > mesh");
    const filename = meshElement?.getAttribute("filename");
    if (!filename) continue;

    totalMeshes += 1;
    const visualGroup = new THREE.Group();
    applyOrigin(visualGroup, visual);
    linkGroup.add(visualGroup);

    const loadMesh = () => {
      loader.load(
        resolveMeshPath(filename),
        (geometry) => {
          geometry.computeVertexNormals();
          const mesh = new THREE.Mesh(geometry, materialFromVisual(visual));
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          visualGroup.add(mesh);
          placeholder.visible = false;
          loadedMeshes += 1;
          if (latestState) applyTelemetry(latestState);
        },
        undefined,
        () => {
          failedMeshes += 1;
          if (latestState) applyTelemetry(latestState);
        },
      );
    };
    window.setTimeout(loadMesh, isDeferredMesh(filename) ? 1200 : 0);
  }
}

async function loadRobot() {
  const response = await fetch(URDF_PATH);
  const text = await response.text();
  const xml = new DOMParser().parseFromString(text, "application/xml");

  const linkGroups = new Map();
  for (const link of xml.querySelectorAll("robot > link")) {
    const group = new THREE.Group();
    group.name = link.getAttribute("name");
    linkGroups.set(group.name, group);
    loadVisualMeshes(link, group);
  }

  robotRoot = new THREE.Group();
  robotRoot.name = "h1_2";
  scene.add(robotRoot);

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

    jointGroups.set(name, {
      group: jointGroup,
      axis,
      baseQuaternion: jointGroup.quaternion.clone(),
      type,
    });

    const parentGroup = linkGroups.get(parent);
    if (parentGroup) {
      parentGroup.add(jointGroup);
    } else {
      robotRoot.add(jointGroup);
    }
    jointGroup.add(childGroup);
    childLinks.add(child);
  }

  for (const [name, group] of linkGroups) {
    if (!childLinks.has(name)) robotRoot.add(group);
  }

  robotRoot.rotation.x = -Math.PI / 2;
  robotRoot.position.y = -0.55;
  modelReady = true;
  if (latestState) applyTelemetry(latestState);
}

function initScene() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0b0e11);

  camera = new THREE.PerspectiveCamera(48, 1, 0.01, 100);
  camera.position.set(2.15, 0.55, 0);

  try {
    renderer = new THREE.WebGLRenderer({ antialias: true });
  } catch (error) {
    showFallback();
    console.error("3D viewer unavailable:", error);
    return false;
  }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  container.appendChild(renderer.domElement);
  hideFallback();

  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, -0.65, 0);
  controls.enableDamping = true;

  const hemi = new THREE.HemisphereLight(0xffffff, 0x24282d, 2.4);
  scene.add(hemi);
  const key = new THREE.DirectionalLight(0xffffff, 2.2);
  key.position.set(3, 5, 3);
  key.castShadow = true;
  scene.add(key);

  gridHelper = new THREE.GridHelper(3, 30, 0x36424d, 0x242a30);
  gridHelper.position.y = FLOOR_Y;
  scene.add(gridHelper);
  quickRoot = createQuickRobot();
  scene.add(quickRoot);
  return true;
}

function resize() {
  if (!renderer || !container) return;
  const rect = container.getBoundingClientRect();
  if (rect.width < 10 || rect.height < 10) return;
  renderer.setSize(rect.width, rect.height, false);
  camera.aspect = rect.width / rect.height;
  camera.updateProjectionMatrix();
}

function animate() {
  requestAnimationFrame(animate);
  controls?.update();
  resize();
  renderer?.render(scene, camera);
}

window.addEventListener("telemetry-state", (event) => applyTelemetry(event.detail.snapshot));
window.addEventListener("telemetry-tab-change", () => setTimeout(resize, 0));
window.addEventListener("resize", resize);

if (initScene()) {
  bindViewTools();
  setFields({ model: "loading", status: "loading URDF", meshes: "0/0" });
  loadRobot().catch((error) => {
    console.error(error);
    setFields({ model: "h1_2.urdf", status: "load failed", error: error.message });
  });
  animate();
} else {
  setFields({ model: "h1_2.urdf", status: "fallback" });
}
