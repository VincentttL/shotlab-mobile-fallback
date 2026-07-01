import * as THREE from 'https://esm.sh/three@0.160.0';
import { OrbitControls } from 'https://esm.sh/three@0.160.0/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'https://esm.sh/three@0.160.0/examples/jsm/loaders/GLTFLoader.js';
import Chart from 'https://esm.sh/chart.js@4.4.1/auto';

import {
  DEFAULT_COURT,
  brancazioOptimum,
  courtWith,
  deg,
  rad,
  makeSpeed,
  entryAngle,
  trajectoryPoints,
  simulateNoiseExperiment,
  rimPlaneDotCloud,
  simulatedLandingPointCloud,
  makeSensitivity,
  heisenbergCurve,
  classifyShot,
  coachingSentence,
  lateralExperiment,
  forceLineReadout,
  rimTargetRadii,
  kineticChainRelease,
  clamp,
  DEMO_PRESET_PHYSICS,
  DEMO_PRESET_ORDER,
  COLD_OPEN_DEMO,
  FORM_MODES,
  SPL_FREE_THROW_SAMPLES,
  SPL_FREE_THROW_SOURCE,
  formForceProfile,
  splShotToRimPlane,
} from './lab_physics.mjs';
import {
  MIXAMO_BONE_MAP,
  RIGGED_AVATAR_ASSET,
  avatarPhasePose,
  mixamoBoneAliases,
} from './lab_avatar.mjs';

const $ = (id) => document.getElementById(id);
const fmt = (x, n = 2) => Number.isFinite(x) ? x.toFixed(n) : '—';

const controls = {
  angle: $('angle'), speed: $('speed'), height: $('height'), distance: $('distance'),
  lateral: $('lateral'), sigmaV: $('sigmaV'), sigmaTheta: $('sigmaTheta'), backspin: $('backspin'),
};
const outs = {
  angle: $('angleOut'), speed: $('speedOut'), height: $('heightOut'), distance: $('distanceOut'),
  lateral: $('lateralOut'), sigmaV: $('sigmaVOut'), sigmaTheta: $('sigmaThetaOut'), backspin: $('backspinOut'),
};
const whipControls = {
  shoulder: $('omShoulder'), elbow: $('omElbow'), wrist: $('omWrist'), legs: $('omLegs'),
};
const whipOuts = {
  shoulder: $('omShoulderOut'), elbow: $('omElbowOut'), wrist: $('omWristOut'), legs: $('omLegsOut'),
};
const aimRows = $('aimRows');
const whipRows = $('whipRows');
const formModeRows = $('formModeRows');
const formFeedback = $('formFeedback');
const formFeedbackText = $('formFeedbackText');
const splRows = $('splRows');
const splLayerToggle = $('splLayerToggle');
const forceLineTape = $('forceLineTape');
const forceLineTapeResult = $('forceLineTapeResult');
const forceLineTapeReadout = $('forceLineTapeReadout');
const forceLineMarker = $('forceLineMarker');
const forceLineTapeExplain = $('forceLineTapeExplain');
const forceLineTapeHome = forceLineTape?.parentNode || null;
const forceLineTapeHomeNext = forceLineTape?.nextSibling || null;
const noisePresetRows = $('noisePresetRows');
const noisePresetRowsHome = noisePresetRows?.parentNode || null;
const noisePresetRowsHomeNext = noisePresetRows?.nextSibling || null;
const splRowsHome = splRows?.parentNode || null;
const splRowsHomeNext = splRows?.nextSibling || null;
const splRowsRail = document.querySelector('.demo-rail');
const splRowsRailBefore = $('firstGestureCue');
const shotVerdictEl = $('shotVerdict');
const verdictFlag = $('verdictFlag');
const verdictEyebrow = $('verdictEyebrow');
const verdictHeadline = $('verdictHeadline');
const verdictHint = $('verdictHint');
const stationTitle = $('stationTitle');
const stationNote = $('stationNote');
const equationBox = $('equationBox');
const legend = $('legend');
const labCanvas = $('labCanvas');
const labCtx = labCanvas.getContext('2d');
const firewallCanvas = $('firewallChart');
const presentationToggle = $('presentationToggle');
const controlsJump = $('controlsJump');
const analysisJump = $('analysisJump');
const controlsPanel = $('controlsPanel');
const analysisPanel = $('analysisPanel');
const takeawayKicker = $('takeawayKicker');
const takeawayTitle = $('takeawayTitle');
const takeawayBody = $('takeawayBody');
const takeawayStep = $('takeawayStep');

let activeStation = 'geometry';
let lastResult = null;
let lastSensitivity = null;
let lastLateral = null;
let lastForceLine = null;
let lastWhip = null;
let lastForm = null;
let activeFormModeKey = null;
let showSplLayer = true;
let activeNoisePresetKey = 'game';
let firewallChart = null;
let launchStart = performance.now();
let isLaunching = true;
let latestTrajectory = [];

const stationCopy = {
  geometry: {
    title: 'Geometry Lab',
    note: '拖动 θ/v/h/d。橙色弧线是当前投篮；蓝绿色圆/椭圆目标面是球心必须穿过的有效篮筐区域。你会看到最优角不是“感觉”，而是 constraint curve 的最低点。',
    eq: 'H − h = d tan θ − g d² / (2v² cos²θ)<br>make: (depth/r)² + (lateral/r)² ≤ 1',
    legend: [['#ff9f43', 'current shot'], ['#60d8ff', 'make ridge'], ['#7cf7a1', 'circular rim target']],
  },
  whip: {
    title: 'Kinetic Chain Lab',
    note: 'WaveBall 的核心论点：投篮是“全身动力链的末端释放”，不是手臂去推。这里你不瞄准 θ/v——你“造”出它们。每个关节的绝对转速是下面关节的累加 (Ω_i = Σ ω_j)，所以手腕继承了肩+肘的转动：单独甩手腕几乎没速度，链条喂给它才有。腿+躯干的 base 速度直接叠加。造好的 release 直接喂给真实弹道，看进不进。',
    eq: 'Ω_i = Σ_{j≤i} ω_j (summation of speed)<br>v_end = v_base + Σ_i Ω_i (k̂ × R_i)',
    legend: [['#ff9f43', 'arm chain'], ['#60d8ff', 'ball velocity'], ['#7cf7a1', 'cumulative speed']],
  },
  force: {
    title: 'Force-Line Lab',
    note: 'WaveBall 说很多真实投篮问题是“弹道偏左”：力线没有对框。这里你拖 v_lat，球不会被重力拉回中线；它只会按 Δx = v_lat · t_flight 直线漂走。现在判定不是矩形边界，而是看 depth + lateral 是否还在圆形篮筐目标内。',
    eq: 'Δx_lateral = v_lat · t_flight<br>make iff (depth/r)² + (lateral/r)² ≤ 1',
    legend: [['#60d8ff', 'circular rim target'], ['#ff9f43', 'current rim offset'], ['#ff6b6b', 'no restoring force']],
  },
  noise: {
    title: 'Noise Lab',
    note: '这里像实验室重复发球：同一个平均出手点，加入速度/角度/force-line jitter。现在 hero 视图优先显示模拟落点点云：绿色是进，红色是偏出；SPL real shots 是真实测量 overlay，不替代圆形 rim-plane 模型。',
    eq: 'P(make) = Monte Carlo landing dots inside circular rim target<br>noise = σv + σθ + σ_lat force-line jitter',
    legend: [['#7cf7a1', 'simulated make'], ['#ff6b6b', 'simulated miss'], ['#f5efe4', 'simulated landing points']],
  },
  eigen: {
    title: 'Eigen Lab',
    note: 'Key result: after standardizing angle/speed error by σ, the make window is thinnest along one direction. Near this setup, the stiff axis is almost pure release speed — short/long control beats tiny angle tweaks.',
    eq: 'z = a·uθ + b·uv,  M = g gᵀ<br>eig(M) → STIFF axis + NULL axis',
    legend: [['#ff9f43', 'STIFF = speed'], ['#60d8ff', 'NULL ≈ angle'], ['#7cf7a1', '|z| ≤ 1 make slab']],
  },
  firewall: {
    title: 'Analogy Test Lab',
    note: '我们故意测试一个诱人的类比：有没有 Δv·Δθ ≥ const？计算结果相反：在最优角附近 tolerance product 变大/发散。类比被实验推翻。',
    eq: 'U(θ) = (w / |∂depth/∂v|) · (w / |∂depth/∂θ|)<br>At θ*: ∂depth/∂θ → 0 → no lower bound',
    legend: [['#b59cff', 'U(θ)'], ['#ff9f43', 'Brancazio optimum'], ['#ff6b6b', 'analogy refuted']],
  },
  form: {
    title: 'Form + Forces Lab',
    note: '你刚指出的关键问题：滑块改变了弹道，但身体到底怎么改？这里把 shoulder / elbow / wrist / legs / backspin 翻译成手型反馈；新增 CMU free throw trace 用真实 124_04 mocap 关节角度曲线显示 knee→shoulder→elbow→wrist 的人体层。注意：CMU 给身体时序，不给进球结果；backspin/Magnus 仍是可视化 cue，不改 projectile core。',
    eq: 'ground → shoulder → elbow → wrist → fingers → ball<br>ω_spin = backspin rpm; Magnus shown as visual/toy lift cue only',
    legend: [['#7cf7a1', 'CMU/ground trace'], ['#ff9f43', 'joint force transfer'], ['#60d8ff', 'backspin / lift cue']],
  },
};

const MATH_EQUATIONS = {
  geometry: `
    <div class="math-line"><math display="block"><mrow><mi>H</mi><mo>−</mo><mi>h</mi><mo>=</mo><mi>d</mi><mo>tan</mo><mi>θ</mi><mo>−</mo><mfrac><mrow><mi>g</mi><msup><mi>d</mi><mn>2</mn></msup></mrow><mrow><mn>2</mn><msup><mi>v</mi><mn>2</mn></msup><msup><mrow><mo>cos</mo><mi>θ</mi></mrow><mn>2</mn></msup></mrow></mfrac></mrow></math></div>
    <div class="math-line"><math display="block"><mrow><mi>make</mi><mo>:</mo><msup><mrow><mo>(</mo><mi>depth</mi><mo>/</mo><mi>r</mi><mo>)</mo></mrow><mn>2</mn></msup><mo>+</mo><msup><mrow><mo>(</mo><mi>lateral</mi><mo>/</mo><mi>r</mi><mo>)</mo></mrow><mn>2</mn></msup><mo>≤</mo><mn>1</mn></mrow></math></div>`,
  whip: `
    <div class="math-line"><math display="block"><mrow><msub><mi>Ω</mi><mi>i</mi></msub><mo>=</mo><munderover><mo>∑</mo><mrow><mi>j</mi><mo>≤</mo><mi>i</mi></mrow><mrow></mrow></munderover><msub><mi>ω</mi><mi>j</mi></msub></mrow></math></div>
    <div class="math-line"><math display="block"><mrow><msub><mi>v</mi><mi>end</mi></msub><mo>=</mo><msub><mi>v</mi><mi>base</mi></msub><mo>+</mo><mo>∑</mo><msub><mi>Ω</mi><mi>i</mi></msub><mo>(</mo><mover><mi>k</mi><mo>^</mo></mover><mo>×</mo><msub><mi>R</mi><mi>i</mi></msub><mo>)</mo></mrow></math></div>`,
  force: `
    <div class="math-line"><math display="block"><mrow><mi>Δx</mi><mo>=</mo><msub><mi>v</mi><mi>lat</mi></msub><mo>·</mo><msub><mi>t</mi><mi>flight</mi></msub></mrow></math></div>
    <div class="math-line"><math display="block"><mrow><msup><mrow><mo>(</mo><mi>depth</mi><mo>/</mo><mi>r</mi><mo>)</mo></mrow><mn>2</mn></msup><mo>+</mo><msup><mrow><mo>(</mo><mi>lateral</mi><mo>/</mo><mi>r</mi><mo>)</mo></mrow><mn>2</mn></msup><mo>≤</mo><mn>1</mn></mrow></math></div>`,
  noise: `
    <div class="math-line"><math display="block"><mrow><mi>P</mi><mo>(</mo><mi>make</mi><mo>)</mo><mo>=</mo><mfrac><mrow><mi># dots inside target</mi></mrow><mrow><mi># simulated shots</mi></mrow></mfrac></mrow></math></div>
    <div class="math-line"><math display="block"><mrow><mi>noise</mi><mo>=</mo><msub><mi>σ</mi><mi>v</mi></msub><mo>+</mo><msub><mi>σ</mi><mi>θ</mi></msub><mo>+</mo><msub><mi>σ</mi><mi>lat</mi></msub><mo>·</mo><msub><mi>t</mi><mi>flight</mi></msub></mrow></math></div>`,
  eigen: `
    <div class="math-line"><math display="block"><mrow><mi>z</mi><mo>=</mo><mi>a</mi><msub><mi>u</mi><mi>θ</mi></msub><mo>+</mo><mi>b</mi><msub><mi>u</mi><mi>v</mi></msub><mo>,</mo><mspace width="0.6em"/><mi>M</mi><mo>=</mo><mi>g</mi><msup><mi>g</mi><mi>T</mi></msup></mrow></math></div>
    <div class="math-line"><math display="block"><mrow><mi>eig</mi><mo>(</mo><mi>M</mi><mo>)</mo><mo>→</mo><mi>STIFF</mi><mo>+</mo><mi>NULL</mi></mrow></math></div>`,
  firewall: `
    <div class="math-line"><math display="block"><mrow><mi>U</mi><mo>(</mo><mi>θ</mi><mo>)</mo><mo>=</mo><mfrac><mi>w</mi><mrow><mo>|</mo><mo>∂</mo><mi>depth</mi><mo>/</mo><mo>∂</mo><mi>v</mi><mo>|</mo></mrow></mfrac><mo>·</mo><mfrac><mi>w</mi><mrow><mo>|</mo><mo>∂</mo><mi>depth</mi><mo>/</mo><mo>∂</mo><mi>θ</mi><mo>|</mo></mrow></mfrac></mrow></math></div>`,
  form: `
    <div class="math-line"><math display="block"><mrow><mi>ground</mi><mo>→</mo><mi>shoulder</mi><mo>→</mo><mi>elbow</mi><mo>→</mo><mi>wrist</mi><mo>→</mo><mi>fingers</mi><mo>→</mo><mi>ball</mi></mrow></math></div>
    <div class="math-line"><math display="block"><mrow><mi>backspin rate</mi><mo>=</mo><mi>rpm control</mi><mo>→</mo><mi>visual lift cue</mi></mrow></math></div>`,
};

const NOISE_PRESETS = Object.freeze({
  tight: { label: 'clean lab', sigmaV: 0.035, sigmaThetaDeg: 0.70, sigmaLateral: 0.010, vLat: 0 },
  game: { label: 'game pressure', sigmaV: 0.095, sigmaThetaDeg: 2.20, sigmaLateral: 0.035, vLat: 0 },
  fatigue: { label: 'fatigue bloom', sigmaV: 0.140, sigmaThetaDeg: 3.20, sigmaLateral: 0.055, vLat: 0 },
  left: { label: 'force-line leak', sigmaV: 0.075, sigmaThetaDeg: 1.50, sigmaLateral: 0.030, vLat: 0.110 },
});

function currentNoisePreset() {
  return NOISE_PRESETS[activeNoisePresetKey] || NOISE_PRESETS.game;
}

const TAKEAWAYS = Object.freeze({
  geometry: {
    kicker: 'geometry takeaway',
    title: 'The basket is a circular target, not a rectangular make box.',
    body: 'Tune angle, speed, height, and distance until the orange arc threads the eroded ball-center rim circle.',
    step: 'Demo: move θ a little, then reset to the make ridge and compare the verdict.'
  },
  whip: {
    kicker: 'kinetic-chain takeaway',
    title: 'The body builds θ/v.',
    body: 'Shoulder, elbow, wrist, and legs sum into the release. Wrist-only looks flashy but loses speed without the chain beneath it.',
    step: 'Demo: hit “Full chain swish,” then “Wrist alone.” The verdict tells the story.'
  },
  force: {
    kicker: 'force-line takeaway',
    title: 'A tiny sideways velocity becomes centimeters at the rim.',
    body: 'There is no magic restoring force: Δx = v_lat · t_flight. Depth error also consumes lateral allowance inside the same circle.',
    step: 'Demo: try “Tiny left drift,” then watch the rim-plane tape go outside.'
  },
  noise: {
    kicker: 'repeatability takeaway',
    title: 'One perfect release is not a player; the cloud is the player.',
    body: 'Speed, angle, and force-line jitter turn a clean mean release into a landing-point distribution with make and miss dots.',
    step: 'Demo: switch clean lab → game pressure → fatigue bloom and read P(make).'
  },
  eigen: {
    kicker: 'eigen takeaway',
    title: 'At this free throw, the dangerous error direction is speed.',
    body: 'The green slab is the make window. The orange stiff axis points where small release errors hurt most; here it is nearly all v.',
    step: 'Demo: show the orange STIFF/speed line, then say: train repeatable depth control.'
  },
  firewall: {
    kicker: 'analogy firewall',
    title: 'The math tool is spectral analysis — not quantum basketball.',
    body: 'The Heisenberg-style product does not create a lower bound here; the simulator explicitly tests and breaks the analogy.',
    step: 'Demo: point to the curve near θ* and label the analogy as refuted.'
  },
  form: {
    kicker: 'form bridge',
    title: 'Physics sliders need a body translation layer.',
    body: 'The lab maps ground → shoulder → elbow → wrist → fingers and can overlay a CMU free-throw trace without claiming outcome labels.',
    step: 'Demo: compare ground-up chain, flat palm push, wrist flick, and CMU trace.'
  },
});

function updateTakeaway() {
  const copy = TAKEAWAYS[activeStation] || TAKEAWAYS.geometry;
  if (takeawayKicker) takeawayKicker.textContent = copy.kicker;
  if (takeawayTitle) takeawayTitle.textContent = copy.title;
  if (takeawayBody) takeawayBody.textContent = copy.body;
  if (takeawayStep) takeawayStep.textContent = copy.step;
}

function renderEquation(copy) {
  equationBox.classList.add('math-equation');
  equationBox.innerHTML = MATH_EQUATIONS[activeStation] || copy.eq;
}

// --- THREE SCENE -----------------------------------------------------------
const canvas3d = $('three');
// WebGL is optional: if the context can't be created (locked-down projector,
// headless browser, GPU disabled), keep the whole 2D lab — controls, plots,
// metrics — alive instead of blanking the page.
let renderer = null;
try {
  renderer = new THREE.WebGLRenderer({ canvas: canvas3d, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
} catch (err) {
  console.warn('[Shot Physics Lab] WebGL unavailable — running 2D-only.', err?.message || err);
  canvas3d.hidden = true;
  const note = document.createElement('div');
  note.textContent = '3D view needs WebGL — the 2D experiment panels still work.';
  note.style.cssText = 'position:fixed;top:50%;left:50%;transform:translate(-50%,-50%);z-index:1;color:#9fb0c2;font:13px Inter,sans-serif;text-align:center;max-width:60vw;pointer-events:none;';
  document.getElementById('app')?.appendChild(note);
}

const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x071019, 0.055);
const camera = new THREE.PerspectiveCamera(46, 1, 0.1, 100);
camera.position.set(4.3, 4.4, 5.2);
const orbit = new OrbitControls(camera, canvas3d);
orbit.enableDamping = true;
orbit.target.set(0, 2.35, -3.45);
orbit.maxPolarAngle = Math.PI * 0.49;
orbit.minDistance = 3.2;
orbit.maxDistance = 12;

scene.add(new THREE.HemisphereLight(0x9fdfff, 0x170b05, 1.9));
const key = new THREE.DirectionalLight(0xffc98e, 3.2);
key.position.set(3.2, 5.4, 2.2);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
scene.add(key);
const rimLight = new THREE.PointLight(0x60d8ff, 2.8, 9);
rimLight.position.set(-2.4, 2.6, -4.2);
scene.add(rimLight);

const world = new THREE.Group();
scene.add(world);

const materials = {
  floor: new THREE.MeshStandardMaterial({ color: 0x0d1c2c, roughness: 0.62, metalness: 0.05 }),
  line: new THREE.LineBasicMaterial({ color: 0x4b6680, transparent: true, opacity: 0.62 }),
  arc: new THREE.LineBasicMaterial({ color: 0xff9f43, linewidth: 2 }),
  ghostMake: new THREE.LineBasicMaterial({ color: 0x7cf7a1, transparent: true, opacity: 0.17 }),
  ghostMiss: new THREE.LineBasicMaterial({ color: 0xff6b6b, transparent: true, opacity: 0.13 }),
  window: new THREE.MeshBasicMaterial({ color: 0x60d8ff, transparent: true, opacity: 0.24, side: THREE.DoubleSide, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1 }),
  opening: new THREE.MeshBasicMaterial({ color: 0x60d8ff, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false, depthTest: false }),
  openingEdge: new THREE.MeshBasicMaterial({ color: 0x60d8ff, transparent: true, opacity: 0.82, side: THREE.DoubleSide, depthWrite: false, depthTest: false }),
  targetEdge: new THREE.MeshBasicMaterial({ color: 0x7cf7a1, transparent: true, opacity: 0.95, side: THREE.DoubleSide, depthWrite: false, depthTest: false }),
  rim: new THREE.MeshStandardMaterial({ color: 0xff6b2a, roughness: 0.35, metalness: 0.1, emissive: 0x351000, emissiveIntensity: 0.5 }),
  ball: new THREE.MeshStandardMaterial({ color: 0xf28a2e, roughness: 0.55, metalness: 0.02, emissive: 0x351000, emissiveIntensity: 0.15 }),
  make: new THREE.PointsMaterial({ size: 0.035, vertexColors: true, transparent: true, opacity: 0.9 }),
  landing: new THREE.PointsMaterial({ size: 0.078, vertexColors: true, transparent: true, opacity: 0.96, depthWrite: false }),
};

function makeCourt() {
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(6.5, 9.2), materials.floor);
  floor.rotation.x = -Math.PI / 2;
  floor.position.set(0, -0.015, -3.7);
  floor.receiveShadow = true;
  world.add(floor);

  const grid = new THREE.GridHelper(8, 16, 0x2b435b, 0x15283b);
  grid.position.set(0, 0.002, -3.7);
  world.add(grid);

  // Free-throw lane / centerline
  const lineGeo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0.012, 0.5), new THREE.Vector3(0, 0.012, -7.7),
    new THREE.Vector3(-1.8, 0.012, -4.2), new THREE.Vector3(1.8, 0.012, -4.2),
  ]);
  world.add(new THREE.LineSegments(lineGeo, materials.line));

  const releaseDisc = new THREE.Mesh(new THREE.CircleGeometry(0.18, 48), new THREE.MeshBasicMaterial({ color: 0xff9f43, transparent: true, opacity: 0.55 }));
  releaseDisc.rotation.x = -Math.PI / 2;
  releaseDisc.position.set(0, 0.018, 0);
  world.add(releaseDisc);
}
makeCourt();

const hoopGroup = new THREE.Group();
world.add(hoopGroup);
const backboard = new THREE.Mesh(new THREE.BoxGeometry(1.25, 0.76, 0.035), new THREE.MeshStandardMaterial({ color: 0xdce8f2, transparent: true, opacity: 0.10, roughness: 0.2 }));
backboard.castShadow = true;
hoopGroup.add(backboard);
const rim = new THREE.Mesh(new THREE.TorusGeometry(DEFAULT_COURT.rimDiameter / 2, 0.018, 12, 72), materials.rim);
rim.rotation.x = Math.PI / 2;
hoopGroup.add(rim);
const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.05, 3.1, 18), new THREE.MeshStandardMaterial({ color: 0x31465d, roughness: 0.6 }));
pole.position.set(0, 1.53, -0.58);
hoopGroup.add(pole);

let mainLine = null;
let ghostGroup = new THREE.Group();
world.add(ghostGroup);
let pointsObj = null;
let makeWindow = null;
let ball = new THREE.Mesh(new THREE.SphereGeometry(DEFAULT_COURT.ballDiameter / 2, 32, 18), materials.ball);
ball.castShadow = true;
const seamMat = new THREE.MeshBasicMaterial({ color: 0x3d1600, transparent: true, opacity: 0.58 });
for (const rot of [[0, 0, 0], [Math.PI / 2, 0, 0], [0, Math.PI / 2, 0]]) {
  const seam = new THREE.Mesh(new THREE.TorusGeometry(DEFAULT_COURT.ballDiameter / 2 * 1.01, 0.0035, 8, 72), seamMat);
  seam.rotation.set(rot[0], rot[1], rot[2]);
  ball.add(seam);
}
world.add(ball);

const avatarGroup = new THREE.Group();
avatarGroup.name = 'rigged-avatar-stage';
world.add(avatarGroup);
let avatarRoot = null;
let avatarBones = new Map();
let avatarRestRotations = new Map();
let pendingAvatarPose = null;
let avatarLoadState = 'idle';

function currentAvatarRates() {
  return {
    shoulder: Number(whipControls.shoulder?.value ?? 0),
    elbow: Number(whipControls.elbow?.value ?? 0),
    wrist: Number(whipControls.wrist?.value ?? 0),
    legs: Number(whipControls.legs?.value ?? 0),
    backspinRpm: Number(controls.backspin?.value ?? 0),
  };
}

function showAvatarForStation() {
  return activeStation === 'whip' || activeStation === 'form';
}

function collectAvatarBones(root) {
  const bones = new Map();
  const rest = new Map();
  root.traverse((node) => {
    if (node.isBone && node.name) {
      bones.set(node.name, node);
      rest.set(node.name, node.rotation.clone());
    }
  });
  avatarRestRotations = rest;
  return bones;
}

function styleAvatar(root) {
  root.traverse((node) => {
    if (node.isMesh || node.isSkinnedMesh) {
      node.castShadow = true;
      node.receiveShadow = true;
      if (node.material) {
        node.material = node.material.clone();
        node.material.roughness = Math.min(0.9, node.material.roughness ?? 0.58);
        node.material.metalness = Math.min(0.18, node.material.metalness ?? 0.02);
      }
    }
  });
}

function findAvatarBone(boneName) {
  for (const alias of mixamoBoneAliases(boneName)) {
    const bone = avatarBones.get(alias);
    if (bone) return bone;
  }
  return null;
}

function applyRiggedAvatarPose(pose) {
  pendingAvatarPose = pose;
  avatarGroup.visible = showAvatarForStation();
  if (!avatarRoot || !pose) return;
  avatarRoot.visible = avatarGroup.visible;
  avatarRoot.position.set(pose.root.position.x, pose.root.position.y, pose.root.position.z);
  avatarRoot.rotation.set(pose.root.rotation.x, pose.root.rotation.y, pose.root.rotation.z);
  avatarRoot.scale.setScalar(pose.root.scale);
  for (const [boneName, r] of Object.entries(pose.rotations)) {
    const bone = findAvatarBone(boneName);
    if (bone) {
      const rest = avatarRestRotations.get(bone.name);
      bone.rotation.set((rest?.x || 0) + r.x, (rest?.y || 0) + r.y, (rest?.z || 0) + r.z);
    }
  }
}

function updateRiggedAvatarPhase(phase = 'release') {
  if (!showAvatarForStation()) {
    avatarGroup.visible = false;
    return;
  }
  applyRiggedAvatarPose(avatarPhasePose(phase, currentAvatarRates()));
}

function initRiggedAvatar() {
  if (!renderer || avatarLoadState !== 'idle') return;
  avatarLoadState = 'loading';
  const loader = new GLTFLoader();
  loader.load(
    RIGGED_AVATAR_ASSET.url,
    (gltf) => {
      avatarRoot = gltf.scene;
      avatarRoot.name = RIGGED_AVATAR_ASSET.id;
      styleAvatar(avatarRoot);
      avatarBones = collectAvatarBones(avatarRoot);
      avatarGroup.add(avatarRoot);
      avatarLoadState = 'ready';
      const missing = Object.values(MIXAMO_BONE_MAP).filter((name) => !findAvatarBone(name));
      if (missing.length) console.warn('[Shot Physics Lab] Rigged avatar missing expected bones:', missing);
      console.info('[Shot Physics Lab] Rigged avatar loaded:', RIGGED_AVATAR_ASSET.label, `${avatarBones.size} bones`);
      applyRiggedAvatarPose(pendingAvatarPose || avatarPhasePose('release', currentAvatarRates()));
    },
    undefined,
    (err) => {
      avatarLoadState = 'error';
      avatarGroup.visible = false;
      console.warn('[Shot Physics Lab] Rigged avatar failed to load — 3D ball lab still works.', err?.message || err);
    },
  );
}
initRiggedAvatar();

function disposeObj(obj) {
  if (!obj) return;
  obj.traverse?.((child) => {
    child.geometry?.dispose?.();
  });
}

function updateHoop(court) {
  hoopGroup.position.set(0, court.H, -court.d);
  backboard.position.set(0, 0.32, -0.32);
  rim.position.set(0, 0, 0);
  pole.position.set(0, -1.52, -0.58);
  if (makeWindow) { makeWindow.parent?.remove(makeWindow); disposeObj(makeWindow); }
  const target = rimTargetRadii(court);
  makeWindow = new THREE.Group();
  // Effective make target for the BALL CENTER, not the full orange metal rim.
  // The faint outer disk shows the physical rim opening; the bright inner ring is
  // the stricter ball-center target: r = rim radius − ball radius.
  const openingDisk = new THREE.Mesh(new THREE.CircleGeometry(court.rimDiameter / 2, 96), materials.opening);
  const openingRing = new THREE.Mesh(new THREE.TorusGeometry(court.rimDiameter / 2, 0.025, 8, 96), materials.openingEdge);
  const targetDisk = new THREE.Mesh(new THREE.CircleGeometry(target.lateral, 96), materials.window);
  const targetRing = new THREE.Mesh(new THREE.TorusGeometry(target.lateral, 0.030, 8, 96), materials.targetEdge);
  const depthScale = target.lateral > 1e-9 ? target.depth / target.lateral : 1;
  for (const obj of [openingDisk, targetDisk]) {
    obj.rotation.x = -Math.PI / 2;
    obj.scale.set(1, depthScale, 1);
  }
  for (const obj of [openingRing, targetRing]) {
    obj.rotation.x = Math.PI / 2;
    obj.scale.set(1, depthScale, 1);
  }
  openingDisk.renderOrder = 2;
  targetDisk.renderOrder = 3;
  openingRing.renderOrder = 7;
  targetRing.renderOrder = 8;
  makeWindow.add(openingDisk, openingRing, targetDisk, targetRing);
  makeWindow.position.set(0, 0.032, 0);
  hoopGroup.add(makeWindow);
}

function lineFromTrajectory(theta, speed, court, material, lateralVelocity = 0, lateralOffset = 0) {
  const cos = Math.cos(theta);
  const pts = trajectoryPoints(theta, speed, court, 86).map((p) => {
    const timeAtX = p.x / Math.max(1e-6, speed * cos);
    const lateral = lateralOffset + lateralVelocity * timeAtX;
    return new THREE.Vector3(lateral, p.y, -p.x);
  });
  return new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), material);
}

function updateScene(result) {
  const court = result.court;
  updateHoop(court);
  if (mainLine) { world.remove(mainLine); disposeObj(mainLine); }
  const vLat = result.vLat || 0;
  mainLine = lineFromTrajectory(result.theta, result.speed, court, materials.arc, vLat, 0);
  world.add(mainLine);
  latestTrajectory = trajectoryPoints(result.theta, result.speed, court, 120);

  world.remove(ghostGroup);
  disposeObj(ghostGroup);
  ghostGroup = new THREE.Group();
  world.add(ghostGroup);

  // Draw ghost arcs lightly. In Noise Lab, keep arcs secondary so the landing
  // dots become the hero instead of reading like a laser/beam.
  const ghostShots = result.shots.slice(0, activeStation === 'noise' ? 18 : 42);
  for (const s of ghostShots) {
    const lateral = activeStation === 'noise' ? s.lateral * 0.18 : 0.018 * s.zTheta;
    const mat = s.make ? materials.ghostMake : materials.ghostMiss;
    const line = lineFromTrajectory(s.theta, s.speed, court, mat, s.vLatShot ?? vLat, lateral);
    ghostGroup.add(line);
  }

  if (pointsObj) { world.remove(pointsObj); disposeObj(pointsObj); }
  const positions = [];
  const colors = [];
  const rimDots = activeStation === 'noise'
    ? simulatedLandingPointCloud(result, { limit: 1100, maxAbsDepth: 0.55, maxAbsLateral: 0.55 })
    : rimPlaneDotCloud(result, { limit: 900, radiusScale: 1.32 });
  for (const dot of rimDots) {
    positions.push(dot.lateral, court.H + 0.055, -court.d - dot.depth);
    const c = new THREE.Color((dot.make ?? dot.rim?.make) ? 0x7cf7a1 : 0xff6b6b);
    colors.push(c.r, c.g, c.b);
  }
  const cloud = new THREE.BufferGeometry();
  cloud.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  cloud.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  pointsObj = new THREE.Points(cloud, activeStation === 'noise' ? materials.landing : materials.make);
  pointsObj.renderOrder = 9;
  world.add(pointsObj);

  updateRiggedAvatarPhase('release');
}

// --- 2D LAB CANVAS ---------------------------------------------------------
function prep2D(canvas) {
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w: rect.width, h: rect.height };
}
function clear2D(ctx, w, h) {
  ctx.clearRect(0, 0, w, h);
  const grd = ctx.createLinearGradient(0, 0, w, h);
  grd.addColorStop(0, 'rgba(96,216,255,.05)');
  grd.addColorStop(1, 'rgba(255,159,67,.05)');
  ctx.fillStyle = grd;
  ctx.fillRect(0, 0, w, h);
}
function drawAxes(ctx, w, h, xLabel, yLabel) {
  ctx.strokeStyle = 'rgba(255,255,255,.18)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(36, h - 30); ctx.lineTo(w - 12, h - 30); ctx.moveTo(36, 12); ctx.lineTo(36, h - 30); ctx.stroke();
  ctx.fillStyle = 'rgba(245,239,228,.62)'; ctx.font = '11px JetBrains Mono, monospace';
  ctx.fillText(xLabel, w - 108, h - 9); ctx.save(); ctx.translate(12, 94); ctx.rotate(-Math.PI/2); ctx.fillText(yLabel, 0, 0); ctx.restore();
}
function renderMiniPlot() {
  const { ctx, w, h } = prep2D(labCanvas);
  clear2D(ctx, w, h);
  if (!lastResult) return;
  if (activeStation === 'geometry') drawGeometryPlot(ctx, w, h, lastResult);
  if (activeStation === 'whip') drawWhipPlot(ctx, w, h, lastWhip);
  if (activeStation === 'form') drawFormForcesPlot(ctx, w, h, lastForm);
  if (activeStation === 'force') drawForcePlot(ctx, w, h, lastLateral);
  if (activeStation === 'noise') drawNoisePlot(ctx, w, h, lastResult);
  if (activeStation === 'eigen') drawEigenPlot(ctx, w, h, lastResult, lastSensitivity);
}
function drawGeometryPlot(ctx, w, h, result) {
  drawAxes(ctx, w, h, 'θ [deg]', 'v [m/s]');
  const court = result.court;
  const thMin = 42, thMax = 62, vMin = 5.6, vMax = 9.1;
  const X = (th) => 36 + (th - thMin) / (thMax - thMin) * (w - 56);
  const Y = (v) => h - 30 - (v - vMin) / (vMax - vMin) * (h - 48);
  ctx.strokeStyle = '#60d8ff'; ctx.lineWidth = 2;
  ctx.beginPath();
  let started = false;
  for (let d = thMin; d <= thMax; d += 0.1) {
    const v = makeSpeed(rad(d), court);
    if (!Number.isFinite(v)) continue;
    if (!started) { ctx.moveTo(X(d), Y(v)); started = true; } else ctx.lineTo(X(d), Y(v));
  }
  ctx.stroke();
  const opt = brancazioOptimum(court);
  ctx.fillStyle = '#ff9f43'; ctx.beginPath(); ctx.arc(X(opt.thetaDeg), Y(opt.speed), 5, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#f5efe4'; ctx.font = '11px JetBrains Mono, monospace'; ctx.fillText('θ*', X(opt.thetaDeg) + 7, Y(opt.speed) - 7);
  ctx.fillStyle = '#7cf7a1'; ctx.beginPath(); ctx.arc(X(deg(result.theta)), Y(result.speed), 4, 0, Math.PI * 2); ctx.fill();
}
function drawWhipPlot(ctx, w, h, whip) {
  if (!whip) return;
  // ---- left half: the arm chain + the ball-velocity arrow it produces -------
  const baseX = 42, baseY = h - 38, leftW = w * 0.47;
  const scale = Math.min(leftW - 28, h - 64) / 0.82; // ~0.79 m reach
  const P = (pt) => [baseX + (-pt[0]) * scale, baseY - pt[1] * scale]; // negate x → up-right
  const pts = whip.points;
  ctx.lineCap = 'round'; ctx.lineWidth = 5;
  const segColors = ['#6f8aa6', '#ff9f43', '#ffd29a'];
  for (let i = 0; i < pts.length - 1; i++) {
    const [x1, y1] = P(pts[i]); const [x2, y2] = P(pts[i + 1]);
    ctx.strokeStyle = segColors[i] || '#ff9f43';
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  }
  const names = ['shoulder', 'elbow', 'wrist'];
  ctx.font = '10px JetBrains Mono, monospace';
  for (let i = 0; i < pts.length - 1; i++) {
    const [x, y] = P(pts[i]);
    ctx.fillStyle = '#9fb0c2'; ctx.beginPath(); ctx.arc(x, y, 3.2, 0, Math.PI * 2); ctx.fill();
    ctx.fillText(names[i], x + 5, y + (i === 0 ? 13 : -5));
  }
  const [bx, by] = P(pts[pts.length - 1]);
  ctx.fillStyle = '#f28a2e'; ctx.beginPath(); ctx.arc(bx, by, 5.5, 0, Math.PI * 2); ctx.fill();
  // ball-velocity arrow at the true launch angle/speed
  const a = whip.launchAngle;
  const arrLen = clamp(whip.speed * 9, 10, leftW * 0.72);
  const ax = bx + Math.cos(a) * arrLen, ay = by - Math.sin(a) * arrLen;
  ctx.strokeStyle = '#60d8ff'; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(ax, ay); ctx.stroke();
  const head = 6.5; ctx.fillStyle = '#60d8ff';
  ctx.beginPath(); ctx.moveTo(ax, ay);
  ctx.lineTo(ax - Math.cos(a - 0.4) * head, ay + Math.sin(a - 0.4) * head);
  ctx.lineTo(ax - Math.cos(a + 0.4) * head, ay + Math.sin(a + 0.4) * head);
  ctx.closePath(); ctx.fill();
  ctx.fillStyle = '#f5efe4'; ctx.font = '700 12px JetBrains Mono, monospace';
  ctx.fillText(`v=${whip.speed.toFixed(2)}`, 14, 20);
  ctx.fillText(`θ=${whip.launchAngleDeg.toFixed(1)}°`, 14, 36);

  // ---- right half: summation of speed (cumulative ball speed, proximal→distal)
  const gx = w * 0.56, gy = h - 34, bw = (w - gx - 14) / 3;
  const maxS = Math.max(whip.cumulativeSpeed[2], whip.speed, 8) * 1.1;
  const barH = (s) => (s / maxS) * (h - 62);
  const stages = ['+sh', '+el', '+wr'];
  const cols = ['#9ecae1', '#4292c6', '#ff9f43'];
  ctx.fillStyle = '#9fb0c2'; ctx.font = '10px JetBrains Mono, monospace';
  ctx.fillText('summation of speed', gx, 16);
  for (let i = 0; i < 3; i++) {
    const x = gx + i * bw + 3, bh = barH(whip.cumulativeSpeed[i]);
    ctx.fillStyle = cols[i]; ctx.fillRect(x, gy - bh, bw - 6, bh);
    ctx.fillStyle = '#f5efe4'; ctx.font = '700 10px JetBrains Mono, monospace';
    ctx.fillText(whip.cumulativeSpeed[i].toFixed(1), x + 1, gy - bh - 4);
    ctx.fillStyle = '#9fb0c2'; ctx.font = '10px JetBrains Mono, monospace';
    ctx.fillText(stages[i], x + 2, gy + 12);
  }
  // legs/trunk base stacks on top of the wrist bar → "lower body strictly adds"
  if (whip.baseSpeed > 0.01) {
    const x = gx + 2 * bw + 3;
    const top = barH(whip.speed), wrist = barH(whip.cumulativeSpeed[2]);
    ctx.fillStyle = 'rgba(124,247,161,.9)';
    ctx.fillRect(x, gy - top, bw - 6, Math.max(0, top - wrist));
    ctx.fillStyle = '#7cf7a1'; ctx.font = '700 10px JetBrains Mono, monospace';
    ctx.fillText(`+legs`, x + 1, gy - top - 4);
  }
}

function drawFormForcesPlot(ctx, w, h, profile) {
  if (!profile) return;
  const whip = profile.whip;
  const baseX = 44, baseY = h - 32, bodyW = w * 0.46;
  const scale = Math.min(bodyW - 24, h - 58) / 0.82;
  const P = (pt) => [baseX + (-pt[0]) * scale, baseY - pt[1] * scale];
  const pts = whip.points;

  // Ground → body → ball force path.
  ctx.lineCap = 'round';
  ctx.strokeStyle = 'rgba(124,247,161,.72)'; ctx.lineWidth = 4;
  ctx.beginPath(); ctx.moveTo(baseX - 10, baseY + 8); ctx.lineTo(baseX + 6, baseY - 38); ctx.stroke();
  ctx.fillStyle = '#7cf7a1'; ctx.font = '700 10px JetBrains Mono, monospace'; ctx.fillText('ground reaction', 12, h - 12);

  const segColors = ['#7cf7a1', '#ff9f43', '#ffd29a'];
  for (let i = 0; i < pts.length - 1; i++) {
    const [x1, y1] = P(pts[i]); const [x2, y2] = P(pts[i + 1]);
    ctx.strokeStyle = segColors[i] || '#ff9f43'; ctx.lineWidth = 5;
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    const share = [profile.jointShares.shoulder, profile.jointShares.elbow, profile.jointShares.wrist][i] || 0;
    ctx.fillStyle = 'rgba(245,239,228,.78)'; ctx.font = '10px JetBrains Mono, monospace';
    ctx.fillText(`${Math.round(share * 100)}%`, x2 + 5, y2 - 4);
  }

  const [bx, by] = P(pts[pts.length - 1]);
  ctx.fillStyle = '#f28a2e'; ctx.beginPath(); ctx.arc(bx, by, 8, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = '#3d1600'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(bx, by, 8, 0.4, Math.PI * 1.45); ctx.stroke();
  // backspin curved arrow
  ctx.strokeStyle = '#60d8ff'; ctx.lineWidth = 2.4;
  ctx.beginPath(); ctx.arc(bx, by, 17, -0.15, -Math.PI * 1.2, true); ctx.stroke();
  ctx.fillStyle = '#60d8ff';
  ctx.beginPath(); ctx.moveTo(bx - 14, by + 9); ctx.lineTo(bx - 5, by + 8); ctx.lineTo(bx - 10, by + 1); ctx.closePath(); ctx.fill();
  ctx.font = '700 10px JetBrains Mono, monospace'; ctx.fillText(`${Math.round(profile.backspinRpm)} rpm`, bx + 17, by + 4);

  // Visual/toy Magnus lift cue; explicitly not a trajectory force in this prototype.
  const liftLen = 18 + profile.liftCue * 210;
  ctx.strokeStyle = '#60d8ff'; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(bx + 34, by + 10); ctx.lineTo(bx + 34, by + 10 - liftLen); ctx.stroke();
  ctx.fillStyle = '#60d8ff';
  ctx.beginPath(); ctx.moveTo(bx + 34, by + 7 - liftLen); ctx.lineTo(bx + 29, by + 17 - liftLen); ctx.lineTo(bx + 39, by + 17 - liftLen); ctx.closePath(); ctx.fill();
  ctx.fillText('toy lift cue', bx + 42, by + 6 - liftLen);

  // Right-side panel: either real CMU trace curves or the generic force path list.
  const x0 = w * 0.55, y0 = 20;
  if (profile.trace?.samples?.length) {
    const trace = profile.trace;
    const gw = w - x0 - 16;
    const gh = h - 62;
    const gx = (t) => x0 + t * gw;
    const gy = (v) => y0 + gh - clamp(v, 0, 1) * gh;
    const channels = [
      ['legs', '#7cf7a1'], ['shoulder', '#ff9f43'], ['elbow', '#ffd29a'], ['wrist', '#60d8ff'],
    ];
    ctx.fillStyle = '#f5efe4'; ctx.font = '700 11px JetBrains Mono, monospace'; ctx.fillText('CMU 124_04 real trace', x0, 14);
    ctx.strokeStyle = 'rgba(255,255,255,.14)'; ctx.lineWidth = 1;
    ctx.strokeRect(x0, y0, gw, gh);
    const releaseX = gx((trace.releaseFrameEstimate || 0) / Math.max(1, trace.frames - 1));
    ctx.strokeStyle = 'rgba(245,239,228,.38)'; ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(releaseX, y0); ctx.lineTo(releaseX, y0 + gh); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(245,239,228,.72)'; ctx.font = '9px JetBrains Mono, monospace'; ctx.fillText('release est.', releaseX + 4, y0 + 10);
    channels.forEach(([key, color], ci) => {
      ctx.strokeStyle = color; ctx.lineWidth = 2;
      ctx.beginPath();
      trace.samples.forEach((s, i) => {
        const x = gx(s.t); const y = gy(s[key]);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();
      ctx.fillStyle = color; ctx.font = '9px JetBrains Mono, monospace'; ctx.fillText(key, x0 + 2 + ci * 42, h - 9);
    });
    ctx.fillStyle = '#9fb0c2'; ctx.font = '9px Inter, sans-serif';
    ctx.fillText(`${trace.frames} frames · joint angles, not outcome labels`, x0, y0 + gh + 13);
    return;
  }

  // Right-side force path list: each joint gets a bar + form instruction.
  const rowH = Math.max(22, (h - 42) / profile.forcePath.length);
  ctx.fillStyle = '#f5efe4'; ctx.font = '700 11px JetBrains Mono, monospace'; ctx.fillText('force path → form change', x0, 14);
  profile.forcePath.forEach((step, i) => {
    const y = y0 + i * rowH;
    const barW = clamp(step.share, 0.02, 1) * (w - x0 - 24);
    ctx.fillStyle = step.joint === 'fingers' ? 'rgba(96,216,255,.32)' : step.joint === 'ground' ? 'rgba(124,247,161,.32)' : 'rgba(255,159,67,.28)';
    ctx.fillRect(x0, y, barW, 8);
    ctx.fillStyle = '#d7e1ed'; ctx.font = '700 10px JetBrains Mono, monospace'; ctx.fillText(step.label, x0, y + 20);
    ctx.fillStyle = '#9fb0c2'; ctx.font = '10px Inter, sans-serif'; ctx.fillText(step.form.slice(0, 42), x0 + 84, y + 20);
  });
}

function drawForcePlot(ctx, w, h, lateral) {
  if (!lateral) return;
  const cx = w * 0.5;
  const cy = h * 0.52;
  const radii = rimTargetRadii(lateral.court);
  const maxM = Math.max(radii.lateral * 1.9, radii.depth * 1.9, Math.abs(lateral.lateralAtRim) * 1.25, 0.18);
  const scale = Math.min(w * 0.38, h * 0.36, 145) / maxM;
  const X = (x) => cx + x * scale;
  const Y = (d) => cy + d * scale;

  ctx.strokeStyle = 'rgba(255,255,255,.22)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(X(-maxM), cy); ctx.lineTo(X(maxM), cy); ctx.moveTo(cx, Y(-maxM)); ctx.lineTo(cx, Y(maxM)); ctx.stroke();
  ctx.fillStyle = 'rgba(96,216,255,.12)';
  ctx.beginPath(); ctx.ellipse(cx, cy, radii.lateral * scale, radii.depth * scale, 0, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = '#60d8ff'; ctx.lineWidth = 2.5; ctx.setLineDash([5, 5]);
  ctx.beginPath(); ctx.ellipse(cx, cy, radii.lateral * scale, radii.depth * scale, 0, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);

  const depth = lateral.rim?.depth ?? 0;
  const missColor = lateral.makeByLateral ? '#7cf7a1' : '#ff6b6b';
  ctx.strokeStyle = '#ff9f43'; ctx.lineWidth = 4;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(X(lateral.lateralAtRim), Y(depth)); ctx.stroke();
  ctx.fillStyle = missColor; ctx.beginPath(); ctx.arc(X(lateral.lateralAtRim), Y(depth), 8, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#f5efe4'; ctx.font = '12px JetBrains Mono, monospace';
  ctx.fillText(`lateral = ${(lateral.lateralAtRim * 100).toFixed(1)} cm`, 24, 28);
  ctx.fillText(`target r = ${(radii.lateral * 100).toFixed(1)} cm`, 24, 46);
  ctx.fillStyle = '#9fb0c2';
  ctx.fillText(`critical v_lat ≈ ${lateral.lateralCriticalVelocity.toFixed(3)} m/s`, 24, h - 18);
  ctx.fillStyle = missColor;
  ctx.font = '700 18px Inter, sans-serif';
  ctx.fillText(lateral.makeByLateral ? 'INSIDE CIRCLE' : 'OUTSIDE CIRCLE', X(lateral.lateralAtRim) + 12, Y(depth) - 12);
}
function drawNoisePlot(ctx, w, h, result) {
  const cx = w * 0.5;
  const cy = h * 0.52;
  const radii = rimTargetRadii(result.court);
  const simDots = simulatedLandingPointCloud(result, { limit: 720, maxAbsDepth: 0.55, maxAbsLateral: 0.55 });
  const splDots = showSplLayer ? splMappedShots(result.court) : [];
  const simExtent = simDots.reduce((m, dot) => Math.max(m, Math.abs(dot.lateral), Math.abs(dot.depth)), 0);
  const splExtent = splDots.reduce((m, dot) => Math.max(m, Math.abs(dot.lateral), Math.abs(dot.depth)), 0);
  const maxM = Math.max(radii.lateral * 1.55, radii.depth * 1.55, Math.abs(result.meanLateral || 0) + radii.lateral, simExtent * 1.12, splExtent * 1.14, 0.18);
  const scale = Math.min(w * 0.39, h * 0.36, 128) / maxM;
  const X = (x) => cx + x * scale;
  const Y = (d) => cy + d * scale;

  ctx.strokeStyle = 'rgba(255,255,255,.20)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(X(-maxM), cy); ctx.lineTo(X(maxM), cy); ctx.moveTo(cx, Y(-maxM)); ctx.lineTo(cx, Y(maxM)); ctx.stroke();
  ctx.fillStyle = 'rgba(96,216,255,.11)';
  ctx.beginPath(); ctx.ellipse(cx, cy, radii.lateral * scale, radii.depth * scale, 0, 0, Math.PI * 2); ctx.fill();
  ctx.strokeStyle = '#60d8ff'; ctx.lineWidth = 2.2; ctx.setLineDash([5, 5]);
  ctx.beginPath(); ctx.ellipse(cx, cy, radii.lateral * scale, radii.depth * scale, 0, 0, Math.PI * 2); ctx.stroke(); ctx.setLineDash([]);

  for (const dot of simDots) {
    ctx.fillStyle = dot.make ? 'rgba(124,247,161,.70)' : 'rgba(255,107,107,.56)';
    ctx.beginPath(); ctx.arc(X(dot.lateral), Y(dot.depth), 1.9, 0, Math.PI * 2); ctx.fill();
  }
  if (showSplLayer) {
    for (const dot of splDots) {
      ctx.fillStyle = dot.actualMake ? 'rgba(124,247,161,.96)' : 'rgba(255,107,107,.92)';
      ctx.strokeStyle = dot.modelAgreesWithResult ? 'rgba(245,239,228,.82)' : '#ff9f43';
      ctx.lineWidth = dot.modelAgreesWithResult ? 1.2 : 2.2;
      ctx.beginPath(); ctx.arc(X(dot.lateral), Y(dot.depth), 4.6, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    }
  }
  ctx.fillStyle = '#f5efe4'; ctx.font = '700 11px JetBrains Mono, monospace';
  ctx.fillText('simulated landing-point cloud', 22, 22);
  ctx.fillStyle = '#9fb0c2'; ctx.font = '10px JetBrains Mono, monospace';
  ctx.fillText(`preset: ${currentNoisePreset().label} · σlat=${fmt(result.sigmaLateral, 3)} m/s`, 22, 38);
  if (showSplLayer) {
    ctx.fillText(`SPL real shots: ${SPL_FREE_THROW_SOURCE.session} ${SPL_FREE_THROW_SOURCE.participant}`, 22, 52);
    ctx.fillText('orange ring = real/model mismatch', 22, 66);
  }
  ctx.fillText('x: lateral miss', w - 112, h - 10);
  ctx.save(); ctx.translate(14, 100); ctx.rotate(-Math.PI / 2); ctx.fillText('depth miss', 0, 0); ctx.restore();
}
function drawEigenPlot(ctx, w, h, result, sens) {
  const cx = w * 0.51, cy = h * 0.52;
  const scale = Math.min(w, h) * 0.27;
  ctx.strokeStyle = 'rgba(255,255,255,.18)'; ctx.beginPath(); ctx.moveTo(18, cy); ctx.lineTo(w - 18, cy); ctx.moveTo(cx, 15); ctx.lineTo(cx, h - 24); ctx.stroke();
  // Make slab: |z| <= 1 roughly translates to speed-error slab at optimum.
  ctx.fillStyle = 'rgba(124,247,161,.12)'; ctx.fillRect(18, cy - scale, w - 36, scale * 2);
  ctx.strokeStyle = 'rgba(124,247,161,.8)'; ctx.setLineDash([5, 5]);
  ctx.beginPath(); ctx.moveTo(18, cy - scale); ctx.lineTo(w - 18, cy - scale); ctx.moveTo(18, cy + scale); ctx.lineTo(w - 18, cy + scale); ctx.stroke(); ctx.setLineDash([]);
  // Cloud in standardized release coordinates.
  for (const s of result.shots.slice(0, 420)) {
    ctx.fillStyle = s.make ? 'rgba(124,247,161,.42)' : 'rgba(255,107,107,.33)';
    ctx.beginPath(); ctx.arc(cx + s.zTheta * scale * 0.38, cy - s.zV * scale * 0.38, 1.4, 0, Math.PI * 2); ctx.fill();
  }
  const drawVec = (vx, vy, color, label) => {
    const x2 = cx + vx * scale * 1.1, y2 = cy - vy * scale * 1.1;
    const x1 = cx - vx * scale * 1.1, y1 = cy + vy * scale * 1.1;
    ctx.strokeStyle = color; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    ctx.fillStyle = color; ctx.font = '12px JetBrains Mono, monospace'; ctx.fillText(label, x2 + 6, y2 - 6);
  };
  drawVec(sens.nullAxis.angle, sens.nullAxis.speed, '#60d8ff', 'NULL/angle');
  drawVec(sens.stiffAxis.angle, sens.stiffAxis.speed, '#ff9f43', 'STIFF/speed');
  ctx.fillStyle = 'rgba(245,239,228,.7)'; ctx.font = '11px JetBrains Mono, monospace';
  ctx.fillText('uθ = δθ/σθ', w - 95, cy - 8);
  ctx.save(); ctx.translate(cx + 7, 26); ctx.rotate(-Math.PI/2); ctx.fillText('uv = δv/σv', 0, 0); ctx.restore();
}

function renderFirewallChart(result) {
  const curve = heisenbergCurve({ court: result.court });
  const opt = brancazioOptimum(result.court);
  const data = curve.map((p) => ({ x: p.thetaDeg, y: Math.min(2.4, p.U) }));
  if (firewallChart) firewallChart.destroy();
  firewallChart = new Chart(firewallCanvas, {
    type: 'line',
    data: {
      datasets: [
        { label: 'U(θ), clipped for display', data, borderColor: '#b59cff', backgroundColor: 'rgba(181,156,255,.16)', pointRadius: 0, borderWidth: 2.4, tension: 0.25, fill: true },
        { label: 'θ* optimum', data: [{ x: opt.thetaDeg, y: 2.35 }, { x: opt.thetaDeg, y: 0 }], borderColor: '#ff9f43', pointRadius: 0, borderWidth: 2, borderDash: [5, 5] },
      ],
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      parsing: false,
      scales: {
        x: { type: 'linear', min: 47, max: 57, ticks: { color: '#9fb0c2' }, grid: { color: 'rgba(255,255,255,.08)' }, title: { display: true, text: 'release angle θ [deg]', color: '#9fb0c2' } },
        y: { min: 0, max: 2.4, ticks: { color: '#9fb0c2' }, grid: { color: 'rgba(255,255,255,.08)' }, title: { display: true, text: 'tolerance product U', color: '#9fb0c2' } },
      },
      plugins: { legend: { labels: { color: '#d7e1ed', boxWidth: 10 } }, tooltip: { enabled: true } },
    },
  });
}

// --- STATE / UI ------------------------------------------------------------
function readConfig() {
  const h = Number(controls.height.value);
  const d = Number(controls.distance.value);
  const court = courtWith({ h, d });
  const noise = currentNoisePreset();
  return {
    court,
    theta: rad(Number(controls.angle.value)),
    speed: Number(controls.speed.value),
    vLat: Number(controls.lateral.value),
    sigmaV: Number(controls.sigmaV.value),
    sigmaTheta: rad(Number(controls.sigmaTheta.value)),
    sigmaLateral: activeStation === 'noise' ? noise.sigmaLateral : 0,
    backspinRpm: Number(controls.backspin.value),
  };
}
function syncLabels(cfg) {
  outs.angle.textContent = `${fmt(deg(cfg.theta), 1)}°`;
  outs.speed.textContent = `${fmt(cfg.speed, 2)} m/s`;
  outs.height.textContent = `${fmt(cfg.court.h, 2)} m`;
  outs.distance.textContent = `${fmt(cfg.court.d, 2)} m`;
  outs.lateral.textContent = `${fmt(cfg.vLat, 3)} m/s`;
  outs.sigmaV.textContent = `${fmt(cfg.sigmaV, 3)} m/s`;
  outs.sigmaTheta.textContent = `${fmt(deg(cfg.sigmaTheta), 2)}°`;
  outs.backspin.textContent = `${Math.round(cfg.backspinRpm)} rpm`;
}
function setMetricLabels(labels) {
  document.querySelectorAll('.metric .k').forEach((el, i) => { el.textContent = labels[i] || el.textContent; });
}
function splMappedShots(court = DEFAULT_COURT) {
  return SPL_FREE_THROW_SAMPLES.map((sample) => splShotToRimPlane(sample, court));
}
function splLayerStats(court = DEFAULT_COURT, result = null) {
  const mapped = splMappedShots(court);
  const made = mapped.filter((shot) => shot.actualMake).length;
  const agree = mapped.filter((shot) => shot.modelAgreesWithResult).length;
  const meanEntry = mapped.reduce((sum, shot) => sum + shot.entryAngleDeg, 0) / Math.max(1, mapped.length);
  const modelEntry = result ? deg(entryAngle(result.theta, result.speed, court)) : NaN;
  return { mapped, total: mapped.length, made, agree, meanEntry, modelEntry };
}
function updateMetrics(result, sens) {
  const opt = brancazioOptimum(result.court);
  if (activeStation === 'noise' && showSplLayer) {
    const spl = splLayerStats(result.court, result);
    setMetricLabels(['sim P(make)', 'SPL real makes', 'model agrees', 'SPL/model entry']);
    $('pMake').textContent = `${Math.round(result.makeProbability * 100)}%`;
    $('thetaOpt').textContent = `${spl.made}/${spl.total}`;
    $('dvDepth').textContent = `${spl.agree}/${spl.total}`;
    $('stiffAxis').textContent = `${fmt(spl.meanEntry, 1)}/${fmt(spl.modelEntry, 1)}°`;
    return;
  }
  if (activeStation === 'force' && lastLateral) {
    setMetricLabels(['rim result', 'radial offset', 'flight time', 'critical v_lat']);
    $('pMake').textContent = lastLateral.makeByLateral ? 'IN' : 'MISS';
    $('thetaOpt').textContent = `${fmt(lastLateral.rim.normalizedRadius, 2)} r`;
    $('dvDepth').textContent = `${fmt(lastLateral.flightTime, 2)} s`;
    $('stiffAxis').textContent = `${fmt(lastLateral.lateralCriticalVelocity, 3)} m/s`;
    return;
  }
  if (activeStation === 'whip' && lastWhip) {
    setMetricLabels(['release speed', 'launch angle', 'wrist adds', 'P(make)']);
    $('pMake').textContent = `${fmt(lastWhip.speed, 2)} m/s`;
    $('thetaOpt').textContent = `${fmt(lastWhip.launchAngleDeg, 1)}°`;
    const wristAdds = lastWhip.cumulativeSpeed[2] - lastWhip.cumulativeSpeed[1];
    $('dvDepth').textContent = `+${fmt(wristAdds, 1)} m/s`;
    $('stiffAxis').textContent = `${Math.round(result.gaussianMakeProbability * 100)}%`;
    return;
  }
  if (activeStation === 'form' && lastForm) {
    if (lastForm.trace) {
      setMetricLabels(['human trace', 'frames', 'release est.', 'source']);
      $('pMake').textContent = lastForm.trace.source.motion;
      $('thetaOpt').textContent = `${lastForm.trace.frames}`;
      $('dvDepth').textContent = `f${lastForm.trace.releaseFrameEstimate}`;
      $('stiffAxis').textContent = 'CMU';
      return;
    }
    setMetricLabels(['form cue', 'backspin', 'lift cue', 'ground share']);
    const cueTag = lastForm.primaryCue.includes('ground-up') ? 'GROUND' : lastForm.primaryCue.includes('fingers') ? 'FINGERS' : lastForm.primaryCue.includes('wrist') ? 'CHAIN' : 'FORM';
    $('pMake').textContent = cueTag;
    $('thetaOpt').textContent = `${Math.round(lastForm.backspinRpm)} rpm`;
    $('dvDepth').textContent = `${Math.round(lastForm.liftCue * 100)}%`;
    $('stiffAxis').textContent = `${Math.round(lastForm.jointShares.ground * 100)}%`;
    return;
  }
  setMetricLabels(['P(make)', 'θ* optimum', '∂depth/∂v', 'stiff axis']);
  $('pMake').textContent = `${Math.round(result.gaussianMakeProbability * 100)}%`;
  $('thetaOpt').textContent = `${fmt(opt.thetaDeg, 1)}°`;
  $('dvDepth').textContent = `${fmt(result.jacobian.depthV, 2)}`;
  $('stiffAxis').textContent = `${Math.round(sens.stiffAxis.speedFraction * 100)}% v`;
}
function updateForceLineTape() {
  if (!forceLineTape) return;
  if (activeStation !== 'force' || !lastForceLine) {
    forceLineTape.hidden = true;
    return;
  }
  const r = lastForceLine;
  const isMiss = r.status === 'MISS';
  const centered = r.direction === 'center';
  const direction = centered ? 'centered' : `${r.direction} at rim`;
  const marginCm = isMiss ? r.outsideByCm : r.insideByCm;
  const marginLabel = isMiss ? 'outside by' : 'inside by';
  const markerLeft = clamp(50 + r.normalizedToAllowance * 42, 6, 94);
  forceLineTape.hidden = false;
  forceLineTape.classList.toggle('miss', isMiss);
  forceLineTapeResult.textContent = isMiss ? 'MISS' : 'IN';
  forceLineTapeReadout.textContent = `${fmt(r.absDriftCm, 1)} cm ${direction} / ${fmt(r.allowanceCm, 1)} cm allowed`;
  forceLineTapeExplain.textContent = `${marginLabel} ${fmt(marginCm, 1)} cm. Δx = v_lat · t_flight; ${r.model}.`;
  if (forceLineMarker) forceLineMarker.style.left = `${markerLeft}%`;
}
function placeForceLineTape() {
  if (!forceLineTape) return;
  if (activeStation === 'force') {
    forceLineTape.classList.add('rail-force-line-tape');
    if (splRowsRail && forceLineTape.parentNode !== splRowsRail) {
      splRowsRail.insertBefore(forceLineTape, splRowsRailBefore || null);
    }
    return;
  }
  forceLineTape.classList.remove('rail-force-line-tape');
  if (forceLineTapeHome && forceLineTape.parentNode !== forceLineTapeHome) {
    if (forceLineTapeHomeNext?.parentNode === forceLineTapeHome) forceLineTapeHome.insertBefore(forceLineTape, forceLineTapeHomeNext);
    else forceLineTapeHome.appendChild(forceLineTape);
  }
}
function placeNoisePresetRows() {
  if (!noisePresetRows) return;
  if (activeStation === 'noise') {
    noisePresetRows.hidden = false;
    noisePresetRows.classList.add('rail-noise-presets');
    if (splRowsRail && noisePresetRows.parentNode !== splRowsRail) {
      splRowsRail.insertBefore(noisePresetRows, splRowsRailBefore || null);
    }
    document.querySelectorAll('.noise-preset').forEach((b) => b.classList.toggle('active', b.dataset.noisePreset === activeNoisePresetKey));
    return;
  }
  noisePresetRows.hidden = true;
  noisePresetRows.classList.remove('rail-noise-presets');
  if (noisePresetRowsHome && noisePresetRows.parentNode !== noisePresetRowsHome) {
    if (noisePresetRowsHomeNext?.parentNode === noisePresetRowsHome) noisePresetRowsHome.insertBefore(noisePresetRows, noisePresetRowsHomeNext);
    else noisePresetRowsHome.appendChild(noisePresetRows);
  }
}
function placeSplRows() {
  if (!splRows) return;
  if (activeStation === 'noise') {
    splRows.hidden = false;
    splRows.classList.add('rail-spl-layer');
    if (splRowsRail && splRows.parentNode !== splRowsRail) {
      splRowsRail.insertBefore(splRows, splRowsRailBefore || null);
    }
    return;
  }
  splRows.hidden = true;
  splRows.classList.remove('rail-spl-layer');
  if (splRowsHome && splRows.parentNode !== splRowsHome) {
    if (splRowsHomeNext?.parentNode === splRowsHome) splRowsHome.insertBefore(splRows, splRowsHomeNext);
    else splRowsHome.appendChild(splRows);
  }
}
function compactVerdictHeadline(verdict) {
  if (!verdict) return '—';
  if (!verdict.reaches) return 'never reaches rim plane';
  if (verdict.make) return `${fmt(verdict.centerDistanceCm, 1)} cm off-center · ${fmt(verdict.marginCm, 1)} cm inside`;
  return verdict.headline;
}
function compactVerdictHint(verdict) {
  if (!verdict) return '—';
  if (verdict.make) return verdict.reason === 'swish'
    ? 'clean center pass; speed + line inside tolerance.'
    : `${fmt(verdict.marginCm, 1)} cm clearance; contact is not modeled here.`;
  return verdict.hint;
}
function updateVerdict(result) {
  if (!shotVerdictEl || !result) return;
  // The verdict reads the deterministic hero shot — the current aimed/built
  // release the orange arc draws, which is the same point the noise cloud is
  // centered on — using the tested circular rimAcceptance core (no new physics).
  const verdict = classifyShot({ depth: result.meanDepth, lateral: result.meanLateral, court: result.court });
  shotVerdictEl.dataset.tone = verdict.tone;
  shotVerdictEl.dataset.reason = verdict.reason;
  verdictFlag.textContent = verdict.flag;
  const compact = window.matchMedia?.('(max-width: 680px)').matches;
  verdictHeadline.textContent = compact ? compactVerdictHeadline(verdict) : verdict.headline;
  if (activeStation === 'noise') {
    verdictEyebrow.textContent = 'mean release · cloud scatters around it';
    verdictHint.textContent = `${compact ? compactVerdictHint(verdict) : verdict.hint} P(make) below counts the scatter.`;
  } else {
    verdictEyebrow.textContent = 'this release';
    verdictHint.textContent = compact ? compactVerdictHint(verdict) : verdict.hint;
  }
}
function updateStationText() {
  const copy = stationCopy[activeStation];
  stationTitle.textContent = copy.title;
  stationNote.textContent = copy.note;
  updateTakeaway();
  renderEquation(copy);
  const legendRows = activeStation === 'noise' && showSplLayer
    ? [...copy.legend, ['#f5efe4', 'SPL real outcome'], ['#ff9f43', 'real/model mismatch']]
    : copy.legend;
  legend.innerHTML = legendRows.map(([c, t]) => `<span><i class="dot" style="background:${c}"></i>${t}</span>`).join('');
  labCanvas.hidden = activeStation === 'firewall';
  firewallCanvas.hidden = activeStation !== 'firewall';
  const chainMode = activeStation === 'whip' || activeStation === 'form';
  document.querySelector('.left')?.classList.toggle('form-active', activeStation === 'form');
  document.querySelector('.right')?.classList.toggle('trace-active', activeStation === 'form' && !!lastForm?.trace);
  aimRows.hidden = chainMode;
  whipRows.hidden = !chainMode;
  formModeRows.hidden = activeStation !== 'form';
  placeForceLineTape();
  placeNoisePresetRows();
  placeSplRows();
  updateForceLineTape();
  splLayerToggle?.classList.toggle('active', showSplLayer);
  formFeedback.hidden = activeStation !== 'form';
  if (activeStation === 'form' && lastForm) {
    const traceCopy = lastForm.trace
      ? ` Human trace: ${lastForm.trace.source.database} ${lastForm.trace.source.motion} (${lastForm.trace.frames} frames).`
      : '';
    formFeedbackText.textContent = `${lastForm.primaryCue}${traceCopy} The 3D human is a rigged avatar driven by these slider/posture cues; backspin is shown as a force cue only, and the orange projectile still uses the tested no-drag core.`;
  }
}
function readWhip() {
  return kineticChainRelease({
    omegaShoulder: Number(whipControls.shoulder.value),
    omegaElbow: Number(whipControls.elbow.value),
    omegaWrist: Number(whipControls.wrist.value),
    baseSpeed: Number(whipControls.legs.value),
  });
}
function readFormProfile(court = DEFAULT_COURT) {
  const activeMode = activeStation === 'form' && activeFormModeKey ? FORM_MODES[activeFormModeKey] : null;
  return formForceProfile({
    omegaShoulder: Number(whipControls.shoulder.value),
    omegaElbow: Number(whipControls.elbow.value),
    omegaWrist: Number(whipControls.wrist.value),
    baseSpeed: Number(whipControls.legs.value),
    backspinRpm: Number(controls.backspin.value),
    trace: activeMode?.trace || null,
    court,
  });
}
function syncWhipLabels() {
  whipOuts.shoulder.textContent = `${fmt(Number(whipControls.shoulder.value), 2)} rad/s`;
  whipOuts.elbow.textContent = `${fmt(Number(whipControls.elbow.value), 2)} rad/s`;
  whipOuts.wrist.textContent = `${fmt(Number(whipControls.wrist.value), 2)} rad/s`;
  whipOuts.legs.textContent = `${fmt(Number(whipControls.legs.value), 2)} m/s`;
}
function updateAll({ relaunch = false } = {}) {
  const cfg = readConfig();
  if (activeStation === 'whip' || activeStation === 'form') {
    lastForm = readFormProfile(cfg.court);
    lastWhip = lastForm.whip;
    // In the Kinetic Chain / Form labs you BUILD the release; θ and v are outputs
    // of the body chain, not aim sliders. Override cfg, then mirror clamped values
    // back into the aim sliders so switching to Geometry continues from the build.
    cfg.theta = lastWhip.launchAngle;
    cfg.speed = lastWhip.speed;
    controls.angle.value = fmt(clamp(deg(cfg.theta), 42, 62), 1);
    controls.speed.value = fmt(clamp(cfg.speed, 5.8, 8.8), 2);
  } else {
    lastForm = null;
  }
  syncLabels(cfg);
  syncWhipLabels();
  lastResult = simulateNoiseExperiment({ ...cfg, n: activeStation === 'noise' ? 1400 : 900, seed: 42 });
  lastLateral = lateralExperiment(cfg);
  lastForceLine = forceLineReadout(cfg);
  lastResult.vLat = cfg.vLat;
  lastResult.lateralAtRim = lastLateral.lateralAtRim;
  lastSensitivity = makeSensitivity(cfg);
  updateScene(lastResult);
  updateMetrics(lastResult, lastSensitivity);
  updateVerdict(lastResult);
  updateStationText();
  if (activeStation === 'firewall') renderFirewallChart(lastResult);
  else renderMiniPlot();
  if (relaunch) { launchStart = performance.now(); isLaunching = true; }
}

// --- GUIDED DEMO / PRESET STORY PATH ---------------------------------------
// Five one-click beats that drive the whole lab without touching a slider:
// build speed (whip) → wrist-alone contrast → break the force-line (drift) →
// add game pressure (noise) → show the optimizer (eigen). The physics numbers
// come from DEMO_PRESET_PHYSICS (pinned in lab_physics.test.mjs); the captions
// here are the narration. "next ▸" walks the story in order.
const demoCaption = $('demoCaption');
const demoBeat = $('demoBeat');
const firstGestureCue = $('firstGestureCue');
const PRESET_CAPTIONS = {
  swish: "1 · Build it from the ground up — legs → shoulder → elbow → wrist all feed one release. Result: a swish.",
  wrist: "2 · Now max the wrist alone. With no chain under it the ball makes barely 3 m/s and falls short — the snap isn't the shot.",
  drift: "3 · Aim is perfect, but the force-line points ~14 cm/s left. Nothing restores it, so the ball drifts out the side.",
  pressure: "4 · Same perfect shot, game-pressure jitter on speed and angle. Watch the make cloud bloom and P(make) fall by half.",
  eigen: "5 · Diagonalize the make-sensitivity. One stiff axis is left — release speed. That's the thing to train.",
};
const PRESET_TITLES = {
  swish: 'Full chain swish',
  wrist: 'Wrist alone',
  drift: 'Tiny left drift',
  pressure: 'Game pressure',
  eigen: 'Eigen sweet spot',
};
let demoIndex = -1;
let firstGestureDismissed = false;
let firstGestureTimer = null;

const FIRST_GESTURE_TARGETS = {
  shoulder: 'omShoulder',
  wrist: 'omWrist',
};

function firstGestureTarget() {
  return $(FIRST_GESTURE_TARGETS[COLD_OPEN_DEMO.firstGesture.control]);
}

function hideFirstGestureCue({ dismiss = false } = {}) {
  if (dismiss) firstGestureDismissed = true;
  if (firstGestureTimer) {
    window.clearTimeout(firstGestureTimer);
    firstGestureTimer = null;
  }
  firstGestureCue?.setAttribute('hidden', '');
  firstGestureTarget()?.classList.remove('first-gesture-target');
}

function showFirstGestureCue() {
  firstGestureTimer = null;
  if (firstGestureDismissed || activeStation !== DEMO_PRESET_PHYSICS[COLD_OPEN_DEMO.preset].station) return;
  const target = firstGestureTarget();
  if (!firstGestureCue || !target) return;
  firstGestureCue.textContent = COLD_OPEN_DEMO.firstGesture.label;
  target.classList.add('first-gesture-target');
  firstGestureCue.removeAttribute('hidden');
}

function scheduleFirstGestureCue() {
  hideFirstGestureCue();
  if (firstGestureDismissed) return;
  firstGestureTimer = window.setTimeout(showFirstGestureCue, COLD_OPEN_DEMO.firstGesture.revealAfterMs);
}

function dismissFirstGestureCue() {
  if (COLD_OPEN_DEMO.firstGesture.dismissOnFirstAction) hideFirstGestureCue({ dismiss: true });
}

function clearDemoHighlight() {
  for (const c of document.querySelectorAll('.chip.active')) c.classList.remove('active');
  if (demoBeat) demoBeat.textContent = 'Beat 0/5 · custom slider mode';
}

function applyPreset(key) {
  const phys = DEMO_PRESET_PHYSICS[key];
  const caption = PRESET_CAPTIONS[key];
  if (!phys || !caption) return;
  // 1. jump to the beat's station
  activeStation = phys.station;
  document.querySelectorAll('.station').forEach((b) => b.classList.toggle('active', b.dataset.station === phys.station));
  // 2. set the court + a clean slate, then the single variable the beat is about
  const court = courtWith(phys.court || {});
  controls.height.value = fmt(court.h, 2);
  controls.distance.value = fmt(court.d, 2);
  controls.lateral.value = fmt(phys.vLat ?? 0, 3);
  controls.sigmaV.value = fmt(phys.sigmaV ?? 0.045, 3);
  controls.sigmaTheta.value = fmt(phys.sigmaThetaDeg ?? 0.9, 2);
  controls.backspin.value = '190';
  if (phys.station === 'whip') {
    // θ/v are BUILT from the chain here; set the joint rates and let updateAll
    // mirror the resulting release back into the aim sliders.
    whipControls.shoulder.value = fmt(phys.whip.omegaShoulder, 2);
    whipControls.elbow.value = fmt(phys.whip.omegaElbow, 2);
    whipControls.wrist.value = fmt(phys.whip.omegaWrist, 1);
    whipControls.legs.value = fmt(phys.whip.baseSpeed, 2);
  } else {
    // aim presets ride the perfect Brancazio optimum, so the beat's variable is alone
    const opt = brancazioOptimum(court);
    controls.angle.value = fmt(clamp(opt.thetaDeg, 42, 62), 1);
    controls.speed.value = fmt(clamp(opt.speed, 5.8, 8.8), 2);
  }
  // 3. highlight the chip + narrate, then recompute & relaunch the shot
  demoIndex = DEMO_PRESET_ORDER.indexOf(key);
  document.querySelectorAll('.chip').forEach((c) => c.classList.toggle('active', c.dataset.preset === key));
  if (demoBeat) demoBeat.textContent = `Beat ${demoIndex + 1}/${DEMO_PRESET_ORDER.length} · ${PRESET_TITLES[key]}`;
  demoCaption.textContent = caption;
  updateAll({ relaunch: true });
}

function applyNoisePreset(key, { relaunch = false } = {}) {
  const preset = NOISE_PRESETS[key];
  if (!preset) return;
  activeNoisePresetKey = key;
  activeStation = 'noise';
  document.querySelectorAll('.station').forEach((b) => b.classList.toggle('active', b.dataset.station === 'noise'));
  document.querySelectorAll('.noise-preset').forEach((b) => b.classList.toggle('active', b.dataset.noisePreset === key));
  const court = courtWith({ h: 2.20, d: 4.20 });
  const opt = brancazioOptimum(court);
  controls.height.value = fmt(court.h, 2);
  controls.distance.value = fmt(court.d, 2);
  controls.angle.value = fmt(clamp(opt.thetaDeg, 42, 62), 1);
  controls.speed.value = fmt(clamp(opt.speed, 5.8, 8.8), 2);
  controls.lateral.value = fmt(preset.vLat, 3);
  controls.sigmaV.value = fmt(preset.sigmaV, 3);
  controls.sigmaTheta.value = fmt(preset.sigmaThetaDeg, 2);
  updateAll({ relaunch });
}

function playNextPreset() {
  const next = DEMO_PRESET_ORDER[(demoIndex + 1) % DEMO_PRESET_ORDER.length];
  applyPreset(next);
}

function clearFormModeHighlight() {
  activeFormModeKey = null;
  document.querySelectorAll('.form-mode.active').forEach((b) => b.classList.remove('active'));
}

function revealFormControls() {
  if (activeStation !== 'form') return;
  window.setTimeout(() => {
    const leftPanel = document.querySelector('.left');
    if (leftPanel && formModeRows) {
      leftPanel.scrollTo({ top: Math.max(0, formModeRows.offsetTop - 20), behavior: 'smooth' });
    }
  }, 0);
}

function applyFormMode(key) {
  const mode = FORM_MODES[key];
  if (!mode) return;
  dismissFirstGestureCue();
  clearDemoHighlight();
  activeFormModeKey = key;
  activeStation = 'form';
  document.querySelectorAll('.station').forEach((b) => b.classList.toggle('active', b.dataset.station === 'form'));
  document.querySelectorAll('.form-mode').forEach((b) => b.classList.toggle('active', b.dataset.formMode === key));
  const court = courtWith(mode.court || {});
  controls.height.value = fmt(court.h, 2);
  controls.distance.value = fmt(court.d, 2);
  controls.lateral.value = '0.000';
  controls.sigmaV.value = '0.045';
  controls.sigmaTheta.value = '0.90';
  controls.backspin.value = String(mode.backspinRpm);
  whipControls.shoulder.value = fmt(mode.whip.omegaShoulder, 2);
  whipControls.elbow.value = fmt(mode.whip.omegaElbow, 2);
  whipControls.wrist.value = fmt(mode.whip.omegaWrist, 1);
  whipControls.legs.value = fmt(mode.whip.baseSpeed, 2);
  updateAll({ relaunch: true });
  revealFormControls();
}

for (const chip of document.querySelectorAll('.chip')) {
  chip.addEventListener('click', () => {
    dismissFirstGestureCue();
    clearFormModeHighlight();
    applyPreset(chip.dataset.preset);
  });
}
for (const btn of document.querySelectorAll('.form-mode')) {
  btn.addEventListener('click', () => applyFormMode(btn.dataset.formMode));
}
for (const btn of document.querySelectorAll('.noise-preset')) {
  btn.addEventListener('click', () => {
    dismissFirstGestureCue();
    clearDemoHighlight();
    clearFormModeHighlight();
    applyNoisePreset(btn.dataset.noisePreset, { relaunch: false });
  });
}
presentationToggle?.addEventListener('click', () => {
  const active = document.body.classList.toggle('presentation-mode');
  presentationToggle.setAttribute('aria-pressed', String(active));
  presentationToggle.textContent = active ? 'Exit presentation' : 'Presentation mode';
  window.setTimeout(resize, 0);
});
controlsJump?.addEventListener('click', () => controlsPanel?.scrollIntoView({ behavior: 'smooth', block: 'start' }));
analysisJump?.addEventListener('click', () => analysisPanel?.scrollIntoView({ behavior: 'smooth', block: 'start' }));

$('demoNext').addEventListener('click', () => {
  dismissFirstGestureCue();
  clearFormModeHighlight();
  playNextPreset();
});
$('formJump').addEventListener('click', () => {
  dismissFirstGestureCue();
  clearDemoHighlight();
  clearFormModeHighlight();
  activeStation = 'form';
  document.querySelectorAll('.station').forEach((b) => b.classList.toggle('active', b.dataset.station === 'form'));
  updateAll({ relaunch: false });
  revealFormControls();
});
splLayerToggle?.addEventListener('click', () => {
  showSplLayer = !showSplLayer;
  updateAll({ relaunch: false });
});

for (const input of Object.values(controls)) {
  input.addEventListener('input', () => { dismissFirstGestureCue(); clearDemoHighlight(); clearFormModeHighlight(); updateAll({ relaunch: false }); });
}
for (const input of Object.values(whipControls)) {
  input.addEventListener('input', () => { dismissFirstGestureCue(); clearDemoHighlight(); clearFormModeHighlight(); updateAll({ relaunch: true }); });
}
$('launchBtn').addEventListener('click', () => { dismissFirstGestureCue(); updateAll({ relaunch: true }); });
$('resetBtn').addEventListener('click', () => {
  dismissFirstGestureCue();
  clearDemoHighlight();
  clearFormModeHighlight();
  controls.height.value = '2.20';
  controls.distance.value = '4.20';
  const opt = brancazioOptimum(courtWith({ h: 2.20, d: 4.20 }));
  controls.angle.value = fmt(opt.thetaDeg, 1);
  controls.speed.value = fmt(opt.speed, 2);
  controls.lateral.value = '0.000';
  controls.sigmaV.value = '0.045';
  controls.sigmaTheta.value = '0.90';
  controls.backspin.value = '190';
  updateAll({ relaunch: true });
});
for (const btn of document.querySelectorAll('.station')) {
  btn.addEventListener('click', () => {
    dismissFirstGestureCue();
    clearDemoHighlight();
    clearFormModeHighlight();
    activeStation = btn.dataset.station;
    document.querySelectorAll('.station').forEach((b) => b.classList.toggle('active', b === btn));
    if (activeStation === 'noise') applyNoisePreset(activeNoisePresetKey, { relaunch: false });
    else updateAll({ relaunch: false });
    revealFormControls();
  });
}

function resize() {
  const w = window.innerWidth, h = window.innerHeight;
  if (renderer) renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  if (activeStation === 'firewall' && lastResult) renderFirewallChart(lastResult);
  else renderMiniPlot();
}
window.addEventListener('resize', resize);

function animate(now) {
  requestAnimationFrame(animate);
  const frameNow = Number.isFinite(now) ? now : performance.now();
  orbit.update();
  const rawT = ((frameNow - launchStart) / 1900) % 1;
  const t = Number.isFinite(rawT) ? (rawT < 0 ? rawT + 1 : rawT) : 0;
  if (latestTrajectory.length) {
    const idx = Math.max(0, Math.min(latestTrajectory.length - 1, Math.floor(t * latestTrajectory.length)));
    const p = latestTrajectory[idx] || latestTrajectory[0];
    if (p && Number.isFinite(p.y) && Number.isFinite(p.x)) {
      const lateral = lastResult
        ? (lastResult.vLat || 0) * (p.x / Math.max(1e-6, lastResult.speed * Math.cos(lastResult.theta)))
        : 0;
      ball.position.set(lateral, p.y, -p.x);
      const rpm = activeStation === 'form' && lastForm ? lastForm.backspinRpm : Number(controls.backspin?.value || 120);
      const spinStep = 0.018 + clamp(rpm / 320, 0, 1) * 0.085;
      ball.rotation.x -= spinStep;
      ball.rotation.z += 0.012;
    }
  }
  if (showAvatarForStation()) {
    const avatarPhase = t < 0.34 ? 'load' : t < 0.64 ? 'release' : 'follow';
    updateRiggedAvatarPhase(avatarPhase);
  } else {
    avatarGroup.visible = false;
  }
  // Slow court breathing: gives the scene physical presence without distracting.
  rimLight.intensity = 2.4 + Math.sin(frameNow * 0.0016) * 0.45;
  if (renderer) renderer.render(scene, camera);
}

resize();
applyPreset(COLD_OPEN_DEMO.preset);
scheduleFirstGestureCue();
requestAnimationFrame(animate);

console.info('[Shot Physics Lab] loaded. Three.js + Chart.js + local lab_physics core.');
console.info('[Shot Physics Lab] coaching note:', coachingSentence(lastResult));
