import assert from 'node:assert/strict';
import test from 'node:test';

import {
  DEFAULT_COURT,
  brancazioOptimum,
  depthAtRim,
  makeSpeed,
  simulateNoiseExperiment,
  makeSensitivity,
  heisenbergCurve,
  flightTimeToRim,
  lateralAtRim,
  lateralCriticalVelocity,
  lateralExperiment,
  forceLineReadout,
  rimPlaneDotCloud,
  simulatedLandingPointCloud,
  rimAcceptance,
  classifyShot,
  rimTargetRadii,
  kineticChainRelease,
  DEMO_PRESET_PHYSICS,
  DEMO_PRESET_ORDER,
  resolveDemoPreset,
  COLD_OPEN_DEMO,
  resolveColdOpenDemo,
  FORM_MODE_ORDER,
  FORM_MODES,
  CMU_124_FREE_THROW_TRACE,
  SPL_FREE_THROW_SAMPLES,
  SPL_FREE_THROW_SOURCE,
  formForceProfile,
  resolveFormMode,
  splShotToRimPlane,
} from './lab_physics.mjs';

const deg = (r) => r * 180 / Math.PI;
const rad = (d) => d * Math.PI / 180;

test('Brancazio optimum reproduces the free-throw headline number', () => {
  const { theta, speed } = brancazioOptimum(DEFAULT_COURT);
  assert.ok(Math.abs(deg(theta) - 50.7205) < 0.02);
  assert.ok(Math.abs(speed - 7.0976) < 0.01);
});

test('make speed places the descending branch through the rim center', () => {
  const theta = rad(50.7205);
  const v = makeSpeed(theta, DEFAULT_COURT);
  assert.ok(Math.abs(depthAtRim(theta, v, DEFAULT_COURT)) < 1e-4);
});

test('rim target uses a circular ball-center clearance, not a rectangular slab', () => {
  const court = DEFAULT_COURT;
  const radii = rimTargetRadii(court);
  assert.ok(Math.abs(radii.depth - court.w) < 1e-12);
  assert.ok(Math.abs(radii.lateral - court.w) < 1e-12);

  assert.equal(rimAcceptance({ depth: court.w, lateral: 0, court }).make, true, 'front/back edge is just inside');
  assert.equal(rimAcceptance({ depth: 0, lateral: -court.w, court }).make, true, 'left/right edge is just inside');
  assert.equal(rimAcceptance({ depth: 0.70 * court.w, lateral: 0.70 * court.w, court }).make, true, 'diagonal inside circle');
  assert.equal(rimAcceptance({ depth: 0.80 * court.w, lateral: 0.80 * court.w, court }).make, false, 'rectangular slab would pass this, circular hoop rejects it');
});

test('noise experiment classifies make/miss by the circular rim target when lateral drift is present', () => {
  const court = DEFAULT_COURT;
  const opt = brancazioOptimum(court);
  // Find a deterministic shot whose descending root is 0.8w long. Combine that
  // with 0.8w lateral drift: each component is inside the old rectangle, but the
  // ball center is outside the circular effective hoop.
  const targetDepth = 0.80 * court.w;
  let lo = opt.speed;
  let hi = opt.speed * 1.08;
  for (let i = 0; i < 48; i++) {
    const mid = (lo + hi) / 2;
    if (depthAtRim(opt.theta, mid, court) < targetDepth) lo = mid;
    else hi = mid;
  }
  const speed = (lo + hi) / 2;
  const vLat = (0.80 * court.w) / flightTimeToRim(opt.theta, speed, court);
  const r = simulateNoiseExperiment({ theta: opt.theta, speed, vLat, sigmaV: 0, sigmaTheta: 0, n: 8, seed: 4, court });
  assert.equal(r.shots.every((s) => s.make === false), true, 'every deterministic shot is outside the circular rim');
  assert.ok(Math.abs(r.shots[0].rim.normalizedRadius - Math.hypot(0.8, 0.8)) < 0.01, r.shots[0].rim.normalizedRadius);
  assert.equal(r.makeProbability, 0);
  assert.equal(r.gaussianMakeProbability, 0);
});

test('force-line lab: about 12 cm/s lateral velocity reaches the rim edge', () => {
  const { theta, speed } = brancazioOptimum(DEFAULT_COURT);
  const t = flightTimeToRim(theta, speed, DEFAULT_COURT);
  const vCrit = lateralCriticalVelocity(theta, speed, DEFAULT_COURT);
  assert.ok(t > 0.90 && t < 0.96, `flight time ${t}`);
  assert.ok(vCrit > 0.11 && vCrit < 0.13, `critical v_lat ${vCrit}`);
  assert.ok(Math.abs(lateralAtRim(vCrit, theta, speed, DEFAULT_COURT) - DEFAULT_COURT.w) < 1e-9);
  assert.ok(Math.abs(lateralAtRim(-vCrit, theta, speed, DEFAULT_COURT) + DEFAULT_COURT.w) < 1e-9);
});

test('force-line lab shrinks lateral allowance when the shot is already long or short', () => {
  const court = DEFAULT_COURT;
  const opt = brancazioOptimum(court);
  const targetDepth = 0.80 * court.w;
  let lo = opt.speed;
  let hi = opt.speed * 1.08;
  for (let i = 0; i < 48; i++) {
    const mid = (lo + hi) / 2;
    if (depthAtRim(opt.theta, mid, court) < targetDepth) lo = mid;
    else hi = mid;
  }
  const speed = (lo + hi) / 2;
  const time = flightTimeToRim(opt.theta, speed, court);
  const insideOldSlabButOutsideCircle = (0.70 * court.w) / time;
  const r = lateralExperiment({ theta: opt.theta, speed, vLat: insideOldSlabButOutsideCircle, court });
  assert.ok(Math.abs(r.depth - targetDepth) < 0.002, `depth ${r.depth}`);
  assert.ok(Math.abs(r.lateralAtRim) < court.w, 'old left-right-only slab would still pass this');
  assert.equal(r.makeByLateral, false, 'combined circular rim target rejects it');
  assert.ok(r.lateralCriticalVelocity < insideOldSlabButOutsideCircle, 'critical v_lat shrinks with depth error');
});

test('force-line readout translates tiny lateral velocity into honest rim centimeters', () => {
  const court = DEFAULT_COURT;
  const opt = brancazioOptimum(court);
  const cue = forceLineReadout({ theta: opt.theta, speed: opt.speed, vLat: -0.14, court });

  assert.equal(cue.direction, 'left');
  assert.equal(cue.status, 'MISS');
  assert.equal(cue.model, 'constant lateral velocity; no restoring force');
  assert.ok(cue.driftCm < -12 && cue.driftCm > -14, `drift ${cue.driftCm} cm`);
  assert.ok(cue.absDriftCm > cue.allowanceCm, `${cue.absDriftCm} should exceed ${cue.allowanceCm}`);
  assert.ok(cue.outsideByCm > 1.5 && cue.outsideByCm < 3.5, `outside ${cue.outsideByCm} cm`);
  assert.ok(cue.normalizedToAllowance < -1, `normalized ${cue.normalizedToAllowance}`);
});

test('noise lab: increasing speed noise lowers make probability at the optimum', () => {
  const low = simulateNoiseExperiment({ sigmaV: 0.045, sigmaTheta: rad(0.9), n: 4000, seed: 7 });
  const high = simulateNoiseExperiment({ sigmaV: 0.09, sigmaTheta: rad(0.9), n: 4000, seed: 7 });
  assert.ok(high.makeProbability < low.makeProbability - 0.08, `${high.makeProbability} !< ${low.makeProbability}`);
});

test('noise lab dot cloud spreads in a circular rim-plane range, not a line or rectangle', () => {
  const court = DEFAULT_COURT;
  const opt = brancazioOptimum(court);
  const result = simulateNoiseExperiment({ theta: opt.theta, speed: opt.speed, vLat: 0, sigmaV: 0.045, sigmaTheta: rad(0.9), n: 96, seed: 14, court });
  const dots = rimPlaneDotCloud(result, { limit: 96, radiusScale: 1.25 });
  assert.equal(dots.length, 96);
  assert.ok(dots.some((d) => Math.abs(d.lateral) > court.w * 0.25), 'visual cloud must not collapse to a lateral=0 line');
  assert.ok(dots.some((d) => Math.abs(d.depth) > court.w * 0.25), 'visual cloud must have depth spread');
  for (const dot of dots) {
    assert.ok(dot.scatterNormalizedRadius <= 1.25 + 1e-12, `dot outside circular scatter range: ${dot.scatterNormalizedRadius}`);
    assert.equal(dot.rim.make, dot.scatterNormalizedRadius <= 1 + 1e-12, 'green/red is tied to circular rim target');
  }
});

test('noise lab: lateral motor jitter creates real simulated landing-point spread', () => {
  const court = DEFAULT_COURT;
  const opt = brancazioOptimum(court);
  const still = simulateNoiseExperiment({ theta: opt.theta, speed: opt.speed, vLat: 0, sigmaV: 0.045, sigmaTheta: rad(0.9), sigmaLateral: 0, n: 160, seed: 19, court });
  const jitter = simulateNoiseExperiment({ theta: opt.theta, speed: opt.speed, vLat: 0, sigmaV: 0.045, sigmaTheta: rad(0.9), sigmaLateral: 0.035, n: 160, seed: 19, court });
  assert.ok(still.shots.every((s) => Math.abs(s.lateral) < 1e-12), 'baseline keeps lateral fixed when no force-line jitter is requested');
  assert.ok(jitter.shots.some((s) => Math.abs(s.vLatShot) > 0.025), 'shot records include sampled lateral velocity');
  assert.ok(jitter.shots.some((s) => Math.abs(s.lateral) > court.w * 0.25), 'real simulated landing dots spread left/right');
  assert.ok(Math.abs(jitter.meanLateral) < 0.015, `mean lateral should remain near centered, got ${jitter.meanLateral}`);
});

test('simulated landing-point cloud exposes actual shot outcomes for 3D rendering', () => {
  const court = DEFAULT_COURT;
  const opt = brancazioOptimum(court);
  const result = simulateNoiseExperiment({ theta: opt.theta, speed: opt.speed, vLat: 0, sigmaV: 0.09, sigmaTheta: rad(2.6), sigmaLateral: 0.045, n: 220, seed: 23, court });
  const cloud = simulatedLandingPointCloud(result, { limit: 180, maxAbsDepth: 0.42, maxAbsLateral: 0.42 });
  assert.equal(cloud.length, 180);
  assert.ok(cloud.some((d) => d.make), 'cloud should include makes');
  assert.ok(cloud.some((d) => !d.make), 'cloud should include misses');
  assert.ok(cloud.some((d) => Math.abs(d.lateral) > court.w * 0.3), 'cloud keeps real lateral spread');
  assert.ok(cloud.every((d) => Math.abs(d.depth) <= 0.42 && Math.abs(d.lateral) <= 0.42), 'cloud clamps only display extremes');
  assert.deepEqual(Object.keys(cloud[0]).sort(), ['depth', 'lateral', 'make', 'rim', 'source'].sort());
});

// ---- SHOT VERDICT ----------------------------------------------------------
// The plain-language readout (SWISH / SHORT / LONG / LEFT / RIGHT) must stay a
// thin, honest skin over the SAME circular rimAcceptance the lab already tests.

test('shot verdict: the optimum release reads as a centered SWISH make', () => {
  const court = DEFAULT_COURT;
  const opt = brancazioOptimum(court);
  const v = classifyShot({ depth: depthAtRim(opt.theta, opt.speed, court), lateral: 0, court });
  assert.equal(v.make, true);
  assert.equal(v.reason, 'swish');
  assert.equal(v.flag, 'SWISH');
  assert.equal(v.tone, 'make');
  assert.ok(v.marginCm > 9, `near-full clearance at the rim center, got ${v.marginCm}`);
});

test('shot verdict: a rim-grazing make is an amber edge MAKE, not a clean SWISH', () => {
  const court = DEFAULT_COURT;
  const w = court.w;
  const clean = classifyShot({ depth: 0.3 * w, lateral: 0, court });
  const edge = classifyShot({ depth: 0.95 * w, lateral: 0, court });
  assert.equal(clean.make, true);
  assert.equal(clean.reason, 'swish');
  assert.equal(clean.tone, 'make');
  assert.equal(edge.make, true, 'still inside the circular target');
  assert.equal(edge.reason, 'make');
  assert.equal(edge.flag, 'MAKE');
  assert.equal(edge.tone, 'edge');
  assert.ok(/rattle|clearance/i.test(edge.hint), edge.hint);
  assert.ok(edge.marginCm < clean.marginCm, 'edge make has less ball-to-rim clearance');
});

test('shot verdict: depth errors read as SHORT / LONG on the speed-sensitive axis', () => {
  const court = DEFAULT_COURT;
  const w = court.w;
  const short = classifyShot({ depth: -1.4 * w, lateral: 0, court });
  const long = classifyShot({ depth: 1.4 * w, lateral: 0, court });
  assert.equal(short.make, false);
  assert.equal(short.reason, 'short');
  assert.equal(short.flag, 'SHORT');
  assert.equal(long.reason, 'long');
  assert.ok(short.hint.toLowerCase().includes('speed'), short.hint);
  assert.ok(Math.abs(short.offsetCm - 1.4 * w * 100) < 0.2, `offset cm ${short.offsetCm}`);
});

test('shot verdict: force-line drift reads as LEFT / RIGHT, not a depth miss', () => {
  const court = DEFAULT_COURT;
  const w = court.w;
  const left = classifyShot({ depth: 0, lateral: -1.4 * w, court });
  const right = classifyShot({ depth: 0, lateral: 1.4 * w, court });
  assert.equal(left.reason, 'left');
  assert.equal(left.flag, 'LEFT');
  assert.equal(right.reason, 'right');
  assert.ok(left.hint.toLowerCase().includes('force-line'), left.hint);
});

test('shot verdict: the edge of the circular target stays a make, just past it misses', () => {
  const court = DEFAULT_COURT;
  const w = court.w;
  assert.equal(classifyShot({ depth: w, lateral: 0, court }).make, true, 'exact edge matches rimAcceptance');
  assert.equal(classifyShot({ depth: w * 1.02, lateral: 0, court }).make, false);
});

test('shot verdict: a release that never reaches the rim plane is reported, not NaN', () => {
  const v = classifyShot({ depth: NaN, lateral: 0, court: DEFAULT_COURT });
  assert.equal(v.reaches, false);
  assert.equal(v.make, false);
  assert.equal(v.reason, 'short');
  assert.ok(/reach/i.test(v.headline), v.headline);
});

test('shot verdict: matches the mean shot of a drifting noise experiment', () => {
  const court = DEFAULT_COURT;
  const opt = brancazioOptimum(court);
  const result = simulateNoiseExperiment({ theta: opt.theta, speed: opt.speed, vLat: 0.30, sigmaV: 0, sigmaTheta: 0, n: 4, seed: 3, court });
  const v = classifyShot({ depth: result.meanDepth, lateral: result.meanLateral, court });
  assert.equal(v.make, false);
  assert.equal(v.reason, 'right');
  assert.equal(v.tone, 'miss');
});

test('eigen lab: at the optimum the make-sensitivity stiff axis is release speed', () => {
  const { theta, speed } = brancazioOptimum(DEFAULT_COURT);
  const s = makeSensitivity({ theta, speed, sigmaV: 0.045, sigmaTheta: rad(0.9) });
  assert.ok(Math.abs(s.angleLeverage) < 1e-5);
  assert.ok(s.speedLeverage > 0.55 && s.speedLeverage < 0.65);
  assert.ok(s.stiffAxis.speedFraction > 0.999);
  assert.ok(s.nullAxis.angleFraction > 0.999);
});

test('firewall: tolerance product peaks near the Brancazio optimum, not a lower bound', () => {
  const curve = heisenbergCurve({ minDeg: 47, maxDeg: 57, steps: 181 });
  const maxPoint = curve.reduce((a, b) => (b.U > a.U ? b : a), curve[0]);
  assert.ok(Math.abs(maxPoint.thetaDeg - 50.72) < 0.15, `max at ${maxPoint.thetaDeg}`);
});

test('whip lab: default kinetic chain reproduces the Python toy-core release', () => {
  // Reference from 02-computation/scripts/kinematic_chain.py with the pose tuned
  // (solve_pose_for_launch) so the default whip launches up-forward at ~52 deg.
  const r = kineticChainRelease();
  assert.ok(Math.abs(r.speed - 7.46827) < 2e-3, `speed ${r.speed}`);
  assert.ok(Math.abs(r.launchAngleDeg - 52.0) < 0.05, `angle ${r.launchAngleDeg}`);
});

test('whip lab: summation of speed grows proximal->distal (distal inherits Omega)', () => {
  const r = kineticChainRelease();
  // Absolute segment omega is the cumulative sum of the relative joint rates.
  assert.deepEqual(r.omegaAbs.map((w) => Math.round(w * 10) / 10), [2.3, 6.6, 13.9]);
  // Cumulative ball speed climbs as each joint is added (the waterfall).
  assert.ok(r.cumulativeSpeed[0] < r.cumulativeSpeed[1]);
  assert.ok(r.cumulativeSpeed[1] < r.cumulativeSpeed[2]);
  assert.ok(Math.abs(r.cumulativeSpeed[0] - 1.8099) < 2e-3, `c0 ${r.cumulativeSpeed[0]}`);
  assert.ok(Math.abs(r.cumulativeSpeed[2] - 7.4683) < 2e-3, `c2 ${r.cumulativeSpeed[2]}`);
  // With no leg base, the final cumulative speed IS the arm-only release speed.
  assert.ok(Math.abs(r.cumulativeSpeed[2] - r.speed) < 1e-9);
});

test('whip lab: a wrist snap ALONE barely moves the ball without the chain below it', () => {
  const full = kineticChainRelease();
  const wristOnly = kineticChainRelease({ omegaShoulder: 0, omegaElbow: 0, omegaWrist: 7.3 });
  assert.ok(wristOnly.speed < 1.6, `wrist-only ${wristOnly.speed}`);
  assert.ok(wristOnly.speed < full.speed * 0.25, 'wrist alone << full whip');
});

test('whip lab: legs/trunk base velocity strictly adds release speed', () => {
  const armOnly = kineticChainRelease({ baseSpeed: 0 });
  const withLegs = kineticChainRelease({ baseSpeed: 1.375 });
  assert.ok(withLegs.speed > armOnly.speed + 1.0, `${withLegs.speed} vs ${armOnly.speed}`);
  assert.ok(Math.abs(withLegs.speed - 8.8433) < 1e-2, `with legs ${withLegs.speed}`);
});

test('noise lab: gaussian make shortcut accounts for mean depth offset, not just window width', () => {
  const court = DEFAULT_COURT;
  const whip = kineticChainRelease();
  const offRidge = simulateNoiseExperiment({
    theta: whip.launchAngle,
    speed: whip.speed,
    sigmaV: 0.045,
    sigmaTheta: rad(0.9),
    n: 8000,
    seed: 11,
    court,
  });
  assert.ok(Math.abs(offRidge.shots[0].depth) > court.w || Math.abs(offRidge.meanDepth) > court.w);
  assert.ok(offRidge.makeProbability < 0.02, `MC ${offRidge.makeProbability}`);
  assert.ok(offRidge.gaussianMakeProbability < 0.02, `Gaussian ${offRidge.gaussianMakeProbability}`);
});

test('SPL derived samples preserve compact real-shot source fields', () => {
  assert.equal(SPL_FREE_THROW_SOURCE.dataset, 'MLSE SPL Open Data');
  assert.equal(SPL_FREE_THROW_SOURCE.license, 'CC BY-NC-SA 4.0');
  assert.ok(SPL_FREE_THROW_SOURCE.rawPath.includes('basketball/freethrow/data/2024-08-28/P0001'));
  assert.ok(SPL_FREE_THROW_SAMPLES.length >= 12, 'derived layer keeps enough shots for a visible comparison');

  const first = SPL_FREE_THROW_SAMPLES[0];
  assert.deepEqual(Object.keys(first).sort(), [
    'entry_angle',
    'landing_x',
    'landing_y',
    'participant_id',
    'result',
    'sampling_rate',
    'source_file',
    'tracking_frames',
    'trial_date',
    'trial_id',
  ].sort());
  assert.equal(first.sampling_rate, 30);
  assert.equal(first.trial_date, '2024-08-28');
  assert.equal(first.participant_id, 'P0001');
  assert.equal(first.trial_id, 'T0001');
  assert.equal(first.result, 'missed');
  assert.equal(first.landing_x, 7.15);
  assert.equal(first.landing_y, 12.755);
  assert.equal(first.entry_angle, 40.9);
  assert.equal(first.tracking_frames, 240);
  assert.equal('tracking' in first, false, 'raw tracking frames must not be committed in the compact derived layer');
});

test('SPL landing inches map deterministically to centered rim-plane meters', () => {
  const mapped = splShotToRimPlane(SPL_FREE_THROW_SAMPLES[0], DEFAULT_COURT);
  const inch = 0.0254;
  assert.ok(Math.abs(mapped.lateral - 7.15 * inch) < 1e-12);
  assert.ok(Math.abs(mapped.depth - (12.755 - 9) * inch) < 1e-12);
  assert.equal(mapped.actualMake, false);
  assert.equal(mapped.modelMake, false);
  assert.equal(mapped.modelAgreesWithResult, true);
  assert.equal(mapped.comparison, 'real_miss_model_miss');
  assert.equal(mapped.entryAngleDeg, 40.9);
});

test('SPL make-miss comparison stays separate from the circular rim model', () => {
  const realMakeOutsideCircularModel = SPL_FREE_THROW_SAMPLES.find((s) => s.trial_id === 'T0021');
  const mapped = splShotToRimPlane(realMakeOutsideCircularModel, DEFAULT_COURT);
  assert.equal(mapped.actualMake, true);
  assert.equal(mapped.modelMake, false);
  assert.equal(mapped.modelAgreesWithResult, false);
  assert.equal(mapped.comparison, 'real_make_model_miss');
  assert.ok(mapped.rim.normalizedRadius > 1, `expected outside circular target, got ${mapped.rim.normalizedRadius}`);
});

// ---- GUIDED DEMO / PRESET STORY PATH --------------------------------------
// Pin every beat of the 30-second guided story so the live demo is un-embarrassable.

test('guided demo: the five beats resolve and each targets its declared station', () => {
  assert.deepEqual([...DEMO_PRESET_ORDER], ['swish', 'wrist', 'drift', 'pressure', 'eigen']);
  for (const key of DEMO_PRESET_ORDER) {
    const r = resolveDemoPreset(key);
    assert.equal(r.station, DEMO_PRESET_PHYSICS[key].station, `${key} station`);
  }
});

test('guided demo: "full chain swish" builds a release that actually swishes', () => {
  const r = resolveDemoPreset('swish');
  assert.equal(r.station, 'whip');
  assert.ok(r.makeByDepth, `swish should make, depth ${r.depth}`);
  assert.ok(Math.abs(r.depth) <= r.court.w, `depth ${r.depth} inside window ±${r.court.w}`);
  // It is the FULL chain: legs/trunk engaged AND a real wrist contribution on top.
  assert.ok(DEMO_PRESET_PHYSICS.swish.whip.baseSpeed > 0, 'legs/trunk engaged');
  assert.ok(r.whip.cumulativeSpeed[2] > r.whip.cumulativeSpeed[1], 'wrist adds onto the chain');
});

test('guided demo: "wrist alone" maxed out still falls short (no chain under it)', () => {
  const r = resolveDemoPreset('wrist');
  const swish = resolveDemoPreset('swish');
  assert.equal(DEMO_PRESET_PHYSICS.wrist.whip.omegaWrist, 16.0, 'wrist is pegged to its max');
  assert.equal(r.reaches, false, `wrist-alone should not even reach rim height, speed ${r.speed}`);
  assert.equal(r.makeByDepth, false);
  assert.ok(r.speed < swish.speed * 0.5, `wrist alone ${r.speed} << full chain ${swish.speed}`);
});

test('guided demo: "tiny left drift" misses on the force-line, with aim left perfect', () => {
  const r = resolveDemoPreset('drift');
  assert.equal(r.makeByLateral, false, 'left drift should miss left-right');
  assert.ok(r.vLat < 0, 'drifts left (negative lateral velocity)');
  assert.ok(Math.abs(r.lateralAtRim) > r.court.w, `|${r.lateralAtRim}| should exceed edge ${r.court.w}`);
  // The aim itself is the perfect optimum, so the miss is purely the force-line.
  const opt = brancazioOptimum(r.court);
  assert.ok(Math.abs(deg(r.theta) - opt.thetaDeg) < 1e-9, 'aim sits exactly on the optimum');
});

test('guided demo: "game pressure" roughly halves make probability vs the sweet spot', () => {
  const pressure = resolveDemoPreset('pressure');
  const sweet = resolveDemoPreset('eigen');
  assert.ok(sweet.gaussianMakeProbability > 0.88, `sweet spot ${sweet.gaussianMakeProbability}`);
  assert.ok(pressure.gaussianMakeProbability < 0.55, `pressure ${pressure.gaussianMakeProbability}`);
  assert.ok(pressure.gaussianMakeProbability < sweet.gaussianMakeProbability * 0.6, 'pressure ~halves P(make)');
});

test('guided demo: "eigen sweet spot" leaves release speed as the lone stiff axis', () => {
  const r = resolveDemoPreset('eigen');
  assert.equal(r.station, 'eigen');
  assert.ok(r.stiffSpeedFraction > 0.999, `stiff axis should be ~100% speed, got ${r.stiffSpeedFraction}`);
});

test('cold-open demo starts with the full-chain make and one visible chain cue', () => {
  const r = resolveColdOpenDemo();
  assert.equal(COLD_OPEN_DEMO.preset, 'swish');
  assert.equal(r.key, 'swish');
  assert.equal(r.station, 'whip');
  assert.equal(r.makeByDepth, true, `cold-open depth ${r.depth}`);
  assert.equal(r.firstGesture.control, 'shoulder');
  assert.equal(r.firstGesture.label, 'drag shoulder ↓');
  assert.equal(r.firstGesture.dismissOnFirstAction, true);
});

// ---- FORM / BACKSPIN / FORCE CHAIN -----------------------------------------
// These are not simulated people. They are form modes that map a visible slider
// state to hand/form feedback: ground reaction → joints → ball → backspin cue.

test('form modes resolve to concrete lab states without pretending to be players', () => {
  assert.deepEqual([...FORM_MODE_ORDER], ['chain', 'flat', 'flick', 'cmuFreeThrow']);
  for (const key of FORM_MODE_ORDER) {
    const r = resolveFormMode(key);
    assert.equal(r.station, 'form');
    assert.equal(r.key, key);
    assert.equal(r.label, FORM_MODES[key].label);
    assert.ok(r.profile.whip.speed > 0, `${key} has a release speed`);
    assert.ok(r.profile.forcePath.length >= 4, `${key} force path includes ground/joints/ball`);
  }
});

test('form mode: CMU free throw trace is grounded in downloaded Subject 124 mocap', () => {
  assert.equal(CMU_124_FREE_THROW_TRACE.source.motion, '124_04');
  assert.equal(CMU_124_FREE_THROW_TRACE.source.label, 'Basketball Free Throw');
  assert.equal(CMU_124_FREE_THROW_TRACE.frames, 726);
  assert.ok(CMU_124_FREE_THROW_TRACE.samples.length >= 48, 'trace keeps enough samples to draw curves');
  assert.ok(CMU_124_FREE_THROW_TRACE.samples.every((s) => s.t >= 0 && s.t <= 1), 'samples are normalized to clip time');
  const r = resolveFormMode('cmuFreeThrow');
  assert.equal(r.label, FORM_MODES.cmuFreeThrow.label);
  assert.equal(r.profile.trace.source.motion, '124_04');
  assert.ok(r.profile.primaryCue.includes('CMU'), r.profile.primaryCue);
  assert.ok(r.profile.forcePath.some((step) => step.joint === 'mocap'), 'force path names real mocap trace');
});

test('form mode: ground-up chain is a make and points to balanced force transfer', () => {
  const r = resolveFormMode('chain');
  assert.equal(r.makeByDepth, true, `chain depth ${r.depth}`);
  assert.ok(r.profile.backspinRpm >= 170, `backspin ${r.profile.backspinRpm}`);
  assert.ok(r.profile.primaryCue.includes('ground-up'), r.profile.primaryCue);
  assert.ok(r.profile.forcePath.some((step) => step.joint === 'ground'), 'ground reaction in force path');
  assert.ok(r.profile.forcePath.some((step) => step.joint === 'fingers'), 'finger/backspin finish in force path');
});

test('form mode: flat palm push tells the user to finish through the fingers/backspin', () => {
  const r = resolveFormMode('flat');
  assert.ok(r.profile.backspinRpm < 100, `flat spin ${r.profile.backspinRpm}`);
  assert.ok(r.profile.primaryCue.includes('fingers'), r.profile.primaryCue);
  assert.ok(r.profile.spinRatio < resolveFormMode('chain').profile.spinRatio, 'flat ball has less spin ratio than chain');
});

test('form mode: wrist flick warns that wrist speed without the chain is not enough', () => {
  const r = resolveFormMode('flick');
  assert.equal(r.makeByDepth, false);
  assert.ok(r.profile.primaryCue.includes('chain'), r.profile.primaryCue);
  assert.ok(r.profile.jointShares.wrist > r.profile.jointShares.shoulder, 'wrist dominates the weak release');
});

test('form force profile scales backspin lift cue with rpm but keeps it labeled visual/toy', () => {
  const low = formForceProfile({ backspinRpm: 60 });
  const high = formForceProfile({ backspinRpm: 240 });
  assert.ok(high.spinRatio > low.spinRatio * 3, `${high.spinRatio} vs ${low.spinRatio}`);
  assert.ok(high.liftCue > low.liftCue, `${high.liftCue} vs ${low.liftCue}`);
  assert.equal(high.liftModel, 'visual-toy, not injected into projectile trajectory');
});
