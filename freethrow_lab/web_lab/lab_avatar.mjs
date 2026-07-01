// lab_avatar.mjs — rigged GLB/SkinnedMesh avatar control contract.
//
// This module is deliberately DOM-free and THREE-free so the slider→joint motion
// mapping can be tested in Node. The browser renderer consumes the returned bone
// rotation recipes and applies them to a loaded Mixamo-style GLB skeleton.

import { clamp } from './lab_physics.mjs';

export const RIGGED_AVATAR_ASSET = Object.freeze({
  id: 'threejs-soldier-mixamo-dev-rig',
  kind: 'rigged-human-placeholder',
  label: 'Rigged human placeholder — replaceable athlete avatar',
  // Directly reachable and parsed during research: 2.1 MB GLB, Mixamo bones,
  // skins, and Idle/Run/TPose/Walk animations. This is the current development
  // rig because it stays visually upright under the v1 direct bone-control map.
  // Final basketball player asset should be sourced with explicit project
  // licensing and can keep this Mixamo map.
  url: 'https://threejs.org/examples/models/gltf/Soldier.glb',
  sourceUrl: 'https://github.com/mrdoob/three.js/blob/dev/examples/models/gltf/Soldier.glb',
  licenseNote: 'three.js example repository asset; verify asset-specific rights before final distribution or replace with a licensed basketball avatar',
});

export const MIXAMO_BONE_MAP = Object.freeze({
  hips: 'mixamorig:Hips',
  spine: 'mixamorig:Spine',
  chest: 'mixamorig:Spine2',
  neck: 'mixamorig:Neck',
  head: 'mixamorig:Head',
  rightShoulder: 'mixamorig:RightShoulder',
  rightUpperArm: 'mixamorig:RightArm',
  rightForeArm: 'mixamorig:RightForeArm',
  rightHand: 'mixamorig:RightHand',
  rightThumb: 'mixamorig:RightHandThumb1',
  rightIndex: 'mixamorig:RightHandIndex1',
  rightMiddle: 'mixamorig:RightHandMiddle1',
  rightRing: 'mixamorig:RightHandRing1',
  rightPinky: 'mixamorig:RightHandPinky1',
  leftShoulder: 'mixamorig:LeftShoulder',
  leftUpperArm: 'mixamorig:LeftArm',
  leftForeArm: 'mixamorig:LeftForeArm',
  leftHand: 'mixamorig:LeftHand',
  rightUpLeg: 'mixamorig:RightUpLeg',
  rightLeg: 'mixamorig:RightLeg',
  rightFoot: 'mixamorig:RightFoot',
  leftUpLeg: 'mixamorig:LeftUpLeg',
  leftLeg: 'mixamorig:LeftLeg',
  leftFoot: 'mixamorig:LeftFoot',
});

export const AVATAR_RATE_MAX = Object.freeze({
  shoulder: 6,
  elbow: 9,
  wrist: 16,
  legs: 2,
  spin: 260,
});

export function mixamoBoneAliases(name) {
  const raw = String(name || '');
  if (!raw) return [];
  const hasPrefix = raw.startsWith('mixamorig:');
  const stripped = hasPrefix ? raw.slice('mixamorig:'.length) : raw;
  const prefixed = hasPrefix ? raw : `mixamorig:${stripped}`;
  const sanitized = `mixamorig_${stripped}`;
  const compact = `mixamorig${stripped}`;
  return [...new Set([raw, stripped, prefixed, sanitized, compact])];
}

const PHASE = Object.freeze({
  load: Object.freeze({ t: 0, wind: -1, rootDip: 1.0, name: 'load' }),
  release: Object.freeze({ t: 0.5, wind: 0, rootDip: 0.15, name: 'release' }),
  follow: Object.freeze({ t: 1, wind: 0.78, rootDip: -0.08, name: 'follow' }),
});

const rot = (x = 0, y = 0, z = 0) => Object.freeze({ x, y, z });

export function normalizeAvatarRates({
  shoulder = 0,
  elbow = 0,
  wrist = 0,
  legs = 0,
  backspinRpm = 0,
  omegaShoulder,
  omegaElbow,
  omegaWrist,
  baseSpeed,
} = {}) {
  const sh = omegaShoulder ?? shoulder;
  const el = omegaElbow ?? elbow;
  const wr = omegaWrist ?? wrist;
  const lg = baseSpeed ?? legs;
  return {
    shoulder: clamp(sh / AVATAR_RATE_MAX.shoulder, 0, 1),
    elbow: clamp(el / AVATAR_RATE_MAX.elbow, 0, 1),
    wrist: clamp(wr / AVATAR_RATE_MAX.wrist, 0, 1),
    legs: clamp(lg / AVATAR_RATE_MAX.legs, 0, 1),
    spin: clamp(backspinRpm / AVATAR_RATE_MAX.spin, 0, 1),
  };
}

export function avatarPhasePose(phaseName = 'release', rates = {}) {
  const phase = PHASE[phaseName] || PHASE.release;
  const n = normalizeAvatarRates(rates);
  const b = MIXAMO_BONE_MAP;
  const wind = phase.wind;
  const legLoad = n.legs * phase.rootDip;
  const shoulderTravel = n.shoulder * wind;
  const elbowTravel = n.elbow * wind;
  const wristTravel = n.wrist * wind;
  const fingerTravel = Math.max(n.spin, n.wrist * 0.65) * Math.max(0, wind + 0.2);

  // Rotations are local Euler deltas applied on top of the avatar's loaded rest
  // pose. They are an expressive shot-form controller, not inverse dynamics.
  // The values were chosen to make the Mixamo right arm visibly form a free-throw
  // release pocket, then follow through through wrist/fingers.
  const rotations = Object.freeze({
    [b.hips]: rot(0.02 + legLoad * 0.04, 0, 0),
    [b.spine]: rot(-0.03 - legLoad * 0.02, 0, 0.02),
    [b.chest]: rot(-0.05 - legLoad * 0.02, 0.02, 0.04),
    [b.neck]: rot(0.03, 0, 0),

    [b.rightShoulder]: rot(0.04, 0.04, -0.05 - n.shoulder * 0.08),
    [b.rightUpperArm]: rot(-0.12 - n.shoulder * 0.05, -0.18, -0.28 - shoulderTravel * 0.72),
    [b.rightForeArm]: rot(0.02, -0.04, -0.34 - elbowTravel * 0.58),
    [b.rightHand]: rot(-0.10 - wristTravel * 0.78, 0.02, 0.06 + n.spin * 0.10),
    [b.rightThumb]: rot(-0.08 - fingerTravel * 0.34, 0.04, 0.04),
    [b.rightIndex]: rot(-0.10 - fingerTravel * 0.62, 0.02, 0.01),
    [b.rightMiddle]: rot(-0.10 - fingerTravel * 0.68, 0.02, 0.01),
    [b.rightRing]: rot(-0.08 - fingerTravel * 0.44, 0.01, 0.01),
    [b.rightPinky]: rot(-0.06 - fingerTravel * 0.38, 0, 0),

    // Left arm balances the shot posture rather than controlling the ball.
    [b.leftShoulder]: rot(0.01, -0.03, 0.06),
    [b.leftUpperArm]: rot(-0.05, 0.16, 0.22),
    [b.leftForeArm]: rot(0.01, 0.04, 0.16),
    [b.leftHand]: rot(-0.02, 0, -0.03),

    // Keep the lower body mostly upright in v1. The legs slider still creates a
    // visible load dip, but not a ragdoll-level crouch that hides the avatar.
    [b.rightUpLeg]: rot(0.03 + legLoad * 0.26, 0.01, -0.02),
    [b.rightLeg]: rot(-0.03 - legLoad * 0.30, 0, 0.01),
    [b.rightFoot]: rot(0.03 + legLoad * 0.08, 0, 0),
    [b.leftUpLeg]: rot(0.03 + legLoad * 0.22, -0.01, 0.02),
    [b.leftLeg]: rot(-0.03 - legLoad * 0.26, 0, -0.01),
    [b.leftFoot]: rot(0.02 + legLoad * 0.06, 0, 0),
  });

  return Object.freeze({
    assetId: RIGGED_AVATAR_ASSET.id,
    label: `rigged avatar ${phase.name} pose · parametric form cue`,
    phase: phase.name,
    t: phase.t,
    provenance: 'parametric-avatar',
    normalizedRates: Object.freeze(n),
    root: Object.freeze({
      // The app positions the avatar behind the release disc and rotates it to
      // face the hoop. Root dip gives the legs slider a visible whole-body load,
      // but the base stays above the floor so the athlete reads as standing, not
      // collapsed into the court.
      position: Object.freeze({ x: 0.92, y: 0.24 - legLoad * 0.08, z: 0.18 }),
      rotation: Object.freeze({ x: 0, y: Math.PI, z: 0 }),
      scale: 1.24,
    }),
    rotations,
  });
}

export function avatarPoseSequence(rates = {}) {
  return ['load', 'release', 'follow'].map((phase) => avatarPhasePose(phase, rates));
}
