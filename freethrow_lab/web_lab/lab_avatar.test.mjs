import assert from 'node:assert/strict';
import test from 'node:test';

import {
  MIXAMO_BONE_MAP,
  RIGGED_AVATAR_ASSET,
  avatarPhasePose,
  avatarPoseSequence,
  mixamoBoneAliases,
  normalizeAvatarRates,
} from './lab_avatar.mjs';

const nearly = (a, b, eps = 1e-9) => Math.abs(a - b) <= eps;

test('rigged avatar asset is a real hosted GLB placeholder with clear licensing caveat', () => {
  assert.equal(RIGGED_AVATAR_ASSET.kind, 'rigged-human-placeholder');
  assert.match(RIGGED_AVATAR_ASSET.url, /^https:\/\/.+\.glb$/);
  assert.match(RIGGED_AVATAR_ASSET.label, /rigged/i);
  assert.match(RIGGED_AVATAR_ASSET.licenseNote, /verify/i);
});

test('mixamo bone map includes the joints needed for a basketball release', () => {
  for (const key of ['hips', 'spine', 'chest', 'neck', 'head', 'rightShoulder', 'rightUpperArm', 'rightForeArm', 'rightHand', 'rightIndex', 'rightMiddle', 'rightUpLeg', 'rightLeg', 'rightFoot']) {
    assert.equal(typeof MIXAMO_BONE_MAP[key], 'string', key);
    assert.match(MIXAMO_BONE_MAP[key], /mixamorig:/, key);
  }
});

test('mixamo bone aliases bridge prefixed, stripped, and GLTFLoader-sanitized names', () => {
  assert.deepEqual(mixamoBoneAliases('mixamorig:RightArm'), ['mixamorig:RightArm', 'RightArm', 'mixamorig_RightArm', 'mixamorigRightArm']);
  assert.deepEqual(mixamoBoneAliases('RightForeArm'), ['RightForeArm', 'mixamorig:RightForeArm', 'mixamorig_RightForeArm', 'mixamorigRightForeArm']);
});

test('avatar rate normalization follows the existing form slider ranges', () => {
  assert.deepEqual(normalizeAvatarRates({ shoulder: 0, elbow: 0, wrist: 0 }), { shoulder: 0, elbow: 0, wrist: 0, legs: 0, spin: 0 });
  assert.deepEqual(normalizeAvatarRates({ shoulder: 6, elbow: 9, wrist: 16, legs: 2, backspinRpm: 260 }), { shoulder: 1, elbow: 1, wrist: 1, legs: 1, spin: 1 });
  assert.deepEqual(normalizeAvatarRates({ shoulder: 3, elbow: 4.5, wrist: 8, legs: 1, backspinRpm: 130 }), { shoulder: 0.5, elbow: 0.5, wrist: 0.5, legs: 0.5, spin: 0.5 });
});

test('slider changes produce visibly different shoulder elbow wrist and finger rotations', () => {
  const low = avatarPhasePose('follow', { shoulder: 0.2, elbow: 0.2, wrist: 0.2, legs: 0, backspinRpm: 20 });
  const high = avatarPhasePose('follow', { shoulder: 6, elbow: 9, wrist: 16, legs: 0, backspinRpm: 260 });
  const b = MIXAMO_BONE_MAP;

  assert.ok(Math.abs(high.rotations[b.rightUpperArm].z - low.rotations[b.rightUpperArm].z) > 0.45, 'shoulder slider must move upper arm');
  assert.ok(Math.abs(high.rotations[b.rightForeArm].z - low.rotations[b.rightForeArm].z) > 0.35, 'elbow slider must move forearm');
  assert.ok(Math.abs(high.rotations[b.rightHand].x - low.rotations[b.rightHand].x) > 0.45, 'wrist slider must move hand');
  assert.ok(Math.abs(high.rotations[b.rightIndex].x - low.rotations[b.rightIndex].x) > 0.35, 'backspin must curl fingers');
});

test('legs slider drives lower-body load while keeping provenance explicit', () => {
  const low = avatarPhasePose('load', { shoulder: 2, elbow: 4, wrist: 7, legs: 0, backspinRpm: 190 });
  const high = avatarPhasePose('load', { shoulder: 2, elbow: 4, wrist: 7, legs: 2, backspinRpm: 190 });
  const b = MIXAMO_BONE_MAP;

  assert.equal(high.provenance, 'parametric-avatar');
  assert.equal(high.assetId, RIGGED_AVATAR_ASSET.id);
  assert.ok(Math.abs(high.rotations[b.rightUpLeg].x - low.rotations[b.rightUpLeg].x) > 0.20, 'leg slider must bend thigh at load');
  assert.ok(Math.abs(high.root.position.y - low.root.position.y) > 0.05, 'leg slider must visibly dip the avatar root at load');
});

test('load release follow sequence changes motion over time, not just static pose', () => {
  const seq = avatarPoseSequence({ shoulder: 4.5, elbow: 7, wrist: 12, legs: 1.2, backspinRpm: 220 });
  assert.deepEqual(seq.map((p) => p.phase), ['load', 'release', 'follow']);
  const b = MIXAMO_BONE_MAP;
  assert.ok(!nearly(seq[0].rotations[b.rightUpperArm].z, seq[1].rotations[b.rightUpperArm].z), 'load to release shoulder changes');
  assert.ok(!nearly(seq[1].rotations[b.rightHand].x, seq[2].rotations[b.rightHand].x), 'release to follow wrist changes');
  assert.ok(seq.every((p) => p.label.includes('rigged avatar')), 'all frames carry avatar label');
});
