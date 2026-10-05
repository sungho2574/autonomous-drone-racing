import test from 'node:test';
import assert from 'node:assert/strict';
import {samplePose} from '../static/pose.mjs';
const near=(a,b)=>assert.ok(Math.abs(a-b)<1e-8, `${a} != ${b}`);
test('position interpolation and shortest-arc yaw interpolation',()=>{
  const p=samplePose([[0,0,0,1,1,0,0,0],[.2,2,4,3,0,0,0,1]],.1);
  assert.deepEqual(p.position,[1,2,2]);near(p.quaternion[2],Math.SQRT1_2);near(p.quaternion[3],Math.SQRT1_2);
});
test('no endpoint clamping or interpolation across missing data',()=>{
  const rows=[[0,0,0,1,1,0,0,0],[1,2,4,3,1,0,0,0]];
  assert.equal(samplePose(rows,-.1),null);assert.equal(samplePose(rows,1.1),null);assert.equal(samplePose(rows,.5),null);
  assert.equal(samplePose([],0),null);assert.equal(samplePose(rows,NaN),null);
  assert.deepEqual(samplePose(rows,1).position,[2,4,3]);
});
test('antipodal quaternions represent the same attitude',()=>{
  const p=samplePose([[0,0,0,0,1,0,0,0],[.2,0,0,0,-1,0,0,0]],.1);
  near(Math.abs(p.quaternion[3]),1);near(p.quaternion[2],0);
});
test('invalid quaternion hides axes without losing position',()=>{
  const p=samplePose([[0,2,3,1,0,0,0,0]],0);
  assert.deepEqual(p.position,[2,3,1]);assert.equal(p.quaternion,null);
});
