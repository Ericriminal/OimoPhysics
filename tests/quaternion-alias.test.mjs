import assert from 'node:assert/strict';
import { test } from 'node:test';
import { oimo } from '../bin/js_modules/OimoPhysics.js';

const { Vec3, Mat3 } = oimo.common;
const { RigidBody, RigidBodyConfig, RigidBodyType, Shape, ShapeConfig } = oimo.dynamics.rigidbody;
const joints = oimo.dynamics.constraint.joint;
const solverTypes = oimo.dynamics.constraint.solver.ConstraintSolverType;

function fixture(type = RigidBodyType.DYNAMIC) {
  const world = new oimo.dynamics.World(oimo.collision.broadphase.BroadPhaseType.BVH, new Vec3());
  function body(bodyType) {
    const config = new RigidBodyConfig();
    config.type = bodyType;
    config.autoSleep = false;
    config.linearDamping = 0;
    config.angularDamping = 0;
    const rb = new RigidBody(config);
    const shapeConfig = new ShapeConfig();
    shapeConfig.geometry = new oimo.collision.geometry.BoxGeometry(new Vec3(0.4, 0.2, 0.3));
    shapeConfig.collisionMask = 0;
    const shape = new Shape(shapeConfig);
    shape.userData = { isTrigger: false };
    rb.addShape(shape);
    rb.userData = { velocityConstraint: new Vec3(1, 1, 1) };
    world.addRigidBody(rb);
    return rb;
  }
  return { world, body, rb: body(type) };
}

function assertRotation(actual, expected, tolerance = 1e-10) {
  const a = actual.toQuat();
  const b = expected.toQuat();
  const dot = Math.abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
  assert.ok(Math.abs(dot - 1) < tolerance, `rotation mismatch: quaternion dot=${dot}`);
}

for (const type of [RigidBodyType.DYNAMIC, RigidBodyType.KINEMATIC]) {
  for (const degrees of [0, 1, 120, 180]) {
    test(`velocity integration composes ${degrees} degrees without quaternion aliasing for type ${type}`, () => {
      const { world, rb } = fixture(type);
      const initial = new Mat3().fromEulerXyz(new Vec3(0.3, -0.5, 0.7));
      const axis = new Vec3(1, 2, -3).normalize();
      const angle = degrees * Math.PI / 180;
      rb.setRotation(initial);
      rb.setAngularVelocity(axis.scale(angle / 0.02));
      const expected = initial.prependRotation(angle, axis.x, axis.y, axis.z);
      world.step(0.02);
      assertRotation(rb.getRotation(), expected);
      assert.ok(rb.getPosition().length() < 1e-10, 'pure rotation must not translate an unconstrained body');
    });
  }
}

for (const degrees of [120, 180]) {
  test(`split impulse integration composes ${degrees} degrees without overwriting its source quaternion`, () => {
    const { rb } = fixture();
    const initial = new Mat3().fromEulerXyz(new Vec3(-0.4, 0.2, 0.8));
    const axis = new Vec3(-2, 1, 3).normalize();
    const angle = degrees * Math.PI / 180;
    rb.setRotation(initial);
    rb._angPseudoVelX = axis.x * angle;
    rb._angPseudoVelY = axis.y * angle;
    rb._angPseudoVelZ = axis.z * angle;
    rb._integratePseudoVelocity();
    assertRotation(rb.getRotation(), initial.prependRotation(angle, axis.x, axis.y, axis.z));
    assert.equal(rb._angPseudoVelX, 0);
    assert.equal(rb._angPseudoVelY, 0);
    assert.equal(rb._angPseudoVelZ, 0);
  });
}

for (const [name, solverType] of [['PGS', solverTypes.ITERATIVE], ['direct', solverTypes.DIRECT]]) {
  for (const degrees of [120, 180]) {
    test(`${name} NGS restores a locked joint after a ${degrees} degree rotation`, () => {
      const { world, body, rb } = fixture();
      const anchor = body(RigidBodyType.KINEMATIC);
      const config = new joints.GenericJointConfig();
      config.init(anchor, rb, new Vec3(), anchor.getRotation(), rb.getRotation());
      config.solverType = solverType;
      config.positionCorrectionAlgorithm = oimo.dynamics.constraint.PositionCorrectionAlgorithm.NGS;
      for (let i = 0; i < 3; i++) {
        config.translationalLimitMotors[i].setLimits(0, 0);
        config.rotationalLimitMotors[i].setLimits(0, 0);
      }
      const joint = new joints.GenericJoint(config);
      world.addJoint(joint);
      const target = new Mat3().prependRotation(degrees * Math.PI / 180, 0.6, 0, 0.8);
      anchor.setRotation(target);
      for (let i = 0; i < 200; i++) world.step(0.02);
      assertRotation(rb.getRotation(), target, 1e-7);
      assert.ok(joint.getAnchor1().sub(joint.getAnchor2()).length() < 1e-7);
      assert.ok(rb.getAngularVelocity().length() < 1e-7, 'NGS position correction must not inject velocity');
    });
  }
}
