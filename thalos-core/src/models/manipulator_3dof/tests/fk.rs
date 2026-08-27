use crate::models::manipulator_3dof::Manipulator3DOFSpec;
use crate::prelude::*;

fn ee_translation(fk: &ForwardKinematics, q: &[f64], ee: &FrameId) -> Vector3 {
    fk.evaluate(q)
        .pose(ee)
        .unwrap()
        .transform()
        .translation
        .clone()
}

fn build() -> (ForwardKinematics, FrameId) {
    let robot = Manipulator3DOFSpec::ideal().build();
    let ee = robot.end_effector().clone();
    let fk = ForwardKinematics::new(robot);
    (fk, ee)
}

// ─── ADR-0001 Z-up regression tests ──────────────────────────

#[test]
fn zero_config_ee_in_z_up() {
    // ADR-0001: Z is vertical. Link 1 translation → Z.
    // At q=[0,0,0]: ee = (l2+l3, 0, l1) = (2, 0, 1)
    let (fk, ee) = build();
    let t = ee_translation(&fk, &[0.0, 0.0, 0.0], &ee);

    assert!(
        (t.x - 2.0).abs() < EPS && t.y.abs() < EPS && (t.z - 1.0).abs() < EPS,
        "Manipulator 3DOF Z-up regression: expected (2, 0, 1), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

// Note: Non-zero config tests for joints 2/3 will be added in Phase 2
// after their Z-up axes are confirmed during migration.

#[test]
fn has_three_segments_and_three_joints() {
    let robot = Manipulator3DOFSpec::ideal().build();
    assert_eq!(
        robot.segments.len(),
        3,
        "Should have exactly three segments"
    );
    assert_eq!(robot.segments[0].joint.id(), 0);
    assert_eq!(robot.segments[1].joint.id(), 1);
    assert_eq!(robot.segments[2].joint.id(), 2);
}
