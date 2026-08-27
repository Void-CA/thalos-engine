use crate::models::single_revolute::SingleRevoluteSpec;
use crate::prelude::*;

// ─── ADR-0001 Z-up regression tests ──────────────────────────

#[test]
fn zero_config_ee_in_z_up() {
    // ADR-0001: Z is vertical. Single revolute spins around Z.
    // At q=0 with l=1: ee = (1, 0, 0) — arm in XY, Z=0.
    let robot = SingleRevoluteSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[0.0]);
    let t = result.ee_pose().unwrap().transform().translation;

    assert!(
        (t.x - 1.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "Single revolute Z-up regression: expected (1, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

// ─── Existing tests ──────────────────────────────────────────

fn setup() -> (SerialChain, ForwardKinematics) {
    let robot = SingleRevoluteSpec::ideal().build();
    let fk = ForwardKinematics::new(robot.clone());
    (robot, fk)
}

#[test]
fn zero_angle_returns_x_position() {
    let (robot, fk) = setup();

    let result = fk.evaluate(&[0.0]);

    // Obtener el frame del efector final
    let end_effector = robot.end_effector();
    let pose = result.pose(end_effector).unwrap();

    // Verificar reference es World
    assert_eq!(
        pose.reference_id(),
        FrameId::World,
        "Reference should be World"
    );

    // Verificar posición: (1, 0, 0) - solo el link en X
    let t = &pose.transform().translation;
    assert!(
        (t.x - 1.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "Position should be (1, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn pi_over_2_returns_y_position() {
    let (robot, fk) = setup();

    // q = π/2 = 90 grados
    let result = fk.evaluate(&[PI / 2.0]);

    let end_effector = robot.end_effector();
    let pose = result.pose(end_effector).unwrap();

    // Verificar posición: (0, 1, 0) - link rotado 90° en Z
    let t = &pose.transform().translation;
    assert!(
        t.x.abs() < EPS && (t.y - 1.0).abs() < EPS && t.z.abs() < EPS,
        "Position should be (0, 1, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn pi_returns_negative_x_position() {
    let (robot, fk) = setup();

    // q = π = 180 grados
    let result = fk.evaluate(&[PI]);

    let end_effector = robot.end_effector();
    let pose = result.pose(end_effector).unwrap();

    // Verificar posición: (-1, 0, 0) - link rotado 180° en Z
    let t = &pose.transform().translation;
    assert!(
        (t.x + 1.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "Position should be (-1, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn has_one_pose() {
    let (robot, fk) = setup();

    let result = fk.evaluate(&[0.0]);
    let frames: Vec<_> = result.frames().collect();

    assert_eq!(
        frames.len(),
        2,
        "Should have exactly two poses (world + child)"
    );
}

#[test]
fn pose_target_is_child_frame() {
    let (robot, fk) = setup();

    let result = fk.evaluate(&[0.0]);

    let end_effector = robot.end_effector();
    let pose = result.pose(end_effector).unwrap();

    assert_eq!(
        &pose.target_id(),
        end_effector,
        "Target frame should be the child frame"
    );
}

#[test]
fn pose_is_global() {
    let (robot, fk) = setup();

    let result = fk.evaluate(&[0.0]);

    let end_effector = robot.end_effector();
    let pose = result.pose(end_effector).unwrap();

    assert!(
        pose.is_global(),
        "Pose should be global (reference == World)"
    );
}
