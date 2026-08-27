use crate::models::planar_3r::Planar3RSpec;
use crate::prelude::*;
use thalos_math::constants::*;
use thalos_math::*;

// ─── ADR-0001 Z-up regression tests ──────────────────────────

#[test]
fn zero_config_ee_in_z_up() {
    // ADR-0001: Z is vertical. Planar 3R operates in XY plane.
    // At q=[0,0,0] with l1=l2=l3=1: ee = (3, 0, 0) — Z is 0.
    let robot = Planar3RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[0.0, 0.0, 0.0]);
    let t = result.ee_pose().unwrap().transform().translation;

    assert!(
        (t.x - 3.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "Planar 3R Z-up regression: expected (3, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn first_joint_90_in_z_up() {
    // Rz(π/2) rotates +X to +Y: ee at (0, 3, 0), Z stays 0.
    let robot = Planar3RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[PI / 2.0, 0.0, 0.0]);
    let t = result.ee_pose().unwrap().transform().translation;

    assert!(
        t.x.abs() < EPS && (t.y - 3.0).abs() < EPS && t.z.abs() < EPS,
        "Planar 3R Rz(90°): expected (0, 3, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

// ─── Existing tests ──────────────────────────────────────────

#[test]
fn returns_three_poses() {
    let robot = Planar3RSpec::ideal().build();

    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[0.0, 0.0, 0.0]);

    let frames: Vec<_> = result.frames().collect();

    assert_eq!(
        frames.len(),
        4,
        "Planar 3R should generate exactly three poses + world pose",
    );
}

#[test]
fn zero_configuration_places_end_effector_at_3_0_0() {
    let robot = Planar3RSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[0.0, 0.0, 0.0]);

    let pose = result.pose(&end_effector).unwrap();

    let t = &pose.transform().translation;

    assert!(
        (t.x - 3.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "End effector should be at (3, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn first_joint_90_deg_places_end_effector_at_0_3_0() {
    let robot = Planar3RSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[PI / 2.0, 0.0, 0.0]);

    let pose = result.pose(&end_effector).unwrap();

    let t = &pose.transform().translation;

    assert!(
        t.x.abs() < EPS && (t.y - 3.0).abs() < EPS && t.z.abs() < EPS,
        "End effector should be at (0, 3, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn folded_configuration_places_end_effector_at_2_1_0() {
    let robot = Planar3RSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    // q1 = π/2
    // q2 = -π/2
    // q3 = 0
    //
    // link1 -> (0,1)
    // link2 -> (1,1)
    // link3 -> (2,1)

    let result = fk.evaluate(&[PI / 2.0, -PI / 2.0, 0.0]);

    let pose = result.pose(&end_effector).unwrap();

    let t = &pose.transform().translation;

    assert!(
        (t.x - 2.0).abs() < EPS && (t.y - 1.0).abs() < EPS && t.z.abs() < EPS,
        "End effector should be at (2, 1, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn third_joint_rotates_relative_to_second_joint() {
    let robot = Planar3RSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    // q1 = 0
    // q2 = 0
    // q3 = π/2
    //
    // link1 -> (1,0)
    // link2 -> (2,0)
    // link3 rotates upward locally
    //
    // expected = (2,1)

    let result = fk.evaluate(&[0.0, 0.0, PI / 2.0]);

    let pose = result.pose(&end_effector).unwrap();

    let t = &pose.transform().translation;

    assert!(
        (t.x - 2.0).abs() < EPS && (t.y - 1.0).abs() < EPS && t.z.abs() < EPS,
        "End effector should be at (2, 1, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn all_joint_rotations_accumulate_correctly() {
    let robot = Planar3RSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    // q1 = π/2
    // q2 = π/2
    // q3 = 0
    //
    // link1 -> (0,1)
    // link2 -> (-1,1)
    // link3 -> (-2,1)

    let result = fk.evaluate(&[PI / 2.0, PI / 2.0, 0.0]);

    let pose = result.pose(&end_effector).unwrap();

    let t = &pose.transform().translation;

    assert!(
        (t.x + 2.0).abs() < EPS && (t.y - 1.0).abs() < EPS && t.z.abs() < EPS,
        "End effector should be at (-2, 1, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}
