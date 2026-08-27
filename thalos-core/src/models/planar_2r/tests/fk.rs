use crate::models::planar_2r::Planar2RSpec;
use crate::prelude::*;
use thalos_math::constants::*;
use thalos_math::*;

// ─── ADR-0001 Z-up regression tests ──────────────────────────

#[test]
fn zero_config_ee_in_z_up() {
    // ADR-0001: Z is vertical. Planar 2R operates in XY plane.
    // At q=[0,0] with l1=l2=1: ee = (2, 0, 0) — Z is 0, arm in XY.
    let robot = Planar2RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[0.0, 0.0]);
    let ee = result.ee_pose().unwrap();
    let t = &ee.transform().translation;

    assert!(
        (t.x - 2.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "Planar 2R Z-up regression: expected (2, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn first_joint_90_in_z_up() {
    // Rz(π/2) rotates +X to +Y: ee at (0, 2, 0), Z stays 0.
    let robot = Planar2RSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[PI / 2.0, 0.0]);
    let t = result.ee_pose().unwrap().transform().translation;

    assert!(
        t.x.abs() < EPS && (t.y - 2.0).abs() < EPS && t.z.abs() < EPS,
        "Planar 2R Rz(90°): expected (0, 2, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

// ─── Existing tests ──────────────────────────────────────────

#[test]
fn returns_two_poses() {
    let robot = Planar2RSpec::ideal().build();

    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[0.0, 0.0]);

    let frames: Vec<_> = result.frames().collect();

    assert_eq!(
        frames.len(),
        3,
        "Planar 2R should generate 3 poses, including world pose",
    );
}

#[test]
fn all_poses_are_global() {
    let robot = Planar2RSpec::ideal().build();

    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[0.0, 0.0]);

    for frame in result.frames() {
        let pose = result.pose(frame).unwrap();

        assert!(pose.is_global(), "All poses should be global");

        assert_eq!(
            pose.reference_id(),
            FrameId::World,
            "Reference frame should be World"
        );
    }
}

#[test]
fn zero_configuration_places_end_effector_at_2_0_0() {
    let robot = Planar2RSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[0.0, 0.0]);

    let pose = result.pose(&end_effector).unwrap();

    let t = &pose.transform().translation;

    assert!(
        (t.x - 2.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "End effector should be at (2, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn first_joint_90_deg_places_end_effector_at_0_2_0() {
    let robot = Planar2RSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[PI / 2.0, 0.0]);

    let pose = result.pose(&end_effector).unwrap();

    let t = &pose.transform().translation;

    assert!(
        t.x.abs() < EPS && (t.y - 2.0).abs() < EPS && t.z.abs() < EPS,
        "End effector should be at (0, 2, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn folded_configuration_places_end_effector_at_1_1_0() {
    let robot = Planar2RSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[PI / 2.0, -PI / 2.0]);

    let pose = result.pose(&end_effector).unwrap();

    let t = &pose.transform().translation;

    assert!(
        (t.x - 1.0).abs() < EPS && (t.y - 1.0).abs() < EPS && t.z.abs() < EPS,
        "End effector should be at (1, 1, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn first_link_pose_is_correct_at_zero_configuration() {
    let robot = Planar2RSpec::ideal().build();

    let first_link = robot.segments.first().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[0.0, 0.0]);

    let pose = result.pose(&first_link).unwrap();

    let t = &pose.transform().translation;

    assert!(
        (t.x - 1.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "First link should be at (1, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn second_joint_rotates_relative_to_first_joint() {
    let robot = Planar2RSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    // q1 = 0
    // q2 = π/2
    //
    // link1 -> (1,0)
    // link2 rotates locally upward
    //
    // expected = (1,1)

    let result = fk.evaluate(&[0.0, PI / 2.0]);

    let pose = result.pose(&end_effector).unwrap();

    let t = &pose.transform().translation;

    assert!(
        (t.x - 1.0).abs() < EPS && (t.y - 1.0).abs() < EPS && t.z.abs() < EPS,
        "End effector should be at (1, 1, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}
