use std::fs;

use thalos_models::urdf::parser::parse_robot;
use thalos_models::{Geometry, JointKind};

const FIXTURE_PATH: &str = "tests/fixtures/ur5.urdf";

#[test]
fn parse_ur5_successfully() {
    let source =
        fs::read_to_string(FIXTURE_PATH).expect("UR5 fixture file not found — run from crate root");

    let robot = parse_robot(&source).expect("UR5 should parse without errors");
    assert_eq!(robot.name, "ur5");
}

#[test]
fn ur5_link_count() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    // base_link, shoulder_link, upper_arm_link, forearm_link,
    // wrist_1_link, wrist_2_link, wrist_3_link, ee_link, base,
    // tool0, world
    assert_eq!(
        robot.links.len(),
        11,
        "expected 11 links (world → base_link → … → tool0 + auxiliary frames)"
    );
}

#[test]
fn ur5_joint_count() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    assert_eq!(
        robot.joints.len(),
        10,
        "expected 10 joints (6 revolute + 4 fixed: world, base_link-base, ee, tool0)"
    );
}

#[test]
fn ur5_root_link_is_world() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    assert_eq!(
        robot.root_link, "world",
        "world link is never a child of any joint → should be root"
    );
}

#[test]
fn ur5_revolute_joints_have_axis_and_limits() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    let revolute_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ];

    for name in &revolute_names {
        let joint = robot
            .joints
            .get(*name)
            .unwrap_or_else(|| panic!("missing joint {name}"));

        assert_eq!(joint.kind, JointKind::Revolute, "{name} should be revolute");
        assert!(joint.axis.is_some(), "{name} must have an axis");
        assert!(joint.limits.is_some(), "{name} must have limits");
    }
}

#[test]
fn ur5_fixed_joints_have_no_axis_or_limits() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    let fixed_names = [
        "world_joint",
        "base_link-base_fixed_joint",
        "ee_fixed_joint",
        "wrist_3_link-tool0_fixed_joint",
    ];

    for name in &fixed_names {
        let joint = robot
            .joints
            .get(*name)
            .unwrap_or_else(|| panic!("missing joint {name}"));

        assert_eq!(joint.kind, JointKind::Fixed, "{name} should be fixed");
        assert!(joint.axis.is_none(), "{name} should not have an axis");
        assert!(joint.limits.is_none(), "{name} should not have limits");
    }
}

#[test]
fn ur5_kinematic_chain_is_connected() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    // Verify that every child link referenced in joints exists
    for joint in robot.joints.values() {
        assert!(
            robot.links.contains_key(&joint.parent),
            "joint {} parent '{}' not found in links",
            joint.name,
            joint.parent
        );
        assert!(
            robot.links.contains_key(&joint.child),
            "joint {} child '{}' not found in links",
            joint.name,
            joint.child
        );
    }
}

#[test]
fn ur5_shoulder_pan_origin_has_z_offset() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    let joint = &robot.joints["shoulder_pan_joint"];
    // shoulder_pan origin: xyz="0 0 0.089159", rpy="0 0 0"
    assert!(
        (joint.origin.translation.z - 0.089159).abs() < 1e-6,
        "shoulder_pan z-offset should be 0.089159"
    );
}

#[test]
fn ur5_elbow_joint_origin() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    let joint = &robot.joints["elbow_joint"];
    // elbow origin: xyz="0 -0.1197 0.425"
    assert!((joint.origin.translation.x).abs() < 1e-6);
    assert!((joint.origin.translation.y - (-0.1197)).abs() < 1e-6);
    assert!((joint.origin.translation.z - 0.425).abs() < 1e-6);
}

#[test]
fn ur5_shoulder_lift_rotation() {
    use thalos_math::UnitQuaternion;

    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    let joint = &robot.joints["shoulder_lift_joint"];
    // shoulder_lift: rpy="0 1.57079632679 0" → 90° around Y
    let expected = UnitQuaternion::from_euler(0.0, 1.57079632679, 0.0);
    let actual = joint.origin.rotation;
    // Compare quaternion components
    let a = actual.inner();
    let e = expected.inner();
    assert!(
        (a.w - e.w).abs() < 1e-6
            && (a.x - e.x).abs() < 1e-6
            && (a.y - e.y).abs() < 1e-6
            && (a.z - e.z).abs() < 1e-6,
        "shoulder_lift rotation mismatch: got ({},{},{},{})",
        a.w,
        a.x,
        a.y,
        a.z
    );
}

#[test]
fn ur5_inertial_data_exists() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    // All structural links (not world, base, tool0, ee_link) should have inertial
    let structural_links = [
        "base_link",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    ];
    for name in &structural_links {
        let link = &robot.links[*name];
        assert!(link.inertial.is_some(), "{name} should have inertial data");
    }
}

#[test]
fn ur5_upper_arm_mass() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    let inertial = robot.links["upper_arm_link"].inertial.as_ref().unwrap();
    assert!(
        (inertial.mass - 8.393).abs() < 1e-6,
        "upper_arm mass should be 8.393, got {}",
        inertial.mass
    );
}

#[test]
fn ur5_visual_mesh_geometry() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    let link = &robot.links["shoulder_link"];
    assert_eq!(link.visual.len(), 1, "shoulder_link should have 1 visual");

    match &link.visual[0].geometry {
        Geometry::Mesh { filename, scale: _ } => {
            assert!(
                filename.contains("shoulder.dae"),
                "mesh filename should contain 'shoulder.dae', got '{filename}'"
            );
        }
        other => panic!("expected Mesh geometry, got {other:?}"),
    }
}
