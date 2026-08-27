use std::fs;

use thalos_models::JointKind;
use thalos_models::urdf::parser::parse_robot;

const FIXTURE_PATH: &str = "tests/fixtures/scara.urdf";

#[test]
fn parse_scara_successfully() {
    let source = fs::read_to_string(FIXTURE_PATH)
        .expect("SCARA fixture file not found — run from crate root");

    let robot = parse_robot(&source).expect("SCARA should parse without errors");
    assert_eq!(robot.name, "scara");
}

#[test]
fn scara_link_count() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    // world, base_link, link_1, link_2, link_3, tool0
    assert_eq!(
        robot.links.len(),
        6,
        "expected 6 links (world → base_link → link_1 → link_2 → link_3 → tool0)"
    );
}

#[test]
fn scara_joint_count() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    assert_eq!(
        robot.joints.len(),
        5,
        "expected 5 joints (base_joint + 4 actuated)"
    );
}

#[test]
fn scara_root_link_is_world() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    assert_eq!(
        robot.root_link, "world",
        "world link is never a child of any joint → should be root"
    );
}

#[test]
fn scara_has_two_revolute_one_prismatic_one_continuous() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    let joints_by_name: std::collections::HashMap<&str, &thalos_models::Joint> =
        robot.joints.iter().map(|(n, j)| (n.as_str(), j)).collect();

    assert_eq!(joints_by_name["joint_1"].kind, JointKind::Revolute);
    assert_eq!(joints_by_name["joint_2"].kind, JointKind::Revolute);
    assert_eq!(joints_by_name["joint_3"].kind, JointKind::Prismatic);
    assert_eq!(joints_by_name["joint_4"].kind, JointKind::Continuous);
    assert_eq!(joints_by_name["base_joint"].kind, JointKind::Fixed);
}

#[test]
fn scara_kinematic_chain_is_connected() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    // The kinematic chain should be:
    // world → base_joint → base_link → joint_1 → link_1 → joint_2 → link_2
    //   → joint_3 → link_3 → joint_4 → tool0
    let links: Vec<&str> = robot.links.keys().map(|s| s.as_str()).collect();
    assert!(links.contains(&"world"), "should contain world");
    assert!(links.contains(&"base_link"), "should contain base_link");
    assert!(links.contains(&"link_1"), "should contain link_1");
    assert!(links.contains(&"link_2"), "should contain link_2");
    assert!(links.contains(&"link_3"), "should contain link_3");
    assert!(links.contains(&"tool0"), "should contain tool0");

    // Verify connectivity through joint parents/children
    assert_eq!(robot.joints["base_joint"].parent, "world");
    assert_eq!(robot.joints["base_joint"].child, "base_link");
    assert_eq!(robot.joints["joint_1"].parent, "base_link");
    assert_eq!(robot.joints["joint_1"].child, "link_1");
    assert_eq!(robot.joints["joint_2"].parent, "link_1");
    assert_eq!(robot.joints["joint_2"].child, "link_2");
    assert_eq!(robot.joints["joint_3"].parent, "link_2");
    assert_eq!(robot.joints["joint_3"].child, "link_3");
    assert_eq!(robot.joints["joint_4"].parent, "link_3");
    assert_eq!(robot.joints["joint_4"].child, "tool0");
}

#[test]
fn scara_prismatic_joint_has_proper_limits() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    let j3 = &robot.joints["joint_3"];
    let limits = j3.limits.expect("prismatic joint must have limits");
    assert!(
        (limits.min - (-0.5)).abs() < 1e-6,
        "joint_3 lower limit should be -0.5, got {}",
        limits.min
    );
    assert!(
        limits.max.abs() < 1e-6,
        "joint_3 upper limit should be 0.0, got {}",
        limits.max
    );
}

#[test]
fn scara_base_joint_has_z_origin() {
    let source = fs::read_to_string(FIXTURE_PATH).unwrap();
    let robot = parse_robot(&source).unwrap();

    let bj = &robot.joints["base_joint"];
    assert!(
        (bj.origin.translation.z - 0.5).abs() < 1e-6,
        "base_joint z-offset should be 0.5, got {}",
        bj.origin.translation.z
    );
}
