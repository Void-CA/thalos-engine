use thalos_importer::import_urdf;
use thalos_models::JointKind;

const FIXTURE: &str = include_str!("../../thalos-models/tests/fixtures/scara.urdf");

#[test]
fn parse_scara_successfully() {
    let robot = import_urdf(FIXTURE).expect("SCARA should parse without errors");
    assert_eq!(robot.name, "scara");
}

#[test]
fn scara_link_count() {
    let robot = import_urdf(FIXTURE).unwrap();
    assert_eq!(robot.links.len(), 6);
}

#[test]
fn scara_joint_count() {
    let robot = import_urdf(FIXTURE).unwrap();
    assert_eq!(robot.joints.len(), 5);
}

#[test]
fn scara_root_link_is_world() {
    let robot = import_urdf(FIXTURE).unwrap();
    assert_eq!(robot.root_link, "world");
}

#[test]
fn scara_has_two_revolute_one_prismatic_one_continuous() {
    let robot = import_urdf(FIXTURE).unwrap();
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
    let robot = import_urdf(FIXTURE).unwrap();
    let links: Vec<&str> = robot.links.keys().map(|s| s.as_str()).collect();
    assert!(links.contains(&"world"));
    assert!(links.contains(&"base_link"));
    assert!(links.contains(&"link_1"));
    assert!(links.contains(&"link_2"));
    assert!(links.contains(&"link_3"));
    assert!(links.contains(&"tool0"));

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
    let robot = import_urdf(FIXTURE).unwrap();
    let j3 = &robot.joints["joint_3"];
    let limits = j3.limits.expect("prismatic joint must have limits");
    assert!((limits.min - (-0.5)).abs() < 1e-6);
    assert!(limits.max.abs() < 1e-6);
}

#[test]
fn scara_base_joint_has_z_origin() {
    let robot = import_urdf(FIXTURE).unwrap();
    let bj = &robot.joints["base_joint"];
    assert!((bj.origin.translation.z - 0.5).abs() < 1e-6);
}
