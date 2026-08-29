use thalos_importer::import_urdf;
use thalos_models::{Geometry, JointKind};

const FIXTURE: &str = include_str!("../../thalos-models/tests/fixtures/ur5.urdf");

#[test]
fn parse_ur5_successfully() {
    let robot = import_urdf(FIXTURE).expect("UR5 should parse without errors");
    assert_eq!(robot.name, "ur5");
}

#[test]
fn ur5_link_count() {
    let robot = import_urdf(FIXTURE).unwrap();
    assert_eq!(robot.links.len(), 11);
}

#[test]
fn ur5_joint_count() {
    let robot = import_urdf(FIXTURE).unwrap();
    assert_eq!(robot.joints.len(), 10);
}

#[test]
fn ur5_root_link_is_world() {
    let robot = import_urdf(FIXTURE).unwrap();
    assert_eq!(robot.root_link, "world");
}

#[test]
fn ur5_revolute_joints_have_axis_and_limits() {
    let robot = import_urdf(FIXTURE).unwrap();
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

        assert_eq!(joint.kind, JointKind::Revolute);
        assert!(joint.axis.is_some());
        assert!(joint.limits.is_some());
    }
}

#[test]
fn ur5_fixed_joints_have_no_axis_or_limits() {
    let robot = import_urdf(FIXTURE).unwrap();
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

        assert_eq!(joint.kind, JointKind::Fixed);
        assert!(joint.axis.is_none());
        assert!(joint.limits.is_none());
    }
}

#[test]
fn ur5_kinematic_chain_is_connected() {
    let robot = import_urdf(FIXTURE).unwrap();
    for joint in robot.joints.values() {
        assert!(robot.links.contains_key(&joint.parent));
        assert!(robot.links.contains_key(&joint.child));
    }
}

#[test]
fn ur5_shoulder_pan_origin_has_z_offset() {
    let robot = import_urdf(FIXTURE).unwrap();
    let joint = &robot.joints["shoulder_pan_joint"];
    assert!((joint.origin.translation.z - 0.089159).abs() < 1e-6);
}

#[test]
fn ur5_elbow_joint_origin() {
    let robot = import_urdf(FIXTURE).unwrap();
    let joint = &robot.joints["elbow_joint"];
    assert!((joint.origin.translation.x).abs() < 1e-6);
    assert!((joint.origin.translation.y - (-0.1197)).abs() < 1e-6);
    assert!((joint.origin.translation.z - 0.425).abs() < 1e-6);
}

#[test]
fn ur5_shoulder_lift_rotation() {
    use thalos_math::UnitQuaternion;

    let robot = import_urdf(FIXTURE).unwrap();
    let joint = &robot.joints["shoulder_lift_joint"];
    let expected = UnitQuaternion::from_euler(0.0, 1.57079632679, 0.0);
    let actual = joint.origin.rotation;
    let a = actual.inner();
    let e = expected.inner();
    assert!(
        (a.w - e.w).abs() < 1e-6
            && (a.x - e.x).abs() < 1e-6
            && (a.y - e.y).abs() < 1e-6
            && (a.z - e.z).abs() < 1e-6
    );
}

#[test]
fn ur5_inertial_data_exists() {
    let robot = import_urdf(FIXTURE).unwrap();
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
        assert!(link.inertial.is_some());
    }
}

#[test]
fn ur5_upper_arm_mass() {
    let robot = import_urdf(FIXTURE).unwrap();
    let inertial = robot.links["upper_arm_link"].inertial.as_ref().unwrap();
    assert!((inertial.mass - 8.393).abs() < 1e-6);
}

#[test]
fn ur5_visual_mesh_geometry() {
    let robot = import_urdf(FIXTURE).unwrap();
    let link = &robot.links["shoulder_link"];
    assert_eq!(link.visual.len(), 1);

    match &link.visual[0].geometry {
        Geometry::Mesh { filename, scale: _ } => {
            assert!(filename.contains("shoulder.dae"));
        }
        other => panic!("expected Mesh geometry, got {other:?}"),
    }
}
