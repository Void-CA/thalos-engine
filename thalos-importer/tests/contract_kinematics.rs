use thalos_importer::import_urdf;
use thalos_engine::core::robot::adapter;
use thalos_engine::core::kinematics::forward::ForwardKinematics;

#[test]
fn scara_imported_robot_fk_zero_pose_contract() {
    let source = include_str!("../../thalos-models/tests/fixtures/scara.urdf");
    let robot = import_urdf(source).expect("SCARA fixture should import cleanly");

    assert_eq!(robot.name, "scara");
    assert_eq!(robot.root_link, "world");

    // Build kinematics chain from imported model
    let chain = adapter::auto(&robot).expect("adapter should build serial chain");
    let fk = ForwardKinematics::new(chain);
    let result = fk.evaluate(&[0.0, 0.0, 0.0, 0.0]);

    let ee_pose = result.ee_pose().expect("End effector pose should exist");
    assert!(ee_pose.translation().z > 0.0, "TCP Z offset should be positive");
}

#[test]
fn dof6_imported_robot_fk_zero_pose_contract() {
    let source = include_str!("../../../thalos-industrial/docs/execution/robot/6dof.urdf");
    let robot = import_urdf(source).expect("6DOF fixture should import cleanly");

    assert_eq!(robot.name, "manipulator_6dof");
    assert_eq!(robot.root_link, "base_link");

    let chain = adapter::auto(&robot).expect("adapter should build 6DOF serial chain");
    assert_eq!(chain.dof_count(), 6, "6DOF manipulator should have 6 active degrees of freedom");

    let fk = ForwardKinematics::new(chain);
    let result = fk.evaluate(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

    let ee_pose = result.ee_pose().expect("End effector pose should exist");
    assert!(ee_pose.translation().z > 0.5, "6DOF TCP at zero pose must be elevated");
}
