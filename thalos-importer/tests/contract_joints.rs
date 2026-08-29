use thalos_importer::import_urdf;
use thalos_models::joint::JointKind;

#[test]
fn joint_types_axes_and_limits_contract() {
    let xml = r#"
        <robot name="joint_test_bot">
            <link name="base"/>
            <link name="l1"/>
            <link name="l2"/>
            <link name="l3"/>
            <link name="l4"/>

            <joint name="j_revolute" type="revolute">
                <parent link="base"/>
                <child link="l1"/>
                <axis xyz="0 1 0"/>
                <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"/>
            </joint>

            <joint name="j_continuous" type="continuous">
                <parent link="l1"/>
                <child link="l2"/>
                <axis xyz="0 0 1"/>
            </joint>

            <joint name="j_prismatic" type="prismatic">
                <parent link="l2"/>
                <child link="l3"/>
                <axis xyz="1 0 0"/>
                <limit lower="0.0" upper="0.5" effort="50.0" velocity="0.5"/>
            </joint>

            <joint name="j_fixed" type="fixed">
                <parent link="l3"/>
                <child link="l4"/>
            </joint>
        </robot>
    "#;

    let robot = import_urdf(xml).expect("URDF joint contract fixture should import cleanly");

    // Revolute joint
    let rev = robot.joints.get("j_revolute").expect("j_revolute exists");
    assert!(matches!(rev.kind, JointKind::Revolute));
    let axis = rev.axis.expect("revolute joint requires an axis");
    assert_eq!((axis.x, axis.y, axis.z), (0.0, 1.0, 0.0));
    let limits = rev.limits.as_ref().expect("revolute joint requires limits");
    assert_eq!(limits.min, -3.14);
    assert_eq!(limits.max, 3.14);

    // Continuous joint (no lower/upper limits requirement)
    let cont = robot.joints.get("j_continuous").expect("j_continuous exists");
    assert!(matches!(cont.kind, JointKind::Continuous));
    let axis = cont.axis.expect("continuous joint requires an axis");
    assert_eq!((axis.x, axis.y, axis.z), (0.0, 0.0, 1.0));

    // Prismatic joint
    let prism = robot.joints.get("j_prismatic").expect("j_prismatic exists");
    assert!(matches!(prism.kind, JointKind::Prismatic));
    let axis = prism.axis.expect("prismatic joint requires an axis");
    assert_eq!((axis.x, axis.y, axis.z), (1.0, 0.0, 0.0));
    let limits = prism.limits.as_ref().expect("prismatic joint requires limits");
    assert_eq!(limits.min, 0.0);
    assert_eq!(limits.max, 0.5);

    // Fixed joint
    let fixed = robot.joints.get("j_fixed").expect("j_fixed exists");
    assert!(matches!(fixed.kind, JointKind::Fixed));
    assert!(fixed.axis.is_none());
}
