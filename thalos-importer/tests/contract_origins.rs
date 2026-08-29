use thalos_importer::import_urdf;

#[test]
fn joint_and_visual_origins_are_strictly_separated() {
    let xml = r#"
        <robot name="origin_test_bot">
            <link name="base_link"/>
            <link name="link1">
                <visual>
                    <origin xyz="0.05 0.10 0.15" rpy="0.1 0.2 0.3"/>
                    <geometry><box size="0.1 0.1 0.1"/></geometry>
                </visual>
            </link>
            <joint name="j1" type="revolute">
                <parent link="base_link"/>
                <child link="link1"/>
                <origin xyz="0.10 0.20 0.30" rpy="0.4 0.5 0.6"/>
                <axis xyz="0 0 1"/>
                <limit lower="-1.57" upper="1.57"/>
            </joint>
        </robot>
    "#;

    let robot = import_urdf(xml).expect("URDF with asymmetric origins should import cleanly");

    // 1. Check joint origin (parent -> joint frame)
    let joint = robot.joints.get("j1").expect("joint j1 should exist");
    assert_eq!(joint.parent, "base_link");
    assert_eq!(joint.child, "link1");
    assert_eq!(joint.origin.translation.x, 0.10);
    assert_eq!(joint.origin.translation.y, 0.20);
    assert_eq!(joint.origin.translation.z, 0.30);
    let (r, p, y) = joint.origin.rotation.to_euler();
    assert!((r - 0.4).abs() < 1e-6);
    assert!((p - 0.5).abs() < 1e-6);
    assert!((y - 0.6).abs() < 1e-6);

    // 2. Check visual origin (link frame -> visual geometry)
    let link = robot.links.get("link1").expect("link1 should exist");
    assert_eq!(link.visual.len(), 1, "link1 should have 1 visual element");
    let visual = &link.visual[0];
    assert_eq!(visual.origin.translation.x, 0.05);
    assert_eq!(visual.origin.translation.y, 0.10);
    assert_eq!(visual.origin.translation.z, 0.15);
    let (vr, vp, vy) = visual.origin.rotation.to_euler();
    assert!((vr - 0.1).abs() < 1e-6);
    assert!((vp - 0.2).abs() < 1e-6);
    assert!((vy - 0.3).abs() < 1e-6);
}
