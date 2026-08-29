use thalos_importer::import_urdf;

#[test]
fn serial_link_hierarchy_topology_contract() {
    let xml = r#"
        <robot name="chain_bot">
            <link name="base"/>
            <link name="link1"/>
            <link name="link2"/>

            <joint name="j1" type="revolute">
                <parent link="base"/>
                <child link="link1"/>
            </joint>

            <joint name="j2" type="revolute">
                <parent link="link1"/>
                <child link="link2"/>
            </joint>
        </robot>
    "#;

    let robot = import_urdf(xml).expect("hierarchy test bot should import");

    assert_eq!(robot.root_link, "base");
    assert_eq!(robot.links.len(), 3);
    assert_eq!(robot.joints.len(), 2);

    let j1 = robot.joints.get("j1").unwrap();
    assert_eq!(j1.parent, "base");
    assert_eq!(j1.child, "link1");

    let j2 = robot.joints.get("j2").unwrap();
    assert_eq!(j2.parent, "link1");
    assert_eq!(j2.child, "link2");
}
