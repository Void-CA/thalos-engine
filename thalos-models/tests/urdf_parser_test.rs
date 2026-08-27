//! Integration tests for the URDF parser — exercises the full public API.

use thalos_models::urdf::parser::parse_robot;
use thalos_models::{Geometry, JointKind};

#[test]
fn minimal_robot() {
    let source = r#"
        <robot name="test">
            <link name="base"/>
        </robot>
    "#;
    let robot = parse_robot(source).unwrap();
    assert_eq!(robot.name, "test");
    assert_eq!(robot.links.len(), 1);
    assert!(robot.links.contains_key("base"));
    assert!(robot.joints.is_empty());
    assert_eq!(robot.root_link, "base");
}

#[test]
fn robot_with_joint() {
    let source = r#"
        <robot name="arm">
            <link name="base"/>
            <link name="tool"/>
            <joint name="j1" type="revolute">
                <parent link="base"/>
                <child link="tool"/>
                <origin xyz="0 0 1" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
                <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
            </joint>
        </robot>
    "#;
    let robot = parse_robot(source).unwrap();
    assert_eq!(robot.joints.len(), 1);
    let joint = &robot.joints["j1"];
    assert_eq!(joint.kind, JointKind::Revolute);
    assert_eq!(joint.parent, "base");
    assert_eq!(joint.child, "tool");
    assert!(joint.axis.is_some());
    let limits = joint.limits.unwrap();
    assert!((limits.min - (-1.57)).abs() < 1e-6);
    assert!((limits.max - 1.57).abs() < 1e-6);
    assert_eq!(limits.velocity, Some(1.0));
    assert_eq!(limits.effort, Some(10.0));
    assert_eq!(robot.root_link, "base");
}

#[test]
fn root_link_detection() {
    let source = r#"
        <robot name="r">
            <link name="a"/>
            <link name="b"/>
            <link name="c"/>
            <joint name="j1" type="fixed">
                <parent link="a"/>
                <child link="b"/>
            </joint>
            <joint name="j2" type="fixed">
                <parent link="b"/>
                <child link="c"/>
            </joint>
        </robot>
    "#;
    let robot = parse_robot(source).unwrap();
    assert_eq!(robot.root_link, "a");
    assert_eq!(robot.links.len(), 3);
    assert_eq!(robot.joints.len(), 2);
}

#[test]
fn geometry_shapes() {
    let source = r#"
        <robot name="geo">
            <link name="base"/>
            <link name="tip">
                <visual>
                    <origin xyz="0 0 0.5" rpy="0 0 0"/>
                    <geometry>
                        <sphere radius="0.1"/>
                    </geometry>
                </visual>
                <collision>
                    <geometry>
                        <box size="0.2 0.2 0.2"/>
                    </geometry>
                </collision>
            </link>
            <joint name="j" type="fixed">
                <parent link="base"/>
                <child link="tip"/>
            </joint>
        </robot>
    "#;
    let robot = parse_robot(source).unwrap();
    let tip = &robot.links["tip"];
    assert_eq!(tip.visual.len(), 1);
    assert_eq!(tip.collision.len(), 1);

    match &tip.visual[0].geometry {
        Geometry::Sphere { radius } => assert!((*radius - 0.1).abs() < 1e-6),
        _ => panic!("expected sphere"),
    }
    match &tip.collision[0].geometry {
        Geometry::Box {
            width,
            height,
            depth,
        } => {
            assert!((*width - 0.2).abs() < 1e-6);
            assert!((*height - 0.2).abs() < 1e-6);
            assert!((*depth - 0.2).abs() < 1e-6);
        }
        _ => panic!("expected box"),
    }
}

#[test]
fn inertial_parsing() {
    let source = r#"
        <robot name="i">
            <link name="base">
                <inertial>
                    <origin xyz="0 0 0" rpy="0 0 0"/>
                    <mass value="2.5"/>
                    <inertia ixx="0.1" ixy="0" ixz="0"
                             iyy="0.1" iyz="0" izz="0.1"/>
                </inertial>
            </link>
        </robot>
    "#;
    let robot = parse_robot(source).unwrap();
    let inertial = robot.links["base"].inertial.as_ref().unwrap();
    assert!((inertial.mass - 2.5).abs() < 1e-6);
    assert!((inertial.inertia.ixx - 0.1).abs() < 1e-6);
}

#[test]
fn visual_material() {
    let source = r#"
        <robot name="m">
            <link name="base">
                <visual>
                    <geometry><sphere radius="1"/></geometry>
                    <material name="red">
                        <color rgba="1 0 0 1"/>
                    </material>
                </visual>
            </link>
        </robot>
    "#;
    let robot = parse_robot(source).unwrap();
    let mat = robot.links["base"].visual[0].material.as_ref().unwrap();
    assert_eq!(mat.name, "");
    let color = mat.color.unwrap();
    assert!((color.r - 1.0).abs() < 1e-6);
    assert!((color.g - 0.0).abs() < 1e-6);
    assert!((color.b - 0.0).abs() < 1e-6);
    assert!((color.a - 1.0).abs() < 1e-6);
}

#[test]
fn multiple_joint_types() {
    for (type_str, expected) in [
        ("revolute", JointKind::Revolute),
        ("continuous", JointKind::Continuous),
        ("prismatic", JointKind::Prismatic),
        ("fixed", JointKind::Fixed),
        ("floating", JointKind::Floating),
        ("planar", JointKind::Planar),
    ] {
        let source = format!(
            r#"
            <robot name="jt">
                <link name="a"/>
                <link name="b"/>
                <joint name="j" type="{type_str}">
                    <parent link="a"/>
                    <child link="b"/>
                </joint>
            </robot>
            "#
        );
        let robot = parse_robot(&source).unwrap();
        assert_eq!(robot.joints["j"].kind, expected, "mismatch for {type_str}");
    }
}

#[test]
fn error_missing_robot_name() {
    let source = r#"<robot><link name="x"/></robot>"#;
    let err = parse_robot(source).unwrap_err();
    assert!(
        err.to_string().contains("missing required attribute"),
        "got: {err}"
    );
}

#[test]
fn error_unknown_joint_type() {
    let source = r#"
        <robot name="e">
            <link name="a"/><link name="b"/>
            <joint name="j" type="hyperloop">
                <parent link="a"/><child link="b"/>
            </joint>
        </robot>
    "#;
    let err = parse_robot(source).unwrap_err();
    assert!(err.to_string().contains("unknown joint type"), "got: {err}");
}

#[test]
fn error_missing_parent_in_joint() {
    let source = r#"
        <robot name="e">
            <link name="a"/><link name="b"/>
            <joint name="j" type="fixed">
                <child link="b"/>
            </joint>
        </robot>
    "#;
    let err = parse_robot(source).unwrap_err();
    assert!(
        err.to_string().contains("missing required child"),
        "got: {err}"
    );
}

#[test]
fn global_material_shared() {
    let source = r#"
        <robot name="g">
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
            <link name="a"/>
        </robot>
    "#;
    let robot = parse_robot(source).unwrap();
    let mat = &robot.materials["blue"];
    assert_eq!(mat.name, "blue");
    let color = mat.color.unwrap();
    assert!((color.b - 1.0).abs() < 1e-6);
}

#[test]
fn continuous_joint_no_limits() {
    let source = r#"
        <robot name="c">
            <link name="a"/><link name="b"/>
            <joint name="j" type="continuous">
                <parent link="a"/><child link="b"/>
                <axis xyz="0 0 1"/>
            </joint>
        </robot>
    "#;
    let robot = parse_robot(source).unwrap();
    let joint = &robot.joints["j"];
    assert_eq!(joint.kind, JointKind::Continuous);
    assert!(joint.limits.is_none());
}

#[test]
fn mesh_geometry_with_scale() {
    let source = r#"
        <robot name="m">
            <link name="base">
                <visual>
                    <geometry>
                        <mesh filename="package://meshes/arm.stl" scale="1 2 1"/>
                    </geometry>
                </visual>
            </link>
        </robot>
    "#;
    let robot = parse_robot(source).unwrap();
    let visual = &robot.links["base"].visual[0];
    match &visual.geometry {
        Geometry::Mesh { filename, scale } => {
            assert_eq!(filename, "package://meshes/arm.stl");
            let s = scale.unwrap();
            assert!((s.x - 1.0).abs() < 1e-6);
            assert!((s.y - 2.0).abs() < 1e-6);
            assert!((s.z - 1.0).abs() < 1e-6);
        }
        _ => panic!("expected mesh"),
    }
}
