use thalos_importer::import_urdf;
use thalos_engine::core::robot::adapter;
use thalos_engine::core::kinematics::forward::ForwardKinematics;

const ABB_IRB140: &str = include_str!("fixtures/robots/abb_irb140/robot.urdf");

#[test]
fn abb_irb140_imports() {
    let robot = import_urdf(ABB_IRB140).expect("ABB IRB 140 should import");
    assert_eq!(robot.name, "abb_irb140");
    assert_eq!(robot.root_link, "base_link");
    // 6 revolute + 2 fixed = 8 joints
    assert_eq!(robot.joints.len(), 8);
    // base_link, link_1..6, tool0, base = 9 links
    assert_eq!(robot.links.len(), 9);
}

#[test]
fn abb_irb140_joint_chain() {
    let robot = import_urdf(ABB_IRB140).expect("ABB IRB 140 should import");

    // Verify the serial chain: base_link -> link_1 -> ... -> link_6 -> tool0
    let expected_chain = [
        ("joint_1", "base_link", "link_1"),
        ("joint_2", "link_1", "link_2"),
        ("joint_3", "link_2", "link_3"),
        ("joint_4", "link_3", "link_4"),
        ("joint_5", "link_4", "link_5"),
        ("joint_6", "link_5", "link_6"),
    ];

    for (name, parent, child) in &expected_chain {
        let joint = robot.joints.get(*name).expect(&format!("missing joint {}", name));
        assert_eq!(joint.parent, *parent, "{} parent", name);
        assert_eq!(joint.child, *child, "{} child", name);
    }
}

#[test]
fn abb_irb140_joint_limits() {
    let robot = import_urdf(ABB_IRB140).expect("ABB IRB 140 should import");

    // All 6 revolute joints should have limits
    for i in 1..=6 {
        let name = format!("joint_{}", i);
        let joint = robot.joints.get(&name).expect(&format!("missing {}", name));
        let limits = joint.limits.as_ref().expect(&format!("{} should have limits", name));
        assert!(limits.min < limits.max, "{} limits should be min < max", name);
    }
}

#[test]
fn abb_irb140_joint_axes() {
    let robot = import_urdf(ABB_IRB140).expect("ABB IRB 140 should import");

    // Verify axes from URDF
    let j1 = robot.joints.get("joint_1").unwrap();
    let axis = j1.axis.unwrap();
    assert!((axis.z - 1.0).abs() < 1e-6, "joint_1 axis should be Z");

    let j2 = robot.joints.get("joint_2").unwrap();
    let axis = j2.axis.unwrap();
    assert!((axis.y - 1.0).abs() < 1e-6, "joint_2 axis should be Y");

    let j4 = robot.joints.get("joint_4").unwrap();
    let axis = j4.axis.unwrap();
    assert!((axis.x - 1.0).abs() < 1e-6, "joint_4 axis should be X");
}

#[test]
fn abb_irb140_fk_zero_pose() {
    let robot = import_urdf(ABB_IRB140).expect("ABB IRB 140 should import");
    let chain = adapter::auto(&robot).expect("adapter should build chain");
    assert_eq!(chain.dof_count(), 6);

    let fk = ForwardKinematics::new(chain);
    let q = [0.0; 6];
    let result = fk.evaluate(&q);
    let ee = result.ee_position().expect("ee position should exist");

    // At zero pose, the EE should be at a non-trivial position
    // (the robot has link lengths of ~1.38m, ~1.42m, ~1.49m)
    eprintln!("ABB IRB 140 FK(q=[0;6]) EE: ({:.4}, {:.4}, {:.4})", ee.x, ee.y, ee.z);

    // The EE should not be at origin (robot has physical extent)
    let dist = (ee.x * ee.x + ee.y * ee.y + ee.z * ee.z).sqrt();
    assert!(dist > 0.5, "EE at zero pose should be far from origin, got dist={}", dist);
}

#[test]
fn abb_irb140_visual_elements() {
    let robot = import_urdf(ABB_IRB140).expect("ABB IRB 140 should import");

    // base_link should have visual
    let base = robot.links.get("base_link").unwrap();
    assert!(!base.visual.is_empty(), "base_link should have visual");

    // link_1 through link_6 should have visual
    for i in 1..=6 {
        let link = robot.links.get(&format!("link_{}", i)).unwrap();
        assert!(!link.visual.is_empty(), "link_{} should have visual", i);
    }
}

#[test]
fn abb_irb140_materials_preserved() {
    let robot = import_urdf(ABB_IRB140).expect("ABB IRB 140 should import");

    // Materials are defined inline in visual blocks, not globally
    // Verify base_link has material with color
    let base = robot.links.get("base_link").unwrap();
    let visual = base.visual.first().expect("base_link should have visual");
    let mat = visual.material.as_ref().expect("base_link visual should have material");
    assert_eq!(mat.name, "abb_orange");
    assert!(mat.color.is_some(), "abb_orange should have color");
}
