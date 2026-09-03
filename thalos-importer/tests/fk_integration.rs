use thalos_importer::import_urdf;
use thalos_engine::core::robot::adapter;
use thalos_engine::core::kinematics::forward::ForwardKinematics;

const PI: f64 = std::f64::consts::PI;

// ── Planar 2R ──────────────────────────────────────────────────
// URDF joint origins: joint_1 at (0,0,0), joint_2 at (1.0, 0, 0)
// FK returns frame positions (link frame origins), not visual geometry tips.
// At q=[0,0]: link_1 frame at (0,0,0), link_2 frame at (1.0, 0, 0)

const PLANAR_2R: &str = include_str!("fixtures/robots/planar_2r/robot.urdf");

#[test]
fn planar_2r_import_and_chain() {
    let robot = import_urdf(PLANAR_2R).expect("planar_2r should import");
    assert_eq!(robot.name, "planar_2r");
    assert_eq!(robot.root_link, "base_link");
    assert_eq!(robot.links.len(), 3);
    assert_eq!(robot.joints.len(), 2);

    let chain = adapter::auto(&robot).expect("adapter should build chain");
    assert_eq!(chain.dof_count(), 2);
}

#[test]
fn planar_2r_fk_zero_pose() {
    let robot = import_urdf(PLANAR_2R).unwrap();
    let chain = adapter::auto(&robot).unwrap();
    let fk = ForwardKinematics::new(chain);

    // q = [0, 0]: link_2 frame at joint_2 origin = (1.0, 0, 0)
    let result = fk.evaluate(&[0.0, 0.0]);
    let ee = result.ee_position().expect("ee position should exist");

    assert!((ee.x - 1.0).abs() < 1e-6, "EE x at zero pose: expected 1.0, got {}", ee.x);
    assert!(ee.y.abs() < 1e-6, "EE y at zero pose: expected 0, got {}", ee.y);
}

#[test]
fn planar_2r_fk_45_degrees() {
    let robot = import_urdf(PLANAR_2R).unwrap();
    let chain = adapter::auto(&robot).unwrap();
    let fk = ForwardKinematics::new(chain);

    // q = [π/4, π/4]:
    // joint_2 frame after q1 rotation + q2 rotation
    // x = 1.0*cos(q1) + 0*cos(q1+q2) = cos(π/4) = √2/2
    // y = 1.0*sin(q1) + 0*sin(q1+q2) = sin(π/4) = √2/2
    // The second joint origin is at distance 1.0 from joint_1, rotated by q1
    let q = [PI / 4.0, PI / 4.0];
    let result = fk.evaluate(&q);
    let ee = result.ee_position().expect("ee position should exist");

    // joint_2 origin is at (1.0, 0, 0) relative to link_1 frame
    // After q1 rotation: (1.0*cos(q1), 1.0*sin(q1), 0)
    let expected_x = 1.0 * (PI / 4.0).cos();
    let expected_y = 1.0 * (PI / 4.0).sin();

    assert!((ee.x - expected_x).abs() < 1e-6,
        "EE x at π/4,π/4: expected {}, got {}", expected_x, ee.x);
    assert!((ee.y - expected_y).abs() < 1e-6,
        "EE y at π/4,π/4: expected {}, got {}", expected_y, ee.y);
}

#[test]
fn planar_2r_fk_first_joint_only() {
    let robot = import_urdf(PLANAR_2R).unwrap();
    let chain = adapter::auto(&robot).unwrap();
    let fk = ForwardKinematics::new(chain);

    // q = [π/2, 0]: joint_2 frame rotated 90° from link_1
    // x = 1.0*cos(π/2) = 0
    // y = 1.0*sin(π/2) = 1.0
    let result = fk.evaluate(&[PI / 2.0, 0.0]);
    let ee = result.ee_position().expect("ee position should exist");

    assert!(ee.x.abs() < 1e-6,
        "EE x at π/2,0: expected 0, got {}", ee.x);
    assert!((ee.y - 1.0).abs() < 1e-6,
        "EE y at π/2,0: expected 1.0, got {}", ee.y);
}

#[test]
fn planar_2r_fk_second_joint_contribution() {
    let robot = import_urdf(PLANAR_2R).unwrap();
    let chain = adapter::auto(&robot).unwrap();
    let fk = ForwardKinematics::new(chain);

    // q = [0, π/2]: link_1 at 0°, link_2 frame rotated by q2 relative to link_1
    // But joint_2 origin is at (1.0, 0, 0) relative to link_1
    // After q1=0: joint_2 frame at (1.0, 0, 0)
    // q2 rotates the link_2 frame around its own Z axis
    // Since joint_2 origin has no offset from link_1's endpoint,
    // the frame position doesn't change with q2 — only the orientation does
    let result = fk.evaluate(&[0.0, PI / 2.0]);
    let ee = result.ee_position().expect("ee position should exist");

    // Frame position stays at (1.0, 0, 0) regardless of q2
    assert!((ee.x - 1.0).abs() < 1e-6,
        "EE x at 0,π/2: expected 1.0, got {}", ee.x);
    assert!(ee.y.abs() < 1e-6,
        "EE y at 0,π/2: expected 0, got {}", ee.y);
}

#[test]
fn planar_2r_fk_intermediate_pose() {
    let robot = import_urdf(PLANAR_2R).unwrap();
    let chain = adapter::auto(&robot).unwrap();
    let fk = ForwardKinematics::new(chain);

    // q = [π/6, -π/4]:
    // joint_2 frame at (1.0*cos(π/6), 1.0*sin(π/6), 0)
    let q1 = PI / 6.0;
    let q2 = -PI / 4.0;

    let expected_x = 1.0 * q1.cos();
    let expected_y = 1.0 * q1.sin();

    let result = fk.evaluate(&[q1, q2]);
    let ee = result.ee_position().expect("ee position should exist");

    assert!((ee.x - expected_x).abs() < 1e-6,
        "EE x at π/6,-π/4: expected {}, got {}", expected_x, ee.x);
    assert!((ee.y - expected_y).abs() < 1e-6,
        "EE y at π/6,-π/4: expected {}, got {}", expected_y, ee.y);
}

#[test]
fn planar_2r_origin_rpy_preserved() {
    let robot = import_urdf(PLANAR_2R).unwrap();

    let j1 = robot.joints.get("joint_1").unwrap();
    assert!((j1.origin.translation.x).abs() < 1e-6);
    assert!((j1.origin.translation.y).abs() < 1e-6);
    assert!((j1.origin.translation.z).abs() < 1e-6);

    let j2 = robot.joints.get("joint_2").unwrap();
    assert!((j2.origin.translation.x - 1.0).abs() < 1e-6);
    assert!((j2.origin.translation.y).abs() < 1e-6);
    assert!((j2.origin.translation.z).abs() < 1e-6);
}

#[test]
fn planar_2r_visual_elements_preserved() {
    let robot = import_urdf(PLANAR_2R).unwrap();

    let link1 = robot.links.get("link_1").unwrap();
    assert_eq!(link1.visual.len(), 1, "link_1 should have 1 visual");

    let link2 = robot.links.get("link_2").unwrap();
    assert_eq!(link2.visual.len(), 1, "link_2 should have 1 visual");
}

#[test]
fn planar_2r_inertial_preserved() {
    let robot = import_urdf(PLANAR_2R).unwrap();

    let link1 = robot.links.get("link_1").unwrap();
    let inertial1 = link1.inertial.as_ref().expect("link_1 should have inertial");
    assert!((inertial1.mass - 1.0).abs() < 1e-6);

    let link2 = robot.links.get("link_2").unwrap();
    let inertial2 = link2.inertial.as_ref().expect("link_2 should have inertial");
    assert!((inertial2.mass - 0.5).abs() < 1e-6);
}
