use crate::models::scara::ScaraSpec;
use crate::prelude::*;
use thalos_math::constants::*;
use thalos_math::*;

// ─── ADR-0001 Z-up regression tests ──────────────────────────

#[test]
fn zero_config_ee_in_z_up() {
    // ADR-0001: Z is the canonical vertical axis.
    // At q=[0,0,0,0] with ideal spec (base_height=0, a1=a2=1):
    //   ee = (a1+a2, 0, 0) = (2, 0, 0)
    // Both Y-up and Z-up agree at zero config with no base offset.
    let robot = ScaraSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[0.0, 0.0, 0.0, 0.0]);
    let ee = result.ee_pose().unwrap();
    let t = &ee.transform().translation;

    assert!(
        (t.x - 2.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "SCARA Z-up regression: expected (2, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn prismatic_moves_ee_in_z() {
    // ADR-0001: Prismatic joint moves EE along Z (vertical).
    // At q=[0, 0, d3, 0]: ee = (a1+a2, 0, d3)
    let mut spec = ScaraSpec::ideal();
    spec.joint_limits[2] = JointLimits::new(-2.0, 2.0);
    let robot = spec.build();
    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[0.0, 0.0, 0.5, 0.0]);
    let t = result.ee_pose().unwrap().transform().translation;
    assert!(
        (t.x - 2.0).abs() < EPS && t.y.abs() < EPS && (t.z - 0.5).abs() < EPS,
        "SCARA prismatic in Z: expected (2, 0, 0.5), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );

    let result = fk.evaluate(&[0.0, 0.0, -1.0, 0.0]);
    let t = result.ee_pose().unwrap().transform().translation;
    assert!(
        (t.x - 2.0).abs() < EPS && t.y.abs() < EPS && (t.z + 1.0).abs() < EPS,
        "SCARA prismatic in Z: expected (2, 0, -1), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn first_joint_90_deg_in_z_up() {
    // ADR-0001: Rz(π/2) rotates +X to +Y in Z-up.
    // ee at q=[π/2, 0, 0, 0] → (0, 2, 0)
    let robot = ScaraSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[PI / 2.0, 0.0, 0.0, 0.0]);
    let t = result.ee_pose().unwrap().transform().translation;

    assert!(
        t.x.abs() < EPS && (t.y - 2.0).abs() < EPS && t.z.abs() < EPS,
        "SCARA Rz(90°) should give (0, 2, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn folded_configuration_in_z_up() {
    // ADR-0001: Rz(π/2)·Rz(-π/2) = Rz(0).
    // ee at q=[π/2, -π/2, 0, 0] → link1 in Y, link2 in X → (1, 1, 0)
    let robot = ScaraSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[PI / 2.0, -PI / 2.0, 0.0, 0.0]);
    let t = result.ee_pose().unwrap().transform().translation;

    assert!(
        (t.x - 1.0).abs() < EPS && (t.y - 1.0).abs() < EPS && t.z.abs() < EPS,
        "SCARA folded in Z-up: expected (1, 1, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn wrist_rotation_in_z_up() {
    // ADR-0001: Wrist Rz(π/2) does not change position.
    let robot = ScaraSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[0.0, 0.0, 0.0, PI / 2.0]);
    let t = result.ee_pose().unwrap().transform().translation;

    assert!(
        (t.x - 2.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "SCARA wrist rotation should not move ee from (2, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn combined_motion_accumulates_correctly_in_z_up() {
    // ADR-0001: q1=45°, q2=45°, d3=0.3, q4=90°
    // ee = (cos45 + cos90, sin45 + sin90, 0.3) = (0.7071, 1.7071, 0.3)
    let robot = ScaraSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[PI / 4.0, PI / 4.0, 0.3, PI / 2.0]);
    let t = result.ee_pose().unwrap().transform().translation;

    let expected_x = (PI / 4.0).cos() + (PI / 2.0).cos();
    let expected_y = (PI / 4.0).sin() + (PI / 2.0).sin();
    let expected_z = 0.3;

    assert!(
        (t.x - expected_x).abs() < EPS
            && (t.y - expected_y).abs() < EPS
            && (t.z - expected_z).abs() < EPS,
        "SCARA combined Z-up: expected ({:.4}, {:.4}, {:.4}), got ({:.4}, {:.4}, {:.4})",
        expected_x,
        expected_y,
        expected_z,
        t.x,
        t.y,
        t.z
    );
}

// ─── Existing tests (will be removed after Phase 2 migration) ─

#[test]
fn returns_six_poses() {
    let robot = ScaraSpec::ideal().build();

    let fk = ForwardKinematics::new(robot);

    // Configuración: 4 DOFs (q no incluye la base fija)
    let result = fk.evaluate(&[0.0, 0.0, 0.0, 0.0]);

    let frames: Vec<_> = result.frames().collect();

    assert_eq!(
        frames.len(),
        6, // 5 segmentos (1 fixed + 4 actuados) = 5 frames móviles + world frame
        "SCARA should generate exactly 6 poses (base + 4 joints + world pose)",
    );
}

#[test]
fn zero_configuration_places_end_effector_at_2_0_0() {
    let robot = ScaraSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    // q1=0, q2=0, d3=0, q4=0
    let result = fk.evaluate(&[0.0, 0.0, 0.0, 0.0]);

    let pose = result.pose(&end_effector).unwrap();

    let t = &pose.transform().translation;

    assert!(
        (t.x - 2.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "SCARA at zero config should be at (2, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn position_independent_of_wrist_rotation() {
    let robot = ScaraSpec::ideal().build();

    let end_effector = robot.segments.last().unwrap().child.clone();

    let fk = ForwardKinematics::new(robot);

    // Configuración base: brazos extendidos en X
    let base_config = [0.0, 0.0, 0.5, 0.0];
    let rotated_config = [0.0, 0.0, 0.5, PI / 2.0];

    let result_base = fk.evaluate(&base_config);
    let result_rotated = fk.evaluate(&rotated_config);

    let pose_base = result_base.pose(&end_effector).unwrap();
    let pose_rotated = result_rotated.pose(&end_effector).unwrap();

    let t_base = &pose_base.transform().translation;
    let t_rotated = &pose_rotated.transform().translation;

    // La posición debe ser idéntica independientemente de la rotación de muñeca
    assert!(
        (t_base.x - t_rotated.x).abs() < EPS
            && (t_base.y - t_rotated.y).abs() < EPS
            && (t_base.z - t_rotated.z).abs() < EPS,
        "Wrist rotation changed position: base ({}, {}, {}) vs rotated ({}, {}, {})",
        t_base.x,
        t_base.y,
        t_base.z,
        t_rotated.x,
        t_rotated.y,
        t_rotated.z
    );
}
