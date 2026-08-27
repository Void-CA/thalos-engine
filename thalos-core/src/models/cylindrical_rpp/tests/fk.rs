use crate::models::cylindrical_rpp::CylindricalRPPSpec;
use crate::prelude::*;

// ─── ADR-0001 Z-up regression tests ──────────────────────────

#[test]
fn zero_config_ee_at_origin() {
    // ADR-0001: Z is vertical. Cylindrical RPP: R(z), P(z), P(x).
    // At q=[0,0,0] with ideal l1=0: ee = (0, 0, 0) — all axes at zero.
    let robot = CylindricalRPPSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[0.0, 0.0, 0.0]);
    let t = result.ee_pose().unwrap().transform().translation;

    assert!(
        t.x.abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "Cylindrical RPP at zero config: expected (0, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn prismatic_z_moves_ee_vertically() {
    // ADR-0001: P(z) moves EE along Z (vertical).
    // At q=[0, dz, 0]: ee = (0, 0, dz)
    let mut spec = CylindricalRPPSpec::ideal();
    spec.joint_limits[1] = JointLimits::new(-2.0, 2.0);
    let robot = spec.build();
    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[0.0, 0.5, 0.0]);
    let t = result.ee_pose().unwrap().transform().translation;
    assert!(
        t.x.abs() < EPS && t.y.abs() < EPS && (t.z - 0.5).abs() < EPS,
        "Cylindrical RPP Pz(0.5): expected (0, 0, 0.5), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn prismatic_x_moves_ee_radially() {
    // ADR-0001: P(x) extends EE along X (radial).
    // At q=[0, 0, r]: ee = (r, 0, 0)
    let mut spec = CylindricalRPPSpec::ideal();
    spec.joint_limits[2] = JointLimits::new(-2.0, 2.0);
    let robot = spec.build();
    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[0.0, 0.0, 1.0]);
    let t = result.ee_pose().unwrap().transform().translation;
    assert!(
        (t.x - 1.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "Cylindrical RPP Px(1.0): expected (1, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn azimuth_rotates_in_xy() {
    // ADR-0001: R(z) rotates arm in XY plane.
    // At q=[π/2, 0, 1]: ee = (cos(π/2), sin(π/2), 0) = (0, 1, 0)
    let mut spec = CylindricalRPPSpec::ideal();
    spec.joint_limits[2] = JointLimits::new(-2.0, 2.0);
    let robot = spec.build();
    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[PI / 2.0, 0.0, 1.0]);
    let t = result.ee_pose().unwrap().transform().translation;
    assert!(
        t.x.abs() < EPS && (t.y - 1.0).abs() < EPS && t.z.abs() < EPS,
        "Cylindrical RPP Rz(90°) with Px(1): expected (0, 1, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}
