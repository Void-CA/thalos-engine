use crate::models::spherical_polar_rrp::SphericalPolarRRPSpec;
use crate::prelude::*;

// ─── ADR-0001 Z-up regression tests ──────────────────────────

#[test]
fn zero_config_ee_at_origin() {
    // ADR-0001: Z is vertical. Spherical polar RRP: R(z), R(y), P(x).
    // At q=[0,0,0] with ideal l1=0: ee = (0, 0, 0).
    let robot = SphericalPolarRRPSpec::ideal().build();
    let fk = ForwardKinematics::new(robot);
    let result = fk.evaluate(&[0.0, 0.0, 0.0]);
    let t = result.ee_pose().unwrap().transform().translation;

    assert!(
        t.x.abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "Spherical polar RRP at zero config: expected (0, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn radial_extension_in_x() {
    // ADR-0001: P(x) extends EE along X (radial).
    // At q=[0, 0, r]: ee = (r, 0, 0) — Z stays 0.
    let mut spec = SphericalPolarRRPSpec::ideal();
    spec.joint_limits[2] = JointLimits::new(-2.0, 2.0);
    let robot = spec.build();
    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[0.0, 0.0, 1.0]);
    let t = result.ee_pose().unwrap().transform().translation;
    assert!(
        (t.x - 1.0).abs() < EPS && t.y.abs() < EPS && t.z.abs() < EPS,
        "Spherical polar Px(1.0): expected (1, 0, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn azimuth_rotates_in_xy() {
    // ADR-0001: R(z) rotates arm in XY plane (horizontal).
    // At q=[π/2, 0, 1]: ee = (0, 1, 0).
    let mut spec = SphericalPolarRRPSpec::ideal();
    spec.joint_limits[2] = JointLimits::new(-2.0, 2.0);
    let robot = spec.build();
    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[PI / 2.0, 0.0, 1.0]);
    let t = result.ee_pose().unwrap().transform().translation;
    assert!(
        t.x.abs() < EPS && (t.y - 1.0).abs() < EPS && t.z.abs() < EPS,
        "Spherical polar Rz(90°) with Px(1): expected (0, 1, 0), got ({}, {}, {})",
        t.x,
        t.y,
        t.z
    );
}

#[test]
fn polar_tilt_moves_ee_in_xz() {
    // ADR-0001: R(y) tilts arm in XZ plane (vertical plane).
    // At q=[0, π/4, 1]: Ry(π/4)·(1,0,0) = (√2/2, 0, -√2/2)
    // ee = (cos(π/4), 0, -sin(π/4)) ≈ (0.7071, 0, -0.7071)
    let mut spec = SphericalPolarRRPSpec::ideal();
    spec.joint_limits[2] = JointLimits::new(-2.0, 2.0);
    let robot = spec.build();
    let fk = ForwardKinematics::new(robot);

    let result = fk.evaluate(&[0.0, PI / 4.0, 1.0]);
    let t = result.ee_pose().unwrap().transform().translation;

    let expected_x = (PI / 4.0).cos();
    let expected_z = -(PI / 4.0).sin();

    assert!(
        (t.x - expected_x).abs() < EPS && t.y.abs() < EPS && (t.z - expected_z).abs() < EPS,
        "Spherical polar Ry(45°) with Px(1): expected ({:.4}, 0, {:.4}), got ({:.4}, {:.4}, {:.4})",
        expected_x,
        expected_z,
        t.x,
        t.y,
        t.z
    );
}
