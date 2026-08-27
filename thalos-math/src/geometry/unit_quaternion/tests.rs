use super::*;
use crate::{Quaternion, UnitVector3, Vector3, constants::EPS};

fn uq(w: f64, x: f64, y: f64, z: f64) -> UnitQuaternion {
    let q = Quaternion::new(w, x, y, z);
    let norm = q.norm();
    UnitQuaternion {
        q: Quaternion::new(w / norm, x / norm, y / norm, z / norm),
    }
}

#[test]
fn unit_norm_invariant() {
    let uq = uq(0.707, 0.707, 0.0, 0.0);
    assert!((uq.inner().norm() - 1.0).abs() < 1e-3);
}

#[test]
fn euler_roundtrip() {
    let original = (0.3, -0.5, 0.8);
    let q = UnitQuaternion::from_euler_angles(original.0, original.1, original.2);
    let result = q.to_euler_angles();
    assert!((result.0 - original.0).abs() < EPS);
    assert!((result.1 - original.1).abs() < EPS);
    assert!((result.2 - original.2).abs() < EPS);
}

#[test]
fn rotate_vector_preserves_length() {
    let q = uq(0.707, 0.0, 0.707, 0.0);
    let v = Vector3::new(1.0, 0.0, 0.0);
    let r = q.rotate_vector(v);
    assert!((r.magnitude() - 1.0).abs() < EPS);
}

#[test]
fn slerp_identity_at_t0() {
    let a = uq(1.0, 0.0, 0.0, 0.0);
    let b = uq(0.0, 1.0, 0.0, 0.0);
    let r = a.slerp(&b, 0.0);
    assert!((r.inner().w - 1.0).abs() < EPS);
}

#[test]
fn slerp_identity_at_t1() {
    let a = uq(1.0, 0.0, 0.0, 0.0);
    let b = uq(0.0, 1.0, 0.0, 0.0);
    let r = a.slerp(&b, 1.0);
    assert!((r.inner().w - 0.0).abs() < EPS);
    assert!((r.inner().x - 1.0).abs() < EPS);
}

#[test]
fn from_axis_angle_roundtrip() {
    let axis = UnitVector3::new_normalize(Vector3::new(1.0, 0.0, 0.0));
    let q = UnitQuaternion::from_axis_angle(axis, 0.5);
    let (roll, pitch, yaw) = q.to_euler_angles();
    assert!((roll - 0.5).abs() < EPS);
    assert!((pitch).abs() < EPS);
    assert!((yaw).abs() < EPS);
}

#[test]
fn rotation_between_parallel_vectors() {
    let a = Vector3::new(1.0, 0.0, 0.0);
    let b = Vector3::new(2.0, 0.0, 0.0);
    let q = UnitQuaternion::rotation_between(a, b);
    let rotated = q.rotate_vector(a);
    assert!((rotated - Vector3::new(1.0, 0.0, 0.0)).magnitude() < EPS);
}

#[test]
fn rotation_between_orthogonal_vectors() {
    let a = Vector3::new(1.0, 0.0, 0.0);
    let b = Vector3::new(0.0, 1.0, 0.0);
    let q = UnitQuaternion::rotation_between(a, b);
    let rotated = q.rotate_vector(a);
    assert!(
        (rotated - b).magnitude() < EPS,
        "rotated ({:.4}, {:.4}, {:.4}) != target ({:.4}, {:.4}, {:.4})",
        rotated.x,
        rotated.y,
        rotated.z,
        b.x,
        b.y,
        b.z
    );
}

// ── angle() tests ─────────────────────────────────────

#[test]
fn angle_identity_returns_zero() {
    let q = UnitQuaternion::identity();
    assert!((q.angle() - 0.0).abs() < EPS);
}

#[test]
fn angle_90_degrees_returns_pi_over_2() {
    let axis = UnitVector3::new_normalize(Vector3::x_axis());
    let q = UnitQuaternion::from_axis_angle(axis, std::f64::consts::FRAC_PI_2);
    assert!((q.angle() - std::f64::consts::FRAC_PI_2).abs() < EPS);
}

#[test]
fn angle_180_degrees_returns_pi() {
    let axis = UnitVector3::new_normalize(Vector3::x_axis());
    let q = UnitQuaternion::from_axis_angle(axis, std::f64::consts::PI);
    assert!((q.angle() - std::f64::consts::PI).abs() < EPS);
}

// ── log() tests ────────────────────────────────────────

#[test]
fn log_identity_returns_zero() {
    let q = UnitQuaternion::identity();
    let v = q.log();
    assert!((v.magnitude() - 0.0).abs() < EPS);
}

#[test]
fn log_90_degrees_about_x_returns_correct_vector() {
    let axis = UnitVector3::new_normalize(Vector3::x_axis());
    let q = UnitQuaternion::from_axis_angle(axis, std::f64::consts::FRAC_PI_2);
    let v = q.log();
    assert!((v.x - std::f64::consts::FRAC_PI_2).abs() < EPS);
    assert!(v.y.abs() < EPS);
    assert!(v.z.abs() < EPS);
}

#[test]
fn log_round_trip_exp_log() {
    let axis = UnitVector3::new_normalize(Vector3::new(1.0, 2.0, 3.0));
    let angle = 0.75;
    let q = UnitQuaternion::from_axis_angle(axis, angle);
    let v = q.log();
    let q2 = UnitQuaternion::exp_map(&v);
    // Compare quaternion components
    assert!((q.inner().w - q2.inner().w).abs() < EPS);
    assert!((q.inner().x - q2.inner().x).abs() < EPS);
    assert!((q.inner().y - q2.inner().y).abs() < EPS);
    assert!((q.inner().z - q2.inner().z).abs() < EPS);
}

// ── exp_map() tests ────────────────────────────────────

#[test]
fn exp_map_zero_returns_identity() {
    let v = Vector3::zero();
    let q = UnitQuaternion::exp_map(&v);
    assert!((q.inner().w - 1.0).abs() < EPS);
    assert!(q.inner().x.abs() < EPS);
    assert!(q.inner().y.abs() < EPS);
    assert!(q.inner().z.abs() < EPS);
}

#[test]
fn exp_map_round_trip_log_exp() {
    let v = Vector3::new(0.5, -0.3, 0.8);
    let q = UnitQuaternion::exp_map(&v);
    let v2 = q.log();
    assert!((v.x - v2.x).abs() < EPS);
    assert!((v.y - v2.y).abs() < EPS);
    assert!((v.z - v2.z).abs() < EPS);
}

// ── orientation_error() tests ─────────────────────────

#[test]
fn orientation_error_same_orientation_returns_zero() {
    let axis = UnitVector3::new_normalize(Vector3::new(1.0, 0.0, 0.0));
    let q = UnitQuaternion::from_axis_angle(axis, 0.5);
    let err = crate::orientation_error(&q, &q);
    assert!(err.magnitude() < EPS);
}

#[test]
fn orientation_error_magnitude_matches_angle() {
    let axis = UnitVector3::new_normalize(Vector3::new(1.0, 0.0, 0.0));
    let q_current = UnitQuaternion::identity();
    let q_target = UnitQuaternion::from_axis_angle(axis, std::f64::consts::FRAC_PI_4);
    let err = crate::orientation_error(&q_target, &q_current);
    let angle_error = err.magnitude();
    assert!((angle_error - std::f64::consts::FRAC_PI_4).abs() < 1e-6);
}
