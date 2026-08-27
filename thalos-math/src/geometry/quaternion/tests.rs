use super::*;
use crate::constants::EPS;

fn approx_eq(a: &Quaternion, b: &Quaternion, tol: f64) -> bool {
    (a.w - b.w).abs() < tol
        && (a.x - b.x).abs() < tol
        && (a.y - b.y).abs() < tol
        && (a.z - b.z).abs() < tol
}

fn is_pure_scalar(q: &Quaternion, tol: f64) -> bool {
    q.x.abs() < tol && q.y.abs() < tol && q.z.abs() < tol
}

#[test]
fn identity_is_multiplicative_neutral() {
    let q = Quaternion::new(2.0, -1.0, 3.0, 0.5);
    let id = Quaternion::identity();
    let r1 = q * id;
    assert!(approx_eq(&r1, &q, EPS), "q * identity != q");
    let r2 = id * q;
    assert!(approx_eq(&r2, &q, EPS), "identity * q != q");
}

#[test]
fn inverse_property() {
    let q = Quaternion::new(0.8, -0.2, 0.3, 0.1);
    let inv = q.inverse().unwrap();
    let r1 = q * inv;
    assert!(is_pure_scalar(&r1, EPS), "q * q⁻¹ should be pure scalar");
    assert!((r1.w - 1.0).abs() < 10.0 * EPS, "scalar part should be ≈ 1");
}

#[test]
fn conjugate_definition() {
    let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    let conj = q.conjugate();
    assert_eq!(conj.w, 1.0);
    assert_eq!(conj.x, -2.0);
    assert_eq!(conj.y, -3.0);
    assert_eq!(conj.z, -4.0);
}

#[test]
fn norm_is_multiplicative() {
    let q1 = Quaternion::new(2.0, -0.5, 1.5, 0.3);
    let q2 = Quaternion::new(0.7, 1.2, -0.8, 0.1);
    let diff = ((q1 * q2).norm() - q1.norm() * q2.norm()).abs();
    assert!(diff < 10.0 * EPS);
}

#[test]
fn normalize_produces_unit_norm() {
    let q = Quaternion::new(3.0, -1.5, 2.0, 0.5);
    let n = q.normalize().unwrap();
    assert!((n.norm() - 1.0).abs() < EPS);
}

#[test]
fn identity_norm_is_one() {
    assert!((Quaternion::identity().norm() - 1.0).abs() < EPS);
}

#[test]
fn associativity_of_product() {
    let q1 = Quaternion::new(0.3, -0.7, 0.2, 1.1);
    let q2 = Quaternion::new(0.8, 0.5, -0.4, 0.6);
    let q3 = Quaternion::new(1.2, -0.3, 0.9, -0.5);
    let left = (q1 * q2) * q3;
    let right = q1 * (q2 * q3);
    assert!(approx_eq(&left, &right, 10.0 * EPS));
}
