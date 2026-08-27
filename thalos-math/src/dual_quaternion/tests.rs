use super::*;
use crate::{Quaternion, Transform3D, UnitQuaternion, UnitVector3, Vector3, constants::EPS};

// ═══════════════════════════════════════════════════════════════════
// Phase 1: DualNumber arithmetic
// ═══════════════════════════════════════════════════════════════════

#[test]
fn dual_number_mul_identity() {
    let a = DualNumber::new(1.0, 0.0);
    let b = DualNumber::new(3.0, 4.0);
    let r = a * b;
    assert!((r.real - 3.0).abs() < EPS);
    assert!((r.dual - 4.0).abs() < EPS);
}

#[test]
fn dual_number_mul_epsilon_squared_zero() {
    let a = DualNumber::new(1.0, 2.0);
    let b = DualNumber::new(3.0, 4.0);
    let r = a * b;
    // (1 + 2ε)(3 + 4ε) = 3 + (1*4 + 2*3)ε = 3 + 10ε
    // ε² = 0 so no 2*4*ε² term
    assert!((r.real - 3.0).abs() < EPS);
    assert!((r.dual - 10.0).abs() < EPS);
}

#[test]
fn dual_number_add() {
    let a = DualNumber::new(1.0, 2.0);
    let b = DualNumber::new(3.0, 4.0);
    let r = a + b;
    assert!((r.real - 4.0).abs() < EPS);
    assert!((r.dual - 6.0).abs() < EPS);
}

#[test]
fn dual_number_mul_scalar() {
    let a = DualNumber::new(1.0, 2.0);
    let r = a * 3.0;
    assert!((r.real - 3.0).abs() < EPS);
    assert!((r.dual - 6.0).abs() < EPS);
}

#[test]
fn dual_number_partial_eq() {
    let a = DualNumber::new(1.0, 2.0);
    let b = DualNumber::new(1.0, 2.0);
    let c = DualNumber::new(1.0, 3.0);
    assert_eq!(a, b);
    assert_ne!(a, c);
}

// ═══════════════════════════════════════════════════════════════════
// Phase 2: DualQuaternion model
// ═══════════════════════════════════════════════════════════════════

#[test]
fn dq_identity_values() {
    let id = DualQuaternion::identity();
    assert!((id.rotation().w - 1.0).abs() < EPS);
    assert!((id.rotation().x - 0.0).abs() < EPS);
    assert!((id.rotation().y - 0.0).abs() < EPS);
    assert!((id.rotation().z - 0.0).abs() < EPS);
    assert!((id.dual_part().w - 0.0).abs() < EPS);
    assert!((id.dual_part().x - 0.0).abs() < EPS);
    assert!((id.dual_part().y - 0.0).abs() < EPS);
    assert!((id.dual_part().z - 0.0).abs() < EPS);
}

#[test]
fn dq_new_rejects_non_unit() {
    let q_r = Quaternion::new(2.0, 0.0, 0.0, 0.0);
    let q_d = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let result = DualQuaternion::new(q_r, q_d);
    assert!(result.is_err(), "non-unit q_r should be rejected");
}

#[test]
fn dq_new_accepts_unit() {
    let q_r = Quaternion::new(1.0, 0.0, 0.0, 0.0);
    let q_d = Quaternion::new(0.0, 1.0, 0.0, 0.0);
    let result = DualQuaternion::new(q_r, q_d);
    assert!(result.is_ok());
}

#[test]
fn dq_from_rotation_translation_roundtrip() {
    let rotation =
        UnitQuaternion::from_axis_angle(UnitVector3::z_axis(), std::f64::consts::FRAC_PI_2);
    let translation = Vector3::new(1.0, 2.0, 3.0);
    let dq = DualQuaternion::from_rotation_translation(rotation, translation);

    let r_out = dq.rotation();
    assert!((r_out.w - rotation.q.w).abs() < EPS);
    assert!((r_out.x - rotation.q.x).abs() < EPS);
    assert!((r_out.y - rotation.q.y).abs() < EPS);
    assert!((r_out.z - rotation.q.z).abs() < EPS);

    let t_out = dq.translation();
    assert!((t_out.x - translation.x).abs() < EPS);
    assert!((t_out.y - translation.y).abs() < EPS);
    assert!((t_out.z - translation.z).abs() < EPS);
}

// ═══════════════════════════════════════════════════════════════════
// Phase 3: DualQuaternion algebra
// ═══════════════════════════════════════════════════════════════════

#[test]
fn dq_mul_identity_neutral() {
    let dq = DualQuaternion::from_rotation_translation(
        UnitQuaternion::from_axis_angle(UnitVector3::z_axis(), 0.5),
        Vector3::new(1.0, 2.0, 3.0),
    );
    let id = DualQuaternion::identity();
    let r1 = dq * id;
    let r2 = id * dq;

    assert!((r1.rotation().w - dq.rotation().w).abs() < EPS);
    assert!((r2.rotation().w - dq.rotation().w).abs() < EPS);
}

#[test]
fn dq_mul_associativity() {
    let a = DualQuaternion::from_rotation_translation(
        UnitQuaternion::from_axis_angle(UnitVector3::z_axis(), 0.3),
        Vector3::new(1.0, 0.0, 0.0),
    );
    let b = DualQuaternion::from_rotation_translation(
        UnitQuaternion::from_axis_angle(UnitVector3::x_axis(), 0.5),
        Vector3::new(0.0, 1.0, 0.0),
    );
    let c = DualQuaternion::from_rotation_translation(
        UnitQuaternion::from_axis_angle(UnitVector3::y_axis(), 0.7),
        Vector3::new(0.0, 0.0, 1.0),
    );
    let left = (a * b) * c;
    let right = a * (b * c);
    assert!((left.rotation().w - right.rotation().w).abs() < EPS);
    assert!((left.rotation().x - right.rotation().x).abs() < EPS);
    assert!((left.rotation().y - right.rotation().y).abs() < EPS);
    assert!((left.rotation().z - right.rotation().z).abs() < EPS);
}

#[test]
fn dq_conjugate_property() {
    let dq = DualQuaternion::from_rotation_translation(
        UnitQuaternion::from_axis_angle(UnitVector3::z_axis(), 0.5),
        Vector3::new(1.0, 2.0, 3.0),
    );
    let conj = dq.conjugate();
    let product = dq * conj;
    // q* × q = ‖q‖² (scalar dual quaternion)
    // For unit DQ, result should have identity rotation part
    assert!((product.rotation().w - 1.0).abs() < EPS);
}

#[test]
fn dq_norm_is_one_for_unit() {
    let dq = DualQuaternion::from_rotation_translation(
        UnitQuaternion::from_axis_angle(UnitVector3::z_axis(), 0.5),
        Vector3::new(1.0, 2.0, 3.0),
    );
    let n = dq.norm();
    assert!((n - 1.0).abs() < EPS);
}

#[test]
fn dq_normalize_preserves_rotation() {
    let dq = DualQuaternion::from_rotation_translation(
        UnitQuaternion::from_axis_angle(UnitVector3::z_axis(), 0.5),
        Vector3::new(1.0, 2.0, 3.0),
    );
    let normalized = dq.normalize();
    let n = normalized.norm();
    assert!((n - 1.0).abs() < EPS);
    // Translation should be preserved (within tolerance)
    let t = normalized.translation();
    assert!((t.x - 1.0).abs() < EPS);
    assert!((t.y - 2.0).abs() < EPS);
    assert!((t.z - 3.0).abs() < EPS);
}

// ═══════════════════════════════════════════════════════════════════
// Phase 4: DQ ↔ Transform3D round-trip
// ═══════════════════════════════════════════════════════════════════

#[test]
fn dq_transform_roundtrip_identity() {
    let dq = DualQuaternion::identity();
    let t: Transform3D = dq.into();
    let dq2: DualQuaternion = t.into();
    assert!((dq.rotation().w - dq2.rotation().w).abs() < EPS);
    assert!((dq.rotation().x - dq2.rotation().x).abs() < EPS);
    assert!((dq.rotation().y - dq2.rotation().y).abs() < EPS);
    assert!((dq.rotation().z - dq2.rotation().z).abs() < EPS);
}

#[test]
fn dq_transform_roundtrip_arbitrary() {
    let dq = DualQuaternion::from_rotation_translation(
        UnitQuaternion::from_axis_angle(UnitVector3::z_axis(), std::f64::consts::FRAC_PI_2),
        Vector3::new(1.0, 2.0, 3.0),
    );
    let t: Transform3D = dq.into();
    let dq2: DualQuaternion = t.into();

    let err_rot = (dq.rotation().w - dq2.rotation().w).abs()
        + (dq.rotation().x - dq2.rotation().x).abs()
        + (dq.rotation().y - dq2.rotation().y).abs()
        + (dq.rotation().z - dq2.rotation().z).abs();
    assert!(err_rot < 1e-12, "rotation round-trip error: {}", err_rot);

    let t1 = dq.translation();
    let t2 = dq2.translation();
    let err_trans = (t1.x - t2.x).abs() + (t1.y - t2.y).abs() + (t1.z - t2.z).abs();
    assert!(
        err_trans < 1e-12,
        "translation round-trip error: {}",
        err_trans
    );
}

#[test]
fn dq_transform_roundtrip_random() {
    use rand::Rng;
    let mut rng = rand::rng();

    for _ in 0..1000 {
        let axis = UnitVector3::new(Vector3::new(
            rng.random::<f64>() * 2.0 - 1.0,
            rng.random::<f64>() * 2.0 - 1.0,
            rng.random::<f64>() * 2.0 - 1.0,
        ))
        .unwrap();
        let angle = rng.random::<f64>() * std::f64::consts::PI;
        let rotation = UnitQuaternion::from_axis_angle(axis, angle);
        let translation = Vector3::new(
            rng.random::<f64>() * 10.0 - 5.0,
            rng.random::<f64>() * 10.0 - 5.0,
            rng.random::<f64>() * 10.0 - 5.0,
        );

        let dq = DualQuaternion::from_rotation_translation(rotation, translation);
        let t: Transform3D = dq.into();
        let dq2: DualQuaternion = t.into();

        let err_rot = (dq.rotation().w - dq2.rotation().w).abs()
            + (dq.rotation().x - dq2.rotation().x).abs()
            + (dq.rotation().y - dq2.rotation().y).abs()
            + (dq.rotation().z - dq2.rotation().z).abs();
        assert!(err_rot < 1e-12, "rotation round-trip error: {}", err_rot);

        let t1 = dq.translation();
        let t2 = dq2.translation();
        let err_trans = (t1.x - t2.x).abs() + (t1.y - t2.y).abs() + (t1.z - t2.z).abs();
        assert!(
            err_trans < 1e-12,
            "translation round-trip error: {}",
            err_trans
        );
    }
}

// ═══════════════════════════════════════════════════════════════════
// Phase 5: Screw axis extraction
// ═══════════════════════════════════════════════════════════════════

#[test]
fn screw_axis_pure_rotation() {
    // ω = 2·Im(q_r) = 2·sin(θ/2)·â
    // For θ = π/3: 2·sin(π/6) = 1.0
    let angle = std::f64::consts::FRAC_PI_3;
    let rotation = UnitQuaternion::from_axis_angle(UnitVector3::z_axis(), angle);
    let translation = Vector3::zero();
    let dq = DualQuaternion::from_rotation_translation(rotation, translation);

    let (omega, v) = dq.to_screw_axis();
    assert!((omega.x - 0.0).abs() < EPS);
    assert!((omega.y - 0.0).abs() < EPS);
    assert!((omega.z - 1.0).abs() < 1e-6);
    assert!((v.x - 0.0).abs() < EPS);
    assert!((v.y - 0.0).abs() < EPS);
    assert!((v.z - 0.0).abs() < EPS);
}

#[test]
fn screw_axis_pure_translation() {
    let rotation = UnitQuaternion::identity();
    let translation = Vector3::new(1.0, 0.0, 0.0);
    let dq = DualQuaternion::from_rotation_translation(rotation, translation);

    let (omega, v) = dq.to_screw_axis();
    assert!((omega.x - 0.0).abs() < EPS);
    assert!((omega.y - 0.0).abs() < EPS);
    assert!((omega.z - 0.0).abs() < EPS);
    assert!((v.x - 1.0).abs() < 1e-6);
    assert!((v.y - 0.0).abs() < EPS);
    assert!((v.z - 0.0).abs() < EPS);
}

#[test]
fn twist_struct_matches_screw_axis() {
    let rotation =
        UnitQuaternion::from_axis_angle(UnitVector3::z_axis(), std::f64::consts::FRAC_PI_3);
    let translation = Vector3::new(1.0, 2.0, 3.0);
    let dq = DualQuaternion::from_rotation_translation(rotation, translation);

    let (omega, v) = dq.to_screw_axis();
    let twist = dq.to_twist();

    assert!((twist.angular.x - omega.x).abs() < EPS);
    assert!((twist.angular.y - omega.y).abs() < EPS);
    assert!((twist.angular.z - omega.z).abs() < EPS);
    assert!((twist.linear.x - v.x).abs() < EPS);
    assert!((twist.linear.y - v.y).abs() < EPS);
    assert!((twist.linear.z - v.z).abs() < EPS);
}
