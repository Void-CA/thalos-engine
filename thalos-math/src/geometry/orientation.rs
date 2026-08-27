use crate::{UnitQuaternion, Vector3};

/// Compute the orientation error vector from the current orientation
/// to the target orientation.
///
/// Returns the so(3) tangent vector ω = log(q_target · q_current⁻¹),
/// representing the orientation deviation as a rotation vector.
/// For identical orientations, returns the zero vector.
///
/// This is the exact orientation error via the SO(3) logarithmic map,
/// replacing the small-angle approximation `ω ≈ 2·imag(q_rel)`.
pub fn orientation_error(target: &UnitQuaternion, current: &UnitQuaternion) -> Vector3 {
    let q_rel = *target * current.inverse();
    q_rel.log()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{UnitVector3, Vector3, constants::EPS};

    #[test]
    fn same_orientation_returns_zero() {
        let axis = UnitVector3::new_normalize(Vector3::new(1.0, 0.0, 0.0));
        let q = UnitQuaternion::from_axis_angle(axis, 0.5);
        let err = orientation_error(&q, &q);
        assert!(err.magnitude() < EPS);
    }

    #[test]
    fn orientation_error_magnitude_matches_angle() {
        let axis = UnitVector3::new_normalize(Vector3::new(1.0, 0.0, 0.0));
        let q_current = UnitQuaternion::identity();
        let q_target = UnitQuaternion::from_axis_angle(axis, std::f64::consts::FRAC_PI_4);
        let err = orientation_error(&q_target, &q_current);
        let angle_error = err.magnitude();
        assert!((angle_error - std::f64::consts::FRAC_PI_4).abs() < 1e-6);
    }
}
