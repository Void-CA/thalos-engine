use thalos_math::{Quaternion, Transform3D, UnitQuaternion, Vector3};

/// SLERP that falls back to lerp+normalise for very small angles
/// to avoid division-by-zero in sin(θ).
fn slerp(q1: &UnitQuaternion, q2: &UnitQuaternion, t: f64) -> UnitQuaternion {
    let a = q1.inner();
    let b = q2.inner();

    let mut cos_half_theta = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

    let (bx, by, bz, bw) = if cos_half_theta < 0.0 {
        cos_half_theta = -cos_half_theta;
        (-b.x, -b.y, -b.z, -b.w)
    } else {
        (b.x, b.y, b.z, b.w)
    };

    const THRESHOLD: f64 = 0.9995;

    let (scale0, scale1) = if cos_half_theta > THRESHOLD {
        (1.0 - t, t)
    } else {
        let half_theta = cos_half_theta.acos();
        let sin_half_theta = (1.0_f64 - cos_half_theta * cos_half_theta).sqrt();
        let s0 = ((1.0 - t) * half_theta).sin() / sin_half_theta;
        let s1 = (t * half_theta).sin() / sin_half_theta;
        (s0, s1)
    };

    let result = Quaternion::new(
        scale0 * a.w + scale1 * bw,
        scale0 * a.x + scale1 * bx,
        scale0 * a.y + scale1 * by,
        scale0 * a.z + scale1 * bz,
    );

    if cos_half_theta > THRESHOLD {
        let norm = result.norm();
        if norm > 1e-12 {
            UnitQuaternion::from_quaternion_unchecked(Quaternion::new(
                result.w / norm,
                result.x / norm,
                result.y / norm,
                result.z / norm,
            ))
        } else {
            UnitQuaternion::identity()
        }
    } else {
        UnitQuaternion::from_quaternion_unchecked(result)
    }
}

pub fn lerp_transform(start: &Transform3D, end: &Transform3D, t: f64) -> Transform3D {
    let t = t.clamp(0.0, 1.0);
    let translation = Vector3::new(
        start.translation.x + (end.translation.x - start.translation.x) * t,
        start.translation.y + (end.translation.y - start.translation.y) * t,
        start.translation.z + (end.translation.z - start.translation.z) * t,
    );
    let rotation = slerp(&start.rotation, &end.rotation, t);
    Transform3D {
        translation,
        rotation,
    }
}

pub fn linear_path(start: &Transform3D, end: &Transform3D, step: f64) -> Vec<Transform3D> {
    let dx = end.translation.x - start.translation.x;
    let dy = end.translation.y - start.translation.y;
    let dz = end.translation.z - start.translation.z;
    let distance = (dx * dx + dy * dy + dz * dz).sqrt();

    let effective_step = step.max(1e-12);
    let num_segments = (distance / effective_step).ceil() as usize;
    let num_points = num_segments + 1;

    (0..num_points)
        .map(|i| {
            let t = if num_points > 1 {
                i as f64 / (num_points - 1) as f64
            } else {
                0.0
            };
            lerp_transform(start, end, t)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_math::UnitVector3;

    fn identity_transform() -> Transform3D {
        Transform3D::identity()
    }

    #[test]
    fn slerp_identity_to_identity() {
        let id = UnitQuaternion::identity();
        let result = slerp(&id, &id, 0.5);
        assert!((result.inner().w - 1.0).abs() < 1e-12);
    }

    #[test]
    fn slerp_interpolates_halfway() {
        let qx90 = UnitQuaternion::from_axis_angle(
            UnitVector3::new(Vector3::new(1.0, 0.0, 0.0)).unwrap(),
            std::f64::consts::FRAC_PI_2,
        );
        let id = UnitQuaternion::identity();
        let half = slerp(&id, &qx90, 0.5);
        let v = half.rotate_vector(Vector3::new(0.0, 1.0, 0.0));
        assert!((v.y - (std::f64::consts::FRAC_PI_4).cos()).abs() < 1e-6);
        assert!((v.z - (std::f64::consts::FRAC_PI_4).sin()).abs() < 1e-6);
    }

    #[test]
    fn lerp_transform_midpoint() {
        let start = identity_transform();
        let end = Transform3D {
            translation: Vector3::new(2.0, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
        };
        let mid = lerp_transform(&start, &end, 0.5);
        assert!((mid.translation.x - 1.0).abs() < 1e-12);
        assert!((mid.translation.y).abs() < 1e-12);
    }

    #[test]
    fn linear_path_at_least_two_points() {
        let start = identity_transform();
        let end = Transform3D {
            translation: Vector3::new(1.0, 0.0, 0.0),
            rotation: UnitQuaternion::identity(),
        };
        let path = linear_path(&start, &end, 10.0);
        assert!(path.len() >= 2);
        assert!((path[0].translation.x - 0.0).abs() < 1e-12);
        assert!((path[path.len() - 1].translation.x - 1.0).abs() < 1e-12);
    }
}
