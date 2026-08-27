//! Distance queries between collision geometries.
//!
//! Provides minimum-distance computation between positioned geometries,
//! extending the existing boolean intersection tests with continuous
//! distance metrics.
//!
//! # Supported pairs
//!
//! - Sphere–Sphere: exact center distance minus radii sum
//! - Sphere–Box: distance from sphere center to closest point on box surface
//! - Box–Box: SAT-based separation distance (approximate)
//!
//! Unsupported pairs (e.g. Cylinder) return `f64::INFINITY`.

use thalos_core::collision::CollisionGeometry;
use thalos_math::{Cross, Dot, Transform3D, Vector3};

/// Epsilon for distance comparisons.
const DIST_EPS: f64 = 1e-9;

/// Minimum distance between two positioned geometries.
///
/// Returns:
/// - `d > 0`  — geometries are separated by distance `d`
/// - `d ≈ 0`  — geometries are touching (within epsilon)
/// - `d < 0`  — geometries are intersecting (penetration depth ≈ -d)
/// - `∞`     — pair type not supported
pub fn geometries_distance(
    geo_a: &CollisionGeometry,
    pose_a: &Transform3D,
    geo_b: &CollisionGeometry,
    pose_b: &Transform3D,
) -> f64 {
    match (geo_a, geo_b) {
        (CollisionGeometry::Sphere(a), CollisionGeometry::Sphere(b)) => {
            sphere_sphere_distance(a.radius, pose_a, b.radius, pose_b)
        }
        (CollisionGeometry::Sphere(s), CollisionGeometry::Box(b)) => {
            sphere_box_distance(s.radius, pose_a, b.half_extents, pose_b)
        }
        (CollisionGeometry::Box(b), CollisionGeometry::Sphere(s)) => {
            sphere_box_distance(s.radius, pose_b, b.half_extents, pose_a)
        }
        (CollisionGeometry::Box(a), CollisionGeometry::Box(b)) => {
            box_box_separation(a.half_extents, pose_a, b.half_extents, pose_b)
        }
        _ => f64::INFINITY,
    }
}

/// Distance between two spheres: `||c₁ - c₂|| - (r₁ + r₂)`.
fn sphere_sphere_distance(r1: f64, pose1: &Transform3D, r2: f64, pose2: &Transform3D) -> f64 {
    let delta = pose1.translation - pose2.translation;
    let center_dist = delta.magnitude();
    center_dist - (r1 + r2)
}

/// Signed distance from a sphere to a box.
///
/// Positive = separated, zero = touching, negative = penetrating.
fn sphere_box_distance(
    sphere_radius: f64,
    sphere_pose: &Transform3D,
    box_he: Vector3,
    box_pose: &Transform3D,
) -> f64 {
    let center = sphere_pose.translation - box_pose.translation;
    let inv_rot = box_pose.rotation.inverse();
    let local_center = inv_rot.rotate_vector(center);

    // Closest point on the box surface (or inside) to the sphere center
    let closest = Vector3::new(
        local_center.x.clamp(-box_he.x, box_he.x),
        local_center.y.clamp(-box_he.y, box_he.y),
        local_center.z.clamp(-box_he.z, box_he.z),
    );

    let delta = local_center - closest;
    let dist = delta.magnitude();

    if dist < DIST_EPS {
        // Sphere center is inside the box — negative distance (penetration)
        // Fall back to distance to nearest face.
        let face_dists = vec![
            box_he.x - local_center.x.abs(),
            box_he.y - local_center.y.abs(),
            box_he.z - local_center.z.abs(),
        ];
        -face_dists.into_iter().fold(f64::INFINITY, f64::min)
    } else {
        dist - sphere_radius
    }
}

/// Minimum separation distance between two oriented boxes using SAT.
///
/// Returns the minimum translation to separate the boxes (positive if
/// separated, negative if penetrating).
fn box_box_separation(
    he_a: Vector3,
    pose_a: &Transform3D,
    he_b: Vector3,
    pose_b: &Transform3D,
) -> f64 {
    let axes_a = obb_axes(&pose_a.rotation);
    let axes_b = obb_axes(&pose_b.rotation);
    let sat_axes = sat_axes(&axes_a, &axes_b);

    let center = pose_b.translation - pose_a.translation;
    let mut min_separation = f64::INFINITY;

    for axis in &sat_axes {
        let proj_a = obb_projection_radius(&axes_a, he_a, axis);
        let proj_b = obb_projection_radius(&axes_b, he_b, axis);
        let center_proj = center.dot(*axis).abs();

        let overlap = proj_a + proj_b - center_proj;
        if overlap < -DIST_EPS {
            // Separated along this axis — this is the separation distance
            return -overlap; // positive separation
        }
        // Overlapping along this axis — track minimum overlap (penetration)
        if overlap < min_separation {
            min_separation = overlap;
        }
    }

    // All axes overlapping: boxes are intersecting
    -min_separation
}

/// OBB local axes in global frame.
fn obb_axes(rotation: &thalos_math::UnitQuaternion) -> [Vector3; 3] {
    let x = rotation.rotate_vector(Vector3::new(1.0, 0.0, 0.0));
    let y = rotation.rotate_vector(Vector3::new(0.0, 1.0, 0.0));
    let z = rotation.rotate_vector(Vector3::new(0.0, 0.0, 1.0));
    [x, y, z]
}

/// Projection radius of an OBB onto an axis: Σ|h_i · (axis · axis_i)|.
fn obb_projection_radius(axes: &[Vector3; 3], half_extents: Vector3, test_axis: &Vector3) -> f64 {
    half_extents.x * axes[0].dot(*test_axis).abs()
        + half_extents.y * axes[1].dot(*test_axis).abs()
        + half_extents.z * axes[2].dot(*test_axis).abs()
}

/// SAT axes (3 face + 3 face + 9 edge = 15 max).
fn sat_axes(axes_a: &[Vector3; 3], axes_b: &[Vector3; 3]) -> Vec<Vector3> {
    let mut axes = Vec::with_capacity(15);
    axes.extend_from_slice(axes_a);
    axes.extend_from_slice(axes_b);

    let cross_eps = 1e-12;
    for i in 0..3 {
        for j in 0..3 {
            let cross = axes_a[i].cross(axes_b[j]);
            if cross.dot(cross) > cross_eps {
                axes.push(cross);
            }
        }
    }
    axes
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::collision::{Box3D, Sphere};

    fn dist(
        geo_a: CollisionGeometry,
        pose_a: Transform3D,
        geo_b: CollisionGeometry,
        pose_b: Transform3D,
    ) -> f64 {
        geometries_distance(&geo_a, &pose_a, &geo_b, &pose_b)
    }

    #[test]
    fn sphere_sphere_separated() {
        let a = CollisionGeometry::Sphere(Sphere::new(1.0));
        let b = CollisionGeometry::Sphere(Sphere::new(1.0));
        let d = dist(
            a,
            Transform3D::identity(),
            b,
            Transform3D::from_translation(Vector3::new(4.0, 0.0, 0.0)),
        );
        assert!((d - 2.0).abs() < 1e-9, "expected 2.0, got {}", d);
    }

    #[test]
    fn sphere_sphere_touching() {
        let a = CollisionGeometry::Sphere(Sphere::new(1.0));
        let b = CollisionGeometry::Sphere(Sphere::new(1.0));
        let d = dist(
            a,
            Transform3D::identity(),
            b,
            Transform3D::from_translation(Vector3::new(2.0, 0.0, 0.0)),
        );
        assert!(d.abs() < 1e-9, "expected ~0, got {}", d);
    }

    #[test]
    fn sphere_sphere_intersecting() {
        let a = CollisionGeometry::Sphere(Sphere::new(1.0));
        let b = CollisionGeometry::Sphere(Sphere::new(1.0));
        let d = dist(
            a,
            Transform3D::identity(),
            b,
            Transform3D::from_translation(Vector3::new(1.5, 0.0, 0.0)),
        );
        assert!(d < 0.0, "expected negative, got {}", d);
    }

    #[test]
    fn sphere_box_separated() {
        let s = CollisionGeometry::Sphere(Sphere::new(0.5));
        let b = CollisionGeometry::Box(Box3D::new(1.0, 1.0, 1.0));
        let d = dist(
            s,
            Transform3D::from_translation(Vector3::new(3.0, 0.0, 0.0)),
            b,
            Transform3D::identity(),
        );
        assert!(d > 1.0, "expected separation > 1.0, got {}", d);
    }

    #[test]
    fn sphere_box_touching() {
        let s = CollisionGeometry::Sphere(Sphere::new(0.5));
        let b = CollisionGeometry::Box(Box3D::new(1.0, 1.0, 1.0));
        // Box extends from -0.5 to 0.5 on each axis, sphere center at (1.0, 0, 0)
        // distance = 1.0 - 0.5 - 0.5 = 0.0
        let d = dist(
            s,
            Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0)),
            b,
            Transform3D::identity(),
        );
        assert!(d.abs() < 1e-9, "expected ~0, got {}", d);
    }
}
