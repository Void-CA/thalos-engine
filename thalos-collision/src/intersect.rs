use thalos_core::collision::CollisionGeometry;
use thalos_math::{Cross, Dot, Transform3D, Vector3};

/// Epsilon global para comparaciones de punto flotante en detección
/// de colisiones. Absorbe errores de redondeo en transformaciones y FK.
const COLLISION_EPS: f64 = 1e-9;

/// Determina si dos geometrías posicionadas se intersectan.
pub fn geometries_intersect(
    geo_a: &CollisionGeometry,
    pose_a: &Transform3D,
    geo_b: &CollisionGeometry,
    pose_b: &Transform3D,
) -> bool {
    match (geo_a, geo_b) {
        (CollisionGeometry::Sphere(a), CollisionGeometry::Sphere(b)) => {
            sphere_vs_sphere(a.radius, pose_a, b.radius, pose_b)
        }
        (CollisionGeometry::Box(a), CollisionGeometry::Box(b)) => {
            box_vs_box(a.half_extents, pose_a, b.half_extents, pose_b)
        }
        (CollisionGeometry::Sphere(s), CollisionGeometry::Box(b)) => {
            sphere_vs_box(s.radius, pose_a, b.half_extents, pose_b)
        }
        (CollisionGeometry::Box(b), CollisionGeometry::Sphere(s)) => {
            sphere_vs_box(s.radius, pose_b, b.half_extents, pose_a)
        }
        _ => false,
    }
}

// ─── Sphere-Sphere ───────────────────────────────────────────────

fn sphere_vs_sphere(r1: f64, pose1: &Transform3D, r2: f64, pose2: &Transform3D) -> bool {
    let delta = pose1.translation - pose2.translation;
    let dist_sq = delta.dot(delta);
    let radius_sum = r1 + r2;
    dist_sq <= radius_sum * radius_sum + COLLISION_EPS
}

// ─── Box-Box (SAT) ──────────────────────────────────────────────

fn box_vs_box(he_a: Vector3, pose_a: &Transform3D, he_b: Vector3, pose_b: &Transform3D) -> bool {
    let axes_a = obb_axes(&pose_a.rotation);
    let axes_b = obb_axes(&pose_b.rotation);

    for axis in sat_axes(&axes_a, &axes_b) {
        let proj_a = obb_projection_radius(&axes_a, he_a, &axis);
        let proj_b = obb_projection_radius(&axes_b, he_b, &axis);

        let center = pose_b.translation - pose_a.translation;
        let center_proj = center.dot(axis).abs();

        if center_proj > proj_a + proj_b + COLLISION_EPS {
            return false;
        }
    }

    true
}

/// Retorna los 3 ejes locales del OBB en el marco global.
fn obb_axes(rotation: &thalos_math::UnitQuaternion) -> [Vector3; 3] {
    let x = rotation.rotate_vector(Vector3::new(1.0, 0.0, 0.0));
    let y = rotation.rotate_vector(Vector3::new(0.0, 1.0, 0.0));
    let z = rotation.rotate_vector(Vector3::new(0.0, 0.0, 1.0));
    [x, y, z]
}

/// Radio de proyección de un OBB sobre un eje.
///
/// Equivale a: Σ |half_extents[i] · dot(axis_i, test_axis)|
fn obb_projection_radius(axes: &[Vector3; 3], half_extents: Vector3, test_axis: &Vector3) -> f64 {
    let h = half_extents;
    h.x * axes[0].dot(*test_axis).abs()
        + h.y * axes[1].dot(*test_axis).abs()
        + h.z * axes[2].dot(*test_axis).abs()
}

/// Genera los 15 ejes de prueba para SAT entre dos OBBs.
///
/// Omite ejes degenerados (producto cruz con magnitud casi cero).
///
/// NOTA sobre normalización: SAT funciona correctamente con ejes no
/// normalizados siempre que proj_a, proj_b y center_proj se calculen
/// contra el MISMO vector. Si en el futuro se reusan estos ejes para
/// distancia de penetración, habrá que normalizar.
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

// ─── Sphere-Box ─────────────────────────────────────────────────

fn sphere_vs_box(
    sphere_radius: f64,
    sphere_pose: &Transform3D,
    box_he: Vector3,
    box_pose: &Transform3D,
) -> bool {
    let center = sphere_pose.translation - box_pose.translation;

    let inv_rot = box_pose.rotation.inverse();
    let local_center = inv_rot.rotate_vector(center);

    let closest = Vector3::new(
        local_center.x.clamp(-box_he.x, box_he.x),
        local_center.y.clamp(-box_he.y, box_he.y),
        local_center.z.clamp(-box_he.z, box_he.z),
    );

    let delta = local_center - closest;
    delta.dot(delta) <= sphere_radius * sphere_radius + COLLISION_EPS
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::collision::{Box3D, Sphere};

    fn check(
        geo_a: CollisionGeometry,
        pose_a: Transform3D,
        geo_b: CollisionGeometry,
        pose_b: Transform3D,
    ) -> bool {
        geometries_intersect(&geo_a, &pose_a, &geo_b, &pose_b)
    }

    #[test]
    fn sphere_sphere_no_contact() {
        let a = CollisionGeometry::Sphere(Sphere::new(1.0));
        let b = CollisionGeometry::Sphere(Sphere::new(1.0));
        assert!(!check(
            a,
            Transform3D::identity(),
            b,
            Transform3D::from_translation(Vector3::new(3.0, 0.0, 0.0)),
        ));
    }

    #[test]
    fn sphere_sphere_contact() {
        let a = CollisionGeometry::Sphere(Sphere::new(1.0));
        let b = CollisionGeometry::Sphere(Sphere::new(1.0));
        assert!(check(
            a,
            Transform3D::identity(),
            b,
            Transform3D::from_translation(Vector3::new(1.5, 0.0, 0.0)),
        ));
    }

    #[test]
    fn box_box_no_contact() {
        let a = CollisionGeometry::Box(Box3D::new(1.0, 1.0, 1.0));
        let b = CollisionGeometry::Box(Box3D::new(1.0, 1.0, 1.0));
        assert!(!check(
            a,
            Transform3D::identity(),
            b,
            Transform3D::from_translation(Vector3::new(2.0, 0.0, 0.0)),
        ));
    }

    #[test]
    fn box_box_contact() {
        let a = CollisionGeometry::Box(Box3D::new(2.0, 2.0, 2.0));
        let b = CollisionGeometry::Box(Box3D::new(2.0, 2.0, 2.0));
        assert!(check(
            a,
            Transform3D::identity(),
            b,
            Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0)),
        ));
    }

    #[test]
    fn sphere_box_contact() {
        assert!(check(
            CollisionGeometry::Sphere(Sphere::new(0.5)),
            Transform3D::identity(),
            CollisionGeometry::Box(Box3D::new(2.0, 2.0, 2.0)),
            Transform3D::identity(),
        ));
    }

    #[test]
    fn sphere_box_no_contact() {
        assert!(!check(
            CollisionGeometry::Sphere(Sphere::new(0.5)),
            Transform3D::from_translation(Vector3::new(5.0, 0.0, 0.0)),
            CollisionGeometry::Box(Box3D::new(2.0, 2.0, 2.0)),
            Transform3D::identity(),
        ));
    }

    #[test]
    fn cylinder_pair_returns_false() {
        let cyl = CollisionGeometry::Cylinder(thalos_core::collision::Cylinder::new(1.0, 2.0));
        assert!(!check(
            cyl.clone(),
            Transform3D::identity(),
            cyl,
            Transform3D::from_translation(Vector3::new(0.0, 0.0, 0.0)),
        ));
    }
}
