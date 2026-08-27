use thalos_core::collision::{
    CollisionBody, CollisionChecker, CollisionMatrix, CollisionPair, CollisionResult, EntityId,
};

use crate::classify::classify_collision;
use crate::intersect::geometries_intersect;

/// Detector de colisiones O(n²) sin optimizaciones.
///
/// Implementa detección exacta para:
/// - Sphere vs Sphere
/// - Box vs Box (OBB via Separating Axis Theorem)
///
/// Los pares que involucran geometrías no soportadas se ignoran
/// (no se reportan como colisión).
pub struct NaiveCollisionChecker;

impl CollisionChecker for NaiveCollisionChecker {
    fn check(&self, bodies: &[CollisionBody], matrix: &CollisionMatrix) -> CollisionResult {
        let mut collisions = Vec::new();

        for i in 0..bodies.len() {
            for j in (i + 1)..bodies.len() {
                let a = &bodies[i];
                let b = &bodies[j];

                if let (EntityId::Link(la), EntityId::Link(lb)) = (&a.entity, &b.entity) {
                    if matrix.is_ignored(*la, *lb) {
                        continue;
                    }
                }

                if geometries_intersect(&a.geometry, &a.pose, &b.geometry, &b.pose) {
                    collisions.push(CollisionPair::new(
                        a.entity.clone(),
                        b.entity.clone(),
                        classify_collision(&a.entity, &b.entity),
                    ));
                }
            }
        }

        CollisionResult::new(collisions)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::collision::{Box3D, CollisionGeometry, CollisionType, Sphere};
    use thalos_math::{Transform3D, Vector3};

    fn body(geometry: CollisionGeometry, pose: Transform3D) -> CollisionBody {
        CollisionBody::new(EntityId::Link(0), geometry, pose)
    }

    #[test]
    fn spheres_not_intersecting() {
        let a = body(
            CollisionGeometry::Sphere(Sphere::new(1.0)),
            Transform3D::identity(),
        );
        let b = body(
            CollisionGeometry::Sphere(Sphere::new(1.0)),
            Transform3D::from_translation(Vector3::new(3.0, 0.0, 0.0)),
        );
        let result = NaiveCollisionChecker.check(&[a, b], &CollisionMatrix::new());
        assert!(result.is_empty());
    }

    #[test]
    fn spheres_intersecting() {
        let a = body(
            CollisionGeometry::Sphere(Sphere::new(1.0)),
            Transform3D::identity(),
        );
        let b = body(
            CollisionGeometry::Sphere(Sphere::new(1.0)),
            Transform3D::from_translation(Vector3::new(1.5, 0.0, 0.0)),
        );
        let result = NaiveCollisionChecker.check(&[a, b], &CollisionMatrix::new());
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn boxes_not_intersecting() {
        let a = body(
            CollisionGeometry::Box(Box3D::new(1.0, 1.0, 1.0)),
            Transform3D::identity(),
        );
        let b = body(
            CollisionGeometry::Box(Box3D::new(1.0, 1.0, 1.0)),
            Transform3D::from_translation(Vector3::new(2.0, 0.0, 0.0)),
        );
        let result = NaiveCollisionChecker.check(&[a, b], &CollisionMatrix::new());
        assert!(result.is_empty());
    }

    #[test]
    fn boxes_intersecting() {
        let a = body(
            CollisionGeometry::Box(Box3D::new(2.0, 2.0, 2.0)),
            Transform3D::identity(),
        );
        let b = body(
            CollisionGeometry::Box(Box3D::new(2.0, 2.0, 2.0)),
            Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0)),
        );
        let result = NaiveCollisionChecker.check(&[a, b], &CollisionMatrix::new());
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn ignored_pairs_are_skipped() {
        let a_link1 = CollisionBody::new(
            EntityId::Link(1),
            CollisionGeometry::Sphere(Sphere::new(1.0)),
            Transform3D::identity(),
        );
        let b_link2 = CollisionBody::new(
            EntityId::Link(2),
            CollisionGeometry::Sphere(Sphere::new(1.0)),
            Transform3D::from_translation(Vector3::new(0.5, 0.0, 0.0)),
        );

        let mut matrix = CollisionMatrix::new();
        matrix.ignore(1, 2);

        let result = NaiveCollisionChecker.check(&[a_link1, b_link2], &matrix);
        assert!(result.is_empty(), "ignored pair should not collide");
    }

    #[test]
    fn self_collision_classified_for_links() {
        let a = CollisionBody::new(
            EntityId::Link(1),
            CollisionGeometry::Sphere(Sphere::new(1.0)),
            Transform3D::identity(),
        );
        let b = CollisionBody::new(
            EntityId::Link(2),
            CollisionGeometry::Sphere(Sphere::new(1.0)),
            Transform3D::from_translation(Vector3::new(0.5, 0.0, 0.0)),
        );
        let result = NaiveCollisionChecker.check(&[a, b], &CollisionMatrix::new());
        assert_eq!(
            result.collisions[0].collision_type,
            CollisionType::SelfCollision
        );
    }

    #[test]
    fn environment_collision_classified_for_obstacle() {
        let link = CollisionBody::new(
            EntityId::Link(0),
            CollisionGeometry::Sphere(Sphere::new(1.0)),
            Transform3D::identity(),
        );
        let obstacle = CollisionBody::new(
            EntityId::Obstacle(0),
            CollisionGeometry::Sphere(Sphere::new(1.0)),
            Transform3D::from_translation(Vector3::new(0.5, 0.0, 0.0)),
        );
        let result = NaiveCollisionChecker.check(&[link, obstacle], &CollisionMatrix::new());
        assert_eq!(
            result.collisions[0].collision_type,
            CollisionType::EnvironmentCollision
        );
    }

    #[test]
    fn sphere_box_intersecting() {
        let sphere = body(
            CollisionGeometry::Sphere(Sphere::new(0.5)),
            Transform3D::identity(),
        );
        let box_body = body(
            CollisionGeometry::Box(Box3D::new(2.0, 2.0, 2.0)),
            Transform3D::identity(),
        );
        let result = NaiveCollisionChecker.check(&[sphere, box_body], &CollisionMatrix::new());
        assert_eq!(result.len(), 1);
    }

    #[test]
    fn sphere_box_not_intersecting() {
        let sphere = body(
            CollisionGeometry::Sphere(Sphere::new(0.5)),
            Transform3D::from_translation(Vector3::new(5.0, 0.0, 0.0)),
        );
        let box_body = body(
            CollisionGeometry::Box(Box3D::new(2.0, 2.0, 2.0)),
            Transform3D::identity(),
        );
        let result = NaiveCollisionChecker.check(&[sphere, box_body], &CollisionMatrix::new());
        assert!(result.is_empty());
    }
}
