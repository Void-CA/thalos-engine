use super::{CollisionBody, CollisionMatrix, CollisionResult};

pub trait CollisionChecker {
    fn check(&self, bodies: &[CollisionBody], matrix: &CollisionMatrix) -> CollisionResult;
}
