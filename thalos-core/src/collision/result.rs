use super::EntityId;

/// Resultado de una evaluación de colisiones.
#[derive(Debug, Clone, Default)]
pub struct CollisionResult {
    pub collisions: Vec<CollisionPair>,
}

impl CollisionResult {
    pub fn new(collisions: Vec<CollisionPair>) -> Self {
        Self { collisions }
    }

    pub fn is_empty(&self) -> bool {
        self.collisions.is_empty()
    }

    pub fn len(&self) -> usize {
        self.collisions.len()
    }
}

/// Un par de entidades en colisión.
#[derive(Debug, Clone)]
pub struct CollisionPair {
    pub a: EntityId,
    pub b: EntityId,
    pub collision_type: CollisionType,
}

impl CollisionPair {
    pub fn new(a: EntityId, b: EntityId, collision_type: CollisionType) -> Self {
        Self {
            a,
            b,
            collision_type,
        }
    }
}

/// Clasificación del tipo de colisión.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionType {
    SelfCollision,
    EnvironmentCollision,
}
