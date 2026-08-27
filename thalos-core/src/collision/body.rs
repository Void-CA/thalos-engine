use super::{CollisionGeometry, EntityId};
use thalos_math::Transform3D;

#[derive(Debug, Clone)]
pub struct CollisionBody {
    pub entity: EntityId,
    pub geometry: CollisionGeometry,
    pub pose: Transform3D,
}

impl CollisionBody {
    pub fn new(
        entity: impl Into<EntityId>,
        geometry: CollisionGeometry,
        pose: Transform3D,
    ) -> Self {
        Self {
            entity: entity.into(),
            geometry,
            pose,
        }
    }
}
