use crate::collision::CollisionGeometry;
use thalos_math::Transform3D;

pub use thalos_models::LinkId;

#[derive(Debug, Clone)]
pub struct Link {
    pub id: LinkId,
    pub transform: Transform3D,
    /// Geometría de colisión asociada a este link, en el marco local
    /// del link. `None` significa que este link no participa en
    /// detección de colisiones.
    pub collision_geometry: Option<CollisionGeometry>,
}

impl Link {
    pub fn new(id: LinkId, transform: Transform3D) -> Self {
        Self {
            id,
            transform,
            collision_geometry: None,
        }
    }

    pub fn id(&self) -> LinkId {
        self.id
    }

    /// Builder-style: asigna geometría de colisión a este link.
    pub fn with_collision_geometry(mut self, geometry: CollisionGeometry) -> Self {
        self.collision_geometry = Some(geometry);
        self
    }
}
