use super::{CollisionBody, EntityId};
use crate::kinematics::forward::result::FKResult;
use crate::robot::serial_chain::SerialChain;
pub struct CollisionBodyBuilder;

impl CollisionBodyBuilder {
    pub fn build(chain: &SerialChain, fk: &FKResult) -> Vec<CollisionBody> {
        let mut bodies = Vec::new();

        for segment in &chain.segments {
            let geometry = match &segment.link.collision_geometry {
                Some(g) => g.clone(),
                None => continue,
            };

            let pose = match fk.pose(&segment.child) {
                Some(p) => p.transform().clone(),
                None => continue,
            };

            bodies.push(CollisionBody {
                entity: EntityId::Link(segment.link.id),
                geometry,
                pose,
            });
        }

        bodies
    }
}
