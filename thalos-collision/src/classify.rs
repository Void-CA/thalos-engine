use thalos_core::collision::{CollisionType, EntityId};

/// Determina semánticamente el tipo de colisión según las entidades
/// involucradas.
///
/// - Link ↔ Link → SelfCollision
/// - Cualquier interacción con Obstacle o Tool → EnvironmentCollision
pub fn classify_collision(a: &EntityId, b: &EntityId) -> CollisionType {
    match (a, b) {
        (EntityId::Link(_), EntityId::Link(_)) => CollisionType::SelfCollision,
        _ => CollisionType::EnvironmentCollision,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn link_link_is_self_collision() {
        assert_eq!(
            classify_collision(&EntityId::Link(0), &EntityId::Link(1)),
            CollisionType::SelfCollision
        );
    }

    #[test]
    fn link_obstacle_is_environment() {
        assert_eq!(
            classify_collision(&EntityId::Link(0), &EntityId::Obstacle(0)),
            CollisionType::EnvironmentCollision
        );
    }

    #[test]
    fn obstacle_link_is_environment() {
        assert_eq!(
            classify_collision(&EntityId::Obstacle(0), &EntityId::Link(0)),
            CollisionType::EnvironmentCollision
        );
    }

    #[test]
    fn link_tool_is_environment() {
        assert_eq!(
            classify_collision(&EntityId::Link(0), &EntityId::Tool(0)),
            CollisionType::EnvironmentCollision
        );
    }
}
