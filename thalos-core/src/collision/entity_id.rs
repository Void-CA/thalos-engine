use crate::robot::link::LinkId;

pub type ObstacleId = u32;
pub type ToolId = u32;

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum EntityId {
    Link(LinkId),
    Obstacle(ObstacleId),
    Tool(ToolId),
}

impl From<LinkId> for EntityId {
    fn from(id: LinkId) -> Self {
        EntityId::Link(id)
    }
}
