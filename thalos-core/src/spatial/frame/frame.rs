#[derive(Clone, Copy, Hash, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
pub enum FrameId {
    World,
    Id(u64),
}

impl FrameId {
    pub fn new(id: u64) -> Self {
        Self::Id(id)
    }
}

impl std::fmt::Display for FrameId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FrameId::World => write!(f, "world"),
            FrameId::Id(id) => write!(f, "frame_{}", id),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Frame {
    id: FrameId,
    name: String,
}

impl Frame {
    pub fn new(id: FrameId, name: String) -> Self {
        Self { id, name }
    }

    pub fn world() -> Self {
        Self {
            id: FrameId::World,
            name: "world".into(),
        }
    }

    pub fn id(&self) -> &FrameId {
        &self.id
    }

    pub fn name(&self) -> &str {
        &self.name
    }
}
