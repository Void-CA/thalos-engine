use crate::spatial::frame::{Frame, FrameId};
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct FrameRegistry {
    frames: HashMap<FrameId, Frame>,
    next_id: u64,
}

impl Default for FrameRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameRegistry {
    pub fn new() -> Self {
        Self {
            frames: HashMap::new(),
            next_id: 0,
        }
    }

    pub fn create(&mut self, name: &str) -> FrameId {
        let id = FrameId::new(self.next_id);
        self.next_id += 1;

        let frame = Frame::new(id, name.to_string());

        self.frames.insert(id, frame);

        id
    }

    pub fn get(&self, id: &FrameId) -> Option<&Frame> {
        self.frames.get(id)
    }

    pub fn contains(&self, id: &FrameId) -> bool {
        self.frames.contains_key(id)
    }

    pub fn register(&mut self, frame: Frame) -> FrameId {
        let id = *frame.id();
        self.frames.insert(id, frame);
        id
    }

    /// Find the `FrameId` for a registered frame by name.
    ///
    /// Returns `None` when no frame with that name exists.
    pub fn resolve_by_name(&self, name: &str) -> Option<FrameId> {
        self.frames
            .iter()
            .find(|(_, f)| f.name() == name)
            .map(|(id, _)| *id)
    }
}
