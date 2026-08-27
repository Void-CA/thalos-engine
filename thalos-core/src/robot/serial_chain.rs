use crate::spatial::frame::{Frame, FrameId, FrameRegistry};

use super::segment::Segment;

#[derive(Debug, Clone)]
pub struct SerialChain {
    pub segments: Vec<Segment>,
    pub frames: FrameRegistry,
    pub end_effector: FrameId,
}

impl SerialChain {
    pub fn frame(&self, id: &FrameId) -> Option<&Frame> {
        self.frames.get(id)
    }

    pub fn end_effector(&self) -> &FrameId {
        &self.end_effector
    }

    pub fn end_effector_frame(&self) -> Option<&Frame> {
        self.frames.get(&self.end_effector)
    }

    /// Number of actuated degrees of freedom (non-fixed joints).
    pub fn dof_count(&self) -> usize {
        self.segments.iter().filter(|s| s.joint.dof() > 0).count()
    }

    /// Number of segments in the chain (includes fixed joints).
    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }
}
