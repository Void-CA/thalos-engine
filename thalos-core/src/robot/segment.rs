use crate::{
    robot::{joint::JointType, link::Link},
    spatial::frame::{Frame, FrameId, FrameRegistry},
};
#[derive(Debug, Clone)]
pub struct Segment {
    pub parent: FrameId,
    pub child: FrameId,

    pub joint: JointType,
    pub link: Link,
}

impl Segment {
    pub fn new(parent: FrameId, child: FrameId, joint: JointType, link: Link) -> Self {
        Self {
            parent,
            child,
            joint,
            link,
        }
    }

    pub fn child_frame<'a>(&self, registry: &'a FrameRegistry) -> Option<&'a Frame> {
        registry.get(&self.child)
    }
}
