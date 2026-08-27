use crate::robot::tool_frame::ToolFrame;
use crate::spatial::{frame::FrameId, pose::Pose};
use std::collections::HashMap;
use thalos_math::Vector3;

#[derive(Debug, Clone)]
pub struct FKResult {
    poses: HashMap<FrameId, Pose>,
    end_effector: FrameId,
}

impl FKResult {
    pub fn new(poses: HashMap<FrameId, Pose>, end_effector: FrameId) -> Self {
        Self {
            poses,
            end_effector,
        }
    }

    pub fn pose(&self, frame: &FrameId) -> Option<&Pose> {
        self.poses.get(frame)
    }

    pub fn frames(&self) -> impl Iterator<Item = &FrameId> {
        self.poses.keys()
    }

    pub fn end_effector(&self) -> &FrameId {
        &self.end_effector
    }

    pub fn ee_pose(&self) -> Option<&Pose> {
        self.poses.get(&self.end_effector)
    }

    pub fn ee_position(&self) -> Option<Vector3> {
        self.poses.get(&self.end_effector).map(|p| p.translation())
    }

    /// Resolve the TCP position given a ToolFrame.
    ///
    /// Returns `None` if the TCP's base frame is not present in the FK result.
    pub fn tcp_position(&self, tcp: &ToolFrame) -> Option<Vector3> {
        tcp.resolve_position(self)
    }

    /// Resolve the TCP pose given a ToolFrame.
    ///
    /// Returns `None` if the TCP's base frame is not present in the FK result.
    pub fn tcp_pose(&self, tcp: &ToolFrame) -> Option<Pose> {
        tcp.resolve_pose(self)
    }
}
