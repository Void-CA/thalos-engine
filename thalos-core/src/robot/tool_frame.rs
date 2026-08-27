use crate::kinematics::forward::result::FKResult;
use crate::spatial::frame::FrameId;
use thalos_math::Transform3D;

/// Represents an active Tool Center Point (TCP) frame.
///
/// A TCP is always a transformation relative to a base frame. When `transform`
/// is the identity, the TCP coincides exactly with `base_frame`. When `transform`
/// is non-identity, it represents a virtual TCP offset (e.g., a calibrated tool
/// or gripper) attached to `base_frame`.
///
/// # Examples
///
/// ```
/// use thalos_core::robot::tool_frame::ToolFrame;
/// use thalos_core::spatial::frame::FrameId;
/// use thalos_math::Transform3D;
/// use thalos_math::Vector3;
///
/// // TCP exactly at the tool0 frame
/// let tcp = ToolFrame::identity(FrameId::new(42));
///
/// // TCP with a 12cm offset below the flange
/// let offset = Transform3D::from_translation(Vector3::new(0.0, 0.0, -0.12));
/// let tcp = ToolFrame::with_offset(FrameId::new(42), offset);
/// ```
#[derive(Debug, Clone)]
pub struct ToolFrame {
    /// The frame this TCP is attached to (e.g., `tool0`, `flange`).
    pub base_frame: FrameId,
    /// Transformation from `base_frame` to the TCP. Identity means the TCP
    /// coincides with `base_frame`.
    pub transform: Transform3D,
}

impl ToolFrame {
    /// Create a TCP that coincides exactly with the given frame (identity offset).
    pub fn identity(base_frame: FrameId) -> Self {
        Self {
            base_frame,
            transform: Transform3D::identity(),
        }
    }

    /// Create a TCP with a non-identity offset from the base frame.
    pub fn with_offset(base_frame: FrameId, transform: Transform3D) -> Self {
        Self {
            base_frame,
            transform,
        }
    }

    /// Check if this TCP has a non-identity offset.
    pub fn has_offset(&self) -> bool {
        self.transform != Transform3D::identity()
    }

    /// Resolve the global pose of this TCP given an FK result.
    ///
    /// Returns `None` if the `base_frame` is not present in the FK result.
    /// Otherwise, composes the FK pose of `base_frame` with the TCP offset.
    pub fn resolve_pose(&self, fk: &FKResult) -> Option<crate::spatial::pose::Pose> {
        let base_pose = fk.pose(&self.base_frame)?;
        let composed_transform = base_pose.transform().compose(&self.transform);
        Some(crate::spatial::pose::Pose::new(
            base_pose.reference_id(),
            base_pose.target_id(),
            composed_transform,
        ))
    }

    /// Resolve the global position (translation) of this TCP given an FK result.
    ///
    /// Returns `None` if the `base_frame` is not present in the FK result.
    pub fn resolve_position(&self, fk: &FKResult) -> Option<thalos_math::Vector3> {
        self.resolve_pose(fk).map(|p| p.translation())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::forward::result::FKResult;
    use crate::spatial::pose::Pose;
    use std::collections::HashMap;
    use thalos_math::Vector3;

    fn make_fk_result(frame_id: u64, translation: Vector3) -> FKResult {
        let mut poses = HashMap::new();
        let frame = FrameId::new(frame_id);
        let transform = Transform3D::from_translation(translation);
        poses.insert(frame.clone(), Pose::new(FrameId::World, frame, transform));
        FKResult::new(poses, FrameId::new(frame_id))
    }

    #[test]
    fn identity_tcp_resolves_to_base_frame() {
        let fk = make_fk_result(42, Vector3::new(1.0, 2.0, 3.0));
        let tcp = ToolFrame::identity(FrameId::new(42));

        let position = tcp.resolve_position(&fk).unwrap();

        assert_eq!(position, Vector3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn tcp_with_offset_composes_transform() {
        let fk = make_fk_result(42, Vector3::new(1.0, 0.0, 0.5));
        let offset = Transform3D::from_translation(Vector3::new(0.0, 0.0, -0.12));
        let tcp = ToolFrame::with_offset(FrameId::new(42), offset);

        let position = tcp.resolve_position(&fk).unwrap();

        // The offset is applied in the local frame of base_frame.
        // Since base_frame has identity rotation, the offset is applied directly.
        assert!((position.x - 1.0).abs() < 1e-10);
        assert!((position.y - 0.0).abs() < 1e-10);
        assert!((position.z - 0.38).abs() < 1e-10); // 0.5 - 0.12
    }

    #[test]
    fn tcp_resolve_returns_none_for_missing_frame() {
        let fk = make_fk_result(42, Vector3::new(1.0, 2.0, 3.0));
        let tcp = ToolFrame::identity(FrameId::new(99)); // Frame 99 not in FK

        assert!(tcp.resolve_position(&fk).is_none());
    }

    #[test]
    fn has_offset_returns_false_for_identity() {
        let tcp = ToolFrame::identity(FrameId::new(42));
        assert!(!tcp.has_offset());
    }

    #[test]
    fn has_offset_returns_true_for_non_identity() {
        let offset = Transform3D::from_translation(Vector3::new(0.0, 0.0, -0.12));
        let tcp = ToolFrame::with_offset(FrameId::new(42), offset);
        assert!(tcp.has_offset());
    }
}
