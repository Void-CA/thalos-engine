use crate::ids::OperationId;
use crate::motion::segment::MotionSegment;

/// A motion primitive with provenance metadata.
///
/// Wraps a `MotionSegment` with semantic provenance (`operation_id`) and
/// a compilation-level role (`MotionRole`). This keeps semantic metadata
/// out of the primitive motion commands.
#[derive(Debug, Clone)]
pub struct MotionNode {
    pub segment: MotionSegment,
    /// The operation that produced this node, if any.
    pub operation_id: Option<OperationId>,
    /// The compilation role of this node.
    pub role: MotionRole,
}

/// Compilation-level role categories.
///
/// These are abstract categories that span all operation types:
/// - Pick's Interaction → CloseGripper
/// - Weld's Interaction → ArcStart/ArcEnd
/// Same role, different concrete actions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum MotionRole {
    /// Moving toward the operation target (pre-positioning).
    Approach,
    /// Active work phase (welding, painting, following seam).
    Execution,
    /// Contact events (grip, release, arc start/stop, force application).
    Interaction,
    /// Moving away after operation completion.
    Departure,
    /// Support actions (tool change, sensor check, wait).
    Auxiliary,
    /// Transit / free motion (default for backward compat).
    Transit,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spatial::frame::FrameId;
    use crate::spatial::pose::Pose;
    use thalos_math::Transform3D;

    fn sample_segment() -> MotionSegment {
        MotionSegment::MoveJ {
            origin: OperationId("test".to_string()),
            target: vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            max_velocity: None,
            max_acceleration: None,
        }
    }

    fn sample_pose() -> Pose {
        Pose::new(FrameId::World, FrameId::Id(1), Transform3D::identity())
    }

    // ── MotionNode construction ───────────────────────────

    #[test]
    fn motion_node_holds_segment_and_role() {
        let node = MotionNode {
            segment: sample_segment(),
            operation_id: None,
            role: MotionRole::Transit,
        };

        assert!(matches!(node.segment, MotionSegment::MoveJ { .. }));
        assert_eq!(node.role, MotionRole::Transit);
    }

    #[test]
    fn motion_node_with_operation_id() {
        let node = MotionNode {
            segment: sample_segment(),
            operation_id: Some(OperationId("42".to_string())),
            role: MotionRole::Approach,
        };

        assert_eq!(node.operation_id, Some(OperationId("42".to_string())));
    }

    #[test]
    fn motion_node_without_operation_id() {
        let node = MotionNode {
            segment: sample_segment(),
            operation_id: None,
            role: MotionRole::Execution,
        };

        assert!(node.operation_id.is_none());
    }

    // ── MotionRole variant semantics ──────────────────────

    #[test]
    fn motion_role_variants_are_distinct() {
        assert_ne!(MotionRole::Approach, MotionRole::Execution);
        assert_ne!(MotionRole::Approach, MotionRole::Interaction);
        assert_ne!(MotionRole::Approach, MotionRole::Departure);
        assert_ne!(MotionRole::Approach, MotionRole::Auxiliary);
        assert_ne!(MotionRole::Approach, MotionRole::Transit);
        assert_ne!(MotionRole::Execution, MotionRole::Interaction);
        assert_ne!(MotionRole::Execution, MotionRole::Departure);
        assert_ne!(MotionRole::Execution, MotionRole::Auxiliary);
        assert_ne!(MotionRole::Execution, MotionRole::Transit);
        assert_ne!(MotionRole::Interaction, MotionRole::Departure);
        assert_ne!(MotionRole::Interaction, MotionRole::Auxiliary);
        assert_ne!(MotionRole::Interaction, MotionRole::Transit);
        assert_ne!(MotionRole::Departure, MotionRole::Auxiliary);
        assert_ne!(MotionRole::Departure, MotionRole::Transit);
        assert_ne!(MotionRole::Auxiliary, MotionRole::Transit);
    }

    #[test]
    fn motion_role_exhaustive_debug_format() {
        // Verify each variant produces a meaningful Debug string
        let variants = [
            (MotionRole::Approach, "Approach"),
            (MotionRole::Execution, "Execution"),
            (MotionRole::Interaction, "Interaction"),
            (MotionRole::Departure, "Departure"),
            (MotionRole::Auxiliary, "Auxiliary"),
            (MotionRole::Transit, "Transit"),
        ];

        for (role, expected) in &variants {
            assert_eq!(&format!("{:?}", role), expected);
        }
    }

    // ── MotionNode with different roles ───────────────────

    #[test]
    fn motion_node_supports_all_roles() {
        let roles = [
            MotionRole::Approach,
            MotionRole::Execution,
            MotionRole::Interaction,
            MotionRole::Departure,
            MotionRole::Auxiliary,
            MotionRole::Transit,
        ];

        for role in &roles {
            let node = MotionNode {
                segment: sample_segment(),
                operation_id: None,
                role: *role,
            };
            assert_eq!(node.role, *role, "MotionNode should accept role {:?}", role);
        }
    }

    // ── Clone behavior (MotionNode implements Clone) ──────

    #[test]
    fn motion_node_is_clonable() {
        let node = MotionNode {
            segment: sample_segment(),
            operation_id: Some(OperationId("1".to_string())),
            role: MotionRole::Interaction,
        };

        let cloned = node.clone();
        assert_eq!(cloned.operation_id, node.operation_id);
        assert_eq!(cloned.role, node.role);
        assert!(matches!(cloned.segment, MotionSegment::MoveJ { .. }));
    }
}
