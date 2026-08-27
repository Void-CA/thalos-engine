use crate::ids::OperationId;
use crate::motion::segment::MotionSegment;
use crate::operation::motion_node::{MotionNode, MotionRole};
use crate::operation::operation::Operation;
use crate::spatial::frame::FrameId;
use crate::spatial::pose::Pose;

/// Helper: creates a minimal MoveL node with placeholder geometry.
///
/// The segment carries the operation's `OperationId` as its `origin`
/// (invariant I2 — origin survives expansion).
fn move_to_pose(target_pose: Pose, id: OperationId, role: MotionRole) -> MotionNode {
    MotionNode {
        segment: MotionSegment::MoveL {
            origin: id.clone(),
            frame: FrameId::World,
            target_pose,
            max_velocity: None,
        },
        operation_id: Some(id),
        role,
    }
}

/// Expands an `Operation` into a sequence of `MotionNode`s.
///
/// Each operation type expands to a fixed number of nodes with specific roles:
///
/// | Operation | Nodes | Roles |
/// |-----------|-------|-------|
/// | Pick      | 5     | Approach → Descend → CloseGripper → Lift → Retreat |
/// | Place     | 4     | Approach → Descend → OpenGripper → Retreat |
/// | Transit   | 1     | Transit |
///
/// # Skeleton Geometry (v0)
///
/// All nodes use placeholder geometry — they target the operation's `target_pose`
/// directly. Exact approach offsets and grasp strategy are deferred until
/// Environment and Gripper models exist.
pub fn expand_operation(op: &Operation) -> Vec<MotionNode> {
    match op {
        Operation::Pick {
            id, target_pose, ..
        } => {
            vec![
                move_to_pose(target_pose.clone(), id.clone(), MotionRole::Approach),
                move_to_pose(target_pose.clone(), id.clone(), MotionRole::Execution),
                move_to_pose(target_pose.clone(), id.clone(), MotionRole::Interaction),
                move_to_pose(target_pose.clone(), id.clone(), MotionRole::Departure),
                move_to_pose(target_pose.clone(), id.clone(), MotionRole::Departure),
            ]
        }
        Operation::Place {
            id, target_pose, ..
        } => {
            vec![
                move_to_pose(target_pose.clone(), id.clone(), MotionRole::Approach),
                move_to_pose(target_pose.clone(), id.clone(), MotionRole::Execution),
                move_to_pose(target_pose.clone(), id.clone(), MotionRole::Interaction),
                move_to_pose(target_pose.clone(), id.clone(), MotionRole::Departure),
            ]
        }
        Operation::Transit {
            id, target_pose, ..
        } => {
            vec![move_to_pose(
                target_pose.clone(),
                id.clone(),
                MotionRole::Transit,
            )]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::operation::operation::OperationConstraints;
    use crate::spatial::pose::Pose;
    use thalos_math::Transform3D;

    fn sample_pose() -> Pose {
        Pose::new(FrameId::World, FrameId::Id(1), Transform3D::identity())
    }

    fn make_pick(id: u64, pose: Pose) -> Operation {
        Operation::Pick {
            id: OperationId(id.to_string()),
            target_pose: pose,
            constraints: OperationConstraints::default(),
        }
    }

    fn make_place(id: u64, pose: Pose) -> Operation {
        Operation::Place {
            id: OperationId(id.to_string()),
            target_pose: pose,
            constraints: OperationConstraints::default(),
        }
    }

    fn make_transit(id: u64, pose: Pose) -> Operation {
        Operation::Transit {
            id: OperationId(id.to_string()),
            target_pose: pose,
            constraints: OperationConstraints::default(),
        }
    }

    // ── Pick expansion ────────────────────────────────────

    #[test]
    fn pick_expands_to_five_nodes() {
        let op = make_pick(1, sample_pose());
        let nodes = expand_operation(&op);
        assert_eq!(nodes.len(), 5, "Pick should expand to 5 nodes");
    }

    #[test]
    fn pick_nodes_have_correct_roles_in_order() {
        let op = make_pick(1, sample_pose());
        let nodes = expand_operation(&op);
        assert_eq!(
            nodes[0].role,
            MotionRole::Approach,
            "Node 0 should be Approach"
        );
        assert_eq!(
            nodes[1].role,
            MotionRole::Execution,
            "Node 1 should be Execution (Descend)"
        );
        assert_eq!(
            nodes[2].role,
            MotionRole::Interaction,
            "Node 2 should be Interaction (CloseGripper)"
        );
        assert_eq!(
            nodes[3].role,
            MotionRole::Departure,
            "Node 3 should be Departure (Lift)"
        );
        assert_eq!(
            nodes[4].role,
            MotionRole::Departure,
            "Node 4 should be Departure (Retreat)"
        );
    }

    #[test]
    fn pick_nodes_carry_operation_id() {
        let id = 42;
        let op = make_pick(id, sample_pose());
        let nodes = expand_operation(&op);
        for (i, node) in nodes.iter().enumerate() {
            assert_eq!(
                node.operation_id,
                Some(OperationId(id.to_string())),
                "Pick node {} should carry operation_id {}",
                i,
                id
            );
        }
    }

    #[test]
    fn pick_nodes_segments_carry_origin() {
        // Invariant I2: the segment inside each expanded node carries the
        // operation's OperationId as its origin.
        let op = make_pick(42, sample_pose());
        let nodes = expand_operation(&op);
        for (i, node) in nodes.iter().enumerate() {
            assert_eq!(
                node.segment.origin(),
                &OperationId("42".to_string()),
                "Pick node {} segment should carry origin 42",
                i
            );
        }
    }

    #[test]
    fn pick_descend_targets_operation_pose() {
        // The Descend (Execution) node should reference the operation's target_pose
        let pose = sample_pose();
        let op = make_pick(1, pose.clone());
        let nodes = expand_operation(&op);

        match &nodes[1].segment {
            MotionSegment::MoveL { target_pose, .. } => {
                assert_eq!(
                    target_pose.target_id(),
                    pose.target_id(),
                    "Descend should target the operation's target_pose"
                );
            }
            other => panic!("Descend node should be MoveL, got {:?}", other),
        }
    }

    #[test]
    fn pick_all_nodes_have_valid_segments() {
        let op = make_pick(1, sample_pose());
        let nodes = expand_operation(&op);
        for (i, node) in nodes.iter().enumerate() {
            assert!(
                matches!(node.segment, MotionSegment::MoveL { .. }),
                "Pick node {} should have a MoveL segment, got {:?}",
                i,
                node.segment
            );
        }
    }

    // ── Place expansion ───────────────────────────────────

    #[test]
    fn place_expands_to_four_nodes() {
        let op = make_place(1, sample_pose());
        let nodes = expand_operation(&op);
        assert_eq!(nodes.len(), 4, "Place should expand to 4 nodes");
    }

    #[test]
    fn place_nodes_have_correct_roles_in_order() {
        let op = make_place(1, sample_pose());
        let nodes = expand_operation(&op);
        assert_eq!(
            nodes[0].role,
            MotionRole::Approach,
            "Node 0 should be Approach"
        );
        assert_eq!(
            nodes[1].role,
            MotionRole::Execution,
            "Node 1 should be Execution (Descend)"
        );
        assert_eq!(
            nodes[2].role,
            MotionRole::Interaction,
            "Node 2 should be Interaction (OpenGripper)"
        );
        assert_eq!(
            nodes[3].role,
            MotionRole::Departure,
            "Node 3 should be Departure (Retreat)"
        );
    }

    #[test]
    fn place_nodes_carry_operation_id() {
        let op = make_place(7, sample_pose());
        let nodes = expand_operation(&op);
        for (i, node) in nodes.iter().enumerate() {
            assert_eq!(
                node.operation_id,
                Some(OperationId("7".to_string())),
                "Place node {} should carry operation_id 7",
                i
            );
        }
    }

    // ── Transit expansion ─────────────────────────────────

    #[test]
    fn transit_expands_to_one_node() {
        let op = make_transit(1, sample_pose());
        let nodes = expand_operation(&op);
        assert_eq!(nodes.len(), 1, "Transit should expand to 1 node");
    }

    #[test]
    fn transit_node_has_transit_role() {
        let op = make_transit(99, sample_pose());
        let nodes = expand_operation(&op);
        assert_eq!(
            nodes[0].role,
            MotionRole::Transit,
            "Transit node should have Transit role"
        );
    }

    #[test]
    fn transit_node_carries_operation_id() {
        let op = make_transit(99, sample_pose());
        let nodes = expand_operation(&op);
        assert_eq!(
            nodes[0].operation_id,
            Some(OperationId("99".to_string())),
            "Transit node should carry operation_id 99"
        );
    }

    // ─── Triangulation: different ops, same id range ──────

    #[test]
    fn pick_with_different_id_propagates_to_all_nodes() {
        let op = make_pick(100, sample_pose());
        let nodes = expand_operation(&op);
        for node in &nodes {
            assert_eq!(node.operation_id, Some(OperationId("100".to_string())));
        }
    }

    #[test]
    fn place_with_different_id_propagates_to_all_nodes() {
        let op = make_place(200, sample_pose());
        let nodes = expand_operation(&op);
        for node in &nodes {
            assert_eq!(node.operation_id, Some(OperationId("200".to_string())));
        }
    }
}
