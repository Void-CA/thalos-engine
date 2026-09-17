use thalos_core::motion::MotionPose;

use crate::resource::ToolId;

/// A grasp plan returned by the `KnowledgeProvider` for a given object.
///
/// Contains only geometric frames — no motion instructions, trajectories,
/// or constraints exist on this type. Lowering converts these frames into
/// `ProgramInstruction` values.
#[derive(Debug, Clone, PartialEq)]
pub struct GraspPlan {
    /// The pose where grasping occurs (the object's position/orientation).
    pub grasp_frame: MotionPose,
    /// The pose to approach from before grasping.
    pub approach_frame: MotionPose,
    /// The pose to retract to after grasping.
    pub retreat_frame: MotionPose,
    /// An optional suggested tool for the grasp operation.
    pub preferred_tool: Option<ToolId>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_pose(x: f64, y: f64, z: f64) -> MotionPose {
        MotionPose {
            position: [x, y, z],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "world".into(),
        }
    }

    // ── Field access ────────────────────────────────────────────────────

    #[test]
    fn grasp_plan_has_grasp_frame() {
        let plan = GraspPlan {
            grasp_frame: sample_pose(1.0, 2.0, 3.0),
            approach_frame: sample_pose(0.0, 0.0, 0.0),
            retreat_frame: sample_pose(2.0, 2.0, 2.0),
            preferred_tool: None,
        };
        assert_eq!(plan.grasp_frame.position, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn grasp_plan_has_approach_frame() {
        let plan = GraspPlan {
            grasp_frame: sample_pose(0.0, 0.0, 0.0),
            approach_frame: sample_pose(10.0, 20.0, 30.0),
            retreat_frame: sample_pose(0.0, 0.0, 0.0),
            preferred_tool: None,
        };
        assert_eq!(plan.approach_frame.position, [10.0, 20.0, 30.0]);
    }

    #[test]
    fn grasp_plan_has_retreat_frame() {
        let plan = GraspPlan {
            grasp_frame: sample_pose(0.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 0.0, 0.0),
            retreat_frame: sample_pose(5.0, 5.0, 5.0),
            preferred_tool: None,
        };
        assert_eq!(plan.retreat_frame.position, [5.0, 5.0, 5.0]);
    }

    #[test]
    fn grasp_plan_has_preferred_tool() {
        let plan = GraspPlan {
            grasp_frame: sample_pose(0.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 0.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 0.0),
            preferred_tool: Some(ToolId("gripper-1".to_string())),
        };
        assert_eq!(plan.preferred_tool, Some(ToolId("gripper-1".to_string())));
    }

    #[test]
    fn grasp_plan_preferred_tool_can_be_none() {
        let plan = GraspPlan {
            grasp_frame: sample_pose(0.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 0.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 0.0),
            preferred_tool: None,
        };
        assert!(plan.preferred_tool.is_none());
    }

    // ── No motion instruction fields ─────────────────────────────────────

    #[test]
    fn grasp_plan_has_no_motion_instruction_fields() {
        // Destructure to prove ONLY the four expected fields exist:
        let plan = GraspPlan {
            grasp_frame: sample_pose(0.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 0.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 0.0),
            preferred_tool: None,
        };
        let GraspPlan {
            grasp_frame: _,
            approach_frame: _,
            retreat_frame: _,
            preferred_tool: _,
        } = plan;
        // If this compiles, there are no extra fields.
    }

    #[test]
    fn grasp_plan_fields_are_motion_poses_not_instructions() {
        let plan = GraspPlan {
            grasp_frame: sample_pose(1.0, 2.0, 3.0),
            approach_frame: sample_pose(4.0, 5.0, 6.0),
            retreat_frame: sample_pose(7.0, 8.0, 9.0),
            preferred_tool: None,
        };
        // Pose fields are MotionPose — confirm they have position/orientation/frame
        assert_eq!(plan.grasp_frame.orientation, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(plan.grasp_frame.frame, "world");
    }

    // ── Clone + Debug ───────────────────────────────────────────────────

    #[test]
    fn grasp_plan_clone_equals_original() {
        let plan = GraspPlan {
            grasp_frame: sample_pose(1.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 1.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 1.0),
            preferred_tool: Some(ToolId("t1".to_string())),
        };
        let cloned = plan.clone();
        assert_eq!(plan, cloned);
    }

    #[test]
    fn grasp_plan_debug_format() {
        let plan = GraspPlan {
            grasp_frame: sample_pose(0.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 0.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 0.0),
            preferred_tool: None,
        };
        let debug = format!("{plan:?}");
        assert!(
            debug.contains("GraspPlan"),
            "Debug should contain type name"
        );
    }
}
