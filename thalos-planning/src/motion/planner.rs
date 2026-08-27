use thalos_core::{
    kinematics::inverse::IKSolver, robot::serial_chain::SerialChain, robot::state::RobotState,
    robot::tool_frame::ToolFrame,
};

use crate::error::PlanningError;

// ─── Legacy types (SegmentPlanningContext retains the old contract) ────

/// Legacy planning context for segment-level planning.
///
/// Used by `PlanCompiler`, `MoveJPlanner`, `MoveLPlanner` and the dispatcher
/// in `compiler.rs`. The program-level planner types that used to live here
/// were removed with the parallel planner path (PR 2).
pub struct SegmentPlanningContext<'a> {
    pub robot: &'a SerialChain,
    pub current_state: &'a RobotState,
    pub ik_solver: &'a dyn IKSolver,
    /// Active Tool Center Point (TCP) frame.
    ///
    /// When `Some`, singularity and manipulability analysis reference the TCP.
    /// When `None`, reference the flange (end effector).
    pub tcp: Option<&'a ToolFrame>,
}

/// Legacy alias for backward compatibility with external consumers.
#[allow(deprecated)]
pub type PlanningContext<'a> = SegmentPlanningContext<'a>;

/// Legacy alias for the segment-level planning result type.
pub type PlanningResult = Result<thalos_core::trajectory::Trajectory, PlanningError>;

/// Legacy segment-level planner trait (used by MoveJPlanner, MoveLPlanner).
///
/// Replaced at the program level by the new `MotionPlanner`. Existing segment
/// planners implement this trait while transitioning to internal helpers.
pub trait SegmentPlanner {
    /// The type of goal this planner accepts.
    type Goal: ?Sized;

    /// Plan a trajectory for a single motion goal.
    fn plan<'a>(&self, ctx: &SegmentPlanningContext<'a>, goal: &Self::Goal) -> PlanningResult;
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── PlanningError variants ────────────────────────────────────────

    #[test]
    fn planning_error_empty_program_variant() {
        let err = PlanningError::EmptyProgram;
        assert_eq!(err.to_string(), "Motion program is empty (no instructions)");
    }

    #[test]
    fn planning_error_invalid_context_variant() {
        let err = PlanningError::InvalidContext("missing robot model".into());
        assert_eq!(
            err.to_string(),
            "Planning context is invalid: missing robot model"
        );
    }

    #[test]
    fn planning_error_ik_failure_variant() {
        let err = PlanningError::IKFailure { pose_index: 5 };
        assert_eq!(
            err.to_string(),
            "Inverse kinematics failed for pose index 5"
        );
        match &err {
            PlanningError::IKFailure { pose_index } => {
                assert_eq!(*pose_index, 5);
            }
            _ => panic!("Expected IKFailure"),
        }
    }
}
