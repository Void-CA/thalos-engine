//! Tier-1 execution preparation.
//!
//! Converts a [`MotionPlan`] into the engine's execution IR **without
//! re-planning**. The engine's responsibility ends at "this plan is prepared to
//! be executed"; running it against a robot or device is the application's job
//! (see `ARCHITECTURE.md`). No loop, clock, scheduler or hardware lives here.

use thalos_core::execution::plan::{ExecutionPlan, PlanInstruction};
use thalos_planning::execution_plan_builder::ExecutionPlanBuilder;

use crate::intent::motion::{MotionKind, MotionPlan};

/// A [`MotionPlan`] prepared for execution (Tier 1).
///
/// Wraps the engine's execution IR so the underlying type can evolve without
/// breaking this surface.
#[derive(Debug, Clone)]
pub struct PreparedPlan {
    execution_plan: ExecutionPlan,
}

impl PreparedPlan {
    pub fn waypoint_count(&self) -> usize {
        self.execution_plan.waypoints.len()
    }

    pub fn duration(&self) -> f64 {
        self.execution_plan.duration
    }

    /// Access the underlying execution IR (escape hatch to Tier 3).
    pub fn execution_plan(&self) -> &ExecutionPlan {
        &self.execution_plan
    }
}

/// Prepare a motion plan for execution.
///
/// Uses the plan's **already-computed** trajectory: it does not re-run inverse
/// kinematics or the planner. The result is the engine's execution IR, ready to
/// hand to the application's runtime (the engine does not execute it).
pub fn prepare_execution(plan: &MotionPlan) -> PreparedPlan {
    let instruction = match plan.kind() {
        MotionKind::Joint => PlanInstruction::MoveJ,
        MotionKind::Linear => PlanInstruction::MoveL,
    };
    let execution_plan =
        ExecutionPlanBuilder::build_from_trajectory(plan.trajectory(), instruction);
    PreparedPlan { execution_plan }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::intent::motion::{JointMove, MotionRequest};
    use crate::intent::Manipulator;
    use thalos_core::models::{RobotModel, RobotRegistry};

    #[test]
    fn prepared_plan_reuses_the_motion_plan_exactly() {
        let robot = Manipulator::from_chain(RobotRegistry::create_default(RobotModel::Planar2R));
        let plan = robot
            .plan_motion(
                &[0.0, 0.0],
                MotionRequest::Joint(JointMove::to(vec![0.3, 0.4])),
            )
            .expect("plans");

        let prepared = prepare_execution(&plan);

        // Same waypoints and duration — nothing re-derived.
        assert_eq!(prepared.waypoint_count(), plan.waypoint_count());
        assert_eq!(prepared.duration(), plan.duration());
        assert_eq!(prepared.execution_plan().waypoints.len(), plan.waypoint_count());
    }
}
