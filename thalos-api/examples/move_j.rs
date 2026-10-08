//! Consumer scenario **B4 — MoveJ (joint-space move)** — *resolved in slice D1*.
//!
//! # Intent
//! "Move the joints to these values."
//!
//! # What the consumer must know now (after D1)
//! - `intent::{Manipulator, MotionRequest, JointMove}` — and nothing else.
//!
//! # Mechanism now hidden
//! Before D1 this needed 7 imports, including the IK solver (dragged in by the
//! mandatory `PlanningContext` field) though a joint move never uses it:
//! `ForwardKinematics`, `DampedLeastSquaresSolver`, `IKConfig`, `RobotState`,
//! `PlanningContext`, `MoveJPlanner`, `MoveJConfig`, `ValidatedGoal`, …
//! All of that is now inside `Manipulator::plan_motion`.
//!
//! # Gate (D1)
//! This file is **conceptually incapable** of naming `IKSolver`: it does not
//! appear, and the joint path never touches it. The internal separation
//! (`JointPlanningContext`, which carries no solver) is what makes that true.

#[path = "common/mod.rs"]
mod common;

use thalos_api::intent::{JointMove, Manipulator, MotionRequest};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot = Manipulator::from_urdf(common::PLANAR_2R_URDF)?;

    let plan = robot.plan_motion(
        &[0.0, 0.0],
        MotionRequest::Joint(JointMove::to(vec![0.3, 0.4])),
    )?;

    println!(
        "MoveJ → {} waypoints, {:.3}s",
        plan.waypoint_count(),
        plan.duration()
    );
    Ok(())
}
