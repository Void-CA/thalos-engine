//! Consumer scenario **B5 — MoveL (cartesian move)** — *resolved in slice D2*.
//!
//! # Intent
//! "Go linearly to this position."
//!
//! # What the consumer must know now (after D2)
//! - `intent::{Manipulator, MotionRequest, LinearMove, CartesianTarget}` — and
//!   nothing else.
//!
//! # Mechanism now hidden
//! Before D2 the consumer chained two engines by hand. First inverse kinematics
//! (`ForwardKinematics`, `DampedLeastSquaresSolver`, `IKConfig`, `IKGoal`,
//! `IKSolver`), then the joint planner (`RobotState`, `PlanningContext`,
//! `MoveJPlanner`, `MoveJConfig`, `ValidatedGoal`). All of it is now inside
//! `Manipulator::plan_motion`.
//!
//! # Gate (D2)
//! This file is **conceptually incapable** of naming `IKGoal`,
//! `ForwardKinematics`, `IKSolver`, `DampedLeastSquaresSolver`, `MoveJPlanner`
//! or any cartesian planning context: none appear, and `LinearMove` exposes no
//! IK/sampling/planner configuration — only the target and an optional profile.

#[path = "common/mod.rs"]
mod common;

use thalos_api::intent::{CartesianTarget, LinearMove, Manipulator, MotionRequest};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot = Manipulator::from_urdf(common::PLANAR_2R_URDF)?;

    let plan = robot.plan_motion(
        &[0.0, 0.0],
        MotionRequest::Linear(LinearMove::to(CartesianTarget::position(1.0, 0.5, 0.0))),
    )?;

    println!(
        "MoveL → {} waypoints, {:.3}s",
        plan.waypoint_count(),
        plan.duration()
    );
    Ok(())
}
