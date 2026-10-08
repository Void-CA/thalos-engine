//! Consumer scenario **B7 — execute a plan** — *resolved in slice D3*.
//!
//! # Intent
//! "Take this motion I just planned and prepare it to run."
//!
//! # What the consumer must know now (after D3)
//! - `intent::{Manipulator, MotionRequest, JointMove, prepare_execution}`.
//!
//! # Mechanism now hidden
//! Before D3 this required the entire planning pipeline **plus** the execution
//! builder: `parse_source`, `SemanticCompiler`, `SemanticResolver`,
//! `PlanningInput`, `PlanCompiler`, `DefaultPlannerDispatcher`,
//! `SegmentPlanningContext`, `ExecutionPlanBuilder`, `ExecutionSession`,
//! `ExecutionConfiguration`, `ExecutionSessionId`. And it re-derived the plan
//! from source just to execute it.
//!
//! # Gate (D3)
//! - The `MotionPlan` produced by `plan_motion` is the **same object** passed to
//!   `prepare_execution` — no re-planning, no second compile.
//! - The file cannot name `PlanCompiler`, `ExecutionRunner`, `Trajectory`,
//!   `ExecutionPlanBuilder` or `ExecutionSession`.
//!
//! # Boundary
//! The engine only says "this plan is prepared to be executed". Running it
//! against a robot/device (loop, clock, hardware) belongs to the application.

#[path = "common/mod.rs"]
mod common;

use thalos_api::intent::{JointMove, Manipulator, MotionRequest, prepare_execution};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot = Manipulator::from_urdf(common::PLANAR_2R_URDF)?;

    // 1. Plan a motion once.
    let plan = robot.plan_motion(
        &[0.0, 0.0],
        MotionRequest::Joint(JointMove::to(vec![0.3, 0.4])),
    )?;

    // 2. Prepare THAT plan for execution — no re-planning.
    let prepared = prepare_execution(&plan);

    println!(
        "prepare → {} waypoints, {:.3}s (from the same MotionPlan)",
        prepared.waypoint_count(),
        prepared.duration()
    );
    // 3. The engine hands this to the application; it does not run it.
    Ok(())
}
