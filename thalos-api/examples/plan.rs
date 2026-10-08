//! Consumer scenario **B6 — plan a `.thls` program** — *still Tier 3*.
//!
//! # Intent
//! "Compile this program for this robot into a plan" →
//! `let plan = robot.plan(source)?;`.
//!
//! # Status: the one deliberate gap
//! Unlike B1–B5/B7, this scenario has **no intent API yet**. Program-level
//! planning (`robot.plan_source`) was scoped as **D3-bis** and deliberately
//! **deferred — no evidence** that programs need it (see
//! `docs/API-INTENT-AUDIT.md`). This file is therefore an honest picture of the
//! current mechanism, not a silent leak: it exists to keep the gap visible.
//!
//! # What the consumer must know today
//! - `lang::parse_source` — parse
//! - `semantic::compiler::SemanticCompiler`, `semantic::resolver::SemanticResolver`
//!   — compile + resolve
//! - `planning::input::PlanningInput` — lower to planning input
//! - `planning::motion::compiler::{DefaultPlannerDispatcher, PlanCompiler}`
//! - `planning::motion::planner::SegmentPlanningContext`
//! - `core::robot::state::RobotState`, `ForwardKinematics`, `DampedLeastSquaresSolver`
//!   — to build the context
//!
//! # Mechanism leaked
//! The consumer orchestrates every stage of the pipeline and must know the
//! `DefaultPlannerDispatcher` + `PlanCompiler` pairing.
//!
//! ```ignore
//! // Does not exist today (candidate for D3-bis):
//! let plan = robot.plan_source(source)?;
//! ```

use thalos_api::catalog::{RobotModel, RobotRegistry};
use thalos_api::core::kinematics::forward::ForwardKinematics;
use thalos_api::core::kinematics::inverse::{DampedLeastSquaresSolver, IKConfig};
use thalos_api::core::robot::state::RobotState;
use thalos_api::lang::parse_source;
use thalos_api::planning::input::PlanningInput;
use thalos_api::planning::motion::compiler::{DefaultPlannerDispatcher, PlanCompiler};
use thalos_api::planning::motion::planner::SegmentPlanningContext;
use thalos_api::semantic::compiler::SemanticCompiler;
use thalos_api::semantic::resolver::SemanticResolver;

const SOURCE: &str = "
target home = joints(10deg, 20deg)
fn main() {
    movej(home);
}
";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let chain = RobotRegistry::create_default(RobotModel::Planar2R);

    // 1..4 — the language front-end, stage by stage.
    let ast = parse_source(SOURCE)
        .map_err(|errs| errs.into_iter().map(|e| e.to_string()).collect::<Vec<_>>().join("; "))?;
    let semantic = SemanticCompiler::compile(&ast).map_err(|errs| errs.join("; "))?;
    let resolved = SemanticResolver::resolve(&semantic).map_err(|errs| errs.join("; "))?;
    let planning_input = PlanningInput::from_resolved(&resolved);

    // 5 — the planning context (needs FK + IK, unused for a pure MoveJ).
    let state = RobotState::zero(chain.dof_count());
    let ik = DampedLeastSquaresSolver::from_config(
        ForwardKinematics::new(chain.clone()),
        *chain.end_effector(),
        IKConfig::default(),
    );
    let context = SegmentPlanningContext {
        robot: &chain,
        current_state: &state,
        ik_solver: &ik,
        tcp: None,
    };

    // 6 — compile the plan.
    let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
    let compiled = compiler
        .compile(&planning_input.to_program(), &context)
        .map_err(|e| e.to_string())?;

    println!(
        "plan → {} waypoints, {:.3}s",
        compiled.waypoint_count, compiled.duration
    );
    Ok(())
}
