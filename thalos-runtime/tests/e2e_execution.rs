//! E2E #2 — execution: compiled artifact → simulated runtime behavior.
//!
//! Builds the compiled artifact from real semantic intent (the same full
//! pipeline as E2E #1, via `thalos_engine::semantic::test_support`), schedules it into
//! a simulated scene, and advances the tick loop to completion. Protects the
//! RUNTIME contract:
//!
//! - the robot is FROZEN during the `Delay` window (clock advances while
//!   joints and trajectory time hold),
//! - `SetOutput(gripper, true)` and `SetOutput(gripper, false)` are each
//!   dispatched exactly once, in chronological order,
//! - the final joint configuration equals the compiled plan's final waypoint,
//! - execution reaches a terminal (`Completed`) state.

use std::sync::Arc;
use std::time::Duration;

use tokio::sync::RwLock;

use thalos_engine::core::{
    execution::runtime::{RuntimeAction, RuntimeProgram},
    ids::OperationId,
    models::{RobotModel, RobotRegistry},
    motion::target::OutputValue,
    robot::state::RobotState,
    spatial::frame::FrameRegistry,
};
use thalos_engine::planning::{
    motion::{
        compiler::{DefaultPlannerDispatcher, PlanCompiler},
        planner::SegmentPlanningContext,
        program::CompiledPlan,
    },
    resolver::MotionResolver,
    timeline::TimelineScheduler,
};
use thalos_runtime::{
    RobotController, SceneService,
    backends::controller::simulation::SimulationController,
    backends::BackendManager,
    plan::SessionStatus,
};
use thalos_engine::semantic::{
    lowering::SemanticLowering,
    test_support::{self, pick_wait_place_home_ir},
};

// ---------------------------------------------------------------------------
// Canonical pipeline (same chain as E2E #1, ending at the timed artifact)
// ---------------------------------------------------------------------------

struct PipelineArtifact {
    compiled: CompiledPlan,
    runtime: RuntimeProgram,
}

fn run_canonical_pipeline() -> PipelineArtifact {
    let program = pick_wait_place_home_ir();

    let provider = test_support::build_provider();
    let ctx = test_support::default_ctx(&provider);
    let exec = SemanticLowering::lower(&program, &ctx).expect("lowering should succeed");

    let mut registry = FrameRegistry::new();
    registry.create("world");
    let ik = test_support::FixedTargetIKSolver;
    let initial = [0.0, 0.0];
    let resolver = MotionResolver::new(&ik, &registry, &initial, 2).expect("2 DOF matches");
    let resolution = resolver.resolve(&exec).expect("resolution should succeed");

    let chain = RobotRegistry::create_default(RobotModel::Planar2R);
    let state = RobotState::zero(chain.dof_count());
    let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
    let seg_ctx = SegmentPlanningContext {
        robot: &chain,
        current_state: &state,
        ik_solver: &ik,
        tcp: None,
    };
    let compiled = compiler
        .compile(&resolution.planning, &seg_ctx)
        .expect("compilation should succeed");

    let runtime = TimelineScheduler::new().schedule(&exec, &compiled, resolution.runtime);

    PipelineArtifact { compiled, runtime }
}

// ---------------------------------------------------------------------------
// Scene harness
// ---------------------------------------------------------------------------

/// A `SceneService` over a concrete `SimulationController`, plus the concrete
/// controller handle for observing dispatched events and clock/traj time.
async fn make_service() -> (SceneService, Arc<RwLock<SimulationController>>) {
    let concrete = Arc::new(RwLock::new(SimulationController::new(
        RobotModel::Planar2R.metadata().dof,
    )));
    let controller = concrete.clone() as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(
        manager.clone(),
        RobotModel::Planar2R,
    );
    (svc, concrete)
}

/// Tick the scene forward in `dt` steps until the controller clock reaches
/// `target` (simulation time, from plan start).
async fn advance_until(
    svc: &SceneService,
    concrete: &Arc<RwLock<SimulationController>>,
    dt: f64,
    target: Duration,
) {
    loop {
        let clock = concrete.read().await.clock_time().await;
        if clock >= target {
            break;
        }
        svc.tick_execution_delta(dt).await.unwrap();
    }
}

// ---------------------------------------------------------------------------
// E2E #2
// ---------------------------------------------------------------------------

#[tokio::test]
async fn compiled_plan_executes_with_frozen_delay_and_ordered_output_dispatch() {
    let art = run_canonical_pipeline();
    let compiled = art.compiled.clone();
    let runtime = art.runtime.clone();

    // ── Expected schedule, taken from the timed artifact (E2E #1 contract) ──
    let delay_dur = runtime
        .events
        .iter()
        .find_map(|e| match &e.action {
            RuntimeAction::Delay(d) => Some(*d),
            _ => None,
        })
        .expect("Delay event present (from Wait)");
    let delay_at = runtime
        .events
        .iter()
        .find(|e| matches!(e.action, RuntimeAction::Delay(_)))
        .expect("Delay event")
        .at_time;
    let grip_at = runtime
        .events
        .iter()
        .find(|e| {
            matches!(
                &e.action,
                RuntimeAction::SetOutput {
                    value: OutputValue::Bool(true),
                    ..
                }
            )
        })
        .expect("grip SetOutput(true) event")
        .at_time;
    let ungrip_at = runtime
        .events
        .iter()
        .find(|e| {
            matches!(
                &e.action,
                RuntimeAction::SetOutput {
                    value: OutputValue::Bool(false),
                    ..
                }
            )
        })
        .expect("ungrip SetOutput(false) event")
        .at_time;

    // Final position contract: assert against the plan's own final waypoint.
    let first_waypoint = compiled
        .merged_trajectory
        .waypoints()
        .first()
        .unwrap()
        .joints()
        .to_vec();
    let final_waypoint = compiled
        .merged_trajectory
        .waypoints()
        .last()
        .unwrap()
        .joints()
        .to_vec();

    let (svc, concrete) = make_service().await;
    svc.schedule_program(compiled, runtime).await.unwrap();
    svc.start_execution().await.unwrap();

    const DT: f64 = 0.01;

    // ── 1. Execution starts at the compiled first waypoint ────────────────
    let start_joints = svc.snapshot().await.unwrap().joints;
    assert_eq!(
        start_joints, first_waypoint,
        "execution starts at the plan origin"
    );

    // ── 2. Grip fires at its scheduled absolute time ──────────────────────
    advance_until(&svc, &concrete, DT, grip_at + Duration::from_millis(1)).await;
    let dispatched = concrete.read().await.dispatched_events().await;
    assert_eq!(
        dispatched.len(),
        1,
        "grip must be the only dispatched event at this point"
    );
    assert_eq!(dispatched[0].operation_id, OperationId("op-pick".into()));
    assert_eq!(
        dispatched[0].at_time, grip_at,
        "grip fires at its scheduled time"
    );
    assert!(
        matches!(
            &dispatched[0].action,
            RuntimeAction::SetOutput {
                value: OutputValue::Bool(true),
                ..
            }
        ),
        "grip closes the gripper (true)"
    );

    // ── 3. The Delay freezes the robot while the clock advances ───────────
    advance_until(&svc, &concrete, DT, delay_at).await;
    let held_joints = svc.snapshot().await.unwrap().joints;
    let traj_at_fire = concrete.read().await.traj_time().await;
    assert_ne!(
        held_joints, start_joints,
        "the trajectory must be moving before the delay (joints changed from start)"
    );

    let window_end = delay_at + delay_dur;
    let mut prev = held_joints.clone();
    let mut frozen_ticks = 0u32;
    loop {
        let clock = concrete.read().await.clock_time().await;
        if clock >= window_end - Duration::from_millis(50) {
            break;
        }
        svc.tick_execution_delta(DT).await.unwrap();
        let clock = concrete.read().await.clock_time().await;
        assert!(
            clock < window_end,
            "sample must be inside the delay window (clock {clock:?})"
        );
        let joints = svc.snapshot().await.unwrap().joints;
        assert_eq!(
            joints, prev,
            "joints must hold while the clock advances (clock {clock:?})"
        );
        assert_eq!(
            concrete.read().await.traj_time().await,
            traj_at_fire,
            "trajectory time must be frozen during the delay (clock {clock:?})"
        );
        prev = joints;
        frozen_ticks += 1;
    }
    assert!(
        frozen_ticks >= 10,
        "the delay window must span several observed ticks (got {frozen_ticks})"
    );

    // The delay event itself is not dispatched as an output — only the two
    // SetOutput events are.
    assert_eq!(
        concrete.read().await.dispatched_events().await.len(),
        1,
        "the Delay must not produce a dispatched output event"
    );

    // ── 4. Ungrip fires after the delay; trajectory resumes ───────────────
    advance_until(&svc, &concrete, DT, ungrip_at + Duration::from_millis(1)).await;
    assert!(
        concrete.read().await.traj_time().await > traj_at_fire,
        "trajectory time must resume after the delay"
    );

    let dispatched = concrete.read().await.dispatched_events().await;
    assert_eq!(
        dispatched.len(),
        2,
        "exactly two SetOutput events dispatched in total"
    );
    assert!(
        matches!(
            &dispatched[0].action,
            RuntimeAction::SetOutput {
                value: OutputValue::Bool(true),
                ..
            }
        ),
        "first dispatched output must be grip (true)"
    );
    assert!(
        matches!(
            &dispatched[1].action,
            RuntimeAction::SetOutput {
                value: OutputValue::Bool(false),
                ..
            }
        ),
        "second dispatched output must be ungrip (false)"
    );
    assert_eq!(dispatched[0].at_time, grip_at);
    assert_eq!(dispatched[1].at_time, ungrip_at);
    assert!(
        dispatched[0].at_time < dispatched[1].at_time,
        "grip must be dispatched before ungrip"
    );

    // ── 5. Run to completion: terminal state + final position ─────────────
    let mut guard = 0u32;
    let delta = loop {
        let delta = svc.tick_execution_delta(DT).await.unwrap();
        let terminal = delta
            .execution
            .as_ref()
            .map(|e| e.status.is_terminal())
            .unwrap_or(false);
        if terminal {
            break delta;
        }
        guard += 1;
        assert!(
            guard < 20_000,
            "execution did not complete within 200s of ticks"
        );
    };
    assert_eq!(
        delta.execution.as_ref().expect("execution session").status,
        SessionStatus::Completed,
        "execution must complete normally"
    );
    assert_eq!(
        svc.snapshot().await.unwrap().joints,
        final_waypoint,
        "final joints must match the compiled plan's final waypoint"
    );
    assert_eq!(
        concrete.read().await.dispatched_events().await.len(),
        2,
        "no further outputs dispatched after completion"
    );
}
