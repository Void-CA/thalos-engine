//! E2E #1 — canonical pipeline: intent → correctly timed program artifact.
//!
//! Chains the FULL compiler pipeline end-to-end — the same chain the
//! `POST /motion/plan` API entry point runs, driven from a real semantic
//! program:
//!
//! ```text
//! SemanticProgram → SemanticLowering → ExecutionProgram → MotionResolver →
//! PlanningProgram + RuntimeProgram → PlanCompiler → CompiledPlan →
//! TimelineScheduler → RuntimeProgram (timed)
//! ```
//!
//! Each layer is unit-tested in isolation; this test protects the COMPILER
//! contract as a whole:
//!
//! - `OperationId` survives every stage (`SemanticOperation` →
//!   `ProgramInstruction` → `PlannedSegment` → `RuntimeEvent`).
//! - The `CompiledPlan` is non-empty and the sequence ends with the Home
//!   `MoveJ`.
//! - The `TimelineScheduler` output is a genuinely *timed* `RuntimeProgram`:
//!   events carry strictly non-decreasing absolute `at_time`, exactly one
//!   `Delay` (from the `Wait`), exactly two `SetOutput` events (gripper
//!   `true` then `false`), and the event times align with the compiled
//!   segment timing.

use std::time::Duration;

use thalos_core::{
    execution::program::{ExecutionProgram, ProgramInstruction},
    execution::runtime::{RuntimeAction, RuntimeEvent, RuntimeProgram},
    ids::OperationId,
    models::{RobotModel, RobotRegistry},
    motion::segment::MotionSegment,
    motion::target::OutputValue,
    robot::{serial_chain::SerialChain, state::RobotState},
    spatial::frame::FrameRegistry,
};
use thalos_planning::{
    motion::{
        compiler::{DefaultPlannerDispatcher, PlanCompiler},
        planner::SegmentPlanningContext,
        program::CompiledPlan,
    },
    resolver::MotionResolver,
    timeline::TimelineScheduler,
};
use thalos_semantic::{
    lowering::SemanticLowering,
    test_support::{self, pick_wait_place_home_program},
};

/// The `OperationId` carried by an `ProgramInstruction` (all four variants).
fn instruction_origin(inst: &ProgramInstruction) -> OperationId {
    match inst {
        ProgramInstruction::MoveJ { origin, .. }
        | ProgramInstruction::MoveL { origin, .. }
        | ProgramInstruction::Delay { origin, .. }
        | ProgramInstruction::SetOutput { origin, .. } => origin.clone(),
    }
}

/// The artifact produced by the full canonical pipeline.
struct PipelineArtifact {
    /// IR-1: lowered instruction program (input to the resolver).
    exec: ExecutionProgram,
    /// IR-3: compiled trajectory with per-segment timing.
    compiled: CompiledPlan,
    /// IR-4: temporal events, absolute `at_time` assigned by `TimelineScheduler`.
    runtime: RuntimeProgram,
}

/// Run the canonical scenario through the entire compiler pipeline, including
/// the `TimelineScheduler` step that the isolated unit tests never reach.
fn run_canonical_pipeline() -> PipelineArtifact {
    let program = pick_wait_place_home_program();

    // ── IR-0 → IR-1: SemanticLowering → ExecutionProgram ────────────────
    let provider = test_support::build_provider();
    let ctx = test_support::default_ctx(&provider);
    let exec = SemanticLowering::lower(&program, &ctx).expect("lowering should succeed");

    // ── IR-1 → IR-2 + runtime events: MotionResolver ─────────────────────
    let mut registry = FrameRegistry::new();
    registry.create("world");
    let ik = test_support::FixedTargetIKSolver;
    let initial = [0.0, 0.0];
    let resolver = MotionResolver::new(&ik, &registry, &initial, 2).expect("2 DOF matches");
    let resolution = resolver.resolve(&exec).expect("resolution should succeed");

    // ── IR-2 → IR-3: PlanCompiler → CompiledPlan ─────────────────────────
    let chain: SerialChain = RobotRegistry::create_default(RobotModel::Planar2R);
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

    // ── IR-3 → IR-4: TimelineScheduler → RuntimeProgram (timed) ──────────
    let runtime = TimelineScheduler::new().schedule(&exec, &compiled, resolution.runtime);

    PipelineArtifact {
        exec,
        compiled,
        runtime,
    }
}

/// Assert that `actual` matches an expected absolute time derived from the
/// compiled segment timing. Tolerates the f64 → `Duration` conversion noise
/// (the scheduler accumulates `Duration::from_secs_f64` per segment, while
/// `PlannedSegment::time_range` stores the summed f64 directly).
fn assert_close_to(actual: Duration, expected_secs: f64, context: &str) {
    let diff = (actual.as_secs_f64() - expected_secs).abs();
    assert!(
        diff < 1e-6,
        "{context}: at_time {actual:?} does not match expected {expected_secs}s (diff {diff})"
    );
}

#[test]
fn canonical_pipeline_compiles_to_nonempty_plan_with_origin_trace() {
    let art = run_canonical_pipeline();

    // ── CompiledPlan artifact is non-empty (compiler contract) ───────────
    assert!(
        !art.compiled.segments.is_empty(),
        "CompiledPlan must have segments"
    );
    assert_eq!(
        art.compiled.segments.len(),
        7,
        "Pick(3) + Place(3) + Home(1) = 7 motion segments"
    );
    assert!(art.compiled.waypoint_count > 0, "plan must have waypoints");
    assert!(
        art.compiled.merged_trajectory.len() >= 2,
        "merged trajectory must have at least 2 waypoints"
    );
    assert!(
        art.compiled.duration > 0.0,
        "plan must have finite duration"
    );

    // Merged trajectory timestamps are monotonic.
    let wps = art.compiled.merged_trajectory.waypoints();
    for pair in wps.windows(2) {
        assert!(
            pair[1].timestamp() >= pair[0].timestamp(),
            "merged trajectory timestamps must be monotonic"
        );
    }

    // ── OperationId preserved at every stage (invariant I2) ──────────────
    let instruction_origins: Vec<OperationId> = art
        .exec
        .instructions
        .iter()
        .map(instruction_origin)
        .collect();
    assert_eq!(
        instruction_origins,
        vec![
            OperationId("op-pick".into()),
            OperationId("op-pick".into()),
            OperationId("op-pick".into()),
            OperationId("op-pick".into()),
            OperationId("op-wait".into()),
            OperationId("op-place".into()),
            OperationId("op-place".into()),
            OperationId("op-place".into()),
            OperationId("op-place".into()),
            OperationId("op-home".into()),
        ],
        "ProgramInstruction origins must follow the canonical program order"
    );

    let segment_origins: Vec<OperationId> = art
        .compiled
        .segments
        .iter()
        .map(|s| s.origin.clone())
        .collect();
    assert_eq!(
        segment_origins,
        vec![
            OperationId("op-pick".into()),
            OperationId("op-pick".into()),
            OperationId("op-pick".into()),
            OperationId("op-place".into()),
            OperationId("op-place".into()),
            OperationId("op-place".into()),
            OperationId("op-home".into()),
        ],
        "PlannedSegment origins must survive compilation"
    );

    let event_origins: Vec<OperationId> = art
        .runtime
        .events
        .iter()
        .map(|e| e.operation_id.clone())
        .collect();
    assert_eq!(
        event_origins,
        vec![
            OperationId("op-pick".into()),
            OperationId("op-wait".into()),
            OperationId("op-place".into()),
        ],
        "RuntimeEvent origins must survive scheduling"
    );

    // ── The sequence ends with the Home MoveJ ────────────────────────────
    match &art.compiled.segments.last().unwrap().source {
        MotionSegment::MoveJ { origin, .. } => {
            assert_eq!(
                *origin,
                OperationId("op-home".into()),
                "last segment is Home MoveJ"
            );
        }
        other => panic!("expected the final segment to be a Home MoveJ, got {other:?}"),
    }
    // The merged trajectory ends at the Home target.
    let last_wp = wps.last().unwrap();
    assert!(
        last_wp.timestamp() > 0.0,
        "final waypoint must be reached after motion"
    );
}

#[test]
fn timed_events_align_to_compiled_segment_timing() {
    let art = run_canonical_pipeline();
    let events = &art.runtime.events;

    // ── Exactly three timed events: grip, delay, ungrip ──────────────────
    assert_eq!(
        events.len(),
        3,
        "canonical scenario produces 3 runtime events"
    );

    // Every event is genuinely timed (the resolver's logical events are all
    // zero; the scheduler must assign positive absolute times).
    assert!(
        events.iter().all(|e| e.at_time > Duration::ZERO),
        "all scheduled events must carry a positive absolute at_time"
    );

    // Strictly non-decreasing absolute time (spec: RuntimeProgram Structure).
    for w in events.windows(2) {
        assert!(
            w[0].at_time <= w[1].at_time,
            "events must be ordered by at_time ({} >= {})",
            w[1].at_time.as_millis(),
            w[0].at_time.as_millis()
        );
    }

    // ── Exactly one Delay, from the Wait(300ms) ──────────────────────────
    let delay_dur = events
        .iter()
        .find_map(|e| match &e.action {
            RuntimeAction::Delay(d) => Some(*d),
            _ => None,
        })
        .expect("a Delay event must be present (from Wait)");
    assert_eq!(
        delay_dur,
        Duration::from_millis(300),
        "Wait(300ms) must lower to a 300ms Delay"
    );

    // ── Exactly two SetOutput events: gripper true (grip) then false (ungrip) ──
    let set_outputs: Vec<&RuntimeEvent> = events
        .iter()
        .filter(|e| matches!(e.action, RuntimeAction::SetOutput { .. }))
        .collect();
    assert_eq!(
        set_outputs.len(),
        2,
        "exactly two SetOutput events (grip + ungrip)"
    );
    match (&set_outputs[0].action, &set_outputs[1].action) {
        (
            RuntimeAction::SetOutput {
                channel,
                value: OutputValue::Bool(true),
            },
            RuntimeAction::SetOutput {
                channel: _,
                value: OutputValue::Bool(false),
            },
        ) => {
            assert_eq!(
                channel.name, "gripper",
                "grip must target the gripper channel"
            );
        }
        _ => panic!(
            "expected SetOutput(true) before SetOutput(false), got {:?}",
            events
        ),
    }
    assert!(
        set_outputs[0].at_time < set_outputs[1].at_time,
        "grip must fire before ungrip"
    );

    // ── Absolute times derived from the compiled segment timing ──────────
    // Instruction stream: MoveJ, MoveL, SetOutput, MoveL, Delay, MoveJ, MoveL,
    // SetOutput, MoveL, MoveJ. The cursor advances by each motion segment's
    // duration (`time_range.end - time_range.start`); segments are contiguous
    // from t=0, so the cursor after segment *k* equals `segments[k].end`.
    let seg = &art.compiled.segments;
    // SetOutput(true) fires when the Pick grasp MoveL (segment 1) completes.
    assert_close_to(
        set_outputs[0].at_time,
        seg[1].time_range.end,
        "grip at_time",
    );
    // The Delay starts when the Pick retract MoveL (segment 2) completes.
    let delay_event = events
        .iter()
        .find(|e| matches!(e.action, RuntimeAction::Delay(_)))
        .expect("Delay event");
    assert_close_to(delay_event.at_time, seg[2].time_range.end, "delay at_time");
    // SetOutput(false) fires after the delay + Place approach + Place drop
    // (segment 4 completes, plus the 300ms delay shifted the cursor).
    assert_close_to(
        set_outputs[1].at_time,
        seg[4].time_range.end + 0.300,
        "ungrip at_time",
    );
}
