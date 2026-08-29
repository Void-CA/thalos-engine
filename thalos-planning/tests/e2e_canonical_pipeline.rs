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
    test_support::{self, pick_wait_place_home_ir},
};

fn instruction_origin(inst: &ProgramInstruction) -> OperationId {
    match inst {
        ProgramInstruction::MoveJ { origin, .. }
        | ProgramInstruction::MoveL { origin, .. }
        | ProgramInstruction::Delay { origin, .. }
        | ProgramInstruction::SetOutput { origin, .. } => origin.clone(),
    }
}

struct PipelineArtifact {
    exec: ExecutionProgram,
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

    let runtime = TimelineScheduler::new().schedule(&exec, &compiled, resolution.runtime);

    PipelineArtifact {
        exec,
        compiled,
        runtime,
    }
}

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

    let wps = art.compiled.merged_trajectory.waypoints();
    for pair in wps.windows(2) {
        assert!(
            pair[1].timestamp() >= pair[0].timestamp(),
            "merged trajectory timestamps must be monotonic"
        );
    }

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

    assert_eq!(
        events.len(),
        3,
        "canonical scenario produces 3 runtime events"
    );

    assert!(
        events.iter().all(|e| e.at_time > Duration::ZERO),
        "all scheduled events must carry a positive absolute at_time"
    );

    for w in events.windows(2) {
        assert!(
            w[0].at_time <= w[1].at_time,
            "events must be ordered by at_time ({} >= {})",
            w[1].at_time.as_millis(),
            w[0].at_time.as_millis()
        );
    }

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

    let seg = &art.compiled.segments;
    assert_close_to(
        set_outputs[0].at_time,
        seg[1].time_range.end,
        "grip at_time",
    );
    let delay_event = events
        .iter()
        .find(|e| matches!(e.action, RuntimeAction::Delay(_)))
        .expect("Delay event");
    assert_close_to(delay_event.at_time, seg[2].time_range.end, "delay at_time");
    assert_close_to(
        set_outputs[1].at_time,
        seg[4].time_range.end + 0.300,
        "ungrip at_time",
    );
}
