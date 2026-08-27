//! Architectural property tests for the semantic IR pipeline.
//!
//! Validates that `ExecutionProgram` is a canonical IR that preserves:
//!
//! - **Shape**: each `SemanticOperation` produces the expected instruction structure.
//! - **Traceability**: `OperationId` propagates through all derived instructions.
//! - **Determinism**: same input → same output, always.
//! - **Order**: operations are never reordered during lowering.
//! - **Structural equivalence**: the lowering is a deterministic transformation,
//!   not a black box.
//! - **I2 identity**: `OperationId` survives every official IR transformation
//!   (`SemanticOperation → ProgramInstruction → MotionSegment →
//!   PlannedSegment → RuntimeEvent`).

use std::time::Duration;

use thalos_core::execution::program::ProgramInstruction;
use thalos_core::ids::OperationId;
use thalos_core::motion::{MotionTarget, OutputValue};
use thalos_semantic::{
    ir::SemanticIr,
    knowledge::{GraspPlan, MockKnowledgeProvider},
    lowering::SemanticLowering,
    operation::{HomeOp, MoveToOp, PickOp, PlaceOp, SemanticOperation, WaitOp},
    resource::{LocationId, ObjectId},
    test_support::{
        build_provider, default_ctx, lower, make_origin,
        pick_wait_place_home_ir_with_wait, sample_pose, FixedTargetIKSolver,
    },
};

// ---------------------------------------------------------------------------
// Helpers — shared with the pipeline e2e tests via `test_support`
// (canonical scenario: `pick_wait_place_home_program`/`_with_wait`, provider,
// context, lowering, and `FixedTargetIKSolver`).
// ---------------------------------------------------------------------------

// =========================================================================
// 1. Shape tests — each operation produces the expected instruction pattern
// =========================================================================

#[test]
fn pick_produces_four_instructions() {
    let program = SemanticIr::from_operations(vec![SemanticOperation::Pick(PickOp {
        origin: make_origin("op-pick"),
        object: ObjectId("bolt".into()),
        tool: None,
    })]);
    let instructions = lower(&program);
    assert_eq!(
        instructions.len(),
        4,
        "Pick should produce exactly 4 instructions"
    );
    // Shape: MoveJ → MoveL → SetOutput → MoveL
    assert!(
        matches!(instructions[0], ProgramInstruction::MoveJ { .. }),
        "pick[0] should be MoveJ"
    );
    assert!(
        matches!(instructions[1], ProgramInstruction::MoveL { .. }),
        "pick[1] should be MoveL"
    );
    assert!(
        matches!(instructions[2], ProgramInstruction::SetOutput { .. }),
        "pick[2] should be SetOutput"
    );
    assert!(
        matches!(instructions[3], ProgramInstruction::MoveL { .. }),
        "pick[3] should be MoveL"
    );
}

#[test]
fn place_produces_four_instructions() {
    let program = SemanticIr::from_operations(vec![SemanticOperation::Place(PlaceOp {
        origin: make_origin("op-place"),
        object: ObjectId("bolt".into()),
        destination: LocationId("tray".into()),
        tool: None,
    })]);
    let instructions = lower(&program);
    assert_eq!(
        instructions.len(),
        4,
        "Place should produce exactly 4 instructions"
    );
    assert!(
        matches!(instructions[0], ProgramInstruction::MoveJ { .. }),
        "place[0] should be MoveJ"
    );
    assert!(
        matches!(instructions[1], ProgramInstruction::MoveL { .. }),
        "place[1] should be MoveL"
    );
    assert!(
        matches!(instructions[2], ProgramInstruction::SetOutput { .. }),
        "place[2] should be SetOutput"
    );
    assert!(
        matches!(instructions[3], ProgramInstruction::MoveL { .. }),
        "place[3] should be MoveL"
    );
}

#[test]
fn move_to_produces_one_instruction() {
    let program = SemanticIr::from_operations(vec![SemanticOperation::MoveTo(MoveToOp {
        origin: make_origin("op-move"),
        destination: LocationId("station".into()),
        tool: None,
    })]);
    let instructions = lower(&program);
    assert_eq!(
        instructions.len(),
        1,
        "MoveTo should produce exactly 1 instruction"
    );
    assert!(
        matches!(instructions[0], ProgramInstruction::MoveJ { .. }),
        "MoveTo should produce MoveJ"
    );
}

#[test]
fn wait_produces_one_delay() {
    let program = SemanticIr::from_operations(vec![SemanticOperation::Wait(WaitOp {
        origin: make_origin("op-wait"),
        duration: Duration::from_millis(500),
    })]);
    let instructions = lower(&program);
    assert_eq!(
        instructions.len(),
        1,
        "Wait should produce exactly 1 instruction"
    );
    assert!(
        matches!(instructions[0], ProgramInstruction::Delay { .. }),
        "Wait should produce Delay"
    );
}

#[test]
fn home_produces_one_move_j() {
    let program = SemanticIr::from_operations(vec![SemanticOperation::Home(HomeOp {
        origin: make_origin("op-home"),
    })]);
    let instructions = lower(&program);
    assert_eq!(
        instructions.len(),
        1,
        "Home should produce exactly 1 instruction"
    );
    assert!(
        matches!(instructions[0], ProgramInstruction::MoveJ { .. }),
        "Home should produce MoveJ"
    );
}

// =========================================================================
// 2. Traceability — OperationId survives through lowering
// =========================================================================

#[test]
fn pick_origin_propagates_to_all_instructions() {
    let origin = make_origin("pick-42");
    let program = SemanticIr::from_operations(vec![SemanticOperation::Pick(PickOp {
        origin: origin.clone(),
        object: ObjectId("bolt".into()),
        tool: None,
    })]);
    let instructions = lower(&program);
    for (i, inst) in instructions.iter().enumerate() {
        let inst_origin = match inst {
            ProgramInstruction::MoveJ { origin, .. }
            | ProgramInstruction::MoveL { origin, .. }
            | ProgramInstruction::SetOutput { origin, .. }
            | ProgramInstruction::Delay { origin, .. } => origin,
        };
        assert_eq!(
            *inst_origin, origin,
            "instruction {i} should carry origin '{origin}'"
        );
    }
}

#[test]
fn place_origin_propagates_to_all_instructions() {
    let origin = make_origin("place-99");
    let program = SemanticIr::from_operations(vec![SemanticOperation::Place(PlaceOp {
        origin: origin.clone(),
        object: ObjectId("bolt".into()),
        destination: LocationId("tray".into()),
        tool: None,
    })]);
    let instructions = lower(&program);
    for (i, inst) in instructions.iter().enumerate() {
        let inst_origin = match inst {
            ProgramInstruction::MoveJ { origin, .. }
            | ProgramInstruction::MoveL { origin, .. }
            | ProgramInstruction::SetOutput { origin, .. }
            | ProgramInstruction::Delay { origin, .. } => origin,
        };
        assert_eq!(
            *inst_origin, origin,
            "instruction {i} should carry origin '{origin}'"
        );
    }
}

#[test]
fn home_origin_propagates() {
    let origin = make_origin("home-7");
    let program = SemanticIr::from_operations(vec![SemanticOperation::Home(HomeOp {
        origin: origin.clone(),
    })]);
    let instructions = lower(&program);
    assert_eq!(instructions.len(), 1);
    match &instructions[0] {
        ProgramInstruction::MoveJ { origin: o, .. } => {
            assert_eq!(*o, origin);
        }
        other => panic!("Expected MoveJ, got {other:?}"),
    }
}

// =========================================================================
// 3. Determinism — same input always produces the same output
// =========================================================================

#[test]
fn lowering_is_deterministic() {
    let program = SemanticIr::from_operations(vec![
        SemanticOperation::Pick(PickOp {
            origin: make_origin("op-1"),
            object: ObjectId("bolt".into()),
            tool: None,
        }),
        SemanticOperation::Place(PlaceOp {
            origin: make_origin("op-2"),
            object: ObjectId("bolt".into()),
            destination: LocationId("tray".into()),
            tool: None,
        }),
        SemanticOperation::Wait(WaitOp {
            origin: make_origin("op-3"),
            duration: Duration::from_secs(1),
        }),
        SemanticOperation::Home(HomeOp {
            origin: make_origin("op-4"),
        }),
    ]);

    let provider = build_provider();
    let ctx = default_ctx(&provider);

    let result_a = SemanticLowering::lower(&program, &ctx).expect("first lower");
    let result_b = SemanticLowering::lower(&program, &ctx).expect("second lower");

    assert_eq!(result_a, result_b, "lowering must be deterministic");
}

// =========================================================================
// 4. Order preservation — operations are never reordered
// =========================================================================

#[test]
fn operation_order_is_preserved() {
    let program = SemanticIr::from_operations(vec![
        SemanticOperation::Wait(WaitOp {
            origin: make_origin("op-1"),
            duration: Duration::from_millis(100),
        }),
        SemanticOperation::Pick(PickOp {
            origin: make_origin("op-2"),
            object: ObjectId("bolt".into()),
            tool: None,
        }),
        SemanticOperation::Home(HomeOp {
            origin: make_origin("op-3"),
        }),
    ]);

    let instructions = lower(&program);

    // Wait → Delay
    assert!(
        matches!(instructions[0], ProgramInstruction::Delay { .. }),
        "first operation (Wait) should produce the first instruction"
    );
    // Pick → 4 instructions (MoveJ, MoveL, SetOutput, MoveL)
    assert!(
        matches!(instructions[1], ProgramInstruction::MoveJ { .. }),
        "Pick should start at instruction 1"
    );
    assert!(
        matches!(instructions[4], ProgramInstruction::MoveL { .. }),
        "Pick should end at instruction 4"
    );
    // Home → MoveJ
    assert!(
        matches!(instructions[5], ProgramInstruction::MoveJ { .. }),
        "Home should start at instruction 5"
    );

    assert_eq!(
        instructions.len(),
        6,
        "Wait(1) + Pick(4) + Home(1) = 6 instructions"
    );
}

// =========================================================================
// 5. Full pipeline — Pick → Wait → Place → Home (cross-cutting)
// =========================================================================

#[test]
fn pick_wait_place_home_full_pipeline() {
    let program = pick_wait_place_home_ir_with_wait(Duration::from_millis(500));

    let instructions = lower(&program);

    // Total: Pick(4) + Wait(1) + Place(4) + Home(1) = 10
    assert_eq!(
        instructions.len(),
        10,
        "full pipeline should produce 10 instructions"
    );

    // ── Pick approach [0]: MoveJ with approach_frame from provider ──
    match &instructions[0] {
        ProgramInstruction::MoveJ {
            target, profile, ..
        } => {
            assert_eq!(
                *target,
                MotionTarget::Pose(sample_pose(0.3, 0.0, 0.2)),
                "Pick approach should use approach_frame from GraspPlan"
            );
            assert!(profile.max_velocity > 0.0);
        }
        _ => panic!("instructions[0] should be MoveJ (Pick approach)"),
    }

    // ── Pick grasp [1]: MoveL with grasp_frame from provider ──
    match &instructions[1] {
        ProgramInstruction::MoveL { target, .. } => {
            assert_eq!(
                *target,
                MotionTarget::Pose(sample_pose(0.5, 0.0, 0.0)),
                "Pick grasp should use grasp_frame from GraspPlan"
            );
        }
        _ => panic!("instructions[1] should be MoveL (Pick grasp)"),
    }

    // ── Pick grip [2]: SetOutput(true) ──
    match &instructions[2] {
        ProgramInstruction::SetOutput { channel, value, .. } => {
            assert_eq!(
                channel.name, "gripper",
                "Pick grip should use gripper channel"
            );
            assert_eq!(
                *value,
                OutputValue::Bool(true),
                "Pick grip should close gripper (true)"
            );
        }
        _ => panic!("instructions[2] should be SetOutput (Pick grip)"),
    }

    // ── Pick retract [3]: MoveL with retreat_frame from provider ──
    match &instructions[3] {
        ProgramInstruction::MoveL { target, .. } => {
            assert_eq!(
                *target,
                MotionTarget::Pose(sample_pose(0.6, 0.0, 0.1)),
                "Pick retract should use retreat_frame from GraspPlan"
            );
        }
        _ => panic!("instructions[3] should be MoveL (Pick retract)"),
    }

    // ── Wait [4]: Delay ──
    match &instructions[4] {
        ProgramInstruction::Delay { duration, .. } => {
            assert_eq!(*duration, Duration::from_millis(500));
        }
        _ => panic!("instructions[4] should be Delay"),
    }

    // ── Place approach [5]: MoveJ with approach_frame from provider ──
    match &instructions[5] {
        ProgramInstruction::MoveJ { target, .. } => {
            assert_eq!(
                *target,
                MotionTarget::Pose(sample_pose(0.4, 0.3, 0.2)),
                "Place approach should use approach_frame from PlacementPlan"
            );
        }
        _ => panic!("instructions[5] should be MoveJ (Place approach)"),
    }

    // ── Place drop [6]: MoveL with drop_frame from provider ──
    match &instructions[6] {
        ProgramInstruction::MoveL { target, .. } => {
            assert_eq!(
                *target,
                MotionTarget::Pose(sample_pose(0.4, 0.5, 0.0)),
                "Place drop should use drop_frame from PlacementPlan"
            );
        }
        _ => panic!("instructions[6] should be MoveL (Place drop)"),
    }

    // ── Place ungrip [7]: SetOutput(false) ──
    match &instructions[7] {
        ProgramInstruction::SetOutput { channel, value, .. } => {
            assert_eq!(
                channel.name, "gripper",
                "Place ungrip should use gripper channel"
            );
            assert_eq!(
                *value,
                OutputValue::Bool(false),
                "Place ungrip should open gripper (false)"
            );
        }
        _ => panic!("instructions[7] should be SetOutput (Place ungrip)"),
    }

    // ── Place retract [8]: MoveL with retreat_frame from provider ──
    match &instructions[8] {
        ProgramInstruction::MoveL { target, .. } => {
            assert_eq!(
                *target,
                MotionTarget::Pose(sample_pose(0.4, 0.6, 0.1)),
                "Place retract should use retreat_frame from PlacementPlan"
            );
        }
        _ => panic!("instructions[8] should be MoveL (Place retract)"),
    }

    // ── Home [9]: MoveJ ──
    match &instructions[9] {
        ProgramInstruction::MoveJ { target, .. } => {
            assert_eq!(
                *target,
                MotionTarget::Pose(sample_pose(0.0, 0.0, 0.0)),
                "Home should use home_pose from provider"
            );
        }
        _ => panic!("instructions[9] should be MoveJ (Home)"),
    }

    // Traceability
    for i in 0..4 {
        let origin = match &instructions[i] {
            ProgramInstruction::MoveJ { origin, .. }
            | ProgramInstruction::MoveL { origin, .. }
            | ProgramInstruction::SetOutput { origin, .. } => origin,
            _ => panic!("unexpected instruction type at {i}"),
        };
        assert_eq!(
            *origin,
            OperationId("op-pick".to_string()),
            "instruction {i} should carry Pick origin"
        );
    }
    match &instructions[4] {
        ProgramInstruction::Delay { origin, .. } => {
            assert_eq!(*origin, OperationId("op-wait".to_string()));
        }
        _ => panic!("instruction[4] should be Delay"),
    }
    for i in 5..9 {
        let origin = match &instructions[i] {
            ProgramInstruction::MoveJ { origin, .. }
            | ProgramInstruction::MoveL { origin, .. }
            | ProgramInstruction::SetOutput { origin, .. } => origin,
            _ => panic!("unexpected instruction type at {i}"),
        };
        assert_eq!(
            *origin,
            OperationId("op-place".to_string()),
            "instruction {i} should carry Place origin"
        );
    }
    match &instructions[9] {
        ProgramInstruction::MoveJ { origin, .. } => {
            assert_eq!(*origin, OperationId("op-home".to_string()));
        }
        _ => panic!("instruction[9] should be MoveJ"),
    }
}

// ── Two independent Pick operations ──
#[test]
fn two_picks_produce_eight_instructions() {
    let program = SemanticIr::from_operations(vec![
        SemanticOperation::Pick(PickOp {
            origin: make_origin("pick-1"),
            object: ObjectId("bolt".into()),
            tool: None,
        }),
        SemanticOperation::Pick(PickOp {
            origin: make_origin("pick-2"),
            object: ObjectId("bolt".into()),
            tool: None,
        }),
    ]);
    let instructions = lower(&program);
    assert_eq!(
        instructions.len(),
        8,
        "two Picks should produce 8 instructions"
    );
    match &instructions[2] {
        ProgramInstruction::SetOutput { value, .. } => {
            assert_eq!(*value, OutputValue::Bool(true), "first Pick grip");
        }
        _ => panic!(),
    }
    match &instructions[6] {
        ProgramInstruction::SetOutput { value, .. } => {
            assert_eq!(*value, OutputValue::Bool(true), "second Pick grip");
        }
        _ => panic!(),
    }
}

// =========================================================================
// 6. Task Script → SemanticProgram integration
// =========================================================================

#[test]
fn task_script_parses_to_semantic_program() {
    let script = "\
# Assemble bolt
pick bolt tool=gripper-1
wait 500ms
place bolt at tray
home";

    let program = thalos_semantic::script::parse(script).expect("parse");
    assert_eq!(program.operations.len(), 4);
    assert!(matches!(program.operations[0], SemanticOperation::Pick(_)));
    assert!(matches!(program.operations[1], SemanticOperation::Wait(_)));
    assert!(matches!(program.operations[2], SemanticOperation::Place(_)));
    assert!(matches!(program.operations[3], SemanticOperation::Home(_)));
}

#[test]
fn task_script_round_trips_through_lowering() {
    let script = "\
pick bolt
wait 500ms
home";

    let program = thalos_semantic::script::parse(script).expect("parse");
    assert_eq!(program.operations.len(), 3);

    // Lower and verify structure
    use thalos_core::motion::MotionProfile;
    use thalos_semantic::lowering::SemanticLowering;
    use thalos_semantic::lowering::context::LoweringContext;

    let provider = MockKnowledgeProvider::new()
        .with_grasp_ok(
            ObjectId("bolt".into()),
            GraspPlan {
                grasp_frame: sample_pose(0.5, 0.0, 0.0),
                approach_frame: sample_pose(0.55, 0.0, 0.0),
                retreat_frame: sample_pose(0.45, 0.0, 0.0),
                preferred_tool: None,
            },
        )
        .with_home_pose(Ok(sample_pose(0.0, 0.0, 0.0)));

    let ctx = LoweringContext {
        provider: &provider,
        default_tool: None,
        default_profile: MotionProfile {
            max_velocity: 1.0,
            max_acceleration: 0.5,
            max_jerk: None,
        },
        // Legacy caller: cartesian instructions fall back to the joint
        // profile (backward compatible).
        default_cartesian_profile: None,
    };

    let motion = SemanticLowering::lower(&program, &ctx).expect("lower");
    // Pick(4) + Wait(1) + Home(1) = 6
    assert_eq!(motion.instructions.len(), 6);
}

// =========================================================================
// 7. I2 — OperationId identity across the full canonical pipeline
// =========================================================================
//
// The SAME `OperationId` must survive every official IR transformation:
//
// ```text
// SemanticOperation → ProgramInstruction → MotionSegment → PlannedSegment
//                                                                      ↘
//                                                           RuntimeEvent
// ```
//
// The full four-stage e2e identity is a PR 4 success criterion; this slice
// asserts every stage reachable after PR 1: lowering (IR-0 → IR-1),
// resolution (IR-1 → IR-2 + runtime events), and compilation (IR-2 → IR-3).

use thalos_core::{
    models::{RobotModel, RobotRegistry},
    robot::{serial_chain::SerialChain, state::RobotState},
    spatial::frame::FrameRegistry,
};
use thalos_planning::{
    motion::{
        compiler::{DefaultPlannerDispatcher, PlanCompiler},
        planner::SegmentPlanningContext,
    },
    resolver::MotionResolver,
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

/// Runs the full canonical pipeline for a `SemanticProgram` and returns the
/// origin observed at each IR stage:
/// 1. every `ProgramInstruction`,
/// 2. every `MotionSegment` (resolver output),
/// 3. every `PlannedSegment` (compiler output),
/// 4. every `RuntimeEvent` (resolver output).
#[allow(clippy::type_complexity)]
fn run_pipeline(
    program: SemanticIr,
) -> (
    Vec<OperationId>,
    Vec<OperationId>,
    Vec<OperationId>,
    Vec<OperationId>,
) {
    // ── Stage 0 → 1: SemanticLowering → ExecutionProgram ──────────────────
    let provider = build_provider();
    let ctx = default_ctx(&provider);
    let exec = SemanticLowering::lower(&program, &ctx).expect("lowering should succeed");
    let instruction_origins: Vec<OperationId> =
        exec.instructions.iter().map(instruction_origin).collect();

    // ── Stage 1 → 2 + runtime: MotionResolver → MotionSegment + RuntimeEvent ──
    let mut registry = FrameRegistry::new();
    registry.create("world");
    let ik = FixedTargetIKSolver;
    let initial = [0.0, 0.0];
    // Planar2R is a 2-DOF robot — expected_dof must match initial_state.
    let resolver = MotionResolver::new(&ik, &registry, &initial, 2).expect("2 DOF matches");
    let resolution = resolver.resolve(&exec).expect("resolution should succeed");

    let segment_origins: Vec<OperationId> = resolution
        .planning
        .segments
        .iter()
        .map(|seg| seg.origin().clone())
        .collect();
    let event_origins: Vec<OperationId> = resolution
        .runtime
        .events
        .iter()
        .map(|ev| ev.operation_id.clone())
        .collect();

    // ── Stage 2 → 3: PlanCompiler → PlannedSegment ────────────────────────
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
    let planned_origins: Vec<OperationId> = compiled
        .segments
        .iter()
        .map(|seg| seg.origin.clone())
        .collect();

    (
        instruction_origins,
        segment_origins,
        planned_origins,
        event_origins,
    )
}

#[test]
fn i2_operation_id_identity_across_full_pipeline() {
    let origin = make_origin("op-7");
    let program = SemanticIr::from_operations(vec![
        SemanticOperation::Pick(PickOp {
            origin: origin.clone(),
            object: ObjectId("bolt".into()),
            tool: None,
        }),
        SemanticOperation::Wait(WaitOp {
            origin: origin.clone(),
            duration: Duration::from_millis(300),
        }),
        SemanticOperation::Home(HomeOp {
            origin: origin.clone(),
        }),
    ]);

    let (instructions, segments, planned, events) = run_pipeline(program);

    // Stage 1 — every ProgramInstruction carries the origin.
    assert!(
        !instructions.is_empty(),
        "pipeline must produce instructions"
    );
    for (i, o) in instructions.iter().enumerate() {
        assert_eq!(
            o, &origin,
            "ProgramInstruction[{i}] must carry origin 'op-7'"
        );
    }
    // Stage 2 — every MotionSegment carries the origin.
    assert!(
        !segments.is_empty(),
        "pipeline must produce motion segments"
    );
    for (i, o) in segments.iter().enumerate() {
        assert_eq!(o, &origin, "MotionSegment[{i}] must carry origin 'op-7'");
    }
    // Stage 3 — every PlannedSegment carries the origin.
    assert!(
        !planned.is_empty(),
        "pipeline must produce planned segments"
    );
    for (i, o) in planned.iter().enumerate() {
        assert_eq!(o, &origin, "PlannedSegment[{i}] must carry origin 'op-7'");
    }
    // Stage 4 — every RuntimeEvent carries the origin.
    assert!(!events.is_empty(), "pipeline must produce runtime events");
    for (i, o) in events.iter().enumerate() {
        assert_eq!(o, &origin, "RuntimeEvent[{i}] must carry origin 'op-7'");
    }
}

#[test]
fn i2_origin_identity_with_different_operations_and_origin() {
    // Triangulation: a distinct origin flowing through different operations
    // (MoveTo → MoveJ; Place → MoveJ, MoveL, SetOutput, MoveL) must survive
    // every stage unchanged.
    let origin = make_origin("op-99");
    let program = SemanticIr::from_operations(vec![
        SemanticOperation::MoveTo(MoveToOp {
            origin: origin.clone(),
            destination: LocationId("station".into()),
            tool: None,
        }),
        SemanticOperation::Place(PlaceOp {
            origin: origin.clone(),
            object: ObjectId("bolt".into()),
            destination: LocationId("tray".into()),
            tool: None,
        }),
    ]);

    let (instructions, segments, planned, events) = run_pipeline(program);

    assert!(!instructions.is_empty());
    for (i, o) in instructions.iter().enumerate() {
        assert_eq!(
            o, &origin,
            "ProgramInstruction[{i}] must carry origin 'op-99'"
        );
    }
    assert!(!segments.is_empty());
    for (i, o) in segments.iter().enumerate() {
        assert_eq!(o, &origin, "MotionSegment[{i}] must carry origin 'op-99'");
    }
    assert!(!planned.is_empty());
    for (i, o) in planned.iter().enumerate() {
        assert_eq!(o, &origin, "PlannedSegment[{i}] must carry origin 'op-99'");
    }
    assert!(!events.is_empty());
    for (i, o) in events.iter().enumerate() {
        assert_eq!(o, &origin, "RuntimeEvent[{i}] must carry origin 'op-99'");
    }
}
