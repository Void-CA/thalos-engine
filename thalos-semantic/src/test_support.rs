//! Shared test support for the canonical semantic scenario.
//!
//! Compiled only under the `test-support` feature. Provides a single source of
//! truth for the canonical `Pick → Wait → Place → Home` program and its
//! lowering/knowledge scaffolding, consumed by:
//!
//! - `tests/ir_properties.rs` — architectural property tests,
//! - `tests/e2e_canonical_pipeline.rs` — E2E #1 (compiler contract),
//! - `thalos-runtime/tests/e2e_execution.rs` — E2E #2 (runtime contract).
//!
//! The canonical scenario is `Pick(bolt, op-pick)` → `Wait(300ms, op-wait)` →
//! `Place(bolt→tray, op-place)` → `Home(op-home)`.

use std::time::Duration;

use thalos_core::{
    execution::program::ExecutionInstruction,
    ids::OperationId,
    kinematics::inverse::{IKGoal, IKResult, IKSolver, IkError},
    motion::{MotionPose, MotionProfile},
};

use crate::{
    knowledge::{GraspPlan, MockKnowledgeProvider, PlacementPlan},
    lowering::{SemanticLowering, context::LoweringContext},
    operation::{HomeOp, PickOp, PlaceOp, SemanticOperation, WaitOp},
    program::SemanticProgram,
    resource::{LocationId, ObjectId, ToolId},
};

/// The canonical `Pick → Wait → Place → Home` program with a 300 ms wait.
pub fn pick_wait_place_home_program() -> SemanticProgram {
    pick_wait_place_home_with_wait(Duration::from_millis(300))
}

/// The canonical `Pick → Wait → Place → Home` program with a configurable wait.
///
/// The only variance between consumers is the `Wait` duration (the existing
/// `pick_wait_place_home_full_pipeline` property test asserts 500 ms); every
/// other field is identical, so construction lives here exactly once.
pub fn pick_wait_place_home_with_wait(wait: Duration) -> SemanticProgram {
    SemanticProgram::new(vec![
        SemanticOperation::Pick(PickOp {
            origin: make_origin("op-pick"),
            object: ObjectId("bolt".into()),
            tool: None,
        }),
        SemanticOperation::Wait(WaitOp {
            origin: make_origin("op-wait"),
            duration: wait,
        }),
        SemanticOperation::Place(PlaceOp {
            origin: make_origin("op-place"),
            object: ObjectId("bolt".into()),
            destination: LocationId("tray".into()),
            tool: None,
        }),
        SemanticOperation::Home(HomeOp {
            origin: make_origin("op-home"),
        }),
    ])
}

pub fn sample_pose(x: f64, y: f64, z: f64) -> MotionPose {
    MotionPose {
        position: [x, y, z],
        orientation: [0.0, 0.0, 0.0, 1.0],
        frame: "world".into(),
    }
}

pub fn make_origin(s: &str) -> OperationId {
    OperationId(s.to_string())
}

/// A `KnowledgeProvider` preconfigured for the canonical scenario: `bolt` has a
/// grasp plan, `bolt@tray` has a placement plan, `station` is a known location,
/// and the home pose is the origin.
pub fn build_provider() -> MockKnowledgeProvider {
    let grasp = GraspPlan {
        grasp_frame: sample_pose(0.5, 0.0, 0.0),
        approach_frame: sample_pose(0.3, 0.0, 0.2),
        retreat_frame: sample_pose(0.6, 0.0, 0.1),
        preferred_tool: Some(ToolId("gripper-1".to_string())),
    };
    let place = PlacementPlan {
        drop_frame: sample_pose(0.4, 0.5, 0.0),
        approach_frame: sample_pose(0.4, 0.3, 0.2),
        retreat_frame: sample_pose(0.4, 0.6, 0.1),
    };

    MockKnowledgeProvider::new()
        .with_grasp_ok(ObjectId("bolt".into()), grasp)
        .with_place_ok(ObjectId("bolt".into()), LocationId("tray".into()), place)
        .with_location_ok(LocationId("station".into()), sample_pose(1.0, 0.0, 0.0))
        .with_home_pose(Ok(sample_pose(0.0, 0.0, 0.0)))
}

/// A `LoweringContext` over the canonical provider.
pub fn default_ctx(provider: &MockKnowledgeProvider) -> LoweringContext<'_> {
    LoweringContext {
        provider,
        default_tool: Some(ToolId("gripper-1".to_string())),
        default_profile: MotionProfile {
            max_velocity: 1.0,
            max_acceleration: 0.5,
            max_jerk: None,
        },
        // Legacy caller: cartesian instructions fall back to the joint
        // profile (backward compatible).
        default_cartesian_profile: None,
    }
}

/// Lower a program with the canonical provider/context and return its
/// instructions (IR-0 → IR-1).
pub fn lower(program: SemanticProgram) -> Vec<ExecutionInstruction> {
    let provider = build_provider();
    let ctx = default_ctx(&provider);
    let mp = SemanticLowering::lower(&program, &ctx).expect("lowering should succeed");
    mp.instructions
}

/// IK solver that returns a FIXED joint target — lets the resolver produce a
/// real joint-space `MoveJ` without coupling the test to a specific robot.
pub struct FixedTargetIKSolver;

impl IKSolver for FixedTargetIKSolver {
    fn solve(&self, _q0: &[f64], _goal: IKGoal) -> Result<IKResult, IkError> {
        Ok(IKResult::converged(vec![0.5, 0.3], 1, 0.0, None))
    }
}
