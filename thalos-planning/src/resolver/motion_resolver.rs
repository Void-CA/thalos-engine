use std::time::Duration;

use thalos_core::{
    execution::{
        program::{ExecutionProgram, ProgramInstruction},
        runtime::{RuntimeAction, RuntimeEvent, RuntimeProgram},
    },
    kinematics::inverse::{IKGoal, IKSolver, IKStatus},
    motion::segment::MotionSegment,
    motion::target::MotionTarget,
    spatial::{
        frame::{FrameId, FrameRegistry},
        pose::Pose,
    },
};
use thalos_math::{Quaternion, Transform3D, UnitQuaternion, Vector3};

use super::types::{MotionResolution, ResolutionError};
use crate::motion::planner::PlanningContext;
use crate::motion::program::{PlanningProgram, SemanticTarget};
use thalos_core::robot::state::RobotState;

/// Resolves an `ExecutionProgram` into separate planning and runtime streams.
///
/// - `MoveJ` and `MoveL` instructions → `PlanningProgram` segments
///   (requires IK for joint-space resolution of `MoveJ`).
/// - `Delay` and `SetOutput` instructions → `RuntimeProgram` events
///
/// # Invariants
///
/// - **Order preservation**: Instructions are processed sequentially; output
///   order matches input order.
/// - **Origin preservation (I2)**: Each segment/event copies the `origin`
///   `OperationId` from its source instruction; no transformation drops or
///   renames an identity.
/// - **Determinism**: No I/O, no side effects, no global state.
/// - **Atomic fail**: On any error, no partial `MotionResolution` is returned.
pub struct MotionResolver<'a> {
    ik_solver: &'a dyn IKSolver,
    frame_registry: &'a FrameRegistry,
    initial_state: &'a [f64],
}

impl<'a> MotionResolver<'a> {
    /// Create a new resolver for a given IK solver, frame registry, and
    /// initial robot joint state.
    ///
    /// The `initial_state` seeds the IK solver on the first `MoveJ`
    /// instruction and is tracked internally through all subsequent moves.
    ///
    /// # DOF validation (invariant I1)
    ///
    /// `expected_dof` is the DOF of the robot model the caller resolved for
    /// (e.g. `robot_model.metadata().dof`). It MUST match the length of
    /// `initial_state`; a mismatch is rejected with
    /// [`ResolutionError::DofMismatch`] at construction — fail fast, the
    /// resolver never sees inconsistent state.
    ///
    /// # Design note
    ///
    /// The `initial_state` parameter extends the design from `design.md`
    /// — the original interface did not include it, but IK requires a `q0`
    /// seed. Without it, `MoveJ` resolution cannot determine the robot's
    /// starting configuration.
    pub fn new(
        ik_solver: &'a dyn IKSolver,
        frame_registry: &'a FrameRegistry,
        initial_state: &'a [f64],
        expected_dof: usize,
    ) -> Result<Self, ResolutionError> {
        if initial_state.len() != expected_dof {
            return Err(ResolutionError::DofMismatch {
                expected: expected_dof,
                actual: initial_state.len(),
            });
        }
        Ok(Self {
            ik_solver,
            frame_registry,
            initial_state,
        })
    }

    /// Resolve an `ExecutionProgram` into `MotionResolution`.
    ///
    /// Processes instructions in order. Each instruction maps to exactly one
    /// output element in either the planning or runtime stream (invariant:
    /// completeness). On failure, no partial result is returned.
    pub fn resolve(&self, program: &ExecutionProgram) -> Result<MotionResolution, ResolutionError> {
        let mut planning_segments: Vec<MotionSegment> = Vec::new();
        let mut runtime_events: Vec<RuntimeEvent> = Vec::new();
        let mut current_joints = self.initial_state.to_vec();

        for (index, instruction) in program.instructions.iter().enumerate() {
            match instruction {
                ProgramInstruction::MoveJ {
                    origin,
                    target,
                    profile,
                } => {
                    let pose = motion_target_to_pose(target, self.frame_registry)?;
                    let ik_result = self
                        .ik_solver
                        .solve(&current_joints, IKGoal::Position(pose.translation()))
                        .map_err(|e| ResolutionError::IkFailed {
                            instruction_index: index,
                            reason: e.to_string(),
                        })?;

                    match ik_result.status {
                        IKStatus::Converged => {
                            planning_segments.push(MotionSegment::MoveJ {
                                origin: origin.clone(),
                                target: ik_result.q.clone(),
                                max_velocity: Some(profile.max_velocity),
                                max_acceleration: Some(profile.max_acceleration),
                            });
                            current_joints = ik_result.q;
                        }
                        IKStatus::MaxIterations => {
                            return Err(ResolutionError::IkFailed {
                                instruction_index: index,
                                reason: format!("{:?}", ik_result.status),
                            });
                        }
                    }
                }

                ProgramInstruction::MoveL {
                    origin,
                    target,
                    profile,
                } => {
                    let frame = resolve_frame(target, self.frame_registry)?;
                    match target {
                        MotionTarget::Position(pos) => {
                            planning_segments.push(MotionSegment::MoveLPosition {
                                origin: origin.clone(),
                                frame,
                                target_position: pos.position,
                                max_velocity: Some(profile.max_velocity),
                            });
                        }
                        MotionTarget::Pose(_) => {
                            let pose = motion_target_to_pose(target, self.frame_registry)?;
                            planning_segments.push(MotionSegment::MoveL {
                                origin: origin.clone(),
                                frame,
                                target_pose: pose,
                                max_velocity: Some(profile.max_velocity),
                            });
                        }
                    }
                }

                ProgramInstruction::Delay { origin, duration } => {
                    runtime_events.push(RuntimeEvent {
                        // Logical event: no timing yet. The TimelineScheduler
                        // assigns absolute at_time from the CompiledPlan (IR-3).
                        at_time: Duration::ZERO,
                        operation_id: origin.clone(),
                        action: RuntimeAction::Delay(*duration),
                    });
                }

                ProgramInstruction::SetOutput {
                    origin,
                    channel,
                    value,
                } => {
                    runtime_events.push(RuntimeEvent {
                        at_time: Duration::ZERO,
                        operation_id: origin.clone(),
                        action: RuntimeAction::SetOutput {
                            channel: channel.clone(),
                            value: value.clone(),
                        },
                    });
                }
            }
        }

        let semantic_targets = program
            .instructions
            .iter()
            .filter(|instruction| {
                matches!(
                    instruction,
                    ProgramInstruction::MoveJ { .. } | ProgramInstruction::MoveL { .. }
                )
            })
            .cloned()
            .collect();

        Ok(MotionResolution {
            planning: PlanningProgram::with_semantic_targets(planning_segments, semantic_targets),
            runtime: RuntimeProgram {
                events: runtime_events,
            },
        })
    }
}

/// The resolved result of re-planning a semantic suffix from a new joint seed.
#[derive(Debug, Clone)]
pub struct PlannedSuffix {
    pub planning: PlanningProgram,
    pub runtime: RuntimeProgram,
    pub final_state: RobotState,
}

/// Re-resolve semantic targets from `current_state` with the existing resolver
/// and IK solver. The caller supplies semantic instructions, never resolved
/// joint segments, so Cartesian targets and profiles remain intact.
pub fn replan_suffix(
    current_state: &RobotState,
    suffix: &[SemanticTarget],
    context: &PlanningContext,
) -> Result<PlannedSuffix, ResolutionError> {
    if context.tcp.as_ref().is_some_and(|tcp| tcp.has_offset()) {
        return Err(ResolutionError::UnsupportedToolOffset);
    }

    let program = ExecutionProgram {
        instructions: suffix.to_vec(),
        metadata: thalos_core::execution::program::ExecutionMetadata {
            schema_version: 1,
            source_project: "h6-replanned-alternate".into(),
        },
    };
    let resolver = MotionResolver::new(
        context.ik_solver,
        &context.robot.frames,
        current_state.as_slice(),
        context.robot.dof_count(),
    )?;
    let resolution = resolver.resolve(&program)?;
    let mut final_state = current_state.clone();
    for segment in &resolution.planning.segments {
        if let MotionSegment::MoveJ { target, .. } = segment {
            final_state = RobotState::new(target.clone());
        }
    }

    Ok(PlannedSuffix {
        planning: resolution.planning,
        runtime: resolution.runtime,
        final_state,
    })
}

// ─── Helper functions ───────────────────────────────────────────────────────

/// Convert a `MotionTarget` to a `Pose`, resolving the frame string via
/// `FrameRegistry`.
fn motion_target_to_pose(
    target: &MotionTarget,
    frame_registry: &FrameRegistry,
) -> Result<Pose, ResolutionError> {
    match target {
        MotionTarget::Pose(mp) => {
            let translation = Vector3::new(mp.position[0], mp.position[1], mp.position[2]);
            let quat = Quaternion::new(
                mp.orientation[0],
                mp.orientation[1],
                mp.orientation[2],
                mp.orientation[3],
            );
            let rotation =
                UnitQuaternion::new(quat).map_err(|_| ResolutionError::UnknownFrame("".into()))?;
            // Map error — quaternion normalisation can fail for zero norm
            let transform = Transform3D::from_translation_rotation(translation, rotation);
            let target_frame = resolve_frame_by_name(&mp.frame, frame_registry)?;
            Ok(Pose::new(FrameId::World, target_frame, transform))
        }
        MotionTarget::Position(pos) => {
            let translation = Vector3::new(pos.position[0], pos.position[1], pos.position[2]);
            let transform = Transform3D::from_translation(translation);
            let target_frame = resolve_frame_by_name(&pos.frame, frame_registry)?;
            Ok(Pose::new(FrameId::World, target_frame, transform))
        }
    }
}

/// Resolve the `frame` field from a `MotionTarget` to a `FrameId`.
fn resolve_frame(
    target: &MotionTarget,
    frame_registry: &FrameRegistry,
) -> Result<FrameId, ResolutionError> {
    match target {
        MotionTarget::Pose(mp) => resolve_frame_by_name(&mp.frame, frame_registry),
        MotionTarget::Position(pos) => resolve_frame_by_name(&pos.frame, frame_registry),
    }
}

fn resolve_frame_by_name(name: &str, registry: &FrameRegistry) -> Result<FrameId, ResolutionError> {
    if name == "world" {
        return Ok(FrameId::World);
    }
    registry
        .resolve_by_name(name)
        .ok_or_else(|| ResolutionError::UnknownFrame(name.to_string()))
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    use thalos_core::{
        execution::program::ExecutionMetadata,
        ids::OperationId,
        kinematics::inverse::{IKResult, IKSolver, IkError},
        kinematics::{forward::ForwardKinematics, inverse::DampedLeastSquaresSolver},
        models::{RobotModel, RobotRegistry},
        motion::target::{MotionPose, MotionProfile, OutputChannel, OutputValue},
        robot::state::RobotState,
    };

    // ── Mock IK solver ───────────────────────────────────────────────────

    struct NoopIKSolver;

    impl IKSolver for NoopIKSolver {
        fn solve(&self, q0: &[f64], _goal: IKGoal) -> Result<IKResult, IkError> {
            Ok(IKResult::converged(q0.to_vec(), 1, 0.0, None))
        }
    }

    /// IK solver that always fails to converge.
    struct FailingIKSolver;

    impl IKSolver for FailingIKSolver {
        fn solve(&self, q0: &[f64], _goal: IKGoal) -> Result<IKResult, IkError> {
            Ok(IKResult::max_iterations(q0.to_vec(), 1000, 999.0, None))
        }
    }

    // ── Helpers ──────────────────────────────────────────────────────────

    fn make_registry() -> FrameRegistry {
        let mut reg = FrameRegistry::new();
        reg.create("world");
        reg
    }

    fn make_resolver<'a>(
        ik: &'a dyn IKSolver,
        registry: &'a FrameRegistry,
        initial_state: &'a [f64],
    ) -> MotionResolver<'a> {
        // Test helper: initial_state length IS the expected DOF.
        MotionResolver::new(ik, registry, initial_state, initial_state.len())
            .expect("test resolvers use a matching DOF")
    }

    fn sample_pose() -> MotionPose {
        MotionPose {
            position: [0.1, 0.2, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "world".into(),
        }
    }

    fn sample_metadata() -> ExecutionMetadata {
        ExecutionMetadata {
            schema_version: 1,
            source_project: "test".into(),
        }
    }

    fn default_profile() -> MotionProfile {
        MotionProfile {
            max_velocity: 500.0,
            max_acceleration: 1000.0,
            max_jerk: None,
        }
    }

    // ── Test: empty program ───────────────────────────────────────────────

    #[test]
    fn empty_program_produces_empty_resolution() {
        let ik = NoopIKSolver;
        let registry = make_registry();
        let resolver = make_resolver(&ik, &registry, &[0.0, 0.0]);

        let program = ExecutionProgram {
            instructions: vec![],
            metadata: sample_metadata(),
        };

        let result = resolver
            .resolve(&program)
            .expect("empty program should resolve");
        assert!(result.planning.segments.is_empty());
        assert!(result.runtime.events.is_empty());
    }

    // ── Test: motion-only (MoveJ, MoveL) ─────────────────────────────────

    #[test]
    fn motion_only_produces_planning_segments() {
        let ik = NoopIKSolver;
        let registry = make_registry();
        let resolver = make_resolver(&ik, &registry, &[0.0, 0.0]);

        let program = ExecutionProgram {
            instructions: vec![
                ProgramInstruction::MoveJ {
                    origin: OperationId("1".to_string()),
                    target: MotionTarget::Pose(sample_pose()),
                    profile: default_profile(),
                },
                ProgramInstruction::MoveL {
                    origin: OperationId("2".to_string()),
                    target: MotionTarget::Pose(sample_pose()),
                    profile: default_profile(),
                },
            ],
            metadata: sample_metadata(),
        };

        let result = resolver
            .resolve(&program)
            .expect("motion-only should resolve");
        assert_eq!(result.planning.segments.len(), 2);
        assert!(result.runtime.events.is_empty());
        assert!(matches!(
            result.planning.segments[0],
            MotionSegment::MoveJ { .. }
        ));
        assert!(matches!(
            result.planning.segments[1],
            MotionSegment::MoveL { .. }
        ));
    }

    // ── Test: runtime-only (Delay, SetOutput) ─────────────────────────────

    #[test]
    fn runtime_only_produces_runtime_events() {
        let ik = NoopIKSolver;
        let registry = make_registry();
        let resolver = make_resolver(&ik, &registry, &[0.0, 0.0]);

        let program = ExecutionProgram {
            instructions: vec![
                ProgramInstruction::Delay {
                    origin: OperationId("1".to_string()),
                    duration: Duration::from_secs(2),
                },
                ProgramInstruction::SetOutput {
                    origin: OperationId("2".to_string()),
                    channel: OutputChannel {
                        name: "gripper".into(),
                        channel_type: "digital".into(),
                    },
                    value: OutputValue::Bool(true),
                },
            ],
            metadata: sample_metadata(),
        };

        let result = resolver
            .resolve(&program)
            .expect("runtime-only should resolve");
        assert!(result.planning.segments.is_empty());
        assert_eq!(result.runtime.events.len(), 2);
        assert!(matches!(
            result.runtime.events[0].action,
            RuntimeAction::Delay(_)
        ));
        assert!(matches!(
            result.runtime.events[1].action,
            RuntimeAction::SetOutput { .. }
        ));
    }

    // ── Test: mixed program ───────────────────────────────────────────────

    #[test]
    fn mixed_program_has_correct_counts() {
        let ik = NoopIKSolver;
        let registry = make_registry();
        let resolver = make_resolver(&ik, &registry, &[0.0, 0.0]);

        let program = ExecutionProgram {
            instructions: vec![
                ProgramInstruction::MoveJ {
                    origin: OperationId("1".to_string()),
                    target: MotionTarget::Pose(sample_pose()),
                    profile: default_profile(),
                },
                ProgramInstruction::Delay {
                    origin: OperationId("2".to_string()),
                    duration: Duration::from_secs(1),
                },
                ProgramInstruction::MoveL {
                    origin: OperationId("3".to_string()),
                    target: MotionTarget::Pose(sample_pose()),
                    profile: default_profile(),
                },
                ProgramInstruction::SetOutput {
                    origin: OperationId("4".to_string()),
                    channel: OutputChannel {
                        name: "gripper".into(),
                        channel_type: "digital".into(),
                    },
                    value: OutputValue::Bool(true),
                },
            ],
            metadata: sample_metadata(),
        };

        let result = resolver.resolve(&program).expect("mixed should resolve");
        assert_eq!(result.planning.segments.len(), 2);
        assert_eq!(result.runtime.events.len(), 2);
    }

    // ── Test: determinism ─────────────────────────────────────────────────

    #[test]
    fn resolve_is_deterministic() {
        let ik = NoopIKSolver;
        let registry = make_registry();
        let resolver = make_resolver(&ik, &registry, &[0.0, 0.0]);

        let program = ExecutionProgram {
            instructions: vec![
                ProgramInstruction::MoveJ {
                    origin: OperationId("1".to_string()),
                    target: MotionTarget::Pose(sample_pose()),
                    profile: default_profile(),
                },
                ProgramInstruction::Delay {
                    origin: OperationId("2".to_string()),
                    duration: Duration::from_secs(1),
                },
            ],
            metadata: sample_metadata(),
        };

        let r1 = resolver.resolve(&program).expect("first resolve");
        let r2 = resolver.resolve(&program).expect("second resolve");

        // Compare manually — MotionResolution cannot derive PartialEq
        // because Pose in MotionSegment::MoveL does not implement it.
        assert_eq!(r1.planning.segments.len(), r2.planning.segments.len());
        assert_eq!(r1.runtime.events.len(), r2.runtime.events.len());
        assert_eq!(r1.runtime.events, r2.runtime.events); // RuntimeEvent IS PartialEq
        // Compare segment types
        for (s1, s2) in r1.planning.segments.iter().zip(&r2.planning.segments) {
            assert_eq!(
                std::mem::discriminant(s1),
                std::mem::discriminant(s2),
                "segment variants must match"
            );
        }
    }

    // ── Test: order preservation ──────────────────────────────────────────

    #[test]
    fn motion_segment_order_matches_instruction_order() {
        let ik = NoopIKSolver;
        let registry = make_registry();
        let resolver = make_resolver(&ik, &registry, &[0.0, 0.0]);

        let program = ExecutionProgram {
            instructions: vec![
                ProgramInstruction::MoveL {
                    origin: OperationId("1".to_string()),
                    target: MotionTarget::Pose(sample_pose()),
                    profile: default_profile(),
                },
                ProgramInstruction::MoveJ {
                    origin: OperationId("2".to_string()),
                    target: MotionTarget::Pose(sample_pose()),
                    profile: default_profile(),
                },
            ],
            metadata: sample_metadata(),
        };

        let result = resolver.resolve(&program).expect("should resolve");
        assert_eq!(result.planning.segments.len(), 2);
        // First instruction is MoveL, second is MoveJ
        assert!(matches!(
            result.planning.segments[0],
            MotionSegment::MoveL { .. }
        ));
        assert!(matches!(
            result.planning.segments[1],
            MotionSegment::MoveJ { .. }
        ));
    }

    #[test]
    fn replan_suffix_preserves_semantic_target_and_uses_new_seed() {
        let robot = RobotRegistry::create_default(RobotModel::Scara);
        let fk = ForwardKinematics::new(robot.clone());
        let solver =
            DampedLeastSquaresSolver::new(fk.clone(), *robot.end_effector(), 500, 1e-6, 0.1);
        let current = RobotState::new(vec![0.0, -1.31, -0.1, 0.0]);
        let alternate = RobotState::new(vec![0.2, -0.9, -0.1, 0.0]);
        let target = [0.5, 0.5, 0.25];
        let context = PlanningContext {
            robot: &robot,
            current_state: &alternate,
            ik_solver: &solver,
            tcp: None,
        };
        let suffix = vec![ProgramInstruction::MoveJ {
            origin: OperationId("op-goal".into()),
            target: MotionTarget::Position(thalos_core::motion::target::MotionPosition {
                position: target,
                frame: "world".into(),
            }),
            profile: default_profile(),
        }];

        let original_context = PlanningContext {
            robot: &robot,
            current_state: &current,
            ik_solver: &solver,
            tcp: None,
        };
        let original =
            replan_suffix(&current, &suffix, &original_context).expect("original suffix resolves");
        let planned = replan_suffix(&alternate, &suffix, &context).expect("suffix replans");
        assert_eq!(planned.planning.semantic_targets.as_ref(), Some(&suffix));
        let MotionSegment::MoveJ {
            target: original_joints,
            ..
        } = &original.planning.segments[0]
        else {
            panic!("expected the original resolved MoveJ suffix");
        };
        let MotionSegment::MoveJ { target: joints, .. } = &planned.planning.segments[0] else {
            panic!("expected a resolved MoveJ suffix");
        };
        assert_ne!(
            joints, original_joints,
            "the alternate seed must be observable in suffix resolution"
        );
        let resolved = fk.evaluate(joints).ee_position().expect("resolved FK");
        assert!((resolved.x - target[0]).abs() < 0.02);
        assert!((resolved.y - target[1]).abs() < 0.02);
        assert!((resolved.z - target[2]).abs() < 0.02);
        assert_ne!(planned.final_state.as_slice(), current.as_slice());
    }

    #[test]
    fn replan_suffix_rejects_unrepresentable_tcp_offset() {
        use thalos_core::robot::tool_frame::ToolFrame;

        let robot = RobotRegistry::create_default(RobotModel::Scara);
        let fk = ForwardKinematics::new(robot.clone());
        let solver = DampedLeastSquaresSolver::new(fk, *robot.end_effector(), 500, 1e-6, 0.1);
        let state = RobotState::zero(robot.dof_count());
        let offset = Transform3D::from_translation(Vector3::new(0.0, 0.0, 0.1));
        let tcp = ToolFrame::with_offset(*robot.end_effector(), offset);
        let context = PlanningContext {
            robot: &robot,
            current_state: &state,
            ik_solver: &solver,
            tcp: Some(&tcp),
        };

        let error = replan_suffix(&state, &[], &context).expect_err("TCP offset is unsupported");
        assert_eq!(error, ResolutionError::UnsupportedToolOffset);
    }

    // ── Test: atomic IK failure ───────────────────────────────────────────

    #[test]
    fn ik_failure_returns_error() {
        let ik = FailingIKSolver;
        let registry = make_registry();
        let resolver = make_resolver(&ik, &registry, &[0.0, 0.0]);

        let program = ExecutionProgram {
            instructions: vec![ProgramInstruction::MoveJ {
                origin: OperationId("1".to_string()),
                target: MotionTarget::Pose(sample_pose()),
                profile: default_profile(),
            }],
            metadata: sample_metadata(),
        };

        let result = resolver.resolve(&program);
        assert!(result.is_err());
        match result.unwrap_err() {
            ResolutionError::IkFailed {
                instruction_index, ..
            } => {
                assert_eq!(instruction_index, 0);
            }
            other => panic!("expected IkFailed, got {other:?}"),
        }
    }

    #[test]
    fn ik_failure_on_second_movej_stops_atomically() {
        struct SecondFailsIKSolver {
            call_count: std::sync::Mutex<usize>,
        }

        impl IKSolver for SecondFailsIKSolver {
            fn solve(&self, q0: &[f64], _goal: IKGoal) -> Result<IKResult, IkError> {
                let mut count = self.call_count.lock().unwrap();
                *count += 1;
                if *count == 2 {
                    Ok(IKResult::max_iterations(q0.to_vec(), 1000, 999.0, None))
                } else {
                    Ok(IKResult::converged(q0.to_vec(), 1, 0.0, None))
                }
            }
        }

        let ik = SecondFailsIKSolver {
            call_count: std::sync::Mutex::new(0),
        };
        let registry = make_registry();
        let resolver = make_resolver(&ik, &registry, &[0.0, 0.0]);

        let program = ExecutionProgram {
            instructions: vec![
                ProgramInstruction::MoveJ {
                    origin: OperationId("1".to_string()),
                    target: MotionTarget::Pose(sample_pose()),
                    profile: default_profile(),
                },
                ProgramInstruction::MoveJ {
                    origin: OperationId("2".to_string()),
                    target: MotionTarget::Pose(sample_pose()),
                    profile: default_profile(),
                },
            ],
            metadata: sample_metadata(),
        };

        let result = resolver.resolve(&program);
        assert!(result.is_err());
        match result.unwrap_err() {
            ResolutionError::IkFailed {
                instruction_index, ..
            } => {
                assert_eq!(instruction_index, 1);
            }
            other => panic!("expected IkFailed at index 1, got {other:?}"),
        }
    }

    // ── Test: OperationId origin propagation (IR-1 → IR-2, invariant I2) ──

    #[test]
    fn movej_segment_carries_instruction_origin() {
        let ik = NoopIKSolver;
        let registry = make_registry();
        let resolver = make_resolver(&ik, &registry, &[0.0, 0.0]);

        let program = ExecutionProgram {
            instructions: vec![ProgramInstruction::MoveJ {
                origin: OperationId("op-j".to_string()),
                target: MotionTarget::Pose(sample_pose()),
                profile: default_profile(),
            }],
            metadata: sample_metadata(),
        };

        let result = resolver.resolve(&program).expect("should resolve");
        assert_eq!(result.planning.segments.len(), 1);
        let seg = &result.planning.segments[0];
        assert_eq!(
            seg.origin(),
            &OperationId("op-j".to_string()),
            "MoveJ segment must carry the instruction origin"
        );
    }

    #[test]
    fn movel_segment_carries_instruction_origin() {
        let ik = NoopIKSolver;
        let registry = make_registry();
        let resolver = make_resolver(&ik, &registry, &[0.0, 0.0]);

        let program = ExecutionProgram {
            instructions: vec![ProgramInstruction::MoveL {
                origin: OperationId("op-l".to_string()),
                target: MotionTarget::Pose(sample_pose()),
                profile: default_profile(),
            }],
            metadata: sample_metadata(),
        };

        let result = resolver.resolve(&program).expect("should resolve");
        assert_eq!(result.planning.segments.len(), 1);
        let seg = &result.planning.segments[0];
        assert_eq!(
            seg.origin(),
            &OperationId("op-l".to_string()),
            "MoveL segment must carry the instruction origin"
        );
    }

    #[test]
    fn distinct_origins_survive_mixed_program() {
        let ik = NoopIKSolver;
        let registry = make_registry();
        let resolver = make_resolver(&ik, &registry, &[0.0, 0.0]);

        let program = ExecutionProgram {
            instructions: vec![
                ProgramInstruction::MoveJ {
                    origin: OperationId("pick-1".to_string()),
                    target: MotionTarget::Pose(sample_pose()),
                    profile: default_profile(),
                },
                ProgramInstruction::SetOutput {
                    origin: OperationId("pick-1".to_string()),
                    channel: OutputChannel {
                        name: "gripper".into(),
                        channel_type: "digital".into(),
                    },
                    value: OutputValue::Bool(true),
                },
                ProgramInstruction::MoveL {
                    origin: OperationId("place-2".to_string()),
                    target: MotionTarget::Pose(sample_pose()),
                    profile: default_profile(),
                },
            ],
            metadata: sample_metadata(),
        };

        let result = resolver.resolve(&program).expect("should resolve");

        // Planning segments keep their own instruction origins.
        assert_eq!(result.planning.segments.len(), 2);
        assert_eq!(
            result.planning.segments[0].origin(),
            &OperationId("pick-1".to_string())
        );
        assert_eq!(
            result.planning.segments[1].origin(),
            &OperationId("place-2".to_string())
        );

        // Runtime events keep their own instruction origins.
        assert_eq!(result.runtime.events.len(), 1);
        assert_eq!(
            result.runtime.events[0].operation_id,
            OperationId("pick-1".to_string())
        );
    }

    // ── Test: DOF validation (invariant I1) ────────────────────────────────
    //
    // I1 — Single Robot Per Compilation: the resolver must reject a robot
    // whose DOF does not match its `initial_state` before producing any
    // output. The validation happens at construction (fail fast).

    #[test]
    fn dof_mismatch_rejects_at_construction() {
        let ik = NoopIKSolver;
        let registry = make_registry();

        // initial_state carries 2 joints but the robot is configured for 4 DOF.
        let result = MotionResolver::new(&ik, &registry, &[0.0, 0.0], 4);
        match result {
            Err(ResolutionError::DofMismatch { expected, actual }) => {
                assert_eq!(expected, 4, "expected DOF comes from the robot model");
                assert_eq!(actual, 2, "actual DOF comes from initial_state length");
            }
            Err(other) => panic!("expected DofMismatch, got {other:?}"),
            Ok(_) => panic!("expected DofMismatch, got Ok"),
        }
    }

    #[test]
    fn matching_dof_constructs_resolver() {
        let ik = NoopIKSolver;
        let registry = make_registry();

        let resolver =
            MotionResolver::new(&ik, &registry, &[0.0, 0.0], 2).expect("2 DOF must construct");

        let program = ExecutionProgram {
            instructions: vec![],
            metadata: sample_metadata(),
        };
        let result = resolver
            .resolve(&program)
            .expect("empty program should resolve");
        assert!(result.planning.segments.is_empty());
        assert!(result.runtime.events.is_empty());
    }

    /// Spec: ir-model "URDF robot uses real chain DOF" — a chain built from
    /// a real 4-DOF URDF (icebot) must construct a resolver whose
    /// `expected_dof` is `chain.dof_count()`, never metadata-derived DOF.
    #[test]
    fn resolver_accepts_real_urdf_chain_dof() {
        // icebot.urdf is owned by the sibling thalos-core crate (copied there in
        // PR3; core's tests use the same fixture). Reference it by sibling path
        // from CARGO_MANIFEST_DIR (crate root) — keeps the fixture owned by its
        // source crate rather than duplicating it in planning.
        let urdf = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../thalos-core/tests/fixtures/icebot.urdf"
        ));
        let chain =
            thalos_core::robot::adapter::from_urdf(urdf).expect("icebot URDF must build a chain");
        assert_eq!(chain.dof_count(), 4, "icebot has 4 actuated DOF");

        let ik = NoopIKSolver;
        let registry = make_registry();
        let resolver = MotionResolver::new(&ik, &registry, &[0.0; 4], chain.dof_count())
            .expect("4-DOF URDF chain must construct a resolver");

        let program = ExecutionProgram {
            instructions: vec![],
            metadata: sample_metadata(),
        };
        let result = resolver
            .resolve(&program)
            .expect("empty program should resolve");
        assert!(result.planning.segments.is_empty());
    }
}
