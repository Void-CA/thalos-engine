use std::ops::Range;

use thalos_core::{
    ids::OperationId,
    motion::{expansion::expand_operation, segment::MotionSegment},
    operation::{
        MotionNode, MotionProvenance, MotionRole, Operation, OperationConstraints,
        RangeConstraintQuery,
    },
    prelude::{RobotState, Trajectory, TrajectoryPoint},
};

use crate::error::{CompileError, PlanningError};
use crate::goal::{
    GoalResolver, GoalResolverConfig, JointGoal, ResolvedPositionGoal, ValidatedGoal,
};
use crate::motion::move_j::{MoveJConfig, MoveJPlanner};
use crate::motion::move_l::{MoveLConfig, MoveLPlanner};
use crate::motion::planner::{SegmentPlanner, SegmentPlanningContext};
use crate::motion::program::{CompiledPlan, PlannedSegment, PlanningProgram};

/// Dispatches a `MotionSegment` to the appropriate `MotionPlanner`.
///
/// This trait exists so that `PlanCompiler` never needs to know about
/// specific movement types. New variants (MoveP, Wait, etc.) register a
/// new arm in the dispatcher without changing the compiler.
pub trait MotionPlannerDispatcher {
    /// Plan a single segment against the given context and return its
    /// time-parameterised trajectory.
    fn plan_segment(
        &self,
        segment: &MotionSegment,
        ctx: &SegmentPlanningContext,
    ) -> Result<Trajectory, PlanningError>;
}

/// Default dispatcher supporting MoveJ and MoveL.
///
/// Uses `GoalResolver` for validation and delegates to `MoveJPlanner` /
/// `MoveLPlanner`. New segment types require a new `match` arm here —
/// the compiler stays untouched.
pub struct DefaultPlannerDispatcher {
    pub goal_resolver_config: GoalResolverConfig,
}

impl DefaultPlannerDispatcher {
    pub fn new(config: GoalResolverConfig) -> Self {
        Self {
            goal_resolver_config: config,
        }
    }
}

impl Default for DefaultPlannerDispatcher {
    fn default() -> Self {
        Self {
            goal_resolver_config: GoalResolverConfig::default(),
        }
    }
}

impl MotionPlannerDispatcher for DefaultPlannerDispatcher {
    fn plan_segment(
        &self,
        segment: &MotionSegment,
        ctx: &SegmentPlanningContext,
    ) -> Result<Trajectory, PlanningError> {
        match segment {
            MotionSegment::MoveJ {
                target,
                max_velocity,
                max_acceleration,
                ..
            } => {
                let resolver = GoalResolver::new(self.goal_resolver_config.clone());
                let goal: ValidatedGoal<JointGoal> = resolver.resolve_joint(ctx, target)?;

                let planner = MoveJPlanner::new(MoveJConfig {
                    max_velocity: max_velocity.unwrap_or(1.0),
                    max_acceleration: max_acceleration.unwrap_or(0.5),
                    time_step: 0.01,
                });
                planner.plan(ctx, &goal)
            }

            MotionSegment::MoveL {
                frame: _,
                target_pose,
                max_velocity,
                ..
            } => {
                let resolver = GoalResolver::new(self.goal_resolver_config.clone());
                let planner = MoveLPlanner::new(MoveLConfig {
                    max_velocity: max_velocity.unwrap_or(0.25),
                    max_acceleration: 0.125,
                    time_step: 0.01,
                    cartesian_step: 0.01,
                });

                // Semantic fallback (design ADR-4, spec semantic-ik-fallback
                // "MoveL pose unreachable but translation reachable"): a
                // MoveL whose FINAL pose has no full-pose IK solution compiles
                // through the translation-only path — gated by the operation
                // type (MoveL allows it; MoveLPosition declares Position from
                // the start). When the position ALSO fails, the resolver's
                // IkFailed propagates unchanged (orientation-mandatory path).
                match resolver.resolve_pose(ctx, target_pose) {
                    Ok(goal) => planner.plan(ctx, &goal),
                    Err(
                        source @ (PlanningError::IkFailed { .. }
                        | PlanningError::IkFailedPosition { .. }
                        | PlanningError::Ik(_)),
                    ) => {
                        let _ = source;
                        let position = target_pose.translation();
                        let goal: ValidatedGoal<ResolvedPositionGoal> =
                            resolver.resolve_position(ctx, position)?;
                        planner.plan_position(ctx, &goal)
                    }
                    Err(other) => Err(other),
                }
            }

            MotionSegment::MoveLPosition {
                frame: _,
                target_position,
                max_velocity,
                ..
            } => {
                let resolver = GoalResolver::new(self.goal_resolver_config.clone());
                let goal: ValidatedGoal<ResolvedPositionGoal> = resolver.resolve_position(
                    ctx,
                    thalos_math::Vector3::new(
                        target_position[0],
                        target_position[1],
                        target_position[2],
                    ),
                )?;

                let planner = MoveLPlanner::new(MoveLConfig {
                    max_velocity: max_velocity.unwrap_or(0.25),
                    max_acceleration: 0.125,
                    time_step: 0.01,
                    cartesian_step: 0.01,
                });
                planner.plan_position(ctx, &goal)
            }
        }
    }
}

/// Compiles a `PlanningProgram` into a `CompiledPlan`.
///
/// The compiler is a pure orchestrator:
/// 1. Iterates segments in order
/// 2. Delegates each to the dispatcher
/// 3. Concatenates trajectories with time offsets
/// 4. Returns the merged plan with per-segment metadata
///
/// It does **not** know about MoveJ, MoveL, or any specific motion type.
pub struct PlanCompiler {
    pub dispatcher: Box<dyn MotionPlannerDispatcher + Send + Sync>,
}

impl std::fmt::Debug for PlanCompiler {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PlanCompiler")
            .field("dispatcher", &format_args!("..."))
            .finish()
    }
}

impl PlanCompiler {
    pub fn new(dispatcher: Box<dyn MotionPlannerDispatcher + Send + Sync>) -> Self {
        Self { dispatcher }
    }

    /// Compile a complete motion program.
    ///
    /// Each segment is planned sequentially. The end state of segment N
    /// becomes the start state of segment N+1. All waypoints are merged
    /// into a single continuous trajectory with monotonically increasing
    /// timestamps.
    ///
    /// This is the **legacy** path: plain `MotionSegment`s carry no
    /// operation context, so every `PlannedSegment` gets `operation_id:
    /// None` and `role: None`. It delegates to the same shared core as
    /// [`compile_with_operations`](Self::compile_with_operations), so any
    /// future compilation improvement benefits both paths.
    ///
    /// # Atomicity
    ///
    /// If **any** segment fails, the entire compilation fails with a
    /// `CompileError` identifying which segment and why. No partial
    /// `CompiledPlan` is returned — the runtime is never modified.
    pub fn compile(
        &self,
        program: &PlanningProgram,
        ctx: &SegmentPlanningContext,
    ) -> Result<CompiledPlan, CompileError> {
        let metadata = vec![
            NodeMetadata {
                operation_id: None,
                role: None,
            };
            program.segments.len()
        ];
        self.compile_segments(
            &program.segments,
            &metadata,
            ctx,
            program.semantic_targets.clone(),
        )
    }

    /// Shared compilation core.
    ///
    /// Plans `segments` in order, merging all waypoints into one continuous
    /// trajectory, and attaches the per-segment `metadata` to each resulting
    /// `PlannedSegment`. Both `compile()` (all-None metadata) and
    /// `compile_with_operations()` (per-node metadata from expansion) route
    /// through here.
    fn compile_segments(
        &self,
        segments: &[MotionSegment],
        metadata: &[NodeMetadata],
        ctx: &SegmentPlanningContext,
        semantic_targets: Option<Vec<crate::motion::program::SemanticTarget>>,
    ) -> Result<CompiledPlan, CompileError> {
        let mut planned = Vec::with_capacity(segments.len());
        let mut all_waypoints: Vec<TrajectoryPoint> = Vec::new();
        let mut time_offset = 0.0_f64;
        let mut current_joints = ctx.current_state.joints.clone();

        for (segment_index, segment) in segments.iter().enumerate() {
            let segment_state = RobotState::new(0.0, current_joints.clone());
            let segment_ctx = SegmentPlanningContext {
                robot: ctx.robot,
                current_state: &segment_state,
                ik_solver: ctx.ik_solver,
                tcp: ctx.tcp,
            };

            let trajectory = self
                .dispatcher
                .plan_segment(segment, &segment_ctx)
                .map_err(|source| CompileError {
                    segment_index,
                    source,
                })?;

            let start_idx = all_waypoints.len();

            // Append waypoints with shifted timestamps
            for wp in trajectory.waypoints() {
                all_waypoints.push(TrajectoryPoint::new(
                    wp.joints().to_vec(),
                    wp.timestamp() + time_offset,
                ));
            }

            let end_idx = all_waypoints.len();
            let seg_duration = trajectory.duration();

            // Advance current state to end of this segment
            if let Some(last) = trajectory.waypoints().last() {
                current_joints = last.joints().iter().map(|&q| thalos_core::prelude::JointState::position(q)).collect();
            }

            let meta = metadata.get(segment_index);
            planned.push(PlannedSegment {
                origin: segment.origin().clone(),
                source: segment.clone(),
                trajectory,
                waypoint_range: Range {
                    start: start_idx,
                    end: end_idx,
                },
                time_range: Range {
                    start: time_offset,
                    end: time_offset + seg_duration,
                },
                operation_id: meta.and_then(|m| m.operation_id.clone()),
                role: meta.and_then(|m| m.role),
            });

            time_offset += seg_duration;
        }

        let merged = Trajectory::new(all_waypoints);
        Ok(CompiledPlan::new_with_semantic_targets(
            merged,
            planned,
            semantic_targets,
        ))
    }

    /// Compile a sequence of Operations into a plan with a built-in ConstraintQuery.
    ///
    /// Expands each Operation into MotionNodes, builds a PlanningProgram,
    /// compiles it, and constructs a RangeConstraintQuery that maps each
    /// operation's waypoint range to its constraints. Also builds a
    /// `Vec<MotionProvenance>` — one entry per expanded node/segment —
    /// preserving each node's `operation_id` and `MotionRole` through
    /// compilation.
    pub fn compile_with_operations(
        &self,
        operations: &[Operation],
        ctx: &SegmentPlanningContext,
    ) -> Result<OperationCompilation, CompileError> {
        // 1. Expand each operation to MotionNodes.
        let mut all_nodes: Vec<MotionNode> = Vec::new();
        let mut op_node_ranges: Vec<(Range<usize>, OperationConstraints)> = Vec::new();

        for op in operations {
            let expansion = expand_operation(op);
            let node_count = expansion.len();
            op_node_ranges.push((
                all_nodes.len()..all_nodes.len() + node_count,
                op.constraints().clone(),
            ));
            all_nodes.extend(expansion);
        }

        // 2. Extract segments + per-node metadata and compile via the
        //    shared core (same path as the legacy `compile()`).
        let segments: Vec<MotionSegment> = all_nodes.iter().map(|n| n.segment.clone()).collect();
        let metadata: Vec<NodeMetadata> = all_nodes
            .iter()
            .map(|n| NodeMetadata {
                operation_id: n.operation_id.clone(),
                role: Some(n.role),
            })
            .collect();
        let plan = self.compile_segments(&segments, &metadata, ctx, None)?;

        // 3. Build waypoint-level ranges from the per-operation node ranges.
        let mut waypoint_ranges: Vec<(Range<usize>, OperationConstraints)> = Vec::new();
        for (node_range, constraints) in &op_node_ranges {
            let start_wp = plan.segments[node_range.start].waypoint_range.start;
            let end_wp = plan.segments[node_range.end - 1].waypoint_range.end;
            waypoint_ranges.push((start_wp..end_wp, constraints.clone()));
        }

        let constraint_query = RangeConstraintQuery::new(waypoint_ranges);

        // 4. Build provenance: one entry per expanded node/segment, preserving
        //    the node's operation_id and role. Expansion always sets
        //    operation_id = Some(op.id), so every node yields an entry.
        let provenance: Vec<MotionProvenance> = all_nodes
            .iter()
            .enumerate()
            .filter_map(|(i, node)| {
                node.operation_id
                    .clone()
                    .map(|operation_id| MotionProvenance {
                        waypoint_range: plan.segments[i].waypoint_range.clone(),
                        operation_id,
                        role: node.role,
                    })
            })
            .collect();

        Ok(OperationCompilation {
            plan,
            constraint_query,
            provenance,
        })
    }
}

/// Deterministic initial joints of `segment_index` (design ADR-3, spec
/// semantic-ik-fallback "Segment-start context for materialization").
///
/// - Segment 0 starts from the caller's `current_joints` (the plan's start
///   configuration).
/// - Segment N > 0 starts from the END joints of segment N−1 (its last
///   waypoint) — exactly the joints the compiler will hand the segment when
///   the program is (re)compiled. NEVER the runtime snapshot.
///
/// Materializers and the availability verifier solve IK from these joints so
/// verification matches compilation. Defensive fallback to `current_joints`
/// when the previous segment has an empty trajectory (cannot happen for a
/// successfully compiled plan).
pub fn segment_start_joints(
    compiled: &CompiledPlan,
    segment_index: usize,
    current_joints: &[f64],
) -> Vec<f64> {
    if segment_index == 0 {
        return current_joints.to_vec();
    }
    compiled
        .segments
        .get(segment_index - 1)
        .and_then(|prev| prev.trajectory.waypoints().last())
        .map(|wp| wp.joints().to_vec())
        .unwrap_or_else(|| current_joints.to_vec())
}

/// Per-node semantic metadata carried from expansion into `PlannedSegment`.
#[derive(Debug, Clone)]
struct NodeMetadata {
    operation_id: Option<OperationId>,
    role: Option<MotionRole>,
}

/// Result of compiling a sequence of Operations.
///
/// Includes the compiled plan, a RangeConstraintQuery built from the
/// operations' constraints (mapped to the compiled waypoint ranges), and
/// the provenance records linking each waypoint range back to its
/// originating operation node.
pub struct OperationCompilation {
    pub plan: CompiledPlan,
    pub constraint_query: RangeConstraintQuery,
    pub provenance: Vec<MotionProvenance>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::{
        ids::OperationId,
        kinematics::inverse::{IKResult, IKSolver, IkError},
        models::{RobotModel, RobotRegistry},
        robot::state::RobotState,
    };

    struct NoopIKSolver;

    impl IKSolver for NoopIKSolver {
        fn solve(
            &self,
            q0: &[f64],
            _goal: thalos_core::kinematics::inverse::IKGoal,
        ) -> Result<IKResult, IkError> {
            Ok(IKResult::converged(q0.to_vec(), 1, 0.0, None))
        }
    }

    /// Helper: create a Planar2R chain and a planning context owning all data.
    struct TestHarness {
        chain: thalos_core::robot::serial_chain::SerialChain,
        state: RobotState,
        ik: NoopIKSolver,
    }

    impl TestHarness {
        fn new() -> Self {
            let chain = RobotRegistry::create_default(RobotModel::Planar2R);
            let state = RobotState::zero(chain.dof_count());
            Self {
                chain,
                state,
                ik: NoopIKSolver,
            }
        }

        fn ctx(&self) -> SegmentPlanningContext<'_> {
            SegmentPlanningContext {
                robot: &self.chain,
                current_state: &self.state,
                ik_solver: &self.ik,
                tcp: None,
            }
        }
    }

    #[test]
    fn compile_empty_program() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let program = PlanningProgram::new(vec![]);

        let result = compiler.compile(&program, &h.ctx());
        assert!(result.is_ok());
        let plan = result.unwrap();
        assert!(plan.merged_trajectory.is_empty());
        assert!(plan.segments.is_empty());
        assert_eq!(plan.duration, 0.0);
        assert_eq!(plan.waypoint_count, 0);
    }

    #[test]
    fn compile_single_movej() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let program = PlanningProgram::new(vec![MotionSegment::MoveJ {
            origin: OperationId("test".into()),
            target: vec![1.0, 1.0],
            max_velocity: None,
            max_acceleration: None,
        }]);

        let plan = compiler
            .compile(&program, &h.ctx())
            .expect("compile failed");
        assert!(!plan.merged_trajectory.is_empty());
        assert_eq!(plan.segments.len(), 1);
        assert_eq!(plan.waypoint_count, plan.merged_trajectory.len());

        // Verify segment metadata
        let seg = &plan.segments[0];
        assert_eq!(seg.waypoint_range.start, 0);
        assert_eq!(seg.waypoint_range.end, plan.waypoint_count);
        assert_eq!(seg.time_range.start, 0.0);
        assert!(seg.time_range.end > 0.0);

        // Verify first waypoint is at timestamp 0.0
        let first = &plan.merged_trajectory.waypoints()[0];
        let last = &plan.merged_trajectory.waypoints()[plan.waypoint_count - 1];
        assert_eq!(first.timestamp(), 0.0);
        assert!((last.timestamp() - plan.duration).abs() < 1e-9);

        // Verify source preservation
        match &seg.source {
            MotionSegment::MoveJ { target, .. } => {
                assert_eq!(target, &vec![1.0, 1.0]);
            }
            _ => panic!("expected MoveJ"),
        }
    }

    #[test]
    fn compile_two_movej_segments() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let program = PlanningProgram::new(vec![
            MotionSegment::MoveJ {
                origin: OperationId("test".into()),
                target: vec![1.0, 0.5],
                max_velocity: None,
                max_acceleration: None,
            },
            MotionSegment::MoveJ {
                origin: OperationId("test".into()),
                target: vec![0.0, 1.0],
                max_velocity: None,
                max_acceleration: None,
            },
        ]);

        let plan = compiler
            .compile(&program, &h.ctx())
            .expect("compile failed");
        assert_eq!(plan.segments.len(), 2);
        assert_eq!(plan.waypoint_count, plan.merged_trajectory.len());

        // Verify segment 1 waypoint range
        let seg0 = &plan.segments[0];
        let seg1 = &plan.segments[1];
        assert_eq!(seg0.waypoint_range.start, 0);
        assert_eq!(seg1.waypoint_range.end, plan.waypoint_count);
        assert_eq!(seg0.waypoint_range.end, seg1.waypoint_range.start);

        // Verify concatenated timestamps are monotonic
        let waypoints = plan.merged_trajectory.waypoints();
        for i in 1..waypoints.len() {
            assert!(
                waypoints[i].timestamp() >= waypoints[i - 1].timestamp(),
                "timestamps must be monotonic at index {}",
                i
            );
        }

        // Verify segment 1 time range starts after segment 0
        assert_eq!(seg0.time_range.start, 0.0);
        assert_eq!(seg1.time_range.start, seg0.time_range.end);
    }

    #[test]
    fn compile_two_movej_first_waypoint_matches_start_state() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let program = PlanningProgram::new(vec![
            MotionSegment::MoveJ {
                origin: OperationId("test".into()),
                target: vec![1.0, 0.5],
                max_velocity: None,
                max_acceleration: None,
            },
            MotionSegment::MoveJ {
                origin: OperationId("test".into()),
                target: vec![0.0, 1.0],
                max_velocity: None,
                max_acceleration: None,
            },
        ]);

        let plan = compiler
            .compile(&program, &h.ctx())
            .expect("compile failed");

        let wps = plan.merged_trajectory.waypoints();
        // First waypoint must be the start position [0, 0], NOT the final target
        assert_eq!(
            wps[0].joints(),
            &[0.0, 0.0],
            "first waypoint must match start position, got {:?}",
            wps[0].joints()
        );
    }

    /// A dispatcher that always fails with `InvalidGoal`.
    struct FailingDispatcher;

    impl MotionPlannerDispatcher for FailingDispatcher {
        fn plan_segment(
            &self,
            _segment: &MotionSegment,
            _ctx: &SegmentPlanningContext,
        ) -> Result<Trajectory, PlanningError> {
            Err(PlanningError::InvalidGoal("always fails".into()))
        }
    }

    #[test]
    fn compile_fails_atomically_on_segment_error() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(FailingDispatcher));
        let program = PlanningProgram::new(vec![
            MotionSegment::MoveJ {
                origin: OperationId("test".into()),
                target: vec![0.5, 0.5],
                max_velocity: None,
                max_acceleration: None,
            },
            MotionSegment::MoveJ {
                origin: OperationId("test".into()),
                target: vec![1.0, 0.0],
                max_velocity: None,
                max_acceleration: None,
            },
        ]);

        // Set state to something non-zero so the first segment also fails
        let err = compiler
            .compile(&program, &h.ctx())
            .expect_err("should fail");
        assert_eq!(err.segment_index, 0);
        assert_eq!(err.segment_1based(), 1);
        assert_eq!(
            err.to_string(),
            "segment 1 failed: Invalid goal: always fails"
        );
    }

    #[test]
    fn compile_fails_on_second_segment() {
        /// Dispatcher: first segment succeeds, second fails.
        struct FailingSecondDispatcher;

        impl MotionPlannerDispatcher for FailingSecondDispatcher {
            fn plan_segment(
                &self,
                segment: &MotionSegment,
                ctx: &SegmentPlanningContext,
            ) -> Result<Trajectory, PlanningError> {
                // Let the first segment through
                match segment {
                    MotionSegment::MoveJ { target, .. } if target == &vec![0.5, 0.5] => {
                        DefaultPlannerDispatcher::default().plan_segment(segment, ctx)
                    }
                    _ => Err(PlanningError::InvalidGoal("second segment fails".into())),
                }
            }
        }

        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(FailingSecondDispatcher));
        let program = PlanningProgram::new(vec![
            MotionSegment::MoveJ {
                origin: OperationId("test".into()),
                target: vec![0.5, 0.5],
                max_velocity: None,
                max_acceleration: None,
            },
            MotionSegment::MoveJ {
                origin: OperationId("test".into()),
                target: vec![1.0, 0.0],
                max_velocity: None,
                max_acceleration: None,
            },
        ]);

        let err = compiler
            .compile(&program, &h.ctx())
            .expect_err("second segment should fail");
        assert_eq!(err.segment_index, 1);
        assert_eq!(err.segment_1based(), 2);
        assert_eq!(
            err.to_string(),
            "segment 2 failed: Invalid goal: second segment fails"
        );
    }

    // ── 3.6 Integration: Operation → expand → compile → constraint query ──
    use thalos_core::{
        operation::{
            ConstraintQuery, Operation as CoreOperation, OperationConstraints, OperationType,
            PrecisionLevel,
        },
        spatial::frame::FrameId,
        spatial::pose::Pose,
    };
    use thalos_math::Transform3D;
    use thalos_optimization::{
        TrajectoryOperator, domain::context::OptimizationContext, operators::JointCenteringOperator,
    };

    fn sample_pose() -> Pose {
        Pose::new(FrameId::World, FrameId::Id(1), Transform3D::identity())
    }

    fn make_pick(id: u64, constraints: OperationConstraints) -> CoreOperation {
        CoreOperation::Pick {
            id: OperationId(id.to_string()),
            target_pose: sample_pose(),
            constraints,
        }
    }

    #[test]
    fn compile_with_operations_builds_range_constraint_query() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));

        let constraints = OperationConstraints {
            position_tolerance: Some(0.001),
            orientation_tolerance: Some(0.5_f64.to_radians()),
            ..Default::default()
        };
        let ops = vec![make_pick(1, constraints)];

        let result = compiler
            .compile_with_operations(&ops, &h.ctx())
            .expect("compile_with_operations failed");
        let opc = result;

        // Should have a valid plan
        assert!(!opc.plan.merged_trajectory.is_empty());
        assert_eq!(opc.plan.segments.len(), 5, "Pick expands to 5 segments");

        // RangeConstraintQuery should cover all waypoints
        let total_wps = opc.plan.waypoint_count;
        for i in 0..total_wps {
            // Pick has tight position_tolerance → can_modify_position should be false
            assert!(
                !opc.constraint_query.can_modify_position(i),
                "waypoint {} should NOT allow position modification (tight tolerance)",
                i
            );
        }

        // Orientation tolerance is 0.5°, do not allow relaxation > 0.5°
        assert!(
            !opc.constraint_query
                .can_relax_orientation(0, 1.0_f64.to_radians()),
            "should forbid relaxation beyond tolerance"
        );
    }

    #[test]
    fn compile_with_transit_produces_no_constraints() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));

        // Transit has default (empty) constraints → no restrictions
        let transit = CoreOperation::Transit {
            id: OperationId("2".to_string()),
            target_pose: sample_pose(),
            constraints: OperationConstraints::default(),
        };
        let ops = vec![transit];

        let result = compiler
            .compile_with_operations(&ops, &h.ctx())
            .expect("compile_with_operations failed");
        let opc = result;

        assert_eq!(opc.plan.segments.len(), 1, "Transit expands to 1 segment");
        // Unconstrained transit should allow everything
        assert!(
            opc.constraint_query.can_modify_position(0),
            "unconstrained transit should allow position modification"
        );
        assert!(
            opc.constraint_query
                .can_relax_orientation(0, 10.0_f64.to_radians()),
            "unconstrained transit should allow orientation relaxation"
        );
    }

    // ── 2.2 Provenance preservation (semantic propagation pipeline) ─────

    #[test]
    fn compile_with_operations_builds_provenance() {
        use thalos_core::operation::MotionRole;

        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));

        let ops = vec![
            make_pick(1, OperationConstraints::default()),
            CoreOperation::Transit {
                id: OperationId("2".to_string()),
                target_pose: sample_pose(),
                constraints: OperationConstraints::default(),
            },
        ];

        let opc = compiler
            .compile_with_operations(&ops, &h.ctx())
            .expect("compile_with_operations failed");
        let prov = &opc.provenance;

        // Pick expands to 5 nodes, Transit to 1 → 6 provenance entries.
        assert_eq!(prov.len(), 6, "Pick(5) + Transit(1) → 6 provenance entries");

        // Each entry preserves the originating node's operation_id.
        for (i, p) in prov.iter().enumerate() {
            let expected = if i < 5 { "1" } else { "2" };
            assert_eq!(
                p.operation_id,
                OperationId(expected.to_string()),
                "entry {i} must keep operation_id {expected}"
            );
        }

        // Roles match the expansion order (pick_nodes_have_correct_roles_in_order).
        let expected_roles = [
            MotionRole::Approach,
            MotionRole::Execution,
            MotionRole::Interaction,
            MotionRole::Departure,
            MotionRole::Departure,
            MotionRole::Transit,
        ];
        for (i, (p, role)) in prov.iter().zip(expected_roles.iter()).enumerate() {
            assert_eq!(p.role, *role, "entry {i} must keep role {role:?}");
        }

        // Waypoint ranges are contiguous, in-bounds, and cover the whole plan.
        assert_eq!(prov.first().unwrap().waypoint_range.start, 0);
        assert_eq!(
            prov.last().unwrap().waypoint_range.end,
            opc.plan.waypoint_count
        );
        for w in prov.windows(2) {
            assert_eq!(w[0].waypoint_range.end, w[1].waypoint_range.start);
        }

        // Per-segment metadata mirrors provenance (segments ↔ nodes 1:1).
        assert_eq!(opc.plan.segments.len(), prov.len());
        for (seg, p) in opc.plan.segments.iter().zip(prov.iter()) {
            assert_eq!(seg.operation_id, Some(p.operation_id.clone()));
            assert_eq!(seg.role, Some(p.role));
        }
    }

    #[test]
    fn compile_legacy_path_segments_have_no_operation_metadata() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let program = PlanningProgram::new(vec![MotionSegment::MoveJ {
            origin: OperationId("test".into()),
            target: vec![1.0, 1.0],
            max_velocity: None,
            max_acceleration: None,
        }]);

        let plan = compiler
            .compile(&program, &h.ctx())
            .expect("compile failed");

        assert_eq!(plan.segments.len(), 1);
        let seg = &plan.segments[0];
        assert!(
            seg.operation_id.is_none(),
            "legacy compile() must leave operation_id None"
        );
        assert!(seg.role.is_none(), "legacy compile() must leave role None");
    }

    // ── 2.7 Provenance survival through optimization ─────────

    #[test]
    fn provenance_survives_through_optimization() {
        use thalos_core::{
            analysis::region::{
                ProblemRegion, RegionId, RegionKind, RegionSeverity, project_semantic_problem,
            },
            evaluation::{
                CollisionMetrics, JointSafetyMetrics, ManipulabilityMetrics, PlanMetrics,
            },
            operation::MotionRole,
        };
        use thalos_optimization::{
            PipelineConfig, TrajectoryOperator, domain::context::OptimizationContext,
            operators::Retime, pipeline::OptimizationPipeline,
        };

        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));

        let ops = vec![
            make_pick(1, OperationConstraints::default()),
            CoreOperation::Transit {
                id: OperationId("2".to_string()),
                target_pose: sample_pose(),
                constraints: OperationConstraints::default(),
            },
        ];
        let opc = compiler
            .compile_with_operations(&ops, &h.ctx())
            .expect("compile_with_operations failed");

        // Retime preserves the waypoint count, so provenance ranges stay
        // valid after optimization. Its temporal post-pass runs with the
        // compiled constraint query.
        let retime = Retime::new(3.0, 10.0);
        let operators: [&dyn TrajectoryOperator; 1] = [&retime];

        let robot = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = &opc.plan.merged_trajectory;
        let full_region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Velocity,
            RegionSeverity::Warning,
            0..traj.len(),
        );

        use std::f64::consts::PI as STD_PI;
        use thalos_optimization::domain::context::JointLimits;
        let opt_ctx = OptimizationContext {
            joint_limits: JointLimits {
                lower: vec![-STD_PI, -STD_PI],
                upper: vec![STD_PI, STD_PI],
                velocity: None,
                acceleration: None,
            },
            ..OptimizationContext::default()
        };
        let metrics = PlanMetrics::new(
            0.0,
            0,
            ManipulabilityMetrics::new(0.0, 0.0, 0, 0),
            JointSafetyMetrics::new(1.0, 0.0, 0),
            CollisionMetrics::new(1.0, 0, 0),
            0.0,
            0.0,
        );

        let pipeline = OptimizationPipeline::new(PipelineConfig::default());
        let result = pipeline
            .optimize_regions(
                &operators,
                &robot,
                traj,
                &[full_region],
                &metrics,
                &opt_ctx,
                Some(&opc.constraint_query as &dyn ConstraintQuery),
            )
            .expect("pipeline optimize failed");

        // Retime preserves the waypoint count.
        assert_eq!(result.trajectory.len(), traj.len());

        // Provenance still resolves semantic context against the OPTIMIZED
        // trajectory (projection uses waypoint ranges, not trajectory data).
        let pick_region = ProblemRegion::new(
            RegionId(1),
            RegionKind::Singularity,
            RegionSeverity::Critical,
            0..2,
        );
        let sp = project_semantic_problem(&pick_region, &opc.provenance);
        assert_eq!(sp.operation_id, Some(OperationId("1".to_string())));
        assert_eq!(sp.role, Some(MotionRole::Approach));

        let transit_start = opc.plan.segments[5].waypoint_range.start;
        let transit_region = ProblemRegion::new(
            RegionId(2),
            RegionKind::Velocity,
            RegionSeverity::Warning,
            transit_start..transit_start + 1,
        );
        let sp2 = project_semantic_problem(&transit_region, &opc.provenance);
        assert_eq!(sp2.operation_id, Some(OperationId("2".to_string())));
        assert_eq!(sp2.role, Some(MotionRole::Transit));
    }

    #[test]
    fn constraint_query_from_compiler_affects_optimization_operator() {
        use thalos_core::analysis::region::{ProblemRegion, RegionId, RegionKind, RegionSeverity};
        use thalos_core::models::{RobotModel, RobotRegistry};

        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));

        // Pick with tight position tolerance → JointCentering should NOT modify
        let constraints = OperationConstraints {
            position_tolerance: Some(0.001),
            ..Default::default()
        };
        let ops = vec![make_pick(1, constraints)];

        let plan = compiler
            .compile_with_operations(&ops, &h.ctx())
            .expect("compile_with_operations failed");

        // Use a Planar2R robot (same as TestHarness)
        let robot = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = &plan.plan.merged_trajectory;

        use std::f64::consts::PI as STD_PI;
        use thalos_optimization::domain::context::JointLimits;
        let opt_ctx = OptimizationContext {
            joint_limits: JointLimits {
                lower: vec![-STD_PI, -STD_PI],
                upper: vec![STD_PI, STD_PI],
                velocity: None,
                acceleration: None,
            },
            ..OptimizationContext::default()
        };

        // Full trajectory region
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Constraint,
            RegionSeverity::Warning,
            0..traj.len(),
        );

        let jc = JointCenteringOperator::new(1.0); // snap to center

        // Apply WITHOUT constraints → joints should center
        let without = jc.apply(&robot, traj, &region, &opt_ctx, None).unwrap();
        let without_joints = without.waypoints()[0].joints().to_vec();

        // Apply WITH constraints → joints should NOT move (position_tolerance is tight)
        let with = jc
            .apply(
                &robot,
                traj,
                &region,
                &opt_ctx,
                Some(&plan.constraint_query as &dyn ConstraintQuery),
            )
            .unwrap();
        let with_joints = with.waypoints()[0].joints().to_vec();

        // Without constraints: joints moved toward center (not all-zero start)
        // With constraints: joints unchanged from original
        let original_joints = traj.waypoints()[0].joints();

        // Verify that with constraints, waypoints are preserved
        for (i, (&w, &o)) in with_joints.iter().zip(original_joints.iter()).enumerate() {
            assert!(
                (w - o).abs() < 1e-10,
                "constrained waypoint[{}] joint {} should match original (diff={})",
                0,
                i,
                (w - o).abs()
            );
        }
    }

    // ── 3.7 Origin preservation (IR-2 → IR-3, invariant I2) ───────────────

    #[test]
    fn compile_preserves_origin_from_movej_segment() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let program = PlanningProgram::new(vec![MotionSegment::MoveJ {
            origin: OperationId("op-j".to_string()),
            target: vec![0.5, 0.3],
            max_velocity: None,
            max_acceleration: None,
        }]);

        let plan = compiler
            .compile(&program, &h.ctx())
            .expect("compile failed");
        assert_eq!(plan.segments.len(), 1);
        assert_eq!(
            plan.segments[0].origin,
            OperationId("op-j".to_string()),
            "PlannedSegment must copy origin from its source MotionSegment"
        );
    }

    #[test]
    fn compile_preserves_origin_from_movel_segment() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let program = PlanningProgram::new(vec![MotionSegment::MoveL {
            origin: OperationId("op-l".to_string()),
            frame: FrameId::World,
            target_pose: sample_pose(),
            max_velocity: None,
        }]);

        let plan = compiler
            .compile(&program, &h.ctx())
            .expect("compile failed");
        assert_eq!(plan.segments.len(), 1);
        assert_eq!(
            plan.segments[0].origin,
            OperationId("op-l".to_string()),
            "PlannedSegment must copy origin from its source MotionSegment"
        );
    }

    #[test]
    fn compile_preserves_distinct_origins_across_segments() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let program = PlanningProgram::new(vec![
            MotionSegment::MoveJ {
                origin: OperationId("pick-1".to_string()),
                target: vec![0.5, 0.3],
                max_velocity: None,
                max_acceleration: None,
            },
            MotionSegment::MoveJ {
                origin: OperationId("place-2".to_string()),
                target: vec![1.0, 1.0],
                max_velocity: None,
                max_acceleration: None,
            },
        ]);

        let plan = compiler
            .compile(&program, &h.ctx())
            .expect("compile failed");
        assert_eq!(plan.segments.len(), 2);
        assert_eq!(plan.segments[0].origin, OperationId("pick-1".to_string()));
        assert_eq!(plan.segments[1].origin, OperationId("place-2".to_string()));
    }

    // ── T8 (M2): deterministic segment-start joints (design ADR-3) ─────────
    //
    // Spec semantic-ik-fallback "Same target from two contexts" + "Segment-
    // start context for materialization": the joints a materializer/verifier
    // solves IK from for segment N are the END joints of segment N-1 — never
    // the runtime snapshot — and segment 0 starts from the caller's current
    // joints. Deterministic per start: same program, same start → same joints.

    #[test]
    fn segment_start_joints_are_deterministic_per_segment() {
        let h = TestHarness::new();
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let program = PlanningProgram::new(vec![
            MotionSegment::MoveJ {
                origin: OperationId("test".into()),
                target: vec![0.5, 0.3],
                max_velocity: None,
                max_acceleration: None,
            },
            MotionSegment::MoveJ {
                origin: OperationId("test".into()),
                target: vec![1.0, 0.7],
                max_velocity: None,
                max_acceleration: None,
            },
        ]);
        let plan = compiler
            .compile(&program, &h.ctx())
            .expect("compile failed");

        let current = vec![0.1, 0.2];

        let seg0_start = segment_start_joints(&plan, 0, &current);
        assert_eq!(
            seg0_start, current,
            "segment 0 must start from the caller's current joints"
        );

        let seg1_start = segment_start_joints(&plan, 1, &current);
        let seg0_end = plan.segments[0]
            .trajectory
            .waypoints()
            .last()
            .expect("segment 0 trajectory")
            .joints()
            .to_vec();
        assert_eq!(
            seg1_start, seg0_end,
            "segment 1 must start from the END joints of segment 0"
        );
        assert_ne!(
            seg1_start, current,
            "segment-start joints must never be the snapshot when a previous segment exists"
        );
    }

    // ── T9 (M2): dispatcher-level semantic fallback (design ADR-4) ──────────
    //
    // Spec semantic-ik-fallback "MoveL pose unreachable but translation
    // reachable": the FINAL pose of a user-authored MoveL is resolved by the
    // dispatcher BEFORE planning (GoalResolver::resolve_pose). When that full
    // pose has no IK solution but the translation converges, the segment
    // compiles through the position-only path (`plan_position`) — the same
    // semantic a MoveLPosition declares from the start.

    #[test]
    fn movel_with_unreachable_final_pose_falls_back_to_position_planning() {
        /// Mock solver with the SCARA profile: full-pose IK exhausts
        /// `MaxIterations`, translation-only IK converges.
        struct PoseFailsPositionConvergesIKSolver;

        impl IKSolver for PoseFailsPositionConvergesIKSolver {
            fn solve(
                &self,
                q0: &[f64],
                goal: thalos_core::kinematics::inverse::IKGoal,
            ) -> Result<IKResult, IkError> {
                match goal {
                    thalos_core::kinematics::inverse::IKGoal::Pose(_) => {
                        Ok(IKResult::max_iterations(q0.to_vec(), 100, 1.5, None))
                    }
                    thalos_core::kinematics::inverse::IKGoal::Position(_) => {
                        Ok(IKResult::converged(q0.to_vec(), 1, 0.0, None))
                    }
                }
            }
        }

        let robot = RobotRegistry::create_default(RobotModel::Planar2R);
        let state = RobotState::zero(2);
        let ik = PoseFailsPositionConvergesIKSolver;
        let ctx = SegmentPlanningContext {
            robot: &robot,
            current_state: &state,
            ik_solver: &ik,
            tcp: None,
        };
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let program = PlanningProgram::new(vec![
            MotionSegment::MoveJ {
                origin: OperationId("op-j".to_string()),
                target: vec![0.5, 0.5],
                max_velocity: None,
                max_acceleration: None,
            },
            MotionSegment::MoveL {
                origin: OperationId("op-l".to_string()),
                frame: FrameId::World,
                target_pose: Pose::new(
                    FrameId::World,
                    FrameId::Id(1),
                    Transform3D::from_translation(thalos_math::Vector3::new(1.5, 0.5, 0.0)),
                ),
                max_velocity: None,
            },
        ]);

        // RED (BUG 2): on current code the dispatcher resolves the final pose
        // with IKGoal::Pose → MaxIterations → "segment 2 failed: Inverse
        // kinematics failed for target pose". The semantic fallback must make
        // the reachable-translation MoveL compile.
        let plan = compiler.compile(&program, &ctx).expect(
            "a MoveL whose final pose is unreachable but translation is reachable must compile via the position fallback",
        );
        assert_eq!(plan.segments.len(), 2);
        assert!(!plan.merged_trajectory.is_empty());

        // The last waypoint is the position-resolved state (mock returns q0).
        let last = plan
            .merged_trajectory
            .waypoints()
            .last()
            .unwrap()
            .joints()
            .to_vec();
        assert_eq!(last, vec![0.5, 0.5]);
    }
}
