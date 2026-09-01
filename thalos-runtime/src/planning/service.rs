use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thalos_language_service::{Diagnostic, DiagnosticSeverity, SourceSpan};

use thalos_engine::core::execution::plan::ExecutionPlan;
use thalos_engine::core::execution::runtime::RuntimeProgram;
use thalos_engine::core::kinematics::{forward::ForwardKinematics, inverse::DampedLeastSquaresSolver};
use thalos_engine::core::motion::segment::MotionSegment;
use thalos_engine::core::robot::serial_chain::SerialChain;
use thalos_engine::core::robot::state::RobotState;
use thalos_engine::core::robot::tool_frame::ToolFrame;
use thalos_engine::core::{
    ids::OperationId,
    operation::{Operation, OperationConstraints},
    spatial::{frame::FrameId, pose::Pose},
};
use thalos_engine::lang::parser::parse_source;
use thalos_engine::math::{Quaternion, Transform3D, UnitQuaternion, Vector3};
use thalos_engine::planning::error::CompileError;
use thalos_engine::planning::input::PlanningInput;
use thalos_engine::planning::motion::compiler::{DefaultPlannerDispatcher, PlanCompiler};
use thalos_engine::planning::motion::planner::PlanningContext;
use thalos_engine::planning::motion::program::PlanningProgram;
use thalos_engine::semantic::compiler::SemanticCompiler;
use thalos_engine::semantic::model::MotionTarget;
use thalos_engine::semantic::resolver::SemanticResolver;

use crate::error::RuntimeError;
use crate::scene::RuntimeSnapshot;
use crate::services::scene::SceneService;

const IK_MAX_ITERS: usize = 500;
const IK_TOLERANCE: f64 = 1e-6;
const IK_LAMBDA: f64 = 0.1;

/// Active robot planning context extracted from SceneRuntime or constructed for offline/CLI planning.
#[derive(Debug, Clone)]
pub struct RobotPlanningContext {
    pub robot_id: String,
    pub chain: SerialChain,
    pub initial_positions: Vec<f64>,
    pub tcp: Option<ToolFrame>,
}

/// Result of contextual planning facade.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", content = "payload")]
pub enum PlanResult {
    Planned(ExecutionPlan),
    Diagnostics(Vec<Diagnostic>),
}

// ── Motion plan request DTOs ──

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "kind", content = "value")]
pub enum RotationDto {
    Quaternion { w: f64, x: f64, y: f64, z: f64 },
    Ypr { roll: f64, pitch: f64, yaw: f64 },
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PoseTargetDto {
    pub translation: [f64; 3],
    #[serde(default)]
    pub rotation: Option<RotationDto>,
}

impl PoseTargetDto {
    pub fn to_pose(&self, target_frame: FrameId) -> Pose {
        let [tx, ty, tz] = self.translation;
        let translation = Vector3::new(tx, ty, tz);

        let rotation = match self.rotation {
            Some(RotationDto::Quaternion { w, x, y, z }) => {
                let q = Quaternion::new(w, x, y, z);
                UnitQuaternion::new(q.normalize_or_identity())
                    .unwrap_or_else(|_| UnitQuaternion::identity())
            }
            Some(RotationDto::Ypr { roll, pitch, yaw }) => {
                UnitQuaternion::from_euler(roll, pitch, yaw)
            }
            None => UnitQuaternion::identity(),
        };

        let transform = Transform3D {
            translation,
            rotation,
        };

        Pose::new(FrameId::World, target_frame, transform)
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "type")]
pub enum MotionSegmentDto {
    #[serde(rename = "movej")]
    MoveJ {
        target: Vec<f64>,
        #[serde(default)]
        max_velocity: Option<f64>,
        #[serde(default)]
        max_acceleration: Option<f64>,
    },
    #[serde(rename = "movel")]
    MoveL {
        #[serde(default)]
        frame_id: Option<u64>,
        target: PoseTargetDto,
        #[serde(default)]
        max_velocity: Option<f64>,
    },
}

impl MotionSegmentDto {
    const MANUAL_ORIGIN: &'static str = "manual";

    fn into_segment(self, default_ee: FrameId) -> MotionSegment {
        match self {
            MotionSegmentDto::MoveJ {
                target,
                max_velocity,
                max_acceleration,
            } => MotionSegment::MoveJ {
                origin: OperationId(Self::MANUAL_ORIGIN.into()),
                target,
                max_velocity,
                max_acceleration,
            },
            MotionSegmentDto::MoveL {
                frame_id,
                target,
                max_velocity,
            } => {
                let frame = frame_id.map_or(default_ee, FrameId::Id);
                if target.rotation.is_some() {
                    let pose = target.to_pose(frame);
                    MotionSegment::MoveL {
                        origin: OperationId(Self::MANUAL_ORIGIN.into()),
                        frame,
                        target_pose: pose,
                        max_velocity,
                    }
                } else {
                    MotionSegment::MoveLPosition {
                        origin: OperationId(Self::MANUAL_ORIGIN.into()),
                        frame,
                        target_position: target.translation,
                        max_velocity,
                    }
                }
            }
        }
    }
}

#[derive(Debug, Default, Serialize, Deserialize, Clone)]
pub struct OperationConstraintsDto {
    #[serde(default)]
    pub position_tolerance: Option<f64>,
    #[serde(default)]
    pub orientation_tolerance: Option<f64>,
    #[serde(default)]
    pub velocity_limit: Option<f64>,
}

impl OperationConstraintsDto {
    fn into_constraints(self) -> OperationConstraints {
        OperationConstraints {
            position_tolerance: self.position_tolerance,
            orientation_tolerance: self.orientation_tolerance,
            joint_deviation_limit: None,
            velocity_limit: self.velocity_limit,
            approach_direction: None,
            retreat_direction: None,
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "type")]
pub enum OperationDto {
    #[serde(rename = "pick")]
    Pick {
        id: u64,
        #[serde(default)]
        frame_id: Option<u64>,
        target: PoseTargetDto,
        #[serde(default)]
        constraints: OperationConstraintsDto,
    },
    #[serde(rename = "place")]
    Place {
        id: u64,
        #[serde(default)]
        frame_id: Option<u64>,
        target: PoseTargetDto,
        #[serde(default)]
        constraints: OperationConstraintsDto,
    },
    #[serde(rename = "transit")]
    Transit {
        id: u64,
        #[serde(default)]
        frame_id: Option<u64>,
        target: PoseTargetDto,
        #[serde(default)]
        constraints: OperationConstraintsDto,
    },
}

impl OperationDto {
    pub fn into_operation(self, default_ee: FrameId) -> Operation {
        match self {
            OperationDto::Pick {
                id,
                frame_id,
                target,
                constraints,
            } => Operation::Pick {
                id: OperationId(id.to_string()),
                target_pose: target.to_pose(frame_id.map_or(default_ee, FrameId::Id)),
                constraints: constraints.into_constraints(),
            },
            OperationDto::Place {
                id,
                frame_id,
                target,
                constraints,
            } => Operation::Place {
                id: OperationId(id.to_string()),
                target_pose: target.to_pose(frame_id.map_or(default_ee, FrameId::Id)),
                constraints: constraints.into_constraints(),
            },
            OperationDto::Transit {
                id,
                frame_id,
                target,
                constraints,
            } => Operation::Transit {
                id: OperationId(id.to_string()),
                target_pose: target.to_pose(frame_id.map_or(default_ee, FrameId::Id)),
                constraints: constraints.into_constraints(),
            },
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct MotionPlanRequest {
    #[serde(default)]
    pub segments: Vec<MotionSegmentDto>,
    #[serde(default)]
    pub operations: Option<Vec<OperationDto>>,
}

impl MotionPlanRequest {
    pub fn into_program(self, default_ee: FrameId) -> PlanningProgram {
        let segments = self
            .segments
            .into_iter()
            .map(|s| s.into_segment(default_ee))
            .collect();
        PlanningProgram::new(segments)
    }
}

use std::sync::Arc;

// ── Planning Application Service ──

use thalos_engine::planning::motion::program::CompiledPlan;

pub struct PlanningService {
    scene: Arc<SceneService>,
}

impl PlanningService {
    pub fn new(scene: Arc<SceneService>) -> Self {
        Self { scene }
    }

    /// Pure facade / application orchestrator for planning THLS source against an explicit robot context.
    /// Does NOT depend on SceneRuntime directly, allowing usage in tests, CLI, and offline planning.
    pub fn plan_thls_source_with_context(
        source: &str,
        program_id: &str,
        revision: u64,
        context: &RobotPlanningContext,
    ) -> PlanResult {
        let (result, _) = Self::plan_thls_source_internal(source, program_id, revision, context);
        result
    }

    fn plan_thls_source_internal(
        source: &str,
        program_id: &str,
        revision: u64,
        context: &RobotPlanningContext,
    ) -> (PlanResult, Option<CompiledPlan>) {
        // 1. Calculate source fingerprint directly on raw source input
        let mut hasher = Sha256::new();
        hasher.update(source.as_bytes());
        let source_fingerprint = format!("{:x}", hasher.finalize());

        // 2. Parse source
        let ast = match parse_source(source) {
            Ok(ast) => ast,
            Err(parse_errors) => {
                let diags = parse_errors
                    .into_iter()
                    .map(|err| {
                        let span = err.span();
                        Diagnostic {
                            severity: DiagnosticSeverity::Error,
                            code: Some("THL_SYNTAX_ERROR".into()),
                            message: err.to_string(),
                            span: SourceSpan::new(span.start as u32, span.end as u32),
                        }
                    })
                    .collect();
                return (PlanResult::Diagnostics(diags), None);
            }
        };

        // 3. Semantic compile
        let sem_program = match SemanticCompiler::compile(&ast) {
            Ok(p) => p,
            Err(errors) => {
                let diags = errors
                    .into_iter()
                    .map(|msg| Diagnostic {
                        severity: DiagnosticSeverity::Error,
                        code: Some("THL_SEMANTIC_ERROR".into()),
                        message: msg,
                        span: SourceSpan::new(0, source.len() as u32),
                    })
                    .collect();
                return (PlanResult::Diagnostics(diags), None);
            }
        };

        // 4. Semantic resolve
        let resolved = match SemanticResolver::resolve(&sem_program) {
            Ok(r) => r,
            Err(errors) => {
                let diag = Diagnostic {
                    severity: DiagnosticSeverity::Error,
                    code: Some("THL_RESOLUTION_ERROR".into()),
                    message: errors.join("; "),
                    span: SourceSpan::new(0, source.len() as u32),
                };
                return (PlanResult::Diagnostics(vec![diag]), None);
            }
        };

        // 5. Build PlanningInput & check DOF / kinematic invariants
        let planning_input = PlanningInput::from_resolved(&resolved);
        for motion in &planning_input.motions {
            if let MotionTarget::Joints(ref j) = motion.target {
                if j.values.len() != context.chain.dof_count() {
                    let span = motion.provenance.span.as_ref().map_or(
                        SourceSpan::new(0, source.len() as u32),
                        |s| SourceSpan::new(s.start as u32, s.end as u32),
                    );
                    let diag = Diagnostic {
                        severity: DiagnosticSeverity::Error,
                        code: Some("THL_DOF_MISMATCH".into()),
                        message: format!(
                            "Joint target has {} degrees of freedom, but robot '{}' has {}",
                            j.values.len(),
                            context.robot_id,
                            context.chain.dof_count()
                        ),
                        span,
                    };
                    return (PlanResult::Diagnostics(vec![diag]), None);
                }
            }
        }

        // 6. Instantiate SegmentPlanningContext with active robot kinematics
        let fk = ForwardKinematics::new(context.chain.clone());
        let solver = DampedLeastSquaresSolver::new(
            fk,
            *context.chain.end_effector(),
            IK_MAX_ITERS,
            IK_TOLERANCE,
            IK_LAMBDA,
        );
        let robot_state = RobotState::from_positions(context.initial_positions.clone());
        let ctx = PlanningContext {
            robot: &context.chain,
            current_state: &robot_state,
            ik_solver: &solver,
            tcp: context.tcp.as_ref(),
        };

        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let program = planning_input.to_program();

        // 7. Compile plan and map errors to Diagnostics with SourceSpan
        let compiled = match compiler.compile(&program, &ctx) {
            Ok(c) => c,
            Err(err_with_ctx) => {
                let seg_idx = err_with_ctx.segment_index;
                let span = planning_input
                    .motions
                    .get(seg_idx)
                    .and_then(|m| m.provenance.span.as_ref())
                    .map_or(SourceSpan::new(0, source.len() as u32), |s| {
                        SourceSpan::new(s.start as u32, s.end as u32)
                    });
                let diag = Diagnostic {
                    severity: DiagnosticSeverity::Error,
                    code: Some("THL_UNREACHABLE_TARGET".into()),
                    message: format!("Kinematic planning error in segment {}: {}", seg_idx, err_with_ctx.source),
                    span,
                };
                return (PlanResult::Diagnostics(vec![diag]), None);
            }
        };

        // 8. Build ExecutionPlan and freeze provenance
        let base_plan: ExecutionPlan = match thalos_engine::planning::execution_plan_builder::ExecutionPlanBuilder::build(&compiled) {
            Ok(p) => p,
            Err(e) => {
                let diag = Diagnostic {
                    severity: DiagnosticSeverity::Error,
                    code: Some("THL_PLAN_BUILD_ERROR".into()),
                    message: e.to_string(),
                    span: SourceSpan::new(0, source.len() as u32),
                };
                return (PlanResult::Diagnostics(vec![diag]), None);
            }
        };

        let plan = base_plan.with_provenance(
            program_id.to_string(),
            revision,
            source_fingerprint,
            Some(context.robot_id.clone()),
        );

        (PlanResult::Planned(plan), Some(compiled))
    }

    /// Orchestrates planning for THLS source using the active robot context from SceneService.
    pub async fn plan_thls_source(
        &self,
        source: &str,
        program_id: &str,
        revision: u64,
    ) -> Result<PlanResult, RuntimeError> {
        let snapshot = self.scene.snapshot().await?;
        let context = RobotPlanningContext {
            robot_id: snapshot.robot_id.clone(),
            chain: snapshot.chain.clone(),
            initial_positions: snapshot.joints.clone(),
            tcp: snapshot.active_tcp.clone(),
        };
        Ok(Self::plan_thls_source_with_context(source, program_id, revision, &context))
    }

    /// Orchestrates planning for THLS source and previews the result in SceneService if successful.
    pub async fn preview_thls_source(
        &self,
        source: &str,
        program_id: &str,
        revision: u64,
    ) -> Result<(PlanResult, RuntimeSnapshot), RuntimeError> {
        let snapshot = self.scene.snapshot().await?;
        let context = RobotPlanningContext {
            robot_id: snapshot.robot_id.clone(),
            chain: snapshot.chain.clone(),
            initial_positions: snapshot.joints.clone(),
            tcp: snapshot.active_tcp.clone(),
        };

        let (plan_result, compiled_plan_opt) = Self::plan_thls_source_internal(source, program_id, revision, &context);

        match plan_result {
            PlanResult::Planned(plan) => {
                if let Some(compiled) = compiled_plan_opt {
                    self.scene.set_program_provenance(
                        program_id,
                        revision,
                        plan.source_fingerprint.clone().unwrap_or_default(),
                    ).await;
                    let updated_snapshot = self.scene.preview_plan(compiled).await?;
                    Ok((PlanResult::Planned(plan), updated_snapshot))
                } else {
                    let updated_snapshot = self.scene.snapshot().await?;
                    Ok((PlanResult::Planned(plan), updated_snapshot))
                }
            }
            PlanResult::Diagnostics(diags) => {
                let updated_snapshot = self.scene.snapshot().await?;
                Ok((PlanResult::Diagnostics(diags), updated_snapshot))
            }
        }
    }

    /// Compile a motion plan request and schedule it on the scene.
    /// Returns the updated runtime state. The robot does NOT move — this is strictly
    /// a "compile + preview" operation. Execution requires a subsequent
    /// call to `start_execution`.
    pub async fn preview_plan(
        &self,
        mut payload: MotionPlanRequest,
    ) -> Result<RuntimeSnapshot, RuntimeError> {
        let snapshot = self.scene.snapshot().await?;
        let default_frame = snapshot.resolve_default_frame();

        let fk = ForwardKinematics::new(snapshot.chain.clone());
        let solver = DampedLeastSquaresSolver::new(
            fk,
            default_frame,
            IK_MAX_ITERS,
            IK_TOLERANCE,
            IK_LAMBDA,
        );
        let robot_state = RobotState::from_positions(snapshot.joints.clone());
        let ctx = PlanningContext {
            robot: &snapshot.chain,
            current_state: &robot_state,
            ik_solver: &solver,
            tcp: snapshot.active_tcp.as_ref(),
        };
        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let map_err = |err: CompileError| RuntimeError::CompileFailed {
            segment_index: err.segment_index,
            message: err.to_string(),
        };

        let compiled = if let Some(ops) = payload.operations.take() {
            if ops.is_empty() {
                return Ok(snapshot);
            }
            let operations: Vec<Operation> = ops
                .into_iter()
                .map(|op| op.into_operation(default_frame))
                .collect();
            compiler
                .compile_with_operations(&operations, &ctx)
                .map_err(map_err)?
                .plan
        } else {
            let program = payload.into_program(default_frame);
            if program.segments.is_empty() {
                return Ok(snapshot);
            }
            compiler.compile(&program, &ctx).map_err(map_err)?
        };

        let updated_snapshot = self
            .scene
            .schedule_program(compiled, RuntimeProgram::default())
            .await?;
        Ok(updated_snapshot)
    }
}
