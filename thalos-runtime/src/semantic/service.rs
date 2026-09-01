use std::sync::Arc;
use serde::{Deserialize, Serialize};

use thalos_document::program_document::ProgramDocument;

use thalos_engine::core::analysis::location::Location;
use thalos_engine::core::analysis::observation::{Observation, Severity};
use thalos_engine::core::{
    execution::{program::ExecutionProgram, runtime::RuntimeProgram},
    kinematics::{
        forward::ForwardKinematics, inverse::DampedLeastSquaresSolver, inverse::IKConfig,
    },
    motion::MotionProfile,
    robot::state::RobotState,
    spatial::frame::FrameRegistry,
};
use thalos_engine::intelligence::semantic::SemanticExpert;
use thalos_engine::planning::{
    motion::{
        compiler::{DefaultPlannerDispatcher, PlanCompiler},
        planner::SegmentPlanningContext,
    },
    resolver::{MotionResolver, ResolutionError},
    timeline::TimelineScheduler,
};
use thalos_engine::semantic::{
    lowering::{SemanticLowering, context::LoweringContext},
    validation::validate,
};

use crate::error::RuntimeError;
use crate::services::scene::SceneService;

/// IK solver configuration for semantic compilation (spec `ik-config`).
const IK_CONFIG: IKConfig = IKConfig {
    max_iterations: 1000,
    tolerance: 1e-4,
    lambda: 0.1,
};

/// Default JOINT-space motion profile for the semantic planner.
const JOINT_PROFILE: MotionProfile = MotionProfile {
    max_velocity: 1.0,
    max_acceleration: 0.5,
    max_jerk: None,
};

/// Default CARTESIAN-space motion profile for the semantic planner.
const CARTESIAN_PROFILE: MotionProfile = MotionProfile {
    max_velocity: 0.1,
    max_acceleration: 0.5,
    max_jerk: None,
};

/// Validation diagnostics from the pipeline.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationSummary {
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
}

/// Processing metadata.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CompileMetadata {
    pub instruction_count: usize,
}

/// Result of compiling a semantic task without executing it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SemanticCompileOutput {
    pub validation: ValidationSummary,
    pub metadata: CompileMetadata,
    pub motion_program: ExecutionProgram,
}

/// Result of running (compiling, planning, scheduling, and loading) a semantic task.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SemanticRunOutput {
    pub segment_count: usize,
    pub duration_secs: f64,
    pub waypoints: Vec<serde_json::Value>,
    pub event_count: usize,
    pub warnings: Vec<String>,
}

/// Application service for semantic compilation and execution.
pub struct SemanticService {
    scene: Arc<SceneService>,
}

impl SemanticService {
    pub fn new(scene: Arc<SceneService>) -> Self {
        Self { scene }
    }

    /// Compile a semantic task program into an `ExecutionProgram` (IR-1).
    pub fn compile_program(&self, task: &ProgramDocument) -> Result<SemanticCompileOutput, RuntimeError> {
        let observations = validate(&task.program);
        if observations.iter().any(|o| o.severity == Severity::Error) {
            let msgs: Vec<String> = observations
                .iter()
                .filter(|o| o.severity == Severity::Error)
                .map(validation_message)
                .collect();
            return Err(RuntimeError::SemanticValidationError {
                message: msgs.join("; "),
            });
        }

        let expert = SemanticExpert::analyze(&task.program);
        let warnings: Vec<String> = observations
            .iter()
            .chain(expert.iter())
            .filter(|o| o.severity != Severity::Error)
            .map(validation_message)
            .collect();

        let provider = task.scene.knowledge();
        let ctx = LoweringContext::new(&provider)
            .with_default_profile(JOINT_PROFILE)
            .with_default_cartesian_profile(Some(CARTESIAN_PROFILE));

        let ir = thalos_engine::semantic::ir::SemanticIr::from(&task.program);
        let mp = SemanticLowering::lower(&ir, &ctx)
            .map_err(|e| RuntimeError::LoweringError { message: format!("{e}") })?;

        Ok(SemanticCompileOutput {
            validation: ValidationSummary {
                errors: vec![],
                warnings,
            },
            metadata: CompileMetadata {
                instruction_count: mp.instructions.len(),
            },
            motion_program: mp,
        })
    }

    /// Compile + plan + schedule + load semantic task into the active scene runtime.
    pub async fn run_program(&self, task: &ProgramDocument) -> Result<SemanticRunOutput, RuntimeError> {
        let snapshot = self.scene.snapshot().await?;
        let chain = snapshot.chain.clone();
        let initial_joints = snapshot.joints.clone();

        let (
            duration_secs,
            segment_count,
            waypoints_json,
            event_count,
            warnings,
            compiled,
            runtime_program,
        ) = {
            let observations = validate(&task.program);
            if observations.iter().any(|o| o.severity == Severity::Error) {
                let msgs: Vec<String> = observations
                    .iter()
                    .filter(|o| o.severity == Severity::Error)
                    .map(validation_message)
                    .collect();
                return Err(RuntimeError::SemanticValidationError {
                    message: msgs.join("; "),
                });
            }

            let expert = SemanticExpert::analyze(&task.program);
            let warnings: Vec<String> = observations
                .iter()
                .chain(expert.iter())
                .filter(|o| o.severity != Severity::Error)
                .map(validation_message)
                .collect();

            let provider = task.scene.knowledge();
            let ctx = LoweringContext::new(&provider)
                .with_default_profile(JOINT_PROFILE)
                .with_default_cartesian_profile(Some(CARTESIAN_PROFILE));

            let ir = thalos_engine::semantic::ir::SemanticIr::from(&task.program);
            let mp = SemanticLowering::lower(&ir, &ctx)
                .map_err(|e| RuntimeError::LoweringError { message: format!("{e}") })?;

            let dof = chain.dof_count();
            let fk = ForwardKinematics::new(chain.clone());
            let ik_solver = DampedLeastSquaresSolver::from_config(fk, *chain.end_effector(), IK_CONFIG);

            let mut registry = FrameRegistry::new();
            registry.create("world");

            let resolver = MotionResolver::new(&ik_solver, &registry, &initial_joints, dof)
                .map_err(map_resolver_error)?;
            let resolution = resolver.resolve(&mp).map_err(map_resolver_error)?;

            let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
            let current_state = RobotState::from_positions(initial_joints.clone());
            let seg_ctx = build_seg_ctx(&snapshot, &chain, &current_state, &ik_solver);
            let compiled = compiler
                .compile(&resolution.planning, &seg_ctx)
                .map_err(|e| RuntimeError::Planning(e.into()))?;

            let runtime_program = TimelineScheduler::new().schedule(&mp, &compiled, resolution.runtime);

            let wps_json: Vec<serde_json::Value> = compiled
                .merged_trajectory
                .waypoints()
                .iter()
                .map(|p| serde_json::json!({"time_secs": p.timestamp(), "joints": p.joints()}))
                .collect();

            (
                compiled.duration,
                compiled.segments.len(),
                wps_json,
                runtime_program.events.len(),
                warnings,
                compiled,
                runtime_program,
            )
        };

        self.scene.schedule_program(compiled, runtime_program).await?;

        Ok(SemanticRunOutput {
            segment_count,
            duration_secs,
            waypoints: waypoints_json,
            event_count,
            warnings,
        })
    }

    /// Compile + plan + schedule + load semantic source code directly into active scene runtime.
    pub async fn run_source(&self, source: &str) -> Result<SemanticRunOutput, RuntimeError> {
        let snapshot = self.scene.snapshot().await?;
        let chain = snapshot.chain.clone();
        let initial_joints = snapshot.joints.clone();

        let ast = thalos_engine::lang::parse_source(source)
            .map_err(|errs| RuntimeError::SemanticValidationError {
                message: errs.into_iter().map(|e| format!("{}", e)).collect::<Vec<_>>().join("; "),
            })?;

        let sem_program = thalos_engine::semantic::compiler::SemanticCompiler::compile(&ast)
            .map_err(|errs| RuntimeError::SemanticValidationError {
                message: errs.join("; "),
            })?;

        let resolved = thalos_engine::semantic::resolver::SemanticResolver::resolve(&sem_program)
            .map_err(|errs| RuntimeError::SemanticValidationError {
                message: errs.join("; "),
            })?;

        let planning_input = thalos_engine::planning::input::PlanningInput::from_resolved(&resolved);

        let fk = ForwardKinematics::new(chain.clone());
        let ik_solver = DampedLeastSquaresSolver::from_config(fk, *chain.end_effector(), IK_CONFIG);
        let current_state = RobotState::from_positions(initial_joints.clone());
        let seg_ctx = build_seg_ctx(&snapshot, &chain, &current_state, &ik_solver);

        let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
        let compiled = compiler
            .compile(&planning_input.to_program(), &seg_ctx)
            .map_err(|e| RuntimeError::Planning(e.into()))?;

        let wps_json: Vec<serde_json::Value> = compiled
            .merged_trajectory
            .waypoints()
            .iter()
            .map(|p| serde_json::json!({"time_secs": p.timestamp(), "joints": p.joints()}))
            .collect();

        let duration_secs = compiled.duration;
        let segment_count = compiled.segments.len();

        let runtime_program = RuntimeProgram { events: vec![] };
        self.scene.schedule_program(compiled, runtime_program).await?;

        Ok(SemanticRunOutput {
            segment_count,
            duration_secs,
            waypoints: wps_json,
            event_count: 0,
            warnings: vec![],
        })
    }
}

fn validation_message(o: &Observation) -> String {
    let op = match &o.location {
        Location::Operation(id) => format!("{id}"),
        other => format!("{other:?}"),
    };
    format!("[{:?}] {:?} (op: {op})", o.severity, o.kind)
}

fn map_resolver_error(e: ResolutionError) -> RuntimeError {
    match e {
        ResolutionError::DofMismatch { .. } => RuntimeError::DofMismatch { message: format!("{e}") },
        _ => RuntimeError::Planning(thalos_engine::planning::error::PlanningError::InvalidContext(format!("{e}"))),
    }
}

pub fn build_seg_ctx<'a>(
    snapshot: &'a crate::RuntimeSnapshot,
    chain: &'a thalos_engine::core::robot::serial_chain::SerialChain,
    current_state: &'a RobotState,
    ik_solver: &'a dyn thalos_engine::core::kinematics::inverse::IKSolver,
) -> SegmentPlanningContext<'a> {
    SegmentPlanningContext {
        robot: chain,
        current_state,
        ik_solver,
        tcp: snapshot.active_tcp.as_ref(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_engine::core::{
        kinematics::{
            forward::ForwardKinematics,
            inverse::{IKGoal, IKResult, IKSolver, IkError},
        },
        models::{RobotModel, RobotRegistry},
        robot::{state::RobotState, tool_frame::ToolFrame},
    };
    use crate::RuntimeSnapshot;

    struct NoopIKSolver;

    impl IKSolver for NoopIKSolver {
        fn solve(&self, q0: &[f64], _goal: IKGoal) -> Result<IKResult, IkError> {
            Ok(IKResult::converged(q0.to_vec(), 1, 0.0, None))
        }
    }

    fn snapshot_with_tcp(active_tcp: Option<ToolFrame>) -> RuntimeSnapshot {
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let joints = vec![0.0, 0.0];
        let fk_result = ForwardKinematics::new(chain.clone()).evaluate(&joints);
        RuntimeSnapshot {
            robot: Some(RobotModel::Planar2R),
            robot_source: None,
            robot_name: "test".into(),
            robot_id: "planar_2r".into(),
            joints_meta: vec![],
            joints,
            chain,
            fk_result,
            ik_result: None,
            active_plan: None,
            execution: None,
            active_tcp,
            generated_at: chrono::Utc::now(),
        }
    }

    #[test]
    fn seg_ctx_tcp_some_when_active_tcp_set() {
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let tcp = ToolFrame::identity(*chain.end_effector());
        let snapshot = snapshot_with_tcp(Some(tcp.clone()));
        let state = RobotState::zero(2);
        let ik = NoopIKSolver;

        let ctx = build_seg_ctx(&snapshot, &chain, &state, &ik);

        let resolved = ctx
            .tcp
            .expect("seg_ctx.tcp must be Some when active_tcp is set");
        assert_eq!(
            resolved.base_frame, tcp.base_frame,
            "seg_ctx.tcp must reference the active TCP frame"
        );
    }

    #[test]
    fn seg_ctx_tcp_none_when_active_tcp_unset() {
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let snapshot = snapshot_with_tcp(None);
        let state = RobotState::zero(2);
        let ik = NoopIKSolver;

        let ctx = build_seg_ctx(&snapshot, &chain, &state, &ik);

        assert!(
            ctx.tcp.is_none(),
            "seg_ctx.tcp must be None when active_tcp is unset"
        );
    }
}
