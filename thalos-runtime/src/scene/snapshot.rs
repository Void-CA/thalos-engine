use std::sync::Arc;

use chrono::{DateTime, Utc};

use thalos_engine::core::{
    kinematics::{forward::result::FKResult, inverse::result::IKResult},
    models::RobotModel,
    robot::serial_chain::SerialChain,
    robot::tool_frame::ToolFrame,
    spatial::frame::FrameId,
};
use thalos_engine::models::Robot;

use crate::plan::{ActiveMotionPlan, ExecutionSession, SessionStatus};
use crate::state::robot_state::{MotionMode, RobotState};

/// Lightweight joint metadata for URDF-imported robots.
#[derive(Debug, Clone)]
pub struct JointMeta {
    pub name: String,
    pub kind: String,
    pub min: Option<f64>,
    pub max: Option<f64>,
}

/// Immutable snapshot of the runtime state at a point in time.
///
/// Field types remain unchanged for API backward compatibility.
/// The `from_robot_state` constructor is the new construction path;
/// direct field construction is still supported for tests.
#[derive(Debug, Clone)]
pub struct RuntimeSnapshot {
    /// Catalog-membership tag (ADR-003): `Some(X)` = internal catalog robot;
    /// `None` = URDF-imported robot (identity carried by `robot_name`,
    /// `robot_source`, `joints_meta`, and `chain`).
    pub robot: Option<RobotModel>,
    pub robot_source: Option<Robot>,
    pub robot_name: String,
    /// Canonical robot identity (spec robot-identity R1) — mirrors
    /// `SceneRuntime.robot_id`: `metadata.id` for catalog robots,
    /// `urdf:<sha256-trunc-12>` for URDF imports.
    pub robot_id: String,
    pub joints_meta: Vec<JointMeta>,
    pub joints: Vec<f64>,
    pub chain: SerialChain,
    pub fk_result: FKResult,
    pub ik_result: Option<IKResult>,
    pub active_plan: Option<ActiveMotionPlan>,
    pub execution: Option<ExecutionSession>,
    /// Active Tool Center Point (TCP) frame.
    ///
    /// When `Some`, all analysis (workspace, singularity, manipulability)
    /// and IK default to this TCP instead of the flange (`chain.end_effector`).
    /// When `None`, the flange is used as the default working frame.
    pub active_tcp: Option<ToolFrame>,
    pub generated_at: DateTime<Utc>,
}

impl RuntimeSnapshot {
    /// Build a snapshot from a RobotState + runtime context.
    ///
    /// ExecutionSession is derived from RobotState.execution for backward compat.
    pub fn from_robot_state(
        state: &Arc<RobotState>,
        robot: Option<RobotModel>,
        robot_source: Option<Robot>,
        robot_name: String,
        robot_id: String,
        joints_meta: Vec<JointMeta>,
        chain: SerialChain,
        fk_result: FKResult,
        active_plan: Option<ActiveMotionPlan>,
        active_tcp: Option<ToolFrame>,
    ) -> Self {
        let execution = session_from_robot_state(state);
        Self {
            robot,
            robot_source,
            robot_name,
            robot_id,
            joints_meta,
            joints: state.joints.positions.clone(),
            chain,
            fk_result,
            ik_result: None,
            active_plan,
            execution,
            active_tcp,
            generated_at: Utc::now(),
        }
    }

    pub fn trajectory_progress(&self) -> Option<f64> {
        self.active_plan.as_ref().map(|p| p.progress())
    }

    /// Resolve the default frame for IK and motion commands.
    ///
    /// Returns the active TCP base_frame if set, otherwise the flange (end_effector).
    /// This is the canonical source of truth for the "working frame" across all
    /// analysis and motion operations.
    pub fn resolve_default_frame(&self) -> FrameId {
        self.active_tcp
            .as_ref()
            .map(|tcp| tcp.base_frame.clone())
            .unwrap_or_else(|| *self.chain.end_effector())
    }
}

/// Lightweight tick result — derived from RobotState.
///
/// Field types remain unchanged for API backward compatibility.
/// The `from_robot_state` constructor is the new construction path.
#[derive(Debug, Clone)]
pub struct TickDelta {
    pub joints: Vec<f64>,
    pub chain: SerialChain,
    pub fk_result: FKResult,
    pub execution: Option<ExecutionSession>,
    pub plan_duration: f64,
    /// Active Tool Center Point (TCP) frame.
    ///
    /// When `Some`, all analysis (workspace, singularity, manipulability)
    /// and IK default to this TCP instead of the flange (`chain.end_effector`).
    /// When `None`, the flange is used as the default working frame.
    pub active_tcp: Option<ToolFrame>,
}

impl TickDelta {
    pub fn from_robot_state(
        state: &Arc<RobotState>,
        chain: SerialChain,
        fk_result: FKResult,
        plan_duration: f64,
        active_tcp: Option<ToolFrame>,
    ) -> Self {
        let execution = session_from_robot_state(state);
        Self {
            joints: state.joints.positions.clone(),
            chain,
            fk_result,
            execution,
            plan_duration,
            active_tcp,
        }
    }
}

/// Derive an ExecutionSession from RobotState.execution info.
fn session_from_robot_state(state: &Arc<RobotState>) -> Option<ExecutionSession> {
    let progress = state.execution.progress;
    let status = match state.motion.mode {
        MotionMode::Idle if progress >= 1.0 => SessionStatus::Completed,
        MotionMode::Moving => SessionStatus::Running,
        MotionMode::Paused => SessionStatus::Paused,
        MotionMode::Stopping => SessionStatus::Cancelled,
        MotionMode::EStop => SessionStatus::Failed,
        _ => SessionStatus::Ready,
    };
    Some(ExecutionSession::derived(status, progress))
}
