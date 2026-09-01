use thiserror::Error;

use thalos_engine::core::analysis::workspace::WorkspaceError;
use thalos_engine::core::kinematics::inverse::IkError;
use thalos_engine::core::models::RobotModelError;

use thalos_engine::planning::error::PlanningError;

/// Errors specific to the RobotController trait.
#[derive(Error, Debug, PartialEq, Clone)]
pub enum ControllerError {
    #[error("controller is already connected")]
    AlreadyConnected,

    #[error("controller is not connected")]
    NotConnected,

    #[error("this capability is not supported by the current controller")]
    UnsupportedCapability,

    #[error("operation timed out")]
    Timeout,

    #[error("protocol error: {0}")]
    Protocol(String),

    #[error("invalid manifest: {0}")]
    InvalidManifest(String),

    /// Backend management (resilience-presentation PR2a): the requested
    /// backend is not registered.
    #[error("backend not found: {0}")]
    NotFound(String),

    /// Backend management (PR2a): the serial port opened but no firmware
    /// answered the handshake.
    #[error("no firmware detected on the serial port — switch to Simulation or check the port")]
    NoFirmware,

    /// Backend management (PR2a): the serial port could not be opened
    /// (missing or occupied device).
    #[error("serial port is in use or cannot be opened: {0}")]
    PortInUse(String),

    /// Backend management (PR2a): the serial connection was lost mid-operation.
    #[error("connection to the execution backend was lost")]
    ConnectionLost,
}

impl ControllerError {
    pub fn error_code(&self) -> &'static str {
        match self {
            ControllerError::AlreadyConnected => "already_connected",
            ControllerError::NotConnected => "not_connected",
            ControllerError::UnsupportedCapability => "unsupported_capability",
            ControllerError::Timeout => "timeout",
            ControllerError::Protocol(_) => "protocol_error",
            ControllerError::InvalidManifest(_) => "invalid_manifest",
            ControllerError::NotFound(_) => "not_found",
            ControllerError::NoFirmware => "no_firmware",
            ControllerError::PortInUse(_) => "port_in_use",
            ControllerError::ConnectionLost => "connection_lost",
        }
    }
}

impl From<ControllerError> for RuntimeError {
    fn from(e: ControllerError) -> Self {
        // R4-001: a controller failure must preserve the REAL error code so the
        // API can surface e.g. `connection_lost` / `not_connected` to the
        // frontend — never degrade into the meaningless joint_count_mismatch
        // placeholder the previous mapping produced.
        RuntimeError::ControllerFailed { source: e }
    }
}

#[derive(Error, Debug)]
pub enum RuntimeError {
    #[error("robot model error: {0}")]
    RobotModel(#[from] RobotModelError),

    #[error("workspace error: {0}")]
    Workspace(#[from] WorkspaceError),

    #[error("planning error: {0}")]
    Planning(#[from] PlanningError),

    #[error("IK error: {0}")]
    Ik(#[from] IkError),

    #[error("joint count mismatch: expected {expected}, received {received}")]
    JointCountMismatch { expected: usize, received: usize },

    #[error("tool frame not found: frame {frame_id} does not exist in the robot chain")]
    ToolFrameNotFound { frame_id: u64 },

    /// A feature-flagged surface was invoked while the flag is disabled (D5).
    #[error("feature disabled: {feature}")]
    FeatureDisabled { feature: &'static str },

    /// A compiled plan failed replacement validation (D4): no waypoints or
    /// zero duration — the runtime refuses to schedule a degenerate plan.
    #[error("invalid compiled plan: {reason}")]
    InvalidCompiledPlan { reason: String },

    /// Undo was requested with an empty command history (spec command-endpoints
    /// "Undo with empty history" → 409). No applied command carries an inverse.
    #[error("no applied command to undo")]
    EmptyCommandHistory,

    /// Undo was requested for a STALE inverse (R4-001 → 409): the active plan
    /// no longer matches the program the command produced (a non-commanded
    /// path — e.g. a re-schedule — replaced it). Applying the inverse would
    /// corrupt the plan, so the runtime refuses without mutation.
    #[error("stale undo: the active plan was replaced by a path that is not the command's pre-state")]
    StaleUndo,

    /// Undo was requested with a STALE expected version (PR2 → 409): the
    /// history mutated between the atomic peek (`last_with_version`) and the
    /// commit — a concurrent apply/undo slipped into the TOCTOU window. The
    /// runtime refuses without mutation; the caller must re-read the pair.
    #[error("undo version mismatch: expected {expected}, got {actual}")]
    UndoVersionMismatch { expected: u64, actual: u64 },

    /// A controller-level failure (R4-001): the underlying `RobotController`
    /// returned an error that must reach the API with its REAL machine-readable
    /// code (`connection_lost`, `not_connected`, `no_firmware`, …). Previously
    /// every controller error collapsed into a meaningless
    /// `JointCountMismatch{0,0}` (422) that the frontend could not act on.
    #[error("{source}")]
    ControllerFailed { source: ControllerError },

    /// `Repeat` execution was requested with no plan loaded (S8). The API
    /// maps this to 400 `no_active_plan` — `Once` keeps the legacy behavior
    /// of starting (and immediately idling) without a plan.
    #[error("no active plan to execute")]
    NoActivePlan,

    /// A requested plan execution was refused because the plan revision does not match the active program revision.
    #[error("stale plan revision: expected revision {expected}, plan has revision {actual}")]
    StalePlanRevision { expected: u64, actual: u64 },

    /// A requested plan execution was refused because the plan source fingerprint does not match the current program source.
    #[error("stale plan fingerprint: expected {expected}, plan has {actual}")]
    StalePlanFingerprint { expected: String, actual: String },

    #[error("{message}")]
    InvalidUrdf { message: String },

    #[error("{message}")]
    UrdfChainError { message: String },

    #[error("segment {segment_index} failed: {message}")]
    CompileFailed {
        segment_index: usize,
        message: String,
    },

    #[error("semantic validation error: {message}")]
    SemanticValidationError { message: String },

    #[error("lowering error: {message}")]
    LoweringError { message: String },

    #[error("dof mismatch: {message}")]
    DofMismatch { message: String },

    #[error("persistence error: {message}")]
    Persistence { message: String },

    #[error("robot not found: {id}")]
    RobotNotFound { id: String },

    #[error("workspace not found: {id}")]
    WorkspaceNotFound { id: String },
}

#[cfg(test)]
mod tests {
    use super::*;

    /// R4-001: a controller error must NOT collapse into `JointCountMismatch{0,0}`.
    /// `ConnectionLost` maps to `ControllerFailed`, preserving the real code so
    /// the frontend can offer the Reconectar CTA.
    #[test]
    fn from_controller_connection_lost_preserves_connection_lost_code() {
        let err: RuntimeError = ControllerError::ConnectionLost.into();
        match err {
            RuntimeError::ControllerFailed { source } => {
                assert_eq!(source, ControllerError::ConnectionLost);
                assert_eq!(source.error_code(), "connection_lost");
            }
            other => panic!("ConnectionLost must map to ControllerFailed, got {other:?}"),
        }
    }

    /// R4-001: every `ControllerError` keeps its real machine-readable code
    /// through the `RuntimeError` conversion — none may degrade to the
    /// meaningless `joint_count_mismatch` placeholder.
    #[test]
    fn from_controller_preserves_real_error_codes() {
        fn code(e: ControllerError) -> &'static str {
            let err: RuntimeError = e.into();
            match err {
                RuntimeError::ControllerFailed { source } => source.error_code(),
                other => panic!("must be ControllerFailed, got {other:?}"),
            }
        }
        assert_eq!(code(ControllerError::AlreadyConnected), "already_connected");
        assert_eq!(code(ControllerError::NotConnected), "not_connected");
        assert_eq!(code(ControllerError::Timeout), "timeout");
        assert_eq!(code(ControllerError::Protocol("boom".into())), "protocol_error");
        assert_eq!(code(ControllerError::NoFirmware), "no_firmware");
        assert_eq!(code(ControllerError::PortInUse("busy".into())), "port_in_use");
        assert_eq!(code(ControllerError::ConnectionLost), "connection_lost");
    }
}

impl RuntimeError {
    /// Machine-readable error code for the API layer.
    ///
    /// This lets the API return specific error codes (e.g. `joint_limit_violation`,
    /// `ik_failed`) without depending on `thalos-planning` or other implementation
    /// crates directly.
    pub fn error_code(&self) -> &'static str {
        match self {
            RuntimeError::RobotModel(e) => match e {
                RobotModelError::InvalidRobotId { .. } => "invalid_robot_id",
                RobotModelError::ModelSpecMismatch { .. } => "model_spec_mismatch",
            },
            RuntimeError::Workspace(e) => match e {
                WorkspaceError::InvalidSampleCount(_) => "invalid_sample_count",
                WorkspaceError::InvalidTolerance(_) => "invalid_tolerance",
                WorkspaceError::InvalidPoint(_) => "invalid_point",
                WorkspaceError::EmptyWorkspace => "empty_workspace",
            },
            RuntimeError::Planning(e) => match e {
                PlanningError::IkFailed { .. } => "ik_failed",
                PlanningError::IkFailedPosition { .. } => "ik_failed",
                PlanningError::JointLimitViolation { .. } => "joint_limit_violation",
                PlanningError::JointCountMismatch { .. } => "joint_count_mismatch",
                PlanningError::InvalidGoal(_) => "invalid_goal",
                PlanningError::UnreachableGoal { .. } => "unreachable_goal",
                PlanningError::CollisionDetected { .. } => "collision_detected",
                PlanningError::EmptyProgram => "empty_program",
                PlanningError::InvalidContext(_) => "invalid_context",
                PlanningError::IKFailure { .. } => "ik_failure",
                PlanningError::Ik(e) => match e {
                    IkError::UnsupportedJointType(_) => "unsupported_joint_type",
                },
            },
            RuntimeError::Ik(e) => match e {
                IkError::UnsupportedJointType(_) => "unsupported_joint_type",
            },
            RuntimeError::JointCountMismatch { .. } => "joint_count_mismatch",
            RuntimeError::ToolFrameNotFound { .. } => "tool_frame_not_found",
            RuntimeError::FeatureDisabled { .. } => "feature_disabled",
            RuntimeError::InvalidCompiledPlan { .. } => "invalid_compiled_plan",
            RuntimeError::EmptyCommandHistory => "empty_command_history",
            RuntimeError::StaleUndo => "stale_undo",
            RuntimeError::UndoVersionMismatch { .. } => "undo_version_mismatch",
            RuntimeError::ControllerFailed { source } => source.error_code(),
            RuntimeError::NoActivePlan => "no_active_plan",
            RuntimeError::StalePlanRevision { .. } => "stale_plan_revision",
            RuntimeError::StalePlanFingerprint { .. } => "stale_plan_fingerprint",
            RuntimeError::InvalidUrdf { .. } => "invalid_urdf",
            RuntimeError::UrdfChainError { .. } => "urdf_chain_error",
            RuntimeError::CompileFailed { .. } => "compile_failed",
            RuntimeError::SemanticValidationError { .. } => "semantic_validation_error",
            RuntimeError::LoweringError { .. } => "lowering_error",
            RuntimeError::DofMismatch { .. } => "dof_mismatch",
            RuntimeError::Persistence { .. } => "persistence_error",
            RuntimeError::RobotNotFound { .. } => "robot_not_found",
            RuntimeError::WorkspaceNotFound { .. } => "workspace_not_found",
        }
    }

    /// Returns `true` if this error represents a stale execution artifact (plan or undo state).
    pub fn is_stale(&self) -> bool {
        matches!(
            self,
            RuntimeError::StalePlanRevision { .. }
                | RuntimeError::StalePlanFingerprint { .. }
                | RuntimeError::StaleUndo
        )
    }
}

