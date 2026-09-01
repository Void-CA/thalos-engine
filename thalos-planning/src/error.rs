use thalos_core::kinematics::inverse::IkError;
use thalos_core::spatial::pose::Pose;
use thiserror::Error;

/// Singularities are not represented here — they are reported as
/// [`GoalMetadata`](crate::goal::GoalMetadata) so the caller decides.
#[derive(Error, Debug, Clone)]
pub enum PlanningError {
    #[error("Inverse kinematics failed for target pose")]
    IkFailed {
        target_pose: Pose,
        reason: IkFailureReason,
    },

    #[error("Inverse kinematics failed for target position")]
    IkFailedPosition {
        target_position: [f64; 3],
        reason: IkFailureReason,
    },

    #[error("IK error: {0}")]
    Ik(#[from] IkError),

    #[error("Joint limit violation at joint {joint_index}: value {value} ∉ [{min}, {max}]")]
    JointLimitViolation {
        joint_index: usize,
        value: f64,
        min: f64,
        max: f64,
    },

    #[error("Invalid goal: {0}")]
    InvalidGoal(String),

    #[error("Joint count mismatch: expected {expected} DOF for robot, got {got} joints")]
    JointCountMismatch {
        expected: usize,
        got: usize,
    },

    #[error("Goal unreachable: {reason}")]
    UnreachableGoal { reason: String },

    /// Se detectó una colisión durante la planificación o validación
    /// de una trayectoria.
    #[error("Collision detected between {:?} and {:?}", involved.0, involved.1)]
    CollisionDetected {
        involved: (
            thalos_core::collision::EntityId,
            thalos_core::collision::EntityId,
        ),
    },

    // ── Program-level planning errors ──────────────────────────────────
    #[error("Motion program is empty (no instructions)")]
    EmptyProgram,

    #[error("Planning context is invalid: {0}")]
    InvalidContext(String),

    #[error("Inverse kinematics failed for pose index {pose_index}")]
    IKFailure { pose_index: usize },
}

impl From<&str> for PlanningError {
    fn from(msg: &str) -> Self {
        PlanningError::InvalidGoal(msg.to_string())
    }
}

/// Compilation of a multi-segment motion program failed.
///
/// Wraps the underlying `PlanningError` with the 0-based index of the
/// segment that failed. The compiler guarantees atomicity — no partial
/// `CompiledPlan` is produced.
#[derive(Error, Debug, Clone)]
#[error("segment {} failed: {source}", .segment_index + 1)]
pub struct CompileError {
    /// 0-based index of the segment that caused the failure.
    pub segment_index: usize,

    /// The underlying planning error.
    #[source]
    pub source: PlanningError,
}

impl CompileError {
    /// 1-based index for user-facing messages ("segment 3").
    pub fn segment_1based(&self) -> usize {
        self.segment_index + 1
    }
}

impl From<CompileError> for PlanningError {
    fn from(e: CompileError) -> Self {
        e.source
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum IkFailureReason {
    MaxIterationsReached,
    NoSolution,
}
