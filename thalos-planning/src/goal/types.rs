use thalos_core::{
    kinematics::jacobian::{ManipulabilityReport, SingularityReport},
    robot::state::RobotState,
    spatial::pose::Pose,
};
use thalos_math::Vector3;

#[derive(Debug, Clone)]
pub struct JointGoal(pub Vec<f64>);
impl JointGoal {
    pub fn new(joints: Vec<f64>) -> Self {
        Self(joints)
    }

    pub fn as_slice(&self) -> &[f64] {
        &self.0
    }
}

#[derive(Debug, Clone)]
pub struct ResolvedPoseGoal {
    pub pose: Pose,
    pub state: RobotState,
}

/// A translation-only resolved goal: `position` is the target, `state` is
/// the converged joint configuration (orientation is left unconstrained).
#[derive(Debug, Clone)]
pub struct ResolvedPositionGoal {
    pub position: Vector3,
    pub state: RobotState,
}

#[derive(Debug, Clone, Default)]
pub struct GoalMetadata {
    pub singularity: Option<SingularityReport>,
    pub manipulability: Option<ManipulabilityReport>,
    pub joint_limits_applied: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum MetricAction {
    Ignore,
    Warn(f64),
    Reject(f64),
}

#[derive(Debug, Clone, Copy)]
pub enum MetricKind {
    ConditionNumber,
    YoshikawaManipulability,
}

#[derive(Debug, Clone, Copy)]
pub enum ThresholdDirection {
    HigherIsWorse,
    LowerIsWorse,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanningDecision {
    Accepted,
    AcceptedWithWarnings,
    Rejected,
}

#[derive(Debug, Clone)]
pub struct AssessmentWarning {
    pub metric: MetricKind,
    pub value: f64,
    pub threshold: f64,
}

#[derive(Debug, Clone)]
pub struct PlanningAssessment {
    pub decision: PlanningDecision,
    pub warnings: Vec<AssessmentWarning>,
}

impl PlanningAssessment {
    pub fn accepted() -> Self {
        Self {
            decision: PlanningDecision::Accepted,
            warnings: Vec::new(),
        }
    }

    pub fn is_rejected(&self) -> bool {
        matches!(self.decision, PlanningDecision::Rejected)
    }
}

#[derive(Debug, Clone)]
pub struct ValidatedGoal<G> {
    pub goal: G,
    pub metadata: GoalMetadata,
    pub assessment: PlanningAssessment,
}
