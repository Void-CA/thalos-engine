use serde::{Deserialize, Serialize};
use thalos_engine::prelude::*;

/// ExecutionTarget (ADR-014)
/// Specifies where the execution is targeted (Simulation vs Physical Hardware).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExecutionTarget {
    Simulation,
    Hardware { robot_id: RobotId },
}

/// ExecutionPolicyMode (ADR-014)
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExecutionPolicyMode {
    Once,
    Repeat { count: usize },
    Continuous,
    Until { condition: String },
}

/// ExecutionPolicy (ADR-014)
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionPolicy {
    pub mode: ExecutionPolicyMode,
}

impl Default for ExecutionPolicy {
    fn default() -> Self {
        Self {
            mode: ExecutionPolicyMode::Once,
        }
    }
}

/// ExecutionRequest (ADR-014)
/// Explicit intent to execute a motion plan under specific policy and target conditions.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionRequest {
    pub plan_id: MotionPlanId,
    pub target: ExecutionTarget,
    pub policy: ExecutionPolicy,
    pub requirements: Vec<ResourceRequirement>,
}

impl ExecutionRequest {
    pub fn new(
        plan_id: impl Into<String>,
        target: ExecutionTarget,
        policy: ExecutionPolicy,
        requirements: Vec<ResourceRequirement>,
    ) -> Self {
        Self {
            plan_id: MotionPlanId(plan_id.into()),
            target,
            policy,
            requirements,
        }
    }
}
