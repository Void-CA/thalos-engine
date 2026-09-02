use serde::{Deserialize, Serialize};

/// Tolerance limits for a single joint.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct JointTolerance {
    pub position: f64,
    pub velocity: f64,
}

impl JointTolerance {
    pub fn new(position: f64, velocity: f64) -> Self {
        Self { position, velocity }
    }

    pub fn uniform(tolerance: f64) -> Self {
        Self {
            position: tolerance,
            velocity: tolerance,
        }
    }
}

/// Abstract contract for querying tolerance thresholds during deviation analysis.
pub trait TolerancePolicy {
    fn joint_tolerance(&self, joint_index: usize) -> JointTolerance;
    fn cartesian_position_tolerance(&self) -> Option<f64>;
}

/// Static tolerance policy implementation configured with explicit per-joint thresholds.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StaticTolerancePolicy {
    pub joint_tolerances: Vec<JointTolerance>,
    pub cartesian_position_tolerance: Option<f64>,
}

impl StaticTolerancePolicy {
    pub fn new(
        joint_tolerances: Vec<JointTolerance>,
        cartesian_position_tolerance: Option<f64>,
    ) -> Self {
        Self {
            joint_tolerances,
            cartesian_position_tolerance,
        }
    }

    pub fn uniform(dof: usize, joint_position_tol: f64, joint_velocity_tol: f64) -> Self {
        let joint_tolerances = vec![JointTolerance::new(joint_position_tol, joint_velocity_tol); dof];
        Self {
            joint_tolerances,
            cartesian_position_tolerance: None,
        }
    }
}

impl TolerancePolicy for StaticTolerancePolicy {
    fn joint_tolerance(&self, joint_index: usize) -> JointTolerance {
        self.joint_tolerances
            .get(joint_index)
            .copied()
            .unwrap_or(JointTolerance::uniform(f64::MAX))
    }

    fn cartesian_position_tolerance(&self) -> Option<f64> {
        self.cartesian_position_tolerance
    }
}
