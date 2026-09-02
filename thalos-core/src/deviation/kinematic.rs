use serde::{Deserialize, Serialize};
use super::expected::{ExpectedState, ObservedState};

/// Raw kinematic error metrics computed between an ExpectedState and an ObservedState.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct KinematicError {
    pub joint_position_errors: Vec<f64>,
    pub joint_velocity_errors: Vec<f64>,
    pub cartesian_position_error: Option<f64>,
}

impl KinematicError {
    pub fn compute(expected: &ExpectedState, observed: &ObservedState) -> Self {
        let joint_position_errors: Vec<f64> = observed
            .joint_positions
            .iter()
            .zip(expected.joint_positions.iter())
            .map(|(obs, exp)| obs - exp)
            .collect();

        let joint_velocity_errors: Vec<f64> = observed
            .joint_velocities
            .iter()
            .zip(expected.joint_velocities.iter())
            .map(|(obs, exp)| obs - exp)
            .collect();

        let cartesian_position_error = match (observed.cartesian_pose, expected.cartesian_pose) {
            (Some(obs_pose), Some(exp_pose)) => {
                let dx = obs_pose[0] - exp_pose[0];
                let dy = obs_pose[1] - exp_pose[1];
                let dz = obs_pose[2] - exp_pose[2];
                Some((dx * dx + dy * dy + dz * dz).sqrt())
            }
            _ => None,
        };

        Self {
            joint_position_errors,
            joint_velocity_errors,
            cartesian_position_error,
        }
    }

    /// L2 norm of the joint position error vector.
    pub fn joint_position_error_norm(&self) -> f64 {
        self.joint_position_errors
            .iter()
            .map(|e| e * e)
            .sum::<f64>()
            .sqrt()
    }
}

/// Status indicating whether the observed error is within defined tolerance envelopes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum EnvelopeStatus {
    WithinTolerance,
    Violated,
}

/// Severity classification of a kinematic deviation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviationSeverity {
    Nominal,
    Warning,
    Critical,
    Fault,
}

/// Domain entity representing a complete kinematic deviation analysis.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct KinematicDeviation {
    pub robot_id: String,
    pub sampled_at_ns: u64,
    pub expected: ExpectedState,
    pub observed: ObservedState,
    pub error: KinematicError,
    pub envelope: EnvelopeStatus,
    pub severity: Option<DeviationSeverity>,
}
