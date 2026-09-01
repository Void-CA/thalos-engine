use serde::{Deserialize, Serialize};
use thalos_engine::prelude::*;
use super::executor::ExecutionSessionState;

pub use thalos_ports::SignalQuality;

/// ExecutionSnapshot (ADR-014)
/// Lightweight operational status snapshot of an active session.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ExecutionSnapshot {
    pub session_id: ExecutionSessionId,
    pub state: ExecutionSessionState,
    pub elapsed_seconds: f64,
    pub progress: f64,
}

/// ObservationSnapshot (ADR-014)
/// Sensor/Robot telemetry snapshot emitted during execution.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ObservationSnapshot {
    pub sampled_at_ns: u64,
    pub joint_positions: Vec<f64>,
    pub joint_velocities: Vec<f64>,
    pub tcp_pose: [f64; 7],
    pub signal_quality: SignalQuality,
}

/// ExecutionDeviation (ADR-014)
/// Computed operational difference between expected trajectory and observed telemetry.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ExecutionDeviation {
    pub tcp_error_mm: f64,
    pub max_joint_error_rad: f64,
    pub tracking_error: f64,
}

/// RunSnapshot (ADR-014)
/// Consolidated observation DTO consumed by the UI (RoboticsRunSurface).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RunSnapshot {
    pub execution: ExecutionSnapshot,
    pub observation: ObservationSnapshot,
    pub deviation: Option<ExecutionDeviation>,
}

impl RunSnapshot {
    pub fn compute_deviation(expected_joints: &[f64], observed: &ObservationSnapshot) -> Option<ExecutionDeviation> {
        if expected_joints.len() != observed.joint_positions.len() {
            return None;
        }

        let max_joint_error_rad = expected_joints
            .iter()
            .zip(&observed.joint_positions)
            .map(|(exp, obs)| (exp - obs).abs())
            .fold(0.0f64, f64::max);

        let tracking_error = max_joint_error_rad; // Euclidean joint error as proxy
        let tcp_error_mm = tracking_error * 100.0; // Simulated mm error scaling

        Some(ExecutionDeviation {
            tcp_error_mm,
            max_joint_error_rad,
            tracking_error,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_deviation_valid() {
        let expected = vec![0.0, 1.0, 0.5];
        let observed = ObservationSnapshot {
            sampled_at_ns: 1000,
            joint_positions: vec![0.01, 0.99, 0.52],
            joint_velocities: vec![0.0, 0.0, 0.0],
            tcp_pose: [0.0; 7],
            signal_quality: SignalQuality::Nominal,
        };

        let dev = RunSnapshot::compute_deviation(&expected, &observed).unwrap();
        assert!((dev.max_joint_error_rad - 0.02).abs() < 1e-6);
        assert!((dev.tcp_error_mm - 2.0).abs() < 1e-6);
    }
}
