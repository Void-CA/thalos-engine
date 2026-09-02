use serde::{Deserialize, Serialize};

/// A snapshot of expected robot state at a given timestamp.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExpectedState {
    pub timestamp_ns: u64,
    pub joint_positions: Vec<f64>,
    pub joint_velocities: Vec<f64>,
    pub cartesian_pose: Option<[f64; 6]>,
}

impl ExpectedState {
    pub fn new(
        timestamp_ns: u64,
        joint_positions: Vec<f64>,
        joint_velocities: Vec<f64>,
        cartesian_pose: Option<[f64; 6]>,
    ) -> Self {
        Self {
            timestamp_ns,
            joint_positions,
            joint_velocities,
            cartesian_pose,
        }
    }
}

/// A snapshot of observed robot state from telemetry at a given timestamp.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ObservedState {
    pub robot_id: String,
    pub sampled_at_ns: u64,
    pub joint_positions: Vec<f64>,
    pub joint_velocities: Vec<f64>,
    pub cartesian_pose: Option<[f64; 6]>,
}

impl ObservedState {
    pub fn new(
        robot_id: impl Into<String>,
        sampled_at_ns: u64,
        joint_positions: Vec<f64>,
        joint_velocities: Vec<f64>,
        cartesian_pose: Option<[f64; 6]>,
    ) -> Self {
        Self {
            robot_id: robot_id.into(),
            sampled_at_ns,
            joint_positions,
            joint_velocities,
            cartesian_pose,
        }
    }
}

/// Trait defining a temporal sampling contract over an expected motion plan or trajectory.
pub trait ExpectedTrajectory {
    /// Returns the expected state at the given nanosecond timestamp,
    /// or None if the timestamp is out of bounds (before start / after end).
    fn sample_at(&self, timestamp_ns: u64) -> Option<ExpectedState>;
}
