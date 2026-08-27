/// The instantaneous physical/kinematic state of a single robot joint.
#[derive(Debug, Clone, PartialEq)]
pub struct JointState {
    /// Angular or linear position (radians or meters).
    pub position: Option<f64>,
    /// Angular or linear velocity (rad/s or m/s).
    pub velocity: Option<f64>,
    /// Actuator effort (torque in Nm or force in N).
    pub effort: Option<f64>,
}

impl JointState {
    /// Create a joint state with only position specified.
    pub fn position(position: f64) -> Self {
        Self {
            position: Some(position),
            velocity: None,
            effort: None,
        }
    }

    /// Create a joint state with position and velocity specified.
    pub fn position_and_velocity(position: f64, velocity: f64) -> Self {
        Self {
            position: Some(position),
            velocity: Some(velocity),
            effort: None,
        }
    }

    /// Create a joint state with position, velocity, and effort specified.
    pub fn full(position: f64, velocity: f64, effort: f64) -> Self {
        Self {
            position: Some(position),
            velocity: Some(velocity),
            effort: Some(effort),
        }
    }

    /// Empty joint state where no interfaces are available.
    pub fn empty() -> Self {
        Self {
            position: None,
            velocity: None,
            effort: None,
        }
    }
}

/// The instantaneous kinematic state of a robot: timestamp and joint configurations.
///
/// This type exists so that subsystems (planning, simulation, control,
/// temporal analysis) can express "I only need the current joint values"
/// without pulling in the full robot description.
#[derive(Debug, Clone, PartialEq)]
pub struct RobotState {
    /// Acquisition timestamp in seconds.
    pub timestamp: f64,
    /// Joint states for all degrees of freedom.
    pub joints: Vec<JointState>,
}

impl RobotState {
    pub fn new(timestamp: f64, joints: Vec<JointState>) -> Self {
        Self { timestamp, joints }
    }

    /// Convenience constructor: create a `RobotState` with timestamp 0.0 from joint positions.
    pub fn from_positions(positions: impl IntoIterator<Item = f64>) -> Self {
        let joints = positions.into_iter().map(JointState::position).collect();
        Self {
            timestamp: 0.0,
            joints,
        }
    }

    /// Convenience constructor: all joints set to zero position at timestamp 0.0.
    pub fn zero(dof: usize) -> Self {
        Self {
            timestamp: 0.0,
            joints: vec![JointState::position(0.0); dof],
        }
    }

    /// Extract joint positions as a vector if all joints have position available.
    ///
    /// Returns `None` if any joint position is `None`.
    pub fn positions(&self) -> Option<Vec<f64>> {
        self.joints.iter().map(|j| j.position).collect()
    }

    /// Extract joint velocities as a vector if all joints have velocity available.
    ///
    /// Returns `None` if any joint velocity is `None`.
    pub fn velocities(&self) -> Option<Vec<f64>> {
        self.joints.iter().map(|j| j.velocity).collect()
    }

    /// Extract joint efforts as a vector if all joints have effort available.
    ///
    /// Returns `None` if any joint effort is `None`.
    pub fn efforts(&self) -> Option<Vec<f64>> {
        self.joints.iter().map(|j| j.effort).collect()
    }

    /// Validate whether this state satisfies the specified `StateRequirement`.
    pub fn validate_requirement(&self, req: &StateRequirement) -> Result<(), StateSatisfactionError> {
        for (i, j) in self.joints.iter().enumerate() {
            if req.require_position && j.position.is_none() {
                return Err(StateSatisfactionError::MissingPosition { joint_index: i });
            }
            if req.require_velocity && j.velocity.is_none() {
                return Err(StateSatisfactionError::MissingVelocity { joint_index: i });
            }
            if req.require_effort && j.effort.is_none() {
                return Err(StateSatisfactionError::MissingEffort { joint_index: i });
            }
        }
        Ok(())
    }

    /// Check whether this state satisfies the specified `StateRequirement`.
    pub fn satisfies(&self, req: &StateRequirement) -> bool {
        self.validate_requirement(req).is_ok()
    }
}

/// Declarative state observation requirements for execution policies, control loops,
/// and deviation monitoring.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct StateRequirement {
    /// Whether position (q) observation is required for every joint.
    pub require_position: bool,
    /// Whether velocity (q̇) observation is required for every joint.
    pub require_velocity: bool,
    /// Whether effort (τ) observation is required for every joint.
    pub require_effort: bool,
}

impl StateRequirement {
    /// Only joint position (q) is required (e.g. kinematic planning / open-loop execution).
    pub fn position_only() -> Self {
        Self {
            require_position: true,
            require_velocity: false,
            require_effort: false,
        }
    }

    /// Joint position (q) and velocity (q̇) are required (e.g. closed-loop velocity tracking).
    pub fn position_and_velocity() -> Self {
        Self {
            require_position: true,
            require_velocity: true,
            require_effort: false,
        }
    }

    /// Joint position (q), velocity (q̇), and effort (τ) are required (e.g. full dynamic/torque control).
    pub fn full() -> Self {
        Self {
            require_position: true,
            require_velocity: true,
            require_effort: true,
        }
    }

    /// No state observation required.
    pub fn none() -> Self {
        Self {
            require_position: false,
            require_velocity: false,
            require_effort: false,
        }
    }

    /// Check if a given `JointState` satisfies this requirement.
    pub fn satisfies_joint(&self, joint: &JointState) -> bool {
        (!self.require_position || joint.position.is_some())
            && (!self.require_velocity || joint.velocity.is_some())
            && (!self.require_effort || joint.effort.is_some())
    }
}

/// Error indicating that a `RobotState` fails to meet a declarative `StateRequirement`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StateSatisfactionError {
    /// Joint at `joint_index` lacks required position data.
    MissingPosition { joint_index: usize },
    /// Joint at `joint_index` lacks required velocity data.
    MissingVelocity { joint_index: usize },
    /// Joint at `joint_index` lacks required effort data.
    MissingEffort { joint_index: usize },
}

impl std::fmt::Display for StateSatisfactionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::MissingPosition { joint_index } => {
                write!(f, "joint at index {joint_index} missing required position")
            }
            Self::MissingVelocity { joint_index } => {
                write!(f, "joint at index {joint_index} missing required velocity")
            }
            Self::MissingEffort { joint_index } => {
                write!(f, "joint at index {joint_index} missing required effort")
            }
        }
    }
}

impl std::error::Error for StateSatisfactionError {}

/// The mathematical deviation between an expected robot state and an observed robot state.
#[derive(Debug, Clone, PartialEq)]
pub struct StateDeviation {
    /// Difference in timestamps (t_obs - t_exp).
    pub timestamp_delta: f64,
    /// Joint position errors (q_obs - q_exp) if both states provide position.
    pub position_error: Option<Vec<f64>>,
    /// Joint velocity errors (q̇_obs - q̇_exp) if both states provide velocity.
    pub velocity_error: Option<Vec<f64>>,
    /// Joint effort errors (τ_obs - τ_exp) if both states provide effort.
    pub effort_error: Option<Vec<f64>>,
}

impl StateDeviation {
    /// Compute the state deviation from an expected state to an observed state.
    ///
    /// Returns `None` if joint vector lengths do not match.
    pub fn compute(expected: &RobotState, observed: &RobotState) -> Option<Self> {
        if expected.joints.len() != observed.joints.len() {
            return None;
        }

        let timestamp_delta = observed.timestamp - expected.timestamp;

        let position_error = {
            let mut diffs = Vec::with_capacity(expected.joints.len());
            let mut valid = true;
            for (exp, obs) in expected.joints.iter().zip(observed.joints.iter()) {
                match (exp.position, obs.position) {
                    (Some(e), Some(o)) => diffs.push(o - e),
                    _ => {
                        valid = false;
                        break;
                    }
                }
            }
            if valid { Some(diffs) } else { None }
        };

        let velocity_error = {
            let mut diffs = Vec::with_capacity(expected.joints.len());
            let mut valid = true;
            for (exp, obs) in expected.joints.iter().zip(observed.joints.iter()) {
                match (exp.velocity, obs.velocity) {
                    (Some(e), Some(o)) => diffs.push(o - e),
                    _ => {
                        valid = false;
                        break;
                    }
                }
            }
            if valid { Some(diffs) } else { None }
        };

        let effort_error = {
            let mut diffs = Vec::with_capacity(expected.joints.len());
            let mut valid = true;
            for (exp, obs) in expected.joints.iter().zip(observed.joints.iter()) {
                match (exp.effort, obs.effort) {
                    (Some(e), Some(o)) => diffs.push(o - e),
                    _ => {
                        valid = false;
                        break;
                    }
                }
            }
            if valid { Some(diffs) } else { None }
        };

        Some(Self {
            timestamp_delta,
            position_error,
            velocity_error,
            effort_error,
        })
    }

    /// Compute the maximum absolute position error across all joints.
    pub fn max_position_error(&self) -> Option<f64> {
        self.position_error.as_ref().map(|errs| {
            errs.iter().map(|e| e.abs()).fold(0.0, f64::max)
        })
    }

    /// Compute the maximum absolute velocity error across all joints.
    pub fn max_velocity_error(&self) -> Option<f64> {
        self.velocity_error.as_ref().map(|errs| {
            errs.iter().map(|e| e.abs()).fold(0.0, f64::max)
        })
    }

    /// Compute the maximum absolute effort error across all joints.
    pub fn max_effort_error(&self) -> Option<f64> {
        self.effort_error.as_ref().map(|errs| {
            errs.iter().map(|e| e.abs()).fold(0.0, f64::max)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_requirement_validation() {
        let pos_state = RobotState::from_positions(vec![1.0, 2.0]);
        let req_pos = StateRequirement::position_only();
        let req_vel = StateRequirement::position_and_velocity();

        assert!(pos_state.satisfies(&req_pos));
        assert!(!pos_state.satisfies(&req_vel));
        assert_eq!(
            pos_state.validate_requirement(&req_vel),
            Err(StateSatisfactionError::MissingVelocity { joint_index: 0 })
        );

        let full_state = RobotState::new(
            0.0,
            vec![
                JointState::position_and_velocity(1.0, 0.1),
                JointState::position_and_velocity(2.0, 0.2),
            ],
        );
        assert!(full_state.satisfies(&req_vel));
        assert_eq!(full_state.velocities(), Some(vec![0.1, 0.2]));
    }

    #[test]
    fn test_state_deviation_computation() {
        let expected = RobotState::new(
            1.0,
            vec![
                JointState::position_and_velocity(0.0, 1.0),
                JointState::position_and_velocity(1.0, 0.5),
            ],
        );
        let observed = RobotState::new(
            1.1,
            vec![
                JointState::position_and_velocity(0.05, 1.1),
                JointState::position_and_velocity(0.98, 0.4),
            ],
        );

        let dev = StateDeviation::compute(&expected, &observed).expect("valid deviation");
        assert!((dev.timestamp_delta - 0.1).abs() < 1e-9);
        assert_eq!(dev.position_error, Some(vec![0.05, -0.020000000000000018]));
        assert_eq!(dev.velocity_error, Some(vec![0.10000000000000009, -0.09999999999999998]));
        assert!((dev.max_position_error().unwrap() - 0.05).abs() < 1e-9);
    }
}

