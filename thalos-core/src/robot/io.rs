use std::time::Duration;
use serde::{Deserialize, Serialize};

use crate::robot::state::{JointState, RobotState};

/// Error types occurring during hardware or simulation adapter I/O operations.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviceIOError {
    CommunicationFailed(String),
    DeviceNotReady(String),
    Timeout,
    HardwareFault(String),
}

impl std::fmt::Display for DeviceIOError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DeviceIOError::CommunicationFailed(msg) => write!(f, "communication failure: {msg}"),
            DeviceIOError::DeviceNotReady(msg) => write!(f, "device not ready: {msg}"),
            DeviceIOError::Timeout => write!(f, "device I/O timeout"),
            DeviceIOError::HardwareFault(msg) => write!(f, "hardware fault: {msg}"),
        }
    }
}

impl std::error::Error for DeviceIOError {}

/// Capability trait for sending control targets/commands to a physical or simulated robot device.
pub trait CommandSink {
    fn send_command(&mut self, target: &RobotState) -> Result<(), DeviceIOError>;
}

/// Capability trait for reading observed state snapshots from a physical or simulated robot device.
pub trait ObservationSource {
    fn read_observation(&mut self) -> Result<RobotState, DeviceIOError>;
}

/// Deterministic behavior scenario for `FakeRobotAdapter` to test closed-loop execution.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FakeRobotScenario {
    /// Follows target state perfectly.
    Nominal,
    /// Adds a fixed position deviation (e.g. mechanical error or slipping).
    PositionDeviation(f64),
    /// Adds a fixed velocity deviation.
    VelocityDeviation(f64),
    /// Simulates stale telemetry by setting timestamp into the past by specified Duration.
    StaleObservation(Duration),
    /// Simulates missing joint state observations (returns None for position).
    MissingObservation,
    /// Simulates hardware fault (returns NaN / invalid values).
    InvalidObservation,
    /// Simulates communication error on I/O operations.
    CommunicationFailure,
}

/// Simulated robot hardware/driver adapter for deterministic closed-loop testing.
#[derive(Debug, Clone)]
pub struct FakeRobotAdapter {
    pub dof: usize,
    pub scenario: FakeRobotScenario,
    pub current_target: Option<RobotState>,
    pub timestamp: f64,
}

impl FakeRobotAdapter {
    pub fn new(dof: usize) -> Self {
        Self {
            dof,
            scenario: FakeRobotScenario::Nominal,
            current_target: None,
            timestamp: 0.0,
        }
    }

    pub fn with_scenario(mut self, scenario: FakeRobotScenario) -> Self {
        self.scenario = scenario;
        self
    }
}

impl CommandSink for FakeRobotAdapter {
    fn send_command(&mut self, target: &RobotState) -> Result<(), DeviceIOError> {
        if matches!(self.scenario, FakeRobotScenario::CommunicationFailure) {
            return Err(DeviceIOError::CommunicationFailed("Simulated bus error".into()));
        }
        self.current_target = Some(target.clone());
        self.timestamp = target.timestamp;
        Ok(())
    }
}

impl ObservationSource for FakeRobotAdapter {
    fn read_observation(&mut self) -> Result<RobotState, DeviceIOError> {
        if matches!(self.scenario, FakeRobotScenario::CommunicationFailure) {
            return Err(DeviceIOError::CommunicationFailed("Simulated bus error".into()));
        }

        let target = self
            .current_target
            .clone()
            .unwrap_or_else(|| RobotState::zero(self.dof));

        match self.scenario {
            FakeRobotScenario::Nominal => Ok(target),
            FakeRobotScenario::PositionDeviation(offset) => {
                let mut joints = target.joints.clone();
                for j in &mut joints {
                    if let Some(pos) = j.position {
                        j.position = Some(pos + offset);
                    }
                }
                Ok(RobotState::new(target.timestamp, joints))
            }
            FakeRobotScenario::VelocityDeviation(vel_offset) => {
                let mut joints = target.joints.clone();
                for j in &mut joints {
                    let v = j.velocity.unwrap_or(0.0);
                    j.velocity = Some(v + vel_offset);
                }
                Ok(RobotState::new(target.timestamp, joints))
            }
            FakeRobotScenario::StaleObservation(staleness) => {
                let stale_ts = (target.timestamp - staleness.as_secs_f64()).max(0.0);
                Ok(RobotState::new(stale_ts, target.joints))
            }
            FakeRobotScenario::MissingObservation => {
                let joints = vec![
                    JointState {
                        position: None,
                        velocity: None,
                        effort: None,
                    };
                    self.dof
                ];
                Ok(RobotState::new(target.timestamp, joints))
            }
            FakeRobotScenario::InvalidObservation => {
                let joints = vec![
                    JointState {
                        position: Some(f64::NAN),
                        velocity: None,
                        effort: None,
                    };
                    self.dof
                ];
                Ok(RobotState::new(target.timestamp, joints))
            }
            FakeRobotScenario::CommunicationFailure => Err(DeviceIOError::CommunicationFailed(
                "Simulated bus error".into(),
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::capability::{JointObservationRequirement, ObservationConstraint, ObservationRequirement};
    use crate::robot::observation::ObservationAssessment;
    use crate::robot::policy::{DeviationThresholds, ObservationResponsePolicy, PolicyDecision};
    use crate::robot::state::StateDeviation;

    #[test]
    fn fake_adapter_closed_loop_nominal_continue() {
        let mut adapter = FakeRobotAdapter::new(1).with_scenario(FakeRobotScenario::Nominal);

        // Given: Target command = 1.0 rad, tolerance = 0.05 rad
        let target = RobotState::from_positions(vec![1.0]);
        adapter.send_command(&target).unwrap();

        // When: Read observation
        let observed = adapter.read_observation().unwrap();

        // Then: Quality is Valid, deviation is 0.0, Decision is Continue
        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement {
                position: Some(ObservationConstraint::max_staleness(Duration::from_millis(100))),
                velocity: None,
                effort: None,
            },
        );
        let assessment = ObservationAssessment::evaluate(&req, &observed, Duration::from_millis(1));
        let dev = StateDeviation::compute(&target, &observed);
        let policy = ObservationResponsePolicy::new(DeviationThresholds::position(0.05));

        assert!(assessment.is_valid());
        assert_eq!(policy.evaluate(&assessment, dev.as_ref()), PolicyDecision::Continue);
    }

    #[test]
    fn fake_adapter_closed_loop_position_deviation_pause() {
        // Given: Fake robot with position deviation offset +0.125 rad
        let mut adapter = FakeRobotAdapter::new(1).with_scenario(FakeRobotScenario::PositionDeviation(0.125));

        let target = RobotState::from_positions(vec![1.0]);
        adapter.send_command(&target).unwrap();

        // When: Read observation (returns 1.125 rad)
        let observed = adapter.read_observation().unwrap();

        // Then: Assessment is Valid, deviation is 0.125 rad (> 0.05 tolerance), Decision is Pause
        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement {
                position: Some(ObservationConstraint::max_staleness(Duration::from_millis(100))),
                velocity: None,
                effort: None,
            },
        );
        let assessment = ObservationAssessment::evaluate(&req, &observed, Duration::from_millis(1));
        let dev = StateDeviation::compute(&target, &observed);
        let policy = ObservationResponsePolicy::new(DeviationThresholds::position(0.05));

        assert!(assessment.is_valid());
        assert_eq!(dev.as_ref().unwrap().max_position_error(), Some(0.125));
        assert_eq!(policy.evaluate(&assessment, dev.as_ref()), PolicyDecision::Pause);
    }

    #[test]
    fn fake_adapter_closed_loop_stale_observation_hold() {
        // Given: Stale observation scenario (50ms old) with max staleness constraint 10ms
        let mut adapter = FakeRobotAdapter::new(1).with_scenario(FakeRobotScenario::StaleObservation(Duration::from_millis(50)));

        let target = RobotState::from_positions(vec![1.0]);
        adapter.send_command(&target).unwrap();

        let observed = adapter.read_observation().unwrap();

        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement {
                position: Some(ObservationConstraint::max_staleness(Duration::from_millis(10))),
                velocity: None,
                effort: None,
            },
        );
        let assessment = ObservationAssessment::evaluate(&req, &observed, Duration::from_millis(50));
        let dev = StateDeviation::compute(&target, &observed);
        let policy = ObservationResponsePolicy::new(DeviationThresholds::position(0.05));

        assert!(assessment.has_stale());
        assert_eq!(policy.evaluate(&assessment, dev.as_ref()), PolicyDecision::Hold);
    }

    #[test]
    fn fake_adapter_closed_loop_invalid_observation_abort() {
        // Given: Hardware fault / NaN invalid observation
        let mut adapter = FakeRobotAdapter::new(1).with_scenario(FakeRobotScenario::InvalidObservation);

        let target = RobotState::from_positions(vec![1.0]);
        adapter.send_command(&target).unwrap();

        let observed = adapter.read_observation().unwrap();

        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement {
                position: Some(ObservationConstraint::max_staleness(Duration::from_millis(100))),
                velocity: None,
                effort: None,
            },
        );
        let assessment = ObservationAssessment::evaluate(&req, &observed, Duration::from_millis(1));
        let dev = StateDeviation::compute(&target, &observed);
        let policy = ObservationResponsePolicy::new(DeviationThresholds::position(0.05));

        assert!(assessment.has_invalid());
        assert_eq!(policy.evaluate(&assessment, dev.as_ref()), PolicyDecision::Abort);
    }
}
