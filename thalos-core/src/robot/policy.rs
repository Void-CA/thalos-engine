use serde::{Deserialize, Serialize};

use crate::robot::observation::ObservationAssessment;
use crate::robot::state::StateDeviation;

/// Operational decisions rendered by an execution response policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PolicyDecision {
    /// Observation is valid and state deviation is within tolerance; proceed with execution.
    Continue,
    /// Temporary hold / maintain current position setpoint (e.g. stale observation or minor velocity deviation).
    Hold,
    /// Controlled execution pause / decelerate to zero (e.g. missing required observation or position deviation exceeded).
    Pause,
    /// Critical safety stop / immediate execution abort (e.g. invalid sensor data, hardware fault).
    Abort,
}

impl PolicyDecision {
    pub fn is_continue(&self) -> bool {
        matches!(self, PolicyDecision::Continue)
    }

    pub fn requires_stopping(&self) -> bool {
        matches!(self, PolicyDecision::Pause | PolicyDecision::Abort)
    }
}

/// Acceptable mathematical deviation thresholds between expected and observed state.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct DeviationThresholds {
    /// Maximum allowed joint position error $|q_{\text{obs}} - q_{\text{exp}}|$ in radians/meters.
    pub max_position_error: Option<f64>,
    /// Maximum allowed joint velocity error $|\dot{q}_{\text{obs}} - \dot{q}_{\text{exp}}|$ in rad/s or m/s.
    pub max_velocity_error: Option<f64>,
    /// Maximum allowed joint effort error $|\tau_{\text{obs}} - \tau_{\text{exp}}|$ in Nm or N.
    pub max_effort_error: Option<f64>,
}

impl DeviationThresholds {
    pub fn unconstrained() -> Self {
        Self::default()
    }

    pub fn position(max_error: f64) -> Self {
        Self {
            max_position_error: Some(max_error),
            ..Default::default()
        }
    }

    pub fn position_and_velocity(max_pos_error: f64, max_vel_error: f64) -> Self {
        Self {
            max_position_error: Some(max_pos_error),
            max_velocity_error: Some(max_vel_error),
            ..Default::default()
        }
    }
}

/// Policy engine that translates observation assessments and mathematical state deviations
/// into operational runtime decisions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct ObservationResponsePolicy {
    /// Acceptable mathematical state deviation thresholds.
    pub deviation_thresholds: DeviationThresholds,
}

impl ObservationResponsePolicy {
    pub fn new(deviation_thresholds: DeviationThresholds) -> Self {
        Self {
            deviation_thresholds,
        }
    }

    pub fn unconstrained() -> Self {
        Self {
            deviation_thresholds: DeviationThresholds::unconstrained(),
        }
    }

    /// Evaluate an `ObservationAssessment` and optional `StateDeviation` to produce a `PolicyDecision`.
    ///
    /// Decision precedence rules:
    /// 1. `Invalid` observation (sensor fault, CRC error) -> `PolicyDecision::Abort`
    /// 2. `Missing` observation (required variable absent) -> `PolicyDecision::Pause`
    /// 3. `Stale` observation (expired timestamp) -> `PolicyDecision::Hold`
    /// 4. `Valid` observation:
    ///    - Evaluate `StateDeviation` against `deviation_thresholds`.
    ///    - Exceeded position error threshold -> `PolicyDecision::Pause`
    ///    - Exceeded velocity error threshold -> `PolicyDecision::Hold`
    ///    - Within thresholds or no deviation supplied -> `PolicyDecision::Continue`
    pub fn evaluate(
        &self,
        assessment: &ObservationAssessment,
        deviation: Option<&StateDeviation>,
    ) -> PolicyDecision {
        if assessment.has_invalid() {
            return PolicyDecision::Abort;
        }

        if assessment.has_missing() {
            return PolicyDecision::Pause;
        }

        if assessment.has_stale() {
            return PolicyDecision::Hold;
        }

        // Observation is Valid. Now evaluate deviation against thresholds (Valid != Correct)
        if let Some(dev) = deviation {
            if let Some(max_pos_err) = self.deviation_thresholds.max_position_error {
                if let Some(actual_pos_err) = dev.max_position_error() {
                    if actual_pos_err > max_pos_err {
                        return PolicyDecision::Pause;
                    }
                }
            }

            if let Some(max_vel_err) = self.deviation_thresholds.max_velocity_error {
                if let Some(actual_vel_err) = dev.max_velocity_error() {
                    if actual_vel_err > max_vel_err {
                        return PolicyDecision::Hold;
                    }
                }
            }

            if let Some(max_eff_err) = self.deviation_thresholds.max_effort_error {
                if let Some(actual_eff_err) = dev.max_effort_error() {
                    if actual_eff_err > max_eff_err {
                        return PolicyDecision::Hold;
                    }
                }
            }
        }

        PolicyDecision::Continue
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use crate::robot::capability::{JointObservationRequirement, ObservationConstraint, ObservationRequirement};
    use crate::robot::observation::ObservationAssessment;
    use crate::robot::state::{JointState, RobotState};

    #[test]
    fn valid_observation_with_small_deviation_continues() {
        let req = ObservationRequirement::uniform(
            2,
            JointObservationRequirement::position_only(),
        );

        let exp = RobotState::from_positions(vec![1.0, 2.0]);
        let obs = RobotState::from_positions(vec![1.01, 2.01]);
        let dev = StateDeviation::compute(&exp, &obs);

        let assessment = ObservationAssessment::evaluate(&req, &obs, Duration::from_millis(1));
        let policy = ObservationResponsePolicy::new(DeviationThresholds::position(0.05));

        let decision = policy.evaluate(&assessment, dev.as_ref());
        assert_eq!(decision, PolicyDecision::Continue);
    }

    #[test]
    fn valid_observation_with_large_position_deviation_pauses() {
        let req = ObservationRequirement::uniform(
            2,
            JointObservationRequirement::position_only(),
        );

        let exp = RobotState::from_positions(vec![1.0, 2.0]);
        let obs = RobotState::from_positions(vec![1.20, 2.01]); // 0.20 rad error on Joint 0
        let dev = StateDeviation::compute(&exp, &obs);

        let assessment = ObservationAssessment::evaluate(&req, &obs, Duration::from_millis(1));
        let policy = ObservationResponsePolicy::new(DeviationThresholds::position(0.05)); // Max allowed 0.05 rad

        let decision = policy.evaluate(&assessment, dev.as_ref());
        assert_eq!(decision, PolicyDecision::Pause);
    }

    #[test]
    fn stale_observation_holds_even_if_deviation_is_zero() {
        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement {
                position: Some(ObservationConstraint::max_staleness(Duration::from_millis(5))),
                velocity: None,
                effort: None,
            },
        );

        let exp = RobotState::from_positions(vec![1.0]);
        let obs = RobotState::from_positions(vec![1.0]);
        let dev = StateDeviation::compute(&exp, &obs);

        // Sample age is 10ms -> Stale!
        let assessment = ObservationAssessment::evaluate(&req, &obs, Duration::from_millis(10));
        let policy = ObservationResponsePolicy::unconstrained();

        let decision = policy.evaluate(&assessment, dev.as_ref());
        assert_eq!(decision, PolicyDecision::Hold);
    }

    #[test]
    fn missing_observation_pauses() {
        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement::position_only(),
        );

        let obs = RobotState::new(0.0, vec![JointState::empty()]);
        let assessment = ObservationAssessment::evaluate(&req, &obs, Duration::from_millis(1));
        let policy = ObservationResponsePolicy::unconstrained();

        let decision = policy.evaluate(&assessment, None);
        assert_eq!(decision, PolicyDecision::Pause);
    }

    #[test]
    fn invalid_observation_aborts() {
        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement::position_only(),
        );

        let obs = RobotState::from_positions(vec![1.0]);
        let mut assessment = ObservationAssessment::evaluate(&req, &obs, Duration::from_millis(1));
        // Flag joint 0 position as Invalid (e.g. sensor hardware fault)
        assessment.joints[0].position = crate::robot::observation::ObservationQuality::Invalid;

        let policy = ObservationResponsePolicy::unconstrained();
        let decision = policy.evaluate(&assessment, None);
        assert_eq!(decision, PolicyDecision::Abort);
    }
}
