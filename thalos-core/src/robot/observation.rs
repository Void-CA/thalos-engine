use serde::{Deserialize, Serialize};
use std::time::Duration;

use crate::robot::capability::{ObservationConstraint, ObservationRequirement};
use crate::robot::state::RobotState;

/// Discrete observation quality classification for a single joint state variable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ObservationQuality {
    /// Observation is present, valid, and satisfies max staleness constraints.
    Valid,
    /// Observation is required by the operation contract, but missing (`None`) in state.
    Missing,
    /// Observation is present in state, but its sample age exceeds the required `max_staleness`.
    Stale,
    /// Observation is present, but explicitly flagged as untrustworthy (e.g. sensor fault, hardware CRC error).
    Invalid,
}

impl ObservationQuality {
    pub fn is_valid(&self) -> bool {
        matches!(self, ObservationQuality::Valid)
    }

    pub fn is_usable(&self) -> bool {
        matches!(self, ObservationQuality::Valid)
    }
}

/// Assessment of observation quality for a single joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct JointObservationAssessment {
    pub position: ObservationQuality,
    pub velocity: ObservationQuality,
    pub effort: ObservationQuality,
}

impl JointObservationAssessment {
    pub fn is_valid(&self) -> bool {
        self.position.is_valid() && self.velocity.is_valid() && self.effort.is_valid()
    }
}

/// Comprehensive runtime assessment of a `RobotState` against an `ObservationRequirement`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObservationAssessment {
    pub joints: Vec<JointObservationAssessment>,
    pub sample_age: Duration,
}

impl ObservationAssessment {
    /// Evaluate a `RobotState` against an `ObservationRequirement` given the current `sample_age`.
    ///
    /// Evaluates per-joint requirements:
    /// - If a component is required (`Some(constraint)`) and missing (`None`), quality is `Missing`.
    /// - If a component is required, present, and `constraint.max_staleness` is set:
    ///   - If `sample_age <= max_staleness`, quality is `Valid`.
    ///   - If `sample_age > max_staleness`, quality is `Stale`.
    /// - If a component is NOT required by the contract (`None`), quality is `Valid` (ignored).
    pub fn evaluate(
        req: &ObservationRequirement,
        state: &RobotState,
        sample_age: Duration,
    ) -> Self {
        let max_dof = req.joints.len().max(state.joints.len());
        let mut joint_assessments = Vec::with_capacity(max_dof);

        for i in 0..max_dof {
            let req_j = req.joints.get(i).copied().unwrap_or_default();
            let joint_state = state.joints.get(i);

            let pos_quality = evaluate_component(
                req_j.position,
                joint_state.and_then(|j| j.position),
                sample_age,
            );
            let vel_quality = evaluate_component(
                req_j.velocity,
                joint_state.and_then(|j| j.velocity),
                sample_age,
            );
            let eff_quality = evaluate_component(
                req_j.effort,
                joint_state.and_then(|j| j.effort),
                sample_age,
            );

            joint_assessments.push(JointObservationAssessment {
                position: pos_quality,
                velocity: vel_quality,
                effort: eff_quality,
            });
        }

        Self {
            joints: joint_assessments,
            sample_age,
        }
    }

    pub fn is_valid(&self) -> bool {
        self.joints.iter().all(|j| j.is_valid())
    }

    pub fn has_missing(&self) -> bool {
        self.joints.iter().any(|j| {
            matches!(j.position, ObservationQuality::Missing)
                || matches!(j.velocity, ObservationQuality::Missing)
                || matches!(j.effort, ObservationQuality::Missing)
        })
    }

    pub fn has_stale(&self) -> bool {
        self.joints.iter().any(|j| {
            matches!(j.position, ObservationQuality::Stale)
                || matches!(j.velocity, ObservationQuality::Stale)
                || matches!(j.effort, ObservationQuality::Stale)
        })
    }

    pub fn has_invalid(&self) -> bool {
        self.joints.iter().any(|j| {
            matches!(j.position, ObservationQuality::Invalid)
                || matches!(j.velocity, ObservationQuality::Invalid)
                || matches!(j.effort, ObservationQuality::Invalid)
        })
    }
}

fn evaluate_component(
    constraint_opt: Option<ObservationConstraint>,
    value_opt: Option<f64>,
    sample_age: Duration,
) -> ObservationQuality {
    match constraint_opt {
        None => ObservationQuality::Valid, // Not required by the contract
        Some(constraint) => match value_opt {
            None => ObservationQuality::Missing,
            Some(_) => {
                if let Some(max_stale) = constraint.max_staleness {
                    if sample_age <= max_stale {
                        ObservationQuality::Valid
                    } else {
                        ObservationQuality::Stale
                    }
                } else {
                    ObservationQuality::Valid
                }
            }
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::capability::{JointObservationRequirement, ObservationConstraint};
    use crate::robot::state::JointState;

    #[test]
    fn required_present_fresh_is_valid() {
        let req = ObservationRequirement::uniform(
            3,
            JointObservationRequirement {
                position: Some(ObservationConstraint::max_staleness(Duration::from_millis(10))),
                velocity: Some(ObservationConstraint::max_staleness(Duration::from_millis(5))),
                effort: None,
            },
        );

        let state = RobotState::new(
            0.0,
            vec![
                JointState {
                    position: Some(0.0),
                    velocity: Some(0.1),
                    effort: None,
                };
                3
            ],
        );

        let assessment = ObservationAssessment::evaluate(&req, &state, Duration::from_millis(2));
        assert!(assessment.is_valid());
        assert!(!assessment.has_missing());
        assert!(!assessment.has_stale());
    }

    #[test]
    fn required_absent_is_missing() {
        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement {
                position: Some(ObservationConstraint::unconstrained()),
                velocity: Some(ObservationConstraint::unconstrained()),
                effort: None,
            },
        );

        let state = RobotState::new(
            0.0,
            vec![JointState {
                position: Some(0.0),
                velocity: None, // Absent!
                effort: None,
            }],
        );

        let assessment = ObservationAssessment::evaluate(&req, &state, Duration::from_millis(1));
        assert!(!assessment.is_valid());
        assert!(assessment.has_missing());
        assert_eq!(assessment.joints[0].velocity, ObservationQuality::Missing);
        assert_eq!(assessment.joints[0].position, ObservationQuality::Valid);
    }

    #[test]
    fn required_present_stale_is_stale() {
        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement {
                position: Some(ObservationConstraint::max_staleness(Duration::from_millis(5))),
                velocity: Some(ObservationConstraint::max_staleness(Duration::from_millis(2))),
                effort: None,
            },
        );

        let state = RobotState::new(
            0.0,
            vec![JointState {
                position: Some(1.0),
                velocity: Some(0.5),
                effort: None,
            }],
        );

        // Sample age is 4ms -> Position (max 5ms) is Valid, Velocity (max 2ms) is Stale
        let assessment = ObservationAssessment::evaluate(&req, &state, Duration::from_millis(4));
        assert!(!assessment.is_valid());
        assert!(assessment.has_stale());
        assert!(!assessment.has_missing());
        assert_eq!(assessment.joints[0].position, ObservationQuality::Valid);
        assert_eq!(assessment.joints[0].velocity, ObservationQuality::Stale);
    }

    #[test]
    fn non_required_absent_is_ignored_as_valid() {
        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement {
                position: Some(ObservationConstraint::unconstrained()),
                velocity: None, // Not required!
                effort: None,   // Not required!
            },
        );

        let state = RobotState::new(
            0.0,
            vec![JointState {
                position: Some(1.0),
                velocity: None, // Absent but not required
                effort: None,   // Absent but not required
            }],
        );

        let assessment = ObservationAssessment::evaluate(&req, &state, Duration::from_millis(100));
        assert!(assessment.is_valid());
        assert_eq!(assessment.joints[0].position, ObservationQuality::Valid);
        assert_eq!(assessment.joints[0].velocity, ObservationQuality::Valid);
        assert_eq!(assessment.joints[0].effort, ObservationQuality::Valid);
    }

    #[test]
    fn exact_freshness_boundary_is_inclusive_valid() {
        let max_stale = Duration::from_millis(10);
        let req = ObservationRequirement::uniform(
            1,
            JointObservationRequirement {
                position: Some(ObservationConstraint::max_staleness(max_stale)),
                velocity: None,
                effort: None,
            },
        );

        let state = RobotState::new(
            0.0,
            vec![JointState {
                position: Some(0.0),
                velocity: None,
                effort: None,
            }],
        );

        // Sample age exactly equals max_staleness (10ms) -> Valid
        let assessment_boundary = ObservationAssessment::evaluate(&req, &state, max_stale);
        assert_eq!(assessment_boundary.joints[0].position, ObservationQuality::Valid);

        // Sample age 10ms + 1ns -> Stale
        let assessment_past = ObservationAssessment::evaluate(&req, &state, max_stale + Duration::from_nanos(1));
        assert_eq!(assessment_past.joints[0].position, ObservationQuality::Stale);
    }

    #[test]
    fn empty_requirements_produce_valid_assessment() {
        let req = ObservationRequirement::new(vec![]);
        let state = RobotState::new(0.0, vec![]);

        let assessment = ObservationAssessment::evaluate(&req, &state, Duration::from_secs(0));
        assert!(assessment.is_valid());
        assert!(assessment.joints.is_empty());
    }

    #[test]
    fn mixed_joints_per_joint_granularity() {
        let req = ObservationRequirement::new(vec![
            JointObservationRequirement::position_only(),
            JointObservationRequirement {
                position: Some(ObservationConstraint::max_staleness(Duration::from_millis(5))),
                velocity: Some(ObservationConstraint::max_staleness(Duration::from_millis(5))),
                effort: None,
            },
        ]);

        let state = RobotState::new(
            0.0,
            vec![
                JointState {
                    position: Some(0.5),
                    velocity: None,
                    effort: None,
                },
                JointState {
                    position: Some(1.0),
                    velocity: None, // Missing!
                    effort: None,
                },
            ],
        );

        let assessment = ObservationAssessment::evaluate(&req, &state, Duration::from_millis(2));
        assert!(!assessment.is_valid());
        assert_eq!(assessment.joints[0].position, ObservationQuality::Valid);
        assert_eq!(assessment.joints[1].position, ObservationQuality::Valid);
        assert_eq!(assessment.joints[1].velocity, ObservationQuality::Missing);
    }
}
