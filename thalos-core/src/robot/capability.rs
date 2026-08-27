use serde::{Deserialize, Serialize};

/// Discrete components of joint state that can be requested or provided.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum JointStateComponent {
    Position,
    Velocity,
    Effort,
}

/// Declarative observation requirements for a single joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub struct JointObservationRequirement {
    pub position: bool,
    pub velocity: bool,
    pub effort: bool,
}

impl JointObservationRequirement {
    pub fn none() -> Self {
        Self {
            position: false,
            velocity: false,
            effort: false,
        }
    }

    pub fn position_only() -> Self {
        Self {
            position: true,
            velocity: false,
            effort: false,
        }
    }

    pub fn position_velocity() -> Self {
        Self {
            position: true,
            velocity: true,
            effort: false,
        }
    }

    pub fn full() -> Self {
        Self {
            position: true,
            velocity: true,
            effort: true,
        }
    }
}

/// Declarative observation requirements across all joints of a robot.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct ObservationRequirement {
    pub joints: Vec<JointObservationRequirement>,
}

impl ObservationRequirement {
    pub fn new(joints: Vec<JointObservationRequirement>) -> Self {
        Self { joints }
    }

    /// Construct a uniform requirement for a robot with `dof` joints.
    pub fn uniform(dof: usize, joint_req: JointObservationRequirement) -> Self {
        Self {
            joints: vec![joint_req; dof],
        }
    }

    pub fn dof(&self) -> usize {
        self.joints.len()
    }
}

/// Declarative observation capabilities for a single joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub struct JointObservationCapability {
    pub position: bool,
    pub velocity: bool,
    pub effort: bool,
}

impl JointObservationCapability {
    pub fn none() -> Self {
        Self {
            position: false,
            velocity: false,
            effort: false,
        }
    }

    pub fn position_only() -> Self {
        Self {
            position: true,
            velocity: false,
            effort: false,
        }
    }

    pub fn position_velocity() -> Self {
        Self {
            position: true,
            velocity: true,
            effort: false,
        }
    }

    pub fn full() -> Self {
        Self {
            position: true,
            velocity: true,
            effort: true,
        }
    }
}

/// Declarative observation capabilities across all joints of a robot.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct RobotCapability {
    pub joints: Vec<JointObservationCapability>,
}

impl RobotCapability {
    pub fn new(joints: Vec<JointObservationCapability>) -> Self {
        Self { joints }
    }

    /// Construct a uniform capability for a robot with `dof` joints.
    pub fn uniform(dof: usize, joint_cap: JointObservationCapability) -> Self {
        Self {
            joints: vec![joint_cap; dof],
        }
    }

    pub fn dof(&self) -> usize {
        self.joints.len()
    }

    /// Evaluate whether this `RobotCapability` satisfies a given `ObservationRequirement`.
    ///
    /// Per-joint set inclusion ($R_i \subseteq C_i$). Returns `CapabilityMatch::Satisfied` if all
    /// required components are supported for all joints. Otherwise returns `CapabilityMatch::Deficient`
    /// containing all missing joint state components.
    pub fn matches(&self, req: &ObservationRequirement) -> CapabilityMatch {
        let mut deficiencies = Vec::new();

        // If degrees of freedom mismatch, flag missing components for out-of-bounds joints
        let max_dof = self.joints.len().max(req.joints.len());

        for i in 0..max_dof {
            let cap = self.joints.get(i).copied().unwrap_or_default();
            let req_j = req.joints.get(i).copied().unwrap_or_default();

            if req_j.position && !cap.position {
                deficiencies.push(ObservationDeficiency {
                    joint: i,
                    missing: JointStateComponent::Position,
                });
            }
            if req_j.velocity && !cap.velocity {
                deficiencies.push(ObservationDeficiency {
                    joint: i,
                    missing: JointStateComponent::Velocity,
                });
            }
            if req_j.effort && !cap.effort {
                deficiencies.push(ObservationDeficiency {
                    joint: i,
                    missing: JointStateComponent::Effort,
                });
            }
        }

        if deficiencies.is_empty() {
            CapabilityMatch::Satisfied
        } else {
            CapabilityMatch::Deficient(deficiencies)
        }
    }
}

/// Specific missing joint state component identified during capability matching.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ObservationDeficiency {
    pub joint: usize,
    pub missing: JointStateComponent,
}

/// Result of evaluating an `ObservationRequirement` against a `RobotCapability`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CapabilityMatch {
    Satisfied,
    Deficient(Vec<ObservationDeficiency>),
}

impl CapabilityMatch {
    pub fn is_satisfied(&self) -> bool {
        matches!(self, CapabilityMatch::Satisfied)
    }

    pub fn deficiencies(&self) -> &[ObservationDeficiency] {
        match self {
            CapabilityMatch::Satisfied => &[],
            CapabilityMatch::Deficient(list) => list.as_slice(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uniform_capability_satisfies_matching_requirement() {
        let cap = RobotCapability::uniform(6, JointObservationCapability::position_velocity());
        let req = ObservationRequirement::uniform(6, JointObservationRequirement::position_velocity());

        let result = cap.matches(&req);
        assert!(result.is_satisfied());
        assert!(result.deficiencies().is_empty());
    }

    #[test]
    fn capability_satisfies_lesser_requirement() {
        let cap = RobotCapability::uniform(6, JointObservationCapability::full());
        let req = ObservationRequirement::uniform(6, JointObservationRequirement::position_only());

        let result = cap.matches(&req);
        assert!(result.is_satisfied());
    }

    #[test]
    fn deficiency_detected_when_velocity_missing() {
        let cap = RobotCapability::uniform(6, JointObservationCapability::position_only());
        let req = ObservationRequirement::uniform(6, JointObservationRequirement::position_velocity());

        let result = cap.matches(&req);
        assert!(!result.is_satisfied());
        assert_eq!(result.deficiencies().len(), 6);
        for (i, def) in result.deficiencies().iter().enumerate() {
            assert_eq!(def.joint, i);
            assert_eq!(def.missing, JointStateComponent::Velocity);
        }
    }

    #[test]
    fn per_joint_heterogeneous_requirements() {
        let cap = RobotCapability::new(vec![
            JointObservationCapability::full(),              // J0: P+V+E
            JointObservationCapability::position_velocity(), // J1: P+V
            JointObservationCapability::position_only(),     // J2: P
        ]);

        let req = ObservationRequirement::new(vec![
            JointObservationRequirement::position_velocity(), // J0: P+V -> OK
            JointObservationRequirement::position_velocity(), // J1: P+V -> OK
            JointObservationRequirement::position_velocity(), // J2: P+V -> Missing V!
        ]);

        let result = cap.matches(&req);
        assert!(!result.is_satisfied());
        assert_eq!(
            result.deficiencies(),
            &[ObservationDeficiency {
                joint: 2,
                missing: JointStateComponent::Velocity,
            }]
        );
    }
}
