//! Capability vocabulary.
//!
//! Two planes, deliberately distinct:
//!
//! - **Plane A — semantic requirement** ([`CapabilityRequirement`]): what a
//!   station/module/program needs. Domain-level (`requires / provides`,.
//! - **Plane B — execution provider contract** ([`CapabilityDescriptor`]): what a
//!   concrete provider accepts and provides, and with what evidence. Mechanism-level.
//!
//! The domain says *what* is needed; the provider says *how* — and whether it can.

use serde::{Deserialize, Serialize};

use crate::robot::capability::{
    CapabilityMatch, ObservationDeficiency, ObservationRequirement, RobotCapability,
};

// ---------------------------------------------------------------------------
// Plane A — semantic capability requirements (domain)
// ---------------------------------------------------------------------------

/// CapabilityRequirement
/// Declares semantic capabilities required or provided by resources in the station.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "type")]
pub enum CapabilityRequirement {
    CartesianMotion,
    JointMotion,
    PayloadCapacity { min_grams: u32 },
    GripperControl,
    TemperatureSensor,
    VibrationSensor,
    Custom { name: String },
}

/// ResourceRequirement
/// Maps a capability requirement to an optional resolution status.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResourceRequirement {
    pub capability: CapabilityRequirement,
    pub is_mandatory: bool,
}

impl ResourceRequirement {
    pub fn mandatory(capability: CapabilityRequirement) -> Self {
        Self {
            capability,
            is_mandatory: true,
        }
    }

    pub fn optional(capability: CapabilityRequirement) -> Self {
        Self {
            capability,
            is_mandatory: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Plane B — execution provider capability contract (`docs/reference/firmware-boundary.md`)
//
// Pure vocabulary. No I/O, no transport, no registry: a provider advertises a
// descriptor, a request is matched against it before dispatch, and the provider's
// claims are bounded by the evidence level it declares.
// ---------------------------------------------------------------------------

/// What an execution provider accepts when it assumes responsibility for motion.
///
/// Responsibility follows capability (`docs/firmware-boundary.md` §3):
/// for `MoveJ`/`MoveL` the resource generates the trajectory; for
/// `TrajectoryBatch` Thalos plans and the resource reproduces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandCapability {
    /// Ordered, time-parameterized joint samples. Thalos plans; the resource reproduces.
    TrajectoryBatch,
    /// Joint target; the resource generates the trajectory. No IK implied.
    MoveJ,
    /// Cartesian pose target; the resource generates the trajectory.
    MoveL,
    /// Single joint objective; the resource interpolates and closes the loop.
    SetJointTarget,
}

/// Where Cartesian → joint resolution (IK) is performed, declared by the provider.
///
/// Only meaningful for `MoveL` (`docs/firmware-boundary.md` §3.1, §7.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TargetResolution {
    /// Thalos supplies a resolved joint path.
    Host,
    /// The resource resolves IK and declares its redundant-configuration semantics.
    Resource,
}

/// What a provider can truthfully attest to about an execution.
///
/// Ordered: a provider that `Provides` `E4` also satisfies `E0`–`E3`. Claims are
/// bounded by the declared maximum (`docs/firmware-boundary.md` §6.2). Only `E4`
/// supports *"the actuator achieved state X"*.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceLevel {
    /// Thalos emitted the command. Proves nothing physical.
    E0Commanded,
    /// The resource assumed responsibility. Not evidence of motion.
    E1Accepted,
    /// The resource's local executor advanced its command timeline.
    E2Reproduced,
    /// Bounded actuator commands were issued to the hardware.
    E3Written,
    /// Physical feedback confirms the actuator state.
    E4Measured,
}

/// Declarative capability descriptor advertised by an execution provider (Plane B).
///
/// Advertised at connection / session initialization. A change requires
/// re-negotiation, never silent drift (`docs/firmware-boundary.md` §7.6).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CapabilityDescriptor {
    pub contract_version: String,
    pub accepted_commands: Vec<CommandCapability>,
    pub provided_observations: RobotCapability,
    pub max_evidence_level: EvidenceLevel,
    pub target_resolution: TargetResolution,
    /// Joint order for every command and observation payload (SI units).
    pub joint_names: Vec<String>,
}

impl CapabilityDescriptor {
    /// Whether the provider accepts a given command capability.
    pub fn accepts(&self, capability: CommandCapability) -> bool {
        self.accepted_commands.contains(&capability)
    }

    /// Evaluate whether this provider satisfies a request.
    ///
    /// Match-before-dispatch: a program's requirements are matched against the
    /// declared descriptor *before* execution. No fallback is implicit
    /// (`docs/firmware-boundary.md` §8).
    pub fn satisfies(&self, request: &CapabilityRequest) -> ProviderMatch {
        if !self.accepts(request.command) {
            return ProviderMatch::MissingCommand(request.command);
        }
        if self.max_evidence_level < request.min_evidence_level {
            return ProviderMatch::InsufficientEvidence {
                required: request.min_evidence_level,
                declared: self.max_evidence_level,
            };
        }
        if let Some(obs_req) = &request.observations
            && let CapabilityMatch::Deficient(deficiencies) =
                self.provided_observations.matches(obs_req)
        {
            return ProviderMatch::ObservationDeficient(deficiencies);
        }
        ProviderMatch::Satisfied
    }
}

/// A capability request made against an execution provider (Plane B).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CapabilityRequest {
    pub command: CommandCapability,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub observations: Option<ObservationRequirement>,
    pub min_evidence_level: EvidenceLevel,
}

impl CapabilityRequest {
    /// A command request accepting the minimum dispatch evidence (`E1`).
    pub fn command(command: CommandCapability) -> Self {
        Self {
            command,
            observations: None,
            min_evidence_level: EvidenceLevel::E1Accepted,
        }
    }

    pub fn with_observations(mut self, observations: ObservationRequirement) -> Self {
        self.observations = Some(observations);
        self
    }

    pub fn requiring_evidence(mut self, level: EvidenceLevel) -> Self {
        self.min_evidence_level = level;
        self
    }
}

/// Result of matching a [`CapabilityRequest`] against a [`CapabilityDescriptor`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ProviderMatch {
    Satisfied,
    MissingCommand(CommandCapability),
    InsufficientEvidence {
        required: EvidenceLevel,
        declared: EvidenceLevel,
    },
    ObservationDeficient(Vec<ObservationDeficiency>),
}

impl ProviderMatch {
    pub fn is_satisfied(&self) -> bool {
        matches!(self, ProviderMatch::Satisfied)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::capability::{
        JointObservationCapability, JointObservationRequirement, RobotCapability,
    };

    fn descriptor() -> CapabilityDescriptor {
        CapabilityDescriptor {
            contract_version: "1.0".into(),
            accepted_commands: vec![CommandCapability::MoveJ, CommandCapability::MoveL],
            provided_observations: RobotCapability::uniform(
                6,
                JointObservationCapability::position_velocity(),
            ),
            max_evidence_level: EvidenceLevel::E2Reproduced,
            target_resolution: TargetResolution::Resource,
            joint_names: (1..=6).map(|i| format!("joint_{i}")).collect(),
        }
    }

    #[test]
    fn satisfies_matching_command() {
        let result = descriptor().satisfies(&CapabilityRequest::command(CommandCapability::MoveJ));
        assert!(result.is_satisfied());
    }

    #[test]
    fn rejects_unaccepted_command() {
        let result = descriptor().satisfies(&CapabilityRequest::command(CommandCapability::MoveL));
        assert!(result.is_satisfied());

        let result =
            descriptor().satisfies(&CapabilityRequest::command(CommandCapability::TrajectoryBatch));
        assert_eq!(
            result,
            ProviderMatch::MissingCommand(CommandCapability::TrajectoryBatch)
        );
    }

    #[test]
    fn rejects_insufficient_evidence() {
        let request =
            CapabilityRequest::command(CommandCapability::MoveJ).requiring_evidence(EvidenceLevel::E4Measured);
        assert_eq!(
            descriptor().satisfies(&request),
            ProviderMatch::InsufficientEvidence {
                required: EvidenceLevel::E4Measured,
                declared: EvidenceLevel::E2Reproduced,
            }
        );
    }

    #[test]
    fn rejects_observation_deficiency() {
        let obs = ObservationRequirement::uniform(6, JointObservationRequirement::full());
        let request = CapabilityRequest::command(CommandCapability::MoveJ).with_observations(obs);
        match descriptor().satisfies(&request) {
            ProviderMatch::ObservationDeficient(deficiencies) => {
                assert_eq!(deficiencies.len(), 6);
            }
            other => panic!("expected ObservationDeficient, got {other:?}"),
        }
    }

    #[test]
    fn evidence_level_is_ordered() {
        assert!(EvidenceLevel::E0Commanded < EvidenceLevel::E1Accepted);
        assert!(EvidenceLevel::E2Reproduced < EvidenceLevel::E3Written);
        assert!(EvidenceLevel::E3Written < EvidenceLevel::E4Measured);
    }

    #[test]
    fn serde_round_trip() {
        let d = descriptor();
        let json = serde_json::to_string(&d).expect("serialize");
        let back: CapabilityDescriptor = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(d, back);
    }
}
