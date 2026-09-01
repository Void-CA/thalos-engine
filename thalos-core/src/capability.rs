use serde::{Deserialize, Serialize};

/// CapabilityRequirement (ADR-014)
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

/// ResourceRequirement (ADR-014)
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
