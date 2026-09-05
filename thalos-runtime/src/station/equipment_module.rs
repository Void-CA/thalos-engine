use serde::{Deserialize, Serialize};
use thalos_engine::prelude::StationId;

/// Strongly typed identity for an equipment module.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EquipmentModuleId(pub String);

impl std::fmt::Display for EquipmentModuleId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// The type of an equipment module. Immutable after creation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EquipmentModuleKind {
    Robotics,
    Acquisition,
}

impl std::fmt::Display for EquipmentModuleKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Robotics => write!(f, "robotics"),
            Self::Acquisition => write!(f, "acquisition"),
        }
    }
}

impl std::str::FromStr for EquipmentModuleKind {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "robotics" => Ok(Self::Robotics),
            "acquisition" => Ok(Self::Acquisition),
            _ => Err(format!("Invalid equipment module kind: {s}")),
        }
    }
}

/// An equipment module belonging to a Station.
///
/// The `kind` field is immutable after creation. The module's type-specific
/// data lives in the corresponding extension struct (RoboticsModuleExtension
/// or AcquisitionModuleExtension), which is stored in a separate table.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EquipmentModule {
    pub id: EquipmentModuleId,
    pub station_id: StationId,
    pub name: String,
    pub kind: EquipmentModuleKind,
}

/// Type-specific data for a Robotics module.
///
/// The `robot_id` is a foreign key to `robots.id` with ON DELETE RESTRICT.
/// A RoboticsModule requires exactly one Robot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RoboticsModuleExtension {
    pub module_id: EquipmentModuleId,
    pub robot_id: String,
    pub configuration_json: String,
}

/// Type-specific data for an Acquisition module.
///
/// Channels are stored in a separate table with FK to this module.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AcquisitionModuleExtension {
    pub module_id: EquipmentModuleId,
    pub configuration_json: String,
}

/// A channel definition within an Acquisition module.
///
/// Channels represent the semantic definition of a data stream
/// (e.g. "temperature" in °C). Observations are a separate concern.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Channel {
    pub id: String,
    pub acquisition_module_id: EquipmentModuleId,
    pub symbol: String,
    pub name: String,
    pub data_type: String,
    pub unit: String,
}

impl Channel {
    /// Validate that channel fields are semantically valid.
    pub fn validate(&self) -> Result<(), String> {
        if self.symbol.trim().is_empty() {
            return Err("Channel symbol must not be empty".into());
        }
        if self.name.trim().is_empty() {
            return Err("Channel name must not be empty".into());
        }
        if self.data_type.trim().is_empty() {
            return Err("Channel data_type must not be empty".into());
        }
        Ok(())
    }
}
