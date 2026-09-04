use async_trait::async_trait;
use serde::{Deserialize, Serialize};

use crate::error::RuntimeError;
use crate::station::{AcquisitionModule, RoboticsModule, Station};

/// Persistent record of a station in the workspace database.
///
/// Stations are serialized as JSON blobs because their internal structure
/// (module maps) is complex and changes independently of the DB schema.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StationRecord {
    pub id: String,
    pub name: String,
    /// JSON-serialized robotics modules.
    pub robotics_modules_json: String,
    /// JSON-serialized acquisition modules.
    pub acquisition_modules_json: String,
}

impl StationRecord {
    /// Create a record from a Station domain object.
    pub fn from_station(station: &Station) -> Self {
        Self {
            id: station.id.0.clone(),
            name: station.name.clone(),
            robotics_modules_json: serde_json::to_string(&station.robotics_modules)
                .unwrap_or_else(|_| "{}".to_string()),
            acquisition_modules_json: serde_json::to_string(&station.acquisition_modules)
                .unwrap_or_else(|_| "{}".to_string()),
        }
    }

    /// Reconstruct the Station domain object from this record.
    pub fn to_station(&self) -> Result<Station, RuntimeError> {
        let robotics_modules = serde_json::from_str(&self.robotics_modules_json)
            .map_err(|e| RuntimeError::Persistence {
                message: format!("Failed to deserialize robotics modules: {e}"),
            })?;
        let acquisition_modules = serde_json::from_str(&self.acquisition_modules_json)
            .map_err(|e| RuntimeError::Persistence {
                message: format!("Failed to deserialize acquisition modules: {e}"),
            })?;

        Ok(Station {
            id: thalos_engine::prelude::StationId(self.id.clone()),
            name: self.name.clone(),
            robotics_modules,
            acquisition_modules,
        })
    }
}

#[async_trait]
pub trait StationRepository: Send + Sync {
    async fn list(&self) -> Result<Vec<StationRecord>, crate::ports::PersistenceError>;
    async fn get(&self, id: &str) -> Result<Option<StationRecord>, crate::ports::PersistenceError>;
    async fn save(&self, station: &StationRecord) -> Result<(), crate::ports::PersistenceError>;
    async fn delete(&self, id: &str) -> Result<(), crate::ports::PersistenceError>;

    /// Save all stations atomically (replaces entire collection).
    async fn save_all(&self, stations: &[StationRecord]) -> Result<(), crate::ports::PersistenceError>;
}
