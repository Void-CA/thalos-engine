use async_trait::async_trait;
use serde::{Deserialize, Serialize};

/// Persistent record of a station in the workspace database.
///
/// Contains only identity. Modules and channels are stored in separate
/// relational tables via EquipmentModuleRepository.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StationRecord {
    pub id: String,
    pub name: String,
    pub created_at: String,
    pub updated_at: String,
}

#[async_trait]
pub trait StationRepository: Send + Sync {
    async fn list(&self) -> Result<Vec<StationRecord>, crate::ports::PersistenceError>;
    async fn get(&self, id: &str) -> Result<Option<StationRecord>, crate::ports::PersistenceError>;
    async fn save(&self, station: &StationRecord) -> Result<(), crate::ports::PersistenceError>;
    async fn delete(&self, id: &str) -> Result<(), crate::ports::PersistenceError>;
}
