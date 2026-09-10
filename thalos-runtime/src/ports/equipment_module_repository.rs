use async_trait::async_trait;

use crate::ports::PersistenceError;
use crate::ports::robot_repository::{RobotRecord, RobotAsset};
use crate::station::{
    Channel, EquipmentModule, EquipmentModuleId, InterconnectionModuleExtension,
    RoboticsModuleExtension,
};

/// Record types for persistence (separate from domain types).
#[derive(Debug, Clone)]
pub struct EquipmentModuleRecord {
    pub id: String,
    pub station_id: String,
    pub kind: String,
    pub name: String,
    pub created_at: String,
    pub updated_at: String,
}

#[derive(Debug, Clone)]
pub struct RoboticsModuleRecord {
    pub module_id: String,
    pub robot_id: String,
    pub configuration_json: String,
}

#[derive(Debug, Clone)]
pub struct InterconnectionModuleRecord {
    pub module_id: String,
}

#[derive(Debug, Clone)]
pub struct ChannelRecord {
    pub id: String,
    pub interconnection_module_id: String,
    pub symbol: String,
    pub name: String,
    pub unit: String,
}

/// Reference to a robot found within a station module.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RobotReference {
    pub station_id: String,
    pub module_id: EquipmentModuleId,
}

/// Repository for equipment modules, extensions, and channels.
///
/// Key design decisions:
/// - `create_robotics_module` and `create_interconnection_module` are ATOMIC:
///   they insert equipment_module + subtype in one transaction.
/// - There is NO generic `save(EquipmentModuleRecord)` for creation.
/// - `kind` is immutable after creation.
/// - Channel creation is a separate operation.
#[async_trait]
pub trait EquipmentModuleRepository: Send + Sync {
    // ─── Module queries ────────────────────────────────────────────────
    async fn list_by_station(&self, station_id: &str) -> Result<Vec<EquipmentModuleRecord>, PersistenceError>;
    async fn get(&self, id: &str) -> Result<Option<EquipmentModuleRecord>, PersistenceError>;

    // ─── Atomic creation ───────────────────────────────────────────────
    async fn create_robotics_module(
        &self,
        module: &EquipmentModuleRecord,
        extension: &RoboticsModuleRecord,
    ) -> Result<(), PersistenceError>;

    /// Create a robotics module with its robot definition in a single transaction.
    ///
    /// Combines:
    /// - robots INSERT
    /// - robot_assets DELETE + INSERT
    /// - equipment_modules INSERT
    /// - robotics_modules INSERT (with real robot UUID)
    async fn create_robotics_module_with_robot(
        &self,
        robot: &RobotRecord,
        assets: &[RobotAsset],
        station_id: &str,
        module_name: &str,
        configuration_json: &str,
    ) -> Result<String, PersistenceError>; // returns module_id

    async fn create_interconnection_module(
        &self,
        module: &EquipmentModuleRecord,
        extension: &InterconnectionModuleRecord,
    ) -> Result<(), PersistenceError>;

    // ─── Updates ───────────────────────────────────────────────────────
    async fn update_module(&self, module: &EquipmentModuleRecord) -> Result<(), PersistenceError>;

    // ─── Deletion (cascades via FK) ────────────────────────────────────
    async fn delete(&self, id: &str) -> Result<(), PersistenceError>;

    // ─── Type-specific queries ─────────────────────────────────────────
    async fn get_robotics_extension(&self, module_id: &str) -> Result<Option<RoboticsModuleRecord>, PersistenceError>;
    async fn get_interconnection_extension(&self, module_id: &str) -> Result<Option<InterconnectionModuleRecord>, PersistenceError>;

    // ─── Channel queries ───────────────────────────────────────────────
    async fn list_channels(&self, interconnection_module_id: &str) -> Result<Vec<ChannelRecord>, PersistenceError>;
    async fn save_channel(&self, channel: &ChannelRecord) -> Result<(), PersistenceError>;
    async fn delete_channel(&self, id: &str) -> Result<(), PersistenceError>;

    // ─── Integrity queries ─────────────────────────────────────────────
    async fn find_robot_references(&self, robot_id: &str) -> Result<Vec<RobotReference>, PersistenceError>;
}
