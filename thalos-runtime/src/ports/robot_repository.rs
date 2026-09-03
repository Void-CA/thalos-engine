use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::str::FromStr;
use thiserror::Error;

use thalos_models::robot_asset::RobotAsset;

#[derive(Error, Debug)]
pub enum PersistenceError {
    #[error("Database error: {0}")]
    Database(String),

    #[error("Record not found: {0}")]
    NotFound(String),

    #[error("Serialization error: {0}")]
    Serialization(String),
}

pub type Result<T> = std::result::Result<T, PersistenceError>;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum RobotSource {
    Canonical,
    ImportedUrdf,
    ImportedPackage,
}

impl std::fmt::Display for RobotSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RobotSource::Canonical => write!(f, "canonical"),
            RobotSource::ImportedUrdf => write!(f, "imported_urdf"),
            RobotSource::ImportedPackage => write!(f, "imported_package"),
        }
    }
}

impl FromStr for RobotSource {
    type Err = String;

    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s {
            "canonical" => Ok(RobotSource::Canonical),
            "imported_urdf" => Ok(RobotSource::ImportedUrdf),
            "imported_package" => Ok(RobotSource::ImportedPackage),
            _ => Err(format!("Unknown robot source: {}", s)),
        }
    }
}

/// Persistent record of a robot in the workspace database.
///
/// The robot's artifacts (URDF, meshes) live in the filesystem under
/// `robots/<id>/`. This record holds identity and metadata only.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotRecord {
    pub id: String,
    pub name: String,
    pub manufacturer: Option<String>,
    pub model: Option<String>,
    pub source_type: RobotSource,
    /// Human-readable label for the import source (e.g. "abb_irb140_support" or "/home/user/robot.urdf").
    pub source_label: Option<String>,
    /// LEGACY: URDF XML stored inline. Retained for backward compatibility
    /// with robots imported before the materialization system. Will be
    /// removed after all existing robots are migrated.
    #[deprecated(note = "Legacy field — robots should use filesystem URDF + assets")]
    pub urdf_xml: Option<String>,
    pub created_at: String,
    pub updated_at: String,
}

#[async_trait]
pub trait RobotRepository: Send + Sync {
    async fn list(&self) -> Result<Vec<RobotRecord>>;
    async fn get(&self, id: &str) -> Result<Option<RobotRecord>>;
    async fn save(&self, robot: &RobotRecord) -> Result<()>;
    async fn delete(&self, id: &str) -> Result<()>;

    /// Retrieve all persisted assets for a robot.
    async fn get_assets(&self, robot_id: &str) -> Result<Vec<RobotAsset>>;

    /// Persist assets for a robot (replaces any existing assets for that robot).
    async fn save_assets(&self, robot_id: &str, assets: &[RobotAsset]) -> Result<()>;
}
