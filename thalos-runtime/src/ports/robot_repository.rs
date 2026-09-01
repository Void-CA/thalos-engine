use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::str::FromStr;
use thiserror::Error;

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
}

impl std::fmt::Display for RobotSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RobotSource::Canonical => write!(f, "canonical"),
            RobotSource::ImportedUrdf => write!(f, "imported_urdf"),
        }
    }
}

impl FromStr for RobotSource {
    type Err = String;

    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        match s {
            "canonical" => Ok(RobotSource::Canonical),
            "imported_urdf" => Ok(RobotSource::ImportedUrdf),
            _ => Err(format!("Unknown robot source: {}", s)),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotRecord {
    pub id: String,
    pub name: String,
    pub manufacturer: Option<String>,
    pub model: Option<String>,
    pub source_type: RobotSource,
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
}
