use std::sync::Arc;

use thalos_engine::core::models::{RobotMetadata, RobotModel, RobotSpec};
use thalos_engine::core::robot::adapter;
use thalos_importer::import_urdf;

use crate::error::RuntimeError;
use crate::ports::{PersistenceError, RobotRecord, RobotRepository, RobotSource};

/// Application service for robot resource lifecycle.
///
/// Owns: create, import, list, get, delete.
/// Does NOT own: scene state, kinematics execution, planning.
pub struct RobotService {
    repo: Option<Arc<dyn RobotRepository>>,
}

impl Default for RobotService {
    fn default() -> Self {
        Self::new(None)
    }
}

impl RobotService {
    pub fn new(repo: Option<Arc<dyn RobotRepository>>) -> Self {
        Self { repo }
    }

    /// List canonical (engine-defined) robot metadata.
    pub fn list_canonical(&self) -> Vec<RobotMetadata> {
        RobotModel::all().iter().map(|m| m.metadata()).collect()
    }

    /// List all robots: canonical + persisted records from the repository.
    pub async fn list_all(&self) -> Vec<RobotRecord> {
        let mut records: Vec<RobotRecord> = RobotModel::all()
            .iter()
            .map(|m| {
                let meta = m.metadata();
                RobotRecord {
                    id: meta.id.to_string(),
                    name: meta.display_name.to_string(),
                    manufacturer: None,
                    model: None,
                    source_type: RobotSource::Canonical,
                    urdf_xml: None,
                    created_at: String::new(),
                    updated_at: String::new(),
                }
            })
            .collect();

        if let Some(ref repo) = self.repo {
            if let Ok(persisted) = repo.list().await {
                for rec in persisted {
                    if rec.source_type == RobotSource::ImportedUrdf
                        && !records.iter().any(|r| r.id == rec.id)
                    {
                        records.push(rec);
                    }
                }
            }
        }
        records
    }

    /// Get a single robot's metadata. Checks canonical first, then repository.
    pub fn get_canonical_metadata(&self, id: &str) -> Option<RobotMetadata> {
        RobotModel::from_id(id).ok().map(|m| m.metadata())
    }

    /// Get a robot record by ID from the repository.
    pub async fn get_record(&self, id: &str) -> Result<RobotRecord, RuntimeError> {
        let repo = self.repo.as_ref().ok_or_else(|| RuntimeError::Persistence {
            message: "No robot repository configured".to_string(),
        })?;

        repo.get(id)
            .await
            .map_err(|e| RuntimeError::Persistence {
                message: e.to_string(),
            })?
            .ok_or_else(|| RuntimeError::RobotNotFound { id: id.to_string() })
    }

    /// Get the default spec for a canonical robot.
    pub fn get_default_spec(&self, id: &str) -> Option<RobotSpec> {
        RobotModel::from_id(id).ok().map(|m| m.default_spec())
    }

    /// Import a URDF XML, validate it, persist the record, and return its metadata.
    ///
    /// Flow: URDF XML → parse → validate chain → persist → return record.
    pub async fn import_urdf(&self, urdf_xml: &str) -> Result<RobotRecord, RuntimeError> {
        // 1. Parse the URDF to validate structure
        let robot = import_urdf(urdf_xml).map_err(|e| RuntimeError::InvalidUrdf {
            message: format!("Invalid URDF: {e}"),
        })?;

        // 2. Validate that a kinematic chain can be built
        let _chain = adapter::auto(&robot).map_err(|e| RuntimeError::UrdfChainError {
            message: format!("Cannot build chain: {e}"),
        })?;

        // 3. Build a RobotRecord from the validated URDF
        let id = urdf_robot_id(urdf_xml);
        let now = chrono::Utc::now().to_rfc3339();
        let record = RobotRecord {
            id: id.clone(),
            name: robot.name.clone(),
            manufacturer: None,
            model: None,
            source_type: RobotSource::ImportedUrdf,
            urdf_xml: Some(urdf_xml.to_string()),
            created_at: now.clone(),
            updated_at: now,
        };

        // 4. Persist via repository
        let repo = self.repo.as_ref().ok_or_else(|| RuntimeError::Persistence {
            message: "No robot repository configured".to_string(),
        })?;

        repo.save(&record)
            .await
            .map_err(|e| RuntimeError::Persistence {
                message: e.to_string(),
            })?;

        tracing::info!(
            robot_id = %id,
            robot_name = %robot.name,
            "Imported URDF robot into persistence"
        );

        Ok(record)
    }

    /// Delete a robot record from persistence.
    pub async fn delete_robot(&self, id: &str) -> Result<(), RuntimeError> {
        let repo = self.repo.as_ref().ok_or_else(|| RuntimeError::Persistence {
            message: "No robot repository configured".to_string(),
        })?;

        repo.delete(id)
            .await
            .map_err(|e| RuntimeError::Persistence {
                message: e.to_string(),
            })
    }

    /// Load a robot (canonical engine model or imported persistence record) into `SceneService`.
    pub async fn load_robot_into_scene(
        &self,
        id: &str,
        scene: &crate::scene::service::SceneService,
    ) -> Result<crate::scene::snapshot::RuntimeSnapshot, RuntimeError> {
        if let Ok(model) = RobotModel::from_id(id) {
            scene.execute(crate::commands::Command::LoadRobot(model)).await
        } else {
            let record = self.get_record(id).await?;
            let urdf_xml = record.urdf_xml.ok_or_else(|| RuntimeError::InvalidUrdf {
                message: format!("Record '{id}' contains no URDF XML"),
            })?;
            scene.load_urdf_robot(&urdf_xml).await
        }
    }

    /// Check if persistence is available.
    pub fn has_persistence(&self) -> bool {
        self.repo.is_some()
    }
}

/// Generate a deterministic ID for a URDF robot from its source content.
fn urdf_robot_id(urdf_source: &str) -> String {
    use sha2::{Sha256, Digest};
    let hash = Sha256::digest(urdf_source.as_bytes());
    format!("urdf-{}", hex::encode(&hash[..8]))
}

impl From<PersistenceError> for RuntimeError {
    fn from(e: PersistenceError) -> Self {
        RuntimeError::Persistence {
            message: e.to_string(),
        }
    }
}
