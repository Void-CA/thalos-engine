use std::path::Path;
use std::sync::Arc;

use thalos_engine::core::models::{RobotMetadata, RobotModel, RobotSpec};
use thalos_engine::core::robot::adapter;
use thalos_importer::import_urdf;
use thalos_importer::import_urdf_resolved;
use thalos_importer::assets::resolver::Resolution;

use crate::error::RuntimeError;
use crate::ports::{PersistenceError, RobotRecord, RobotRepository, RobotSource};
use crate::robot::catalog::RobotCatalog;
use crate::robot::importer::{ImportError, RobotImporter};

/// Application service for robot resource lifecycle.
///
/// Owns: create, import, list, get, delete.
/// Does NOT own: scene state, kinematics execution, planning.
pub struct RobotService {
    repo: Option<Arc<dyn RobotRepository>>,
    /// Catálogo canónico de definiciones de robots — única autoridad de
    /// identidad técnica del robot (A2.4).
    catalog: RobotCatalog,
}

impl Default for RobotService {
    fn default() -> Self {
        Self::new(None)
    }
}

impl RobotService {
    pub fn new(repo: Option<Arc<dyn RobotRepository>>) -> Self {
        Self {
            repo,
            catalog: RobotCatalog::canonical(),
        }
    }

    /// Acceso al catálogo canónico de definiciones.
    pub fn catalog(&self) -> &RobotCatalog {
        &self.catalog
    }

    /// Lista las definiciones del catálogo (identidad + metadata declarativa).
    pub fn list_definitions(&self) -> Vec<crate::robot::RobotCatalogEntry> {
        self.catalog.definitions().to_vec()
    }

    /// Import a URDF with materialization: copies assets to workspace, persists record + assets.
    ///
    /// This is the primary import path for new robots. The URDF and all referenced
    /// meshes are materialized into the workspace directory, and both the robot record
    /// and asset metadata are persisted to SQLite.
    pub async fn import_urdf_materialized(
        &self,
        workspace_root: &Path,
        urdf_xml: &str,
        source_label: Option<&str>,
        extra_roots: &[std::path::PathBuf],
    ) -> Result<RobotRecord, RuntimeError> {
        let result = RobotImporter::import_urdf(workspace_root, urdf_xml, source_label, extra_roots)
            .map_err(|e| match e {
                ImportError::InvalidUrdf(msg) => RuntimeError::InvalidUrdf { message: msg },
                ImportError::ChainError(msg) => RuntimeError::UrdfChainError { message: msg },
                ImportError::MissingAssets(missing) => RuntimeError::InvalidUrdf {
                    message: format!("Missing assets: {:?}", missing),
                },
                other => RuntimeError::InvalidUrdf {
                    message: other.to_string(),
                },
            })?;

        // Persist record + assets to SQLite
        let repo = self.repo.as_ref().ok_or_else(|| RuntimeError::Persistence {
            message: "No robot repository configured".to_string(),
        })?;
        repo.save(&result.record).await.map_err(|e| RuntimeError::Persistence {
            message: e.to_string(),
        })?;
        repo.save_assets(&result.robot_id, &result.assets).await.map_err(|e| RuntimeError::Persistence {
            message: e.to_string(),
        })?;

        Ok(result.record)
    }

    /// Import a robot package with materialization.
    pub async fn import_package_materialized(
        &self,
        workspace_root: &Path,
        package_dir: &Path,
    ) -> Result<RobotRecord, RuntimeError> {
        let result = RobotImporter::import_package(workspace_root, package_dir)
            .map_err(|e| match e {
                ImportError::InvalidUrdf(msg) => RuntimeError::InvalidUrdf { message: msg },
                ImportError::ChainError(msg) => RuntimeError::UrdfChainError { message: msg },
                ImportError::MissingAssets(missing) => RuntimeError::InvalidUrdf {
                    message: format!("Missing assets: {:?}", missing),
                },
                other => RuntimeError::InvalidUrdf {
                    message: other.to_string(),
                },
            })?;

        let repo = self.repo.as_ref().ok_or_else(|| RuntimeError::Persistence {
            message: "No robot repository configured".to_string(),
        })?;
        repo.save(&result.record).await.map_err(|e| RuntimeError::Persistence {
            message: e.to_string(),
        })?;
        repo.save_assets(&result.robot_id, &result.assets).await.map_err(|e| RuntimeError::Persistence {
            message: e.to_string(),
        })?;

        Ok(result.record)
    }

    /// Load a materialized robot from workspace filesystem.
    ///
    /// Reads the URDF and assets from the workspace directory, builds the
    /// kinematic chain, and loads into SceneService.
    pub async fn load_materialized_robot(
        &self,
        id: &str,
        workspace_root: &Path,
        scene: &crate::scene::service::SceneService,
    ) -> Result<crate::scene::snapshot::RuntimeSnapshot, RuntimeError> {
        let record = self.get_record(id).await?;

        // Try filesystem first (materialized robot)
        let urdf_path = workspace_root.join("robots").join(id).join("robot.urdf");
        if urdf_path.exists() {
            let urdf_xml = std::fs::read_to_string(&urdf_path)
                .map_err(|e| RuntimeError::InvalidUrdf {
                    message: format!("Cannot read URDF: {e}"),
                })?;

            // Load assets from SQLite and build resolution
            let repo = self.repo.as_ref().ok_or_else(|| RuntimeError::Persistence {
                message: "No robot repository configured".to_string(),
            })?;
            let assets = repo.get_assets(id).await.unwrap_or_default();

            let resolution = if assets.is_empty() {
                Resolution::default()
            } else {
                build_resolution_from_assets(&assets, workspace_root)
            };

            let result = import_urdf_resolved(&urdf_xml, &resolution)
                .map_err(|e| RuntimeError::InvalidUrdf {
                    message: format!("Invalid URDF: {e}"),
                })?;

            let chain = adapter::auto(&result.robot).map_err(|e| RuntimeError::UrdfChainError {
                message: format!("Cannot build chain: {e}"),
            })?;

            return scene
                .load_urdf_robot_command(
                    result.robot,
                    chain,
                    record.name.clone(),
                    id,
                )
                .await;
        }

        // Fallback: legacy robot with urdf_xml in SQLite
        #[allow(deprecated)]
        let urdf_xml = record.urdf_xml.ok_or_else(|| RuntimeError::InvalidUrdf {
            message: format!("Robot '{id}' has no URDF on filesystem and no legacy URDF in database"),
        })?;
        scene.load_urdf_robot(&urdf_xml).await
    }

    /// Load a catalog definition into the scene, materializing it to the workspace.
    pub async fn load_definition_into_scene(
        &self,
        definition_id: &str,
        workspace_root: &Path,
        scene: &crate::scene::service::SceneService,
    ) -> Result<crate::scene::snapshot::RuntimeSnapshot, RuntimeError> {
        // Check if already materialized in workspace
        let repo = self.repo.as_ref().ok_or_else(|| RuntimeError::Persistence {
            message: "No robot repository configured".to_string(),
        })?;

        // Look for existing record with this definition_id
        if let Ok(Some(_record)) = repo.get(definition_id).await {
            // Already materialized — load from filesystem
            return self.load_materialized_robot(definition_id, workspace_root, scene).await;
        }

        // Not yet materialized — materialize from catalog
        let resolution = self
            .catalog
            .load_catalog_entry(definition_id)
            .map_err(|e| RuntimeError::RobotDefinition(e))?;

        let result = RobotImporter::import_package(
            workspace_root,
            &resolution.0.definition.asset_root,
        )
        .map_err(|e| RuntimeError::InvalidUrdf {
            message: format!("Failed to materialize catalog robot: {e}"),
        })?;

        // Persist
        repo.save(&result.record).await.map_err(|e| RuntimeError::Persistence {
            message: e.to_string(),
        })?;
        repo.save_assets(&result.robot_id, &result.assets).await.map_err(|e| RuntimeError::Persistence {
            message: e.to_string(),
        })?;

        scene
            .load_urdf_robot_command(
                result.robot,
                result.chain,
                result.record.name.clone(),
                &result.robot_id,
            )
            .await
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
                    source_label: None,
                    urdf_xml: None,
                    created_at: String::new(),
                    updated_at: String::new(),
                }
            })
            .collect();

        if let Some(ref repo) = self.repo {
            if let Ok(persisted) = repo.list().await {
                for rec in persisted {
                    if (rec.source_type == RobotSource::ImportedUrdf
                        || rec.source_type == RobotSource::ImportedPackage)
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

    /// LEGACY: Import a URDF XML without materialization.
    ///
    /// Prefer `import_urdf_materialized` for new imports. This method is retained
    /// for backward compatibility with code that doesn't have a workspace root.
    #[deprecated(note = "Use import_urdf_materialized for new imports")]
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
        #[allow(deprecated)]
        let record = RobotRecord {
            id: id.clone(),
            name: robot.name.clone(),
            manufacturer: None,
            model: None,
            source_type: RobotSource::ImportedUrdf,
            source_label: None,
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
            "Imported URDF robot into persistence (legacy)"
        );

        Ok(record)
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
            #[allow(deprecated)]
            let urdf_xml = record.urdf_xml.ok_or_else(|| RuntimeError::InvalidUrdf {
                message: format!("Record '{id}' contains no URDF XML"),
            })?;
            scene.load_urdf_robot(&urdf_xml).await
        }
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

    /// Check if persistence is available.
    pub fn has_persistence(&self) -> bool {
        self.repo.is_some()
    }
}

/// Build a Resolution from persisted RobotAsset entries.
fn build_resolution_from_assets(
    assets: &[thalos_models::robot_asset::RobotAsset],
    workspace_root: &Path,
) -> Resolution {
    let mut resolved = std::collections::HashMap::new();
    for asset in assets {
        let absolute = workspace_root.join(&asset.stored_path);
        resolved.insert(asset.original_uri.clone(), absolute);
    }
    Resolution {
        resolved,
        missing: vec![],
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
