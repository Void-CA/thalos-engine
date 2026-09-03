use std::path::{Path, PathBuf};
use thiserror::Error;

use sha2::{Digest, Sha256};
use thalos_importer::assets::discovery::UrdfAssetDiscovery;
use thalos_importer::assets::{AssetDiscovery, AssetRole};
use thalos_importer::assets::resolver::{Resolution, UriResolver};
use thalos_importer::import_urdf_resolved;
use thalos_models::robot::Robot;
use thalos_models::robot_asset::RobotAsset;
use thalos_engine::core::models::RobotSpec;
use thalos_engine::core::robot::adapter;
use thalos_engine::core::robot::serial_chain::SerialChain;

use crate::ports::robot_repository::{RobotRecord, RobotSource};

/// Errors that can occur during robot import.
#[derive(Debug, Error)]
pub enum ImportError {
    #[error("Invalid URDF: {0}")]
    InvalidUrdf(String),

    #[error("Cannot build kinematic chain: {0}")]
    ChainError(String),

    #[error("Missing assets: {0:?}")]
    MissingAssets(Vec<String>),

    #[error("Filesystem error: {0}")]
   Fs(#[from] std::io::Error),

    #[error("Workspace root does not exist: {0}")]
    WorkspaceNotFound(PathBuf),

    #[error("Asset hash mismatch for {path}: expected {expected}, got {actual}")]
    HashMismatch {
        path: String,
        expected: String,
        actual: String,
    },
}

/// Result of a successful robot import.
pub struct RobotImportResult {
    pub robot_id: String,
    pub record: RobotRecord,
    pub assets: Vec<RobotAsset>,
    pub robot: Robot,
    pub chain: SerialChain,
}

/// Imports a robot from external files into the Thalos workspace.
///
/// The importer performs a transactional copy: staging directory → validate → commit.
/// If anything fails, the staging directory is cleaned up and the workspace is left intact.
pub struct RobotImporter;

impl RobotImporter {
    /// Import a robot from URDF XML with resolved asset roots.
    ///
    /// - `workspace_root`: The workspace directory (contains `robots/` and `workspace.db`).
    /// - `urdf_xml`: The URDF XML content.
    /// - `source_label`: Optional human-readable label (e.g. "abb_irb140_support").
    /// - `extra_roots`: Filesystem roots to resolve mesh URIs against.
    pub fn import_urdf(
        workspace_root: &Path,
        urdf_xml: &str,
        source_label: Option<&str>,
        extra_roots: &[PathBuf],
    ) -> Result<RobotImportResult, ImportError> {
        if !workspace_root.exists() {
            return Err(ImportError::WorkspaceNotFound(workspace_root.to_path_buf()));
        }

        // 1. Parse URDF to discover asset references
        let discovery = UrdfAssetDiscovery::new();
        let references = discovery
            .discover(urdf_xml)
            .map_err(|e| ImportError::InvalidUrdf(e.to_string()))?;

        // 2. Resolve assets against provided roots
        let mut resolver = UriResolver::new();
        for root in extra_roots {
            if root.exists() {
                resolver = resolver.with_base_dir(root);
            }
        }
        let resolution = resolver.resolve(&references);

        // 3. Validate — all assets must be resolved
        if !resolution.missing.is_empty() {
            let missing: Vec<String> = resolution.missing.iter().map(|r| r.uri.clone()).collect();
            return Err(ImportError::MissingAssets(missing));
        }

        // 4. Generate robot_id (UUID-based, per design decision #3)
        let robot_id = format!("robot-{}", uuid::Uuid::new_v4());

        // 5. Perform the materialization
        Self::materialize(
            workspace_root,
            &robot_id,
            urdf_xml,
            source_label,
            &resolution,
            &references,
            RobotSource::ImportedUrdf,
        )
    }

    /// Import a robot from a package directory (auto-discovers URDF + assets).
    ///
    /// - `workspace_root`: The workspace directory.
    /// - `package_dir`: The robot package directory (e.g. `abb_irb140_support/`).
    pub fn import_package(
        workspace_root: &Path,
        package_dir: &Path,
    ) -> Result<RobotImportResult, ImportError> {
        if !workspace_root.exists() {
            return Err(ImportError::WorkspaceNotFound(workspace_root.to_path_buf()));
        }
        if !package_dir.exists() {
            return Err(ImportError::WorkspaceNotFound(package_dir.to_path_buf()));
        }

        // 1. Auto-discover URDF in the package
        let urdf_xml = Self::discover_urdf_in_package(package_dir)
            .ok_or_else(|| ImportError::InvalidUrdf(
                format!("No URDF file found in {}", package_dir.display())
            ))?;

        // 2. Discover asset references
        let discovery = UrdfAssetDiscovery::new();
        let references = discovery
            .discover(&urdf_xml)
            .map_err(|e| ImportError::InvalidUrdf(e.to_string()))?;

        // 3. Resolve against the package directory
        let resolver = UriResolver::new().with_base_dir(package_dir);
        let resolution = resolver.resolve(&references);

        // 4. Validate
        if !resolution.missing.is_empty() {
            let missing: Vec<String> = resolution.missing.iter().map(|r| r.uri.clone()).collect();
            return Err(ImportError::MissingAssets(missing));
        }

        // 5. Generate robot_id
        let robot_id = format!("robot-{}", uuid::Uuid::new_v4());

        // 6. Materialize
        let source_label = package_dir.file_name()
            .and_then(|n| n.to_str())
            .map(|s| s.to_string());

        Self::materialize(
            workspace_root,
            &robot_id,
            &urdf_xml,
            source_label.as_deref(),
            &resolution,
            &references,
            RobotSource::ImportedPackage,
        )
    }

    /// Core materialization logic: staging → validate → commit.
    fn materialize(
        workspace_root: &Path,
        robot_id: &str,
        urdf_xml: &str,
        source_label: Option<&str>,
        resolution: &Resolution,
        references: &[thalos_importer::assets::AssetReference],
        source_type: RobotSource,
    ) -> Result<RobotImportResult, ImportError> {
        let robots_dir = workspace_root.join("robots");
        let robot_dir = robots_dir.join(robot_id);
        let staging_dir = workspace_root.join(".staging").join(robot_id);

        // Clean up any leftover staging directory
        if staging_dir.exists() {
            std::fs::remove_dir_all(&staging_dir)?;
        }

        // Create staging directory structure
        let staging_assets = staging_dir.join("assets");
        let staging_visual = staging_assets.join("visual");
        let staging_collision = staging_assets.join("collision");
        std::fs::create_dir_all(&staging_visual)?;
        std::fs::create_dir_all(&staging_collision)?;

        // Copy URDF to staging
        std::fs::write(staging_dir.join("robot.urdf"), urdf_xml)?;

        // Copy each resolved asset to staging, compute hash, build RobotAsset
        let mut assets = Vec::new();
        for asset_ref in references {
            let source_path = resolution.resolved.get(&asset_ref.uri)
                .ok_or_else(|| ImportError::InvalidUrdf(
                    format!("Asset not in resolution: {}", asset_ref.uri)
                ))?;

            // Compute SHA-256
            let content = std::fs::read(source_path)?;
            let hash = Sha256::digest(&content);
            let sha256_hex = hex::encode(hash);

            // Determine role and destination
            let (role, dest_dir) = match asset_ref.role {
                AssetRole::Visual => (AssetRole::Visual, &staging_visual),
                AssetRole::Collision => (AssetRole::Collision, &staging_collision),
            };

            let filename = source_path.file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("unknown.stl");
            let dest_path = dest_dir.join(filename);

            std::fs::copy(source_path, &dest_path)?;

            // Build stored_path relative to workspace root
            let stored_path = PathBuf::from("robots")
                .join(robot_id)
                .join("assets")
                .join(role.to_string())
                .join(filename);

            // Short hash for asset ID
            let asset_id = sha256_hex[..12].to_string();

            assets.push(RobotAsset {
                id: asset_id,
                role,
                original_uri: asset_ref.uri.clone(),
                stored_path,
                sha256: sha256_hex,
                filename: filename.to_string(),
            });
        }

        // Build Resolution that points to staging paths for the importer
        let staging_resolution = build_staging_resolution(references, resolution, &staging_dir);

        // Import URDF with staging paths
        let result = import_urdf_resolved(urdf_xml, &staging_resolution)
            .map_err(|e| ImportError::InvalidUrdf(e.to_string()))?;

        let robot = result.robot;

        // Build kinematic chain
        let chain = adapter::auto(&robot)
            .map_err(|e| ImportError::ChainError(e.to_string()))?;

        // Build the record
        let now = chrono::Utc::now().to_rfc3339();
        #[allow(deprecated)]
        let record = RobotRecord {
            id: robot_id.to_string(),
            name: robot.name.clone(),
            manufacturer: None,
            model: None,
            source_type,
            source_label: source_label.map(|s| s.to_string()),
            urdf_xml: None, // New import — URDF lives in filesystem
            created_at: now.clone(),
            updated_at: now,
        };

        // COMMIT: move staging → final location
        // Ensure parent directory exists
        std::fs::create_dir_all(&robots_dir)?;

        // If a robot with this ID already exists, remove it (re-import)
        if robot_dir.exists() {
            std::fs::remove_dir_all(&robot_dir)?;
        }

        std::fs::rename(&staging_dir, &robot_dir)?;

        tracing::info!(
            robot_id = %robot_id,
            robot_name = %robot.name,
            asset_count = assets.len(),
            "Imported and materialized robot into workspace"
        );

        Ok(RobotImportResult {
            robot_id: robot_id.to_string(),
            record,
            assets,
            robot,
            chain,
        })
    }

    /// Verify integrity of a materialized robot's assets.
    ///
    /// Returns Ok(()) if all assets match their stored hashes, or an error
    /// listing the mismatched files.
    pub fn verify_integrity(
        workspace_root: &Path,
        robot_id: &str,
        assets: &[RobotAsset],
    ) -> Result<(), Vec<ImportError>> {
        let mut errors = Vec::new();

        for asset in assets {
            let absolute_path = workspace_root.join(&asset.stored_path);
            match std::fs::read(&absolute_path) {
                Ok(content) => {
                    let hash = Sha256::digest(&content);
                    let actual = hex::encode(hash);
                    if actual != asset.sha256 {
                        errors.push(ImportError::HashMismatch {
                            path: asset.stored_path.display().to_string(),
                            expected: asset.sha256.clone(),
                            actual,
                        });
                    }
                }
                Err(e) => {
                    errors.push(ImportError::Fs(e));
                }
            }
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
}

/// Find a URDF file in a package directory.
fn find_urdf_in_package(dir: &Path) -> Option<PathBuf> {
    // Check urdf/ subdirectory first
    let urdf_dir = dir.join("urdf");
    if urdf_dir.is_dir() {
        if let Ok(entries) = std::fs::read_dir(&urdf_dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().map_or(false, |ext| ext == "urdf") {
                    return Some(path);
                }
            }
        }
    }

    // Check root of package
    if let Ok(entries) = std::fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().map_or(false, |ext| ext == "urdf") {
                return Some(path);
            }
        }
    }

    None
}

/// Build a Resolution mapping original URIs to staging paths.
fn build_staging_resolution(
    references: &[thalos_importer::assets::AssetReference],
    original_resolution: &Resolution,
    staging_dir: &Path,
) -> Resolution {
    let mut resolved = std::collections::HashMap::new();

    for asset_ref in references {
        if let Some(original_path) = original_resolution.resolved.get(&asset_ref.uri) {
            let filename = original_path.file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("unknown.stl");
            let role_dir = asset_ref.role.to_string();
            let staging_path = staging_dir.join("assets").join(role_dir).join(filename);
            resolved.insert(asset_ref.uri.clone(), staging_path);
        }
    }

    Resolution {
        resolved,
        missing: vec![],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    #[test]
    fn find_urdf_in_package_subdir() {
        let dir = tempdir().unwrap();
        let urdf_dir = dir.path().join("urdf");
        std::fs::create_dir_all(&urdf_dir).unwrap();
        std::fs::write(urdf_dir.join("robot.urdf"), "<robot/>").unwrap();

        let found = find_urdf_in_package(dir.path());
        assert!(found.is_some());
        assert!(found.unwrap().ends_with("robot.urdf"));
    }

    #[test]
    fn find_urdf_in_package_root() {
        let dir = tempdir().unwrap();
        std::fs::write(dir.path().join("robot.urdf"), "<robot/>").unwrap();

        let found = find_urdf_in_package(dir.path());
        assert!(found.is_some());
    }

    #[test]
    fn find_urdf_returns_none_when_empty() {
        let dir = tempdir().unwrap();
        assert!(find_urdf_in_package(dir.path()).is_none());
    }
}
