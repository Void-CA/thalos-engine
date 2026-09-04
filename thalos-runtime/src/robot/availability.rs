use std::path::Path;

use crate::ports::RobotRepository;
use crate::robot::importer::RobotImporter;

/// Indicates whether a robot's assets are fully materialized in the workspace.
///
/// This is NOT an error state — `RequiresReimport` is a migration/compatibility
/// status that the UI can display to prompt the user for action.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RobotAvailability {
    /// Robot is fully materialized: URDF + all assets exist and integrity passes.
    Materialized,
    /// Robot record exists but has no materialized assets.
    /// The user needs to re-import from the original source.
    RequiresReimport,
    /// Robot record exists but asset files are corrupted or missing.
    /// Integrity verification failed.
    Corrupted { missing: Vec<String> },
    /// Robot record is legacy (urdf_xml in SQLite) with no filesystem artifacts.
    Legacy,
}

/// Check the availability of a robot's materialized assets in the workspace.
///
/// The conditions for `Materialized` are strict:
/// 1. Robot record exists in SQLite
/// 2. `robots/<id>/robot.urdf` exists on filesystem
/// 3. At least one `RobotAsset` is persisted
/// 4. All asset files exist and SHA-256 hashes match
pub async fn check_robot_availability(
    robot_id: &str,
    workspace_root: &Path,
    repo: &dyn RobotRepository,
) -> RobotAvailability {
    // 1. Check record exists
    let record = match repo.get(robot_id).await {
        Ok(Some(r)) => r,
        Ok(None) => return RobotAvailability::Legacy,
        Err(_) => return RobotAvailability::Legacy,
    };

    // 2. Check URDF exists on filesystem
    let urdf_path = workspace_root.join("robots").join(robot_id).join("robot.urdf");
    if !urdf_path.exists() {
        #[allow(deprecated)]
        if record.urdf_xml.is_some() {
            return RobotAvailability::Legacy;
        }
        return RobotAvailability::RequiresReimport;
    }

    // 3. Check assets
    let assets = match repo.get_assets(robot_id).await {
        Ok(a) => a,
        Err(_) => return RobotAvailability::RequiresReimport,
    };

    if assets.is_empty() {
        #[allow(deprecated)]
        if record.urdf_xml.is_some() {
            return RobotAvailability::Legacy;
        }
        return RobotAvailability::RequiresReimport;
    }

    // 4. Verify integrity
    match RobotImporter::verify_integrity(workspace_root, robot_id, &assets) {
        Ok(()) => RobotAvailability::Materialized,
        Err(errors) => {
            let missing: Vec<String> = errors.iter().map(|e| e.to_string()).collect();
            RobotAvailability::Corrupted { missing }
        }
    }
}
