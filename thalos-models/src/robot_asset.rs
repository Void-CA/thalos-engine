use serde::{Deserialize, Serialize};
use std::path::PathBuf;

/// The role an asset plays in the robot description.
///
/// Visual assets are used for rendering. Collision assets are used for
/// planning, safety validation, and collision detection. They are
/// modeled separately because a robot may use different meshes for
/// each purpose (e.g. a high-poly visual mesh vs a simplified collision mesh).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssetRole {
    Visual,
    Collision,
}

impl std::fmt::Display for AssetRole {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AssetRole::Visual => write!(f, "visual"),
            AssetRole::Collision => write!(f, "collision"),
        }
    }
}

/// A single external asset (mesh, texture, etc.) that belongs to a robot.
///
/// Each asset is identified by a content hash (`sha256`) and stored at a
/// path relative to the workspace root. The `original_uri` preserves the
/// source reference from the URDF for traceability.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotAsset {
    /// Short content hash used as a local identifier.
    pub id: String,
    /// Whether this asset serves visual or collision purposes.
    pub role: AssetRole,
    /// The URI as declared in the original URDF (e.g. `package://abb_irb140/meshes/visual/link_1.stl`).
    pub original_uri: String,
    /// Path relative to the workspace root (e.g. `robots/<id>/assets/visual/link_1.stl`).
    pub stored_path: PathBuf,
    /// Full SHA-256 hex digest of the file content.
    pub sha256: String,
    /// The file name (e.g. `link_1.stl`).
    pub filename: String,
}
