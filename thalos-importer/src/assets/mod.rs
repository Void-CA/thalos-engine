pub mod discovery;
pub mod resolver;

use serde::{Deserialize, Serialize};

use crate::candidate::ImportedCandidate;
use crate::diagnostic::{DiagnosticCode, ImportDiagnostic};
use crate::error::ImportError;
use resolver::Resolution;
use thalos_models::geometry::Geometry;
pub use thalos_models::robot_asset::AssetRole;

/// The kind of external asset referenced by a URDF geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AssetKind {
    Mesh,
}

impl AssetKind {
    /// Infer asset kind from a file extension, defaulting to [`AssetKind::Mesh`].
    pub fn from_extension(ext: &str) -> Self {
        match ext.to_ascii_lowercase().as_str() {
            "stl" | "dae" | "obj" | "ply" => Self::Mesh,
            _ => Self::Mesh,
        }
    }
}

/// A reference to an external file (mesh, texture, etc.) declared in a URDF.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct AssetReference {
    /// The raw URI as declared in the URDF (e.g. `package://abb_irb140_support/meshes/link_1.stl`).
    pub uri: String,
    /// Inferred kind of the asset.
    pub kind: AssetKind,
    /// Whether this asset is used for visual or collision purposes.
    pub role: AssetRole,
}

/// Discovery trait: scans a URDF source and returns the external assets it references.
///
/// This is a pure scan — it does not check whether the files exist or are resolvable.
/// Resolution is handled separately by [`resolver::UriResolver`].
pub trait AssetDiscovery {
    fn discover(&self, source: &str) -> Result<Vec<AssetReference>, ImportError>;
}

/// Rewrite mesh URIs in an [`ImportedCandidate`] using a [`Resolution`].
///
/// For each `Geometry::Mesh { filename }` in visual and collision elements,
/// if `filename` appears in `resolution.resolved`, it is replaced with the
/// resolved filesystem path. Meshes not in the resolution map are left untouched
/// and generate an [`ImportDiagnostic::Warning`] with [`DiagnosticCode::UnresolvedMeshReference`].
///
/// Returns the modified candidate and any diagnostics emitted during resolution.
pub fn resolve_candidate(
    mut candidate: ImportedCandidate,
    resolution: &Resolution,
) -> (ImportedCandidate, Vec<ImportDiagnostic>) {
    let mut diagnostics = Vec::new();

    for body in &mut candidate.raw_bodies {
        for visual in &mut body.visual {
            rewrite_geometry(&mut visual.geometry, resolution, &mut diagnostics);
        }
        for collision in &mut body.collision {
            rewrite_geometry(&mut collision.geometry, resolution, &mut diagnostics);
        }
    }

    (candidate, diagnostics)
}

fn rewrite_geometry(
    geometry: &mut Geometry,
    resolution: &Resolution,
    diagnostics: &mut Vec<ImportDiagnostic>,
) {
    if let Geometry::Mesh { filename, .. } = geometry {
        if let Some(resolved_path) = resolution.resolved.get(filename) {
            *filename = resolved_path.to_string_lossy().into_owned();
        } else {
            diagnostics.push(ImportDiagnostic::warning(
                DiagnosticCode::UnresolvedMeshReference,
                format!("Mesh asset not resolved: {filename}"),
            ));
        }
    }
}
