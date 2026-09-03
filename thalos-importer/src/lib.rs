pub mod assets;
pub mod candidate;
pub mod diagnostic;
pub mod error;
pub mod normalize;
pub mod urdf;

pub use assets::{AssetDiscovery, AssetKind, AssetReference, resolve_candidate};
pub use assets::resolver::{Resolution, UriResolver, UriResolverError};
pub use candidate::{CandidateBody, CandidateJoint, ImportedCandidate};
pub use diagnostic::{DiagnosticCode, ImportDiagnostic};
pub use error::ImportError;
pub use normalize::{CandidateNormalizer, NormalizedRobotResult, Normalizer};

pub use urdf::import_urdf;

/// Import a URDF XML string into a native [`Robot`](thalos_models::Robot) domain model,
/// resolving mesh assets via a [`Resolution`].
///
/// This is the full pipeline:
/// 1. Parse XML into an [`ImportedCandidate`]
/// 2. Rewrite mesh URIs using the provided resolution
/// 3. Validate and normalize the candidate into a domain [`Robot`]
///
/// Meshes whose URIs appear in `resolution.resolved` are replaced with
/// filesystem paths. Meshes not in the resolution map generate warnings
/// (not errors) — the robot is still semantically valid.
pub fn import_urdf_resolved(
    source: &str,
    resolution: &Resolution,
) -> Result<NormalizedRobotResult, ImportError> {
    let candidate: ImportedCandidate = urdf::parse(source).map_err(ImportError::from)?;
    let (candidate, diagnostics) = resolve_candidate(candidate, resolution);
    let normalizer = CandidateNormalizer::new();
    let mut result = normalizer.normalize(&candidate)?;
    result.diagnostics.extend(diagnostics);
    Ok(result)
}
