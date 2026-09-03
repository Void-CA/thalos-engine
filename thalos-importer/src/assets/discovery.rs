use std::collections::HashSet;

use crate::assets::{AssetKind, AssetReference};
use crate::error::ImportError;
use crate::urdf;

/// Default URDF asset discovery implementation.
///
/// Parses the URDF source, extracts all mesh filenames from visual and collision
/// elements, and returns deduplicated [`AssetReference`]s.
pub struct UrdfAssetDiscovery;

impl UrdfAssetDiscovery {
    pub fn new() -> Self {
        Self
    }
}

impl Default for UrdfAssetDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

impl crate::assets::AssetDiscovery for UrdfAssetDiscovery {
    fn discover(&self, source: &str) -> Result<Vec<AssetReference>, ImportError> {
        let candidate = urdf::parse(source).map_err(ImportError::from)?;
        Ok(collect_asset_references(&candidate))
    }
}

/// Collect all asset references from a parsed [`ImportedCandidate`].
///
/// Walks `visual_sources` and `collision_sources` from every body, deduplicates,
/// and infers [`AssetKind`] from the file extension.
pub fn collect_asset_references(candidate: &crate::ImportedCandidate) -> Vec<AssetReference> {
    let mut seen = HashSet::new();
    let mut refs = Vec::new();

    for body in &candidate.raw_bodies {
        for uri in body.visual_sources.iter().chain(body.collision_sources.iter()) {
            if seen.insert(uri.clone()) {
                let kind = uri
                    .rsplit('.')
                    .next()
                    .map(AssetKind::from_extension)
                    .unwrap_or(AssetKind::Mesh);
                refs.push(AssetReference {
                    uri: uri.clone(),
                    kind,
                });
            }
        }
    }

    refs
}
