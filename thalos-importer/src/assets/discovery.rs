use std::collections::HashSet;

use crate::assets::{AssetKind, AssetReference, AssetRole};
use crate::error::ImportError;
use crate::urdf;

/// Default URDF asset discovery implementation.
///
/// Parses the URDF source, extracts all mesh filenames from visual and collision
/// elements, and returns deduplicated [`AssetReference`]s with their role.
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
/// Walks `visual_sources` and `collision_sources` from every body. The same
/// URI may appear in both roles (unlikely but possible); in that case both
/// references are kept since they serve different purposes.
pub fn collect_asset_references(candidate: &crate::ImportedCandidate) -> Vec<AssetReference> {
    let mut seen_visual = HashSet::new();
    let mut seen_collision = HashSet::new();
    let mut refs = Vec::new();

    for body in &candidate.raw_bodies {
        for uri in &body.visual_sources {
            if seen_visual.insert(uri.clone()) {
                let kind = infer_kind(uri);
                refs.push(AssetReference {
                    uri: uri.clone(),
                    kind,
                    role: AssetRole::Visual,
                });
            }
        }
        for uri in &body.collision_sources {
            if seen_collision.insert(uri.clone()) {
                let kind = infer_kind(uri);
                refs.push(AssetReference {
                    uri: uri.clone(),
                    kind,
                    role: AssetRole::Collision,
                });
            }
        }
    }

    refs
}

fn infer_kind(uri: &str) -> AssetKind {
    uri.rsplit('.')
        .next()
        .map(AssetKind::from_extension)
        .unwrap_or(AssetKind::Mesh)
}
