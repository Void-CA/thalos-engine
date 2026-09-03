use std::collections::HashMap;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use super::AssetReference;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum UriResolverError {
    #[error("File not found: {0}")]
    FileNotFound(PathBuf),
    #[error("Package not found: {0}")]
    PackageNotFound(String),
    #[error("Invalid asset URI: {0}")]
    InvalidUri(String),
}

/// The result of resolving a set of asset URIs.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Resolution {
    /// URIs that resolved to a filesystem path.
    pub resolved: HashMap<String, PathBuf>,
    /// URIs that could not be resolved (files not found).
    pub missing: Vec<AssetReference>,
}

/// Resolves asset URIs (`package://`, `file://`, relative) to absolute filesystem paths.
///
/// This is a layer-0 resolver that lives in `thalos-importer` so both the importer
/// pipeline and higher-level crates (`thalos-visual`) can share the same logic.
#[derive(Debug, Clone, Default)]
pub struct UriResolver {
    base_dir: Option<PathBuf>,
    package_mappings: HashMap<String, PathBuf>,
}

impl UriResolver {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_base_dir<P: AsRef<Path>>(mut self, base_dir: P) -> Self {
        self.base_dir = Some(base_dir.as_ref().to_path_buf());
        self
    }

    pub fn register_package<P: AsRef<Path>>(mut self, package_name: impl Into<String>, path: P) -> Self {
        self.package_mappings.insert(package_name.into(), path.as_ref().to_path_buf());
        self
    }

    /// Resolve a single URI without checking if the file exists.
    ///
    /// Returns `Ok(path)` for syntactically valid URIs. Does NOT verify `path.exists()`.
    pub fn resolve_uri(&self, uri: &str) -> Result<PathBuf, UriResolverError> {
        if let Some(package_path) = uri.strip_prefix("package://") {
            self.resolve_package_uri(package_path, false)
        } else if let Some(file_path) = uri.strip_prefix("file://") {
            Ok(PathBuf::from(file_path))
        } else {
            let relative = PathBuf::from(uri);
            if relative.is_absolute() {
                Ok(relative)
            } else if let Some(base) = &self.base_dir {
                Ok(base.join(relative))
            } else {
                Ok(relative)
            }
        }
    }

    /// Resolve a URI and verify the file exists on disk.
    pub fn resolve_uri_strict(&self, uri: &str) -> Result<PathBuf, UriResolverError> {
        let path = self.resolve_uri(uri)?;
        if path.exists() {
            Ok(path)
        } else {
            Err(UriResolverError::FileNotFound(path))
        }
    }

    /// Resolve a batch of asset references.
    ///
    /// URIs that map to existing files go into `resolved`; the rest go into `missing`.
    pub fn resolve(&self, references: &[AssetReference]) -> Resolution {
        let mut resolved = HashMap::new();
        let mut missing = Vec::new();

        for asset_ref in references {
            match self.resolve_uri_strict(&asset_ref.uri) {
                Ok(path) => {
                    resolved.insert(asset_ref.uri.clone(), path);
                }
                Err(_) => {
                    missing.push(asset_ref.clone());
                }
            }
        }

        Resolution { resolved, missing }
    }

    /// Resolve a batch strictly — if ANY file is missing, return an error.
    pub fn resolve_strict(&self, references: &[AssetReference]) -> Result<Resolution, UriResolverError> {
        let mut resolved = HashMap::new();
        let mut missing = Vec::new();

        for asset_ref in references {
            match self.resolve_uri_strict(&asset_ref.uri) {
                Ok(path) => {
                    resolved.insert(asset_ref.uri.clone(), path);
                }
                Err(_) => {
                    missing.push(asset_ref.clone());
                }
            }
        }

        if !missing.is_empty() {
            // Still return the partial resolution alongside the missing refs.
            return Ok(Resolution { resolved, missing });
        }

        Ok(Resolution { resolved, missing })
    }

    fn resolve_package_uri(&self, package_rel_path: &str, check_exists: bool) -> Result<PathBuf, UriResolverError> {
        let mut parts = package_rel_path.splitn(2, '/');
        let pkg_name = parts
            .next()
            .ok_or_else(|| UriResolverError::InvalidUri(package_rel_path.to_string()))?;
        let sub_path = parts.next().unwrap_or("");

        // 1. Check registered package mappings
        if let Some(pkg_base) = self.package_mappings.get(pkg_name) {
            let path = pkg_base.join(sub_path);
            if check_exists && !path.exists() {
                return Err(UriResolverError::FileNotFound(path));
            }
            return Ok(path);
        }

        // 2. Heuristic: walk up from base_dir looking for pkg_name
        if let Some(base) = &self.base_dir {
            let mut curr = Some(base.as_path());
            while let Some(dir) = curr {
                if dir.file_name().and_then(|n| n.to_str()) == Some(pkg_name) {
                    let path = dir.join(sub_path);
                    if check_exists && !path.exists() {
                        return Err(UriResolverError::FileNotFound(path));
                    }
                    return Ok(path);
                }
                let candidate = dir.join(pkg_name);
                if candidate.is_dir() {
                    let path = candidate.join(sub_path);
                    if check_exists && !path.exists() {
                        return Err(UriResolverError::FileNotFound(path));
                    }
                    return Ok(path);
                }
                curr = dir.parent();
            }

            // Fallback: join directly to base_dir
            let path = base.join(sub_path);
            if check_exists && !path.exists() {
                return Err(UriResolverError::FileNotFound(path));
            }
            return Ok(path);
        }

        Err(UriResolverError::PackageNotFound(pkg_name.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assets::{AssetKind, AssetRole};
    use std::fs::{create_dir_all, File};
    use tempfile::tempdir;

    #[test]
    fn resolve_relative_path() {
        let dir = tempdir().unwrap();
        let mesh_path = dir.path().join("meshes").join("link.stl");
        create_dir_all(mesh_path.parent().unwrap()).unwrap();
        File::create(&mesh_path).unwrap();

        let resolver = UriResolver::new().with_base_dir(dir.path());
        let resolved = resolver.resolve_uri_strict("meshes/link.stl").unwrap();
        assert_eq!(resolved, mesh_path);
    }

    #[test]
    fn resolve_package_uri_with_mapping() {
        let dir = tempdir().unwrap();
        let mesh_path = dir.path().join("ur10").join("meshes").join("base.stl");
        create_dir_all(mesh_path.parent().unwrap()).unwrap();
        File::create(&mesh_path).unwrap();

        let resolver = UriResolver::new()
            .register_package("ur_description", dir.path().join("ur10"));
        let resolved = resolver.resolve_uri_strict("package://ur_description/meshes/base.stl").unwrap();
        assert_eq!(resolved, mesh_path);
    }

    #[test]
    fn resolve_package_uri_heuristic() {
        let dir = tempdir().unwrap();
        let pkg_dir = dir.path().join("abb_irb1300");
        let mesh_path = pkg_dir.join("meshes").join("visual").join("link_1.stl");
        create_dir_all(mesh_path.parent().unwrap()).unwrap();
        File::create(&mesh_path).unwrap();

        let resolver = UriResolver::new().with_base_dir(pkg_dir.join("urdf"));
        let resolved = resolver.resolve_uri_strict("package://abb_irb1300/meshes/visual/link_1.stl").unwrap();
        assert_eq!(resolved, mesh_path);
    }

    #[test]
    fn missing_file_returns_error() {
        let dir = tempdir().unwrap();
        let resolver = UriResolver::new().with_base_dir(dir.path());
        let err = resolver.resolve_uri_strict("non_existent.stl").unwrap_err();
        assert!(matches!(err, UriResolverError::FileNotFound(_)));
    }

    #[test]
    fn resolve_without_checking_existence() {
        let dir = tempdir().unwrap();
        let resolver = UriResolver::new().with_base_dir(dir.path());
        // File does NOT exist, but resolve_uri returns Ok
        let path = resolver.resolve_uri("meshes/nonexistent.stl").unwrap();
        assert_eq!(path, dir.path().join("meshes").join("nonexistent.stl"));
    }

    #[test]
    fn resolve_batch_partial() {
        let dir = tempdir().unwrap();
        let mesh_path = dir.path().join("meshes").join("base.stl");
        create_dir_all(mesh_path.parent().unwrap()).unwrap();
        File::create(&mesh_path).unwrap();

        let resolver = UriResolver::new().with_base_dir(dir.path());
        let references = vec![
            AssetReference { uri: "meshes/base.stl".into(), kind: AssetKind::Mesh, role: AssetRole::Visual },
            AssetReference { uri: "meshes/missing.stl".into(), kind: AssetKind::Mesh, role: AssetRole::Visual },
        ];
        let resolution = resolver.resolve(&references);

        assert_eq!(resolution.resolved.len(), 1);
        assert_eq!(resolution.missing.len(), 1);
    }
}
