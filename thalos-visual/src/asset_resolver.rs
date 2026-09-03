use std::collections::HashMap;
use std::path::{Path, PathBuf};
use thiserror::Error;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum AssetResolverError {
    #[error("File not found: {0}")]
    FileNotFound(PathBuf),
    #[error("Package not found: {0}")]
    PackageNotFound(String),
    #[error("Invalid asset URI: {0}")]
    InvalidUri(String),
}

/// Resolves asset paths (such as `package://package_name/path/to/mesh.stl` or relative paths)
/// to absolute filesystem paths.
#[derive(Debug, Clone, Default)]
pub struct AssetResolver {
    /// Directory of the URDF file or project root.
    base_dir: Option<PathBuf>,
    /// Explicit mappings from package name to base directory on disk.
    package_mappings: HashMap<String, PathBuf>,
}

impl AssetResolver {
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

    /// Resolve an asset URI or path to an absolute, existing [`PathBuf`].
    pub fn resolve(&self, uri: &str) -> Result<PathBuf, AssetResolverError> {
        let path = if let Some(package_path) = uri.strip_prefix("package://") {
            self.resolve_package_uri(package_path)?
        } else if let Some(file_path) = uri.strip_prefix("file://") {
            PathBuf::from(file_path)
        } else {
            // Relative path
            let relative = PathBuf::from(uri);
            if relative.is_absolute() {
                relative
            } else if let Some(base) = &self.base_dir {
                base.join(relative)
            } else {
                relative
            }
        };

        if path.exists() {
            Ok(path)
        } else {
            Err(AssetResolverError::FileNotFound(path))
        }
    }

    fn resolve_package_uri(&self, package_rel_path: &str) -> Result<PathBuf, AssetResolverError> {
        let mut parts = package_rel_path.splitn(2, '/');
        let pkg_name = parts
            .next()
            .ok_or_else(|| AssetResolverError::InvalidUri(package_rel_path.to_string()))?;
        let sub_path = parts.next().unwrap_or("");

        // 1. Check registered package mappings
        if let Some(pkg_base) = self.package_mappings.get(pkg_name) {
            return Ok(pkg_base.join(sub_path));
        }

        // 2. Heuristic check relative to base_dir
        if let Some(base) = &self.base_dir {
            // Check if base_dir itself ends with pkg_name or parent contains pkg_name folder
            let mut curr = Some(base.as_path());
            while let Some(dir) = curr {
                if dir.file_name().and_then(|n| n.to_str()) == Some(pkg_name) {
                    return Ok(dir.join(sub_path));
                }
                let candidate = dir.join(pkg_name);
                if candidate.is_dir() {
                    return Ok(candidate.join(sub_path));
                }
                curr = dir.parent();
            }

            // Fallback: join directly to base_dir
            return Ok(base.join(sub_path));
        }

        Err(AssetResolverError::PackageNotFound(pkg_name.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs::{create_dir_all, File};
    use tempfile::tempdir;

    #[test]
    fn resolve_relative_path() {
        let dir = tempdir().unwrap();
        let mesh_path = dir.path().join("meshes").join("link.stl");
        create_dir_all(mesh_path.parent().unwrap()).unwrap();
        File::create(&mesh_path).unwrap();

        let resolver = AssetResolver::new().with_base_dir(dir.path());
        let resolved = resolver.resolve("meshes/link.stl").unwrap();
        assert_eq!(resolved, mesh_path);
    }

    #[test]
    fn resolve_package_uri_with_mapping() {
        let dir = tempdir().unwrap();
        let mesh_path = dir.path().join("ur10").join("meshes").join("base.stl");
        create_dir_all(mesh_path.parent().unwrap()).unwrap();
        File::create(&mesh_path).unwrap();

        let resolver = AssetResolver::new().register_package("ur_description", dir.path().join("ur10"));
        let resolved = resolver.resolve("package://ur_description/meshes/base.stl").unwrap();
        assert_eq!(resolved, mesh_path);
    }

    #[test]
    fn resolve_package_uri_heuristic() {
        let dir = tempdir().unwrap();
        let pkg_dir = dir.path().join("abb_irb1300");
        let mesh_path = pkg_dir.join("meshes").join("visual").join("link_1.stl");
        create_dir_all(mesh_path.parent().unwrap()).unwrap();
        File::create(&mesh_path).unwrap();

        let resolver = AssetResolver::new().with_base_dir(pkg_dir.join("urdf"));
        let resolved = resolver.resolve("package://abb_irb1300/meshes/visual/link_1.stl").unwrap();
        assert_eq!(resolved, mesh_path);
    }

    #[test]
    fn missing_file_returns_error() {
        let dir = tempdir().unwrap();
        let resolver = AssetResolver::new().with_base_dir(dir.path());
        let err = resolver.resolve("non_existent.stl").unwrap_err();
        assert!(matches!(err, AssetResolverError::FileNotFound(_)));
    }
}
