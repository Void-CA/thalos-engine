use std::path::{Path, PathBuf};
pub use thalos_importer::UriResolverError;

/// Resolves asset paths (such as `package://package_name/path/to/mesh.stl` or relative paths)
/// to absolute filesystem paths.
///
/// This is a thin wrapper around [`thalos_importer::UriResolver`] that preserves the
/// original `AssetResolver` API for scene-handling and visual-mapping consumers.
/// For the canonical implementation and batch resolution, see
/// [`thalos_importer::UriResolver`] and [`thalos_importer::Resolution`].
#[derive(Debug, Clone, Default)]
pub struct AssetResolver {
    inner: thalos_importer::UriResolver,
}

impl AssetResolver {
    pub fn new() -> Self {
        Self { inner: thalos_importer::UriResolver::new() }
    }

    pub fn with_base_dir<P: AsRef<Path>>(mut self, base_dir: P) -> Self {
        self.inner = self.inner.with_base_dir(base_dir);
        self
    }

    pub fn register_package<P: AsRef<Path>>(mut self, package_name: impl Into<String>, path: P) -> Self {
        self.inner = self.inner.register_package(package_name, path);
        self
    }

    /// Resolve an asset URI or path to an absolute, existing [`PathBuf`].
    pub fn resolve(&self, uri: &str) -> Result<PathBuf, UriResolverError> {
        self.inner.resolve_uri_strict(uri)
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
        assert!(matches!(err, UriResolverError::FileNotFound(_)));
    }
}
