use std::path::{Path, PathBuf};
use serde::{Deserialize, Serialize};

/// A record of a previously opened workspace.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KnownWorkspace {
    /// The display name of the workspace.
    pub name: String,
    /// The absolute path to the workspace directory.
    pub path: PathBuf,
    /// ISO 8601 timestamp of when this workspace was last opened.
    pub last_opened: String,
}

/// Application-level registry of known workspaces.
///
/// Persisted at `~/.thalos/known_workspaces.json`. This allows the
/// application to remember which workspaces the user has opened, so they
/// can be reopened without manual directory selection.
///
/// This is APPLICATION state, not workspace state. It lives outside any
/// individual workspace directory.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct KnownWorkspaces {
    pub workspaces: Vec<KnownWorkspace>,
}

impl KnownWorkspaces {
    /// Load known workspaces from the default location (`~/.thalos/known_workspaces.json`).
    ///
    /// Returns an empty registry if the file doesn't exist or can't be parsed.
    pub fn load() -> Self {
        let path = Self::default_path();
        Self::load_from(&path)
    }

    /// Load known workspaces from a specific file path.
    pub fn load_from(path: &Path) -> Self {
        match std::fs::read_to_string(path) {
            Ok(content) => serde_json::from_str(&content).unwrap_or_default(),
            Err(_) => Self::default(),
        }
    }

    /// Save known workspaces to the default location.
    pub fn save(&self) -> Result<(), std::io::Error> {
        let path = Self::default_path();
        self.save_to(&path)
    }

    /// Save known workspaces to a specific file path.
    pub fn save_to(&self, path: &Path) -> Result<(), std::io::Error> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let json = serde_json::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        std::fs::write(path, json)
    }

    /// Record or update a workspace as recently opened.
    ///
    /// If the workspace path already exists, updates its name and timestamp.
    /// Otherwise, adds a new entry.
    pub fn record_open(&mut self, name: &str, path: &Path) {
        let now = chrono::Utc::now().to_rfc3339();
        let abs_path = path.to_path_buf();

        if let Some(existing) = self.workspaces.iter_mut().find(|w| w.path == abs_path) {
            existing.name = name.to_string();
            existing.last_opened = now;
        } else {
            self.workspaces.push(KnownWorkspace {
                name: name.to_string(),
                path: abs_path,
                last_opened: now,
            });
        }

        // Sort by last_opened descending (most recent first)
        self.workspaces.sort_by(|a, b| b.last_opened.cmp(&a.last_opened));
    }

    /// Remove a workspace from the known list.
    pub fn remove(&mut self, path: &Path) {
        self.workspaces.retain(|w| w.path != path);
    }

    /// Get the most recently opened workspace, if any.
    pub fn most_recent(&self) -> Option<&KnownWorkspace> {
        self.workspaces.first()
    }

    /// Get all known workspaces, sorted by most recently opened.
    pub fn list(&self) -> &[KnownWorkspace] {
        &self.workspaces
    }

    /// The default path for the known workspaces file.
    fn default_path() -> PathBuf {
        dirs().join("known_workspaces.json")
    }
}

/// Get the application config directory (`~/.thalos/`).
pub fn dirs() -> PathBuf {
    dirs_impl().join("thalos")
}

#[cfg(target_os = "linux")]
fn dirs_impl() -> PathBuf {
    std::env::var("HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from("."))
        .join(".config")
}

#[cfg(target_os = "macos")]
fn dirs_impl() -> PathBuf {
    std::env::var("HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from("."))
        .join("Library/Application Support")
}

#[cfg(target_os = "windows")]
fn dirs_impl() -> PathBuf {
    std::env::var("APPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from("."))
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    #[test]
    fn known_workspaces_record_and_retrieve() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("known_workspaces.json");

        let mut known = KnownWorkspaces::default();
        known.record_open("Cell A", Path::new("/home/user/cell_a"));
        known.record_open("Cell B", Path::new("/home/user/cell_b"));
        known.save_to(&path).unwrap();

        let loaded = KnownWorkspaces::load_from(&path);
        assert_eq!(loaded.workspaces.len(), 2);
        assert_eq!(loaded.workspaces[0].name, "Cell B"); // most recent first
        assert_eq!(loaded.workspaces[1].name, "Cell A");
    }

    #[test]
    fn known_workspaces_update_existing() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("known_workspaces.json");

        let mut known = KnownWorkspaces::default();
        known.record_open("Cell A", Path::new("/home/user/cell_a"));
        known.record_open("Cell A Renamed", Path::new("/home/user/cell_a"));
        known.save_to(&path).unwrap();

        let loaded = KnownWorkspaces::load_from(&path);
        assert_eq!(loaded.workspaces.len(), 1);
        assert_eq!(loaded.workspaces[0].name, "Cell A Renamed");
    }

    #[test]
    fn known_workspaces_remove() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("known_workspaces.json");

        let mut known = KnownWorkspaces::default();
        known.record_open("Cell A", Path::new("/home/user/cell_a"));
        known.record_open("Cell B", Path::new("/home/user/cell_b"));
        known.remove(Path::new("/home/user/cell_a"));
        known.save_to(&path).unwrap();

        let loaded = KnownWorkspaces::load_from(&path);
        assert_eq!(loaded.workspaces.len(), 1);
        assert_eq!(loaded.workspaces[0].name, "Cell B");
    }

    #[test]
    fn load_returns_empty_on_missing_file() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("nonexistent.json");
        let known = KnownWorkspaces::load_from(&path);
        assert!(known.workspaces.is_empty());
    }
}
