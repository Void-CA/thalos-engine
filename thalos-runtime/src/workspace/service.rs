use std::path::{Path, PathBuf};
use std::sync::Arc;
use tokio::sync::RwLock;

use crate::error::RuntimeError;
use crate::ports::WorkspaceRepository;
use crate::robot::service::RobotService;
use crate::scene::service::SceneService;
use crate::scene::snapshot::RuntimeSnapshot;
use crate::workspace::aggregate::{ActiveWorkspace, RobotId, Workspace, WorkspaceId};
use crate::workspace::known::{KnownWorkspace, KnownWorkspaces};

/// Application output representing an opened workspace session.
///
/// Contains the persistent Workspace resource, the active robot identity,
/// and the reconstructed application runtime snapshot.
#[derive(Debug, Clone)]
pub struct OpenedWorkspace {
    pub workspace: Workspace,
    pub active_robot_id: RobotId,
    pub runtime_snapshot: RuntimeSnapshot,
}

/// Application service for Workspace resource lifecycle and orchestration.
///
/// This is the **authority** for workspace location and identity. It owns:
/// - The active workspace root directory (where `workspace.db` and `robots/` live)
/// - Workspace persistence (create, open, save, delete)
/// - Known workspaces registry (recent workspaces across sessions)
/// - Robot loading coordination (delegates to `RobotService`)
///
/// Downstream services (`RobotService`, `RobotImporter`) receive the workspace
/// root as a parameter — they do NOT depend on `WorkspaceService` directly.
pub struct WorkspaceService {
    workspace_repo: Arc<dyn WorkspaceRepository>,
    robot_service: Arc<RobotService>,
    scene_service: Arc<SceneService>,
    active_workspace: Arc<RwLock<Option<ActiveWorkspace>>>,
    /// The root directory of the currently active workspace.
    /// This is runtime state, NOT persisted in the Workspace aggregate.
    active_root: Arc<RwLock<Option<PathBuf>>>,
    /// Application-level registry of known workspaces.
    known_workspaces: Arc<RwLock<KnownWorkspaces>>,
}

impl WorkspaceService {
    pub fn new(
        workspace_repo: Arc<dyn WorkspaceRepository>,
        robot_service: Arc<RobotService>,
        scene_service: Arc<SceneService>,
    ) -> Self {
        Self {
            workspace_repo,
            robot_service,
            scene_service,
            active_workspace: Arc::new(RwLock::new(None)),
            active_root: Arc::new(RwLock::new(None)),
            known_workspaces: Arc::new(RwLock::new(KnownWorkspaces::load())),
        }
    }

    /// Retrieve the currently active workspace context, if one is loaded.
    pub async fn active_workspace(&self) -> Option<ActiveWorkspace> {
        self.active_workspace.read().await.clone()
    }

    /// Get the root directory of the currently active workspace.
    ///
    /// Returns `None` if no workspace is open. Callers should handle this
    /// case (e.g. fall back to a default or return an error).
    pub async fn root(&self) -> Option<PathBuf> {
        self.active_root.read().await.clone()
    }

    /// Get the list of recently opened workspaces, sorted by most recent.
    pub async fn recent_workspaces(&self) -> Vec<KnownWorkspace> {
        self.known_workspaces.read().await.list().to_vec()
    }

    /// Open a workspace from a directory path.
    ///
    /// The directory must contain a `workspace.db`. This method:
    /// 1. Validates the directory exists and contains a workspace database
    /// 2. Opens/creates the SQLite repository from `workspace.db`
    /// 3. Loads the workspace aggregate
    /// 4. Resolves the robot and reconstructs computational state
    /// 5. Sets this as the active workspace
    pub async fn open_at(
        &self,
        path: &Path,
    ) -> Result<OpenedWorkspace, RuntimeError> {
        if !path.exists() {
            return Err(RuntimeError::WorkspaceNotFound {
                id: path.display().to_string(),
            });
        }

        let db_path = path.join("workspace.db");

        // If workspace.db doesn't exist, this is a fresh directory —
        // we can still use it, but there's no persisted workspace to load.
        // The caller should use create_at() for new workspaces.
        if !db_path.exists() {
            return Err(RuntimeError::WorkspaceNotFound {
                id: format!("No workspace.db in {}", path.display()),
            });
        }

        // Set the active root
        *self.active_root.write().await = Some(path.to_path_buf());

        // List workspaces from the repository — there should be exactly one
        // in a self-contained workspace directory.
        let workspaces = self.workspace_repo.list().await
            .map_err(|e| RuntimeError::Persistence {
                message: format!("Failed to list workspaces: {e}"),
            })?;

        if let Some(workspace) = workspaces.into_iter().next() {
            // Load robot into scene
            let snapshot = self
                .robot_service
                .load_materialized_robot(
                    &workspace.robot_id.0,
                    path,
                    &self.scene_service,
                )
                .await?;

            let active_ws = ActiveWorkspace {
                workspace: workspace.clone(),
                active_robot_id: workspace.robot_id.clone(),
            };

            *self.active_workspace.write().await = Some(active_ws);

            tracing::info!(
                workspace_id = %workspace.id,
                workspace_name = %workspace.name,
                workspace_root = %path.display(),
                "Opened workspace from directory"
            );

            // Record in known workspaces
            {
                let mut known = self.known_workspaces.write().await;
                known.record_open(&workspace.name, path);
                let _ = known.save(); // best-effort persistence
            }

            Ok(OpenedWorkspace {
                active_robot_id: workspace.robot_id.clone(),
                workspace,
                runtime_snapshot: snapshot,
            })
        } else {
            Err(RuntimeError::WorkspaceNotFound {
                id: format!("No workspace records in {}", path.display()),
            })
        }
    }

    /// Create a new workspace in a directory.
    ///
    /// Creates `workspace.db` and the `robots/` subdirectory. If the directory
    /// already has a `workspace.db`, this method returns an error to prevent
    /// accidental overwrites.
    pub async fn create_at(
        &self,
        path: &Path,
        name: impl Into<String>,
        robot_id: RobotId,
    ) -> Result<Workspace, RuntimeError> {
        if !path.exists() {
            std::fs::create_dir_all(path).map_err(|e| RuntimeError::Persistence {
                message: format!("Failed to create workspace directory: {e}"),
            })?;
        }

        let db_path = path.join("workspace.db");
        if db_path.exists() {
            return Err(RuntimeError::Persistence {
                message: format!("Workspace already exists at {}", path.display()),
            });
        }

        // Create robots directory
        let robots_dir = path.join("robots");
        std::fs::create_dir_all(&robots_dir).map_err(|e| RuntimeError::Persistence {
            message: format!("Failed to create robots directory: {e}"),
        })?;

        // Create workspace aggregate
        let workspace = Workspace::new(name, robot_id);

        // Persist
        self.workspace_repo.save(&workspace).await
            .map_err(|e| RuntimeError::Persistence {
                message: format!("Failed to persist workspace: {e}"),
            })?;

        // Set as active
        *self.active_root.write().await = Some(path.to_path_buf());
        let active_ws = ActiveWorkspace {
            workspace: workspace.clone(),
            active_robot_id: workspace.robot_id.clone(),
        };
        *self.active_workspace.write().await = Some(active_ws);

        tracing::info!(
            workspace_id = %workspace.id,
            workspace_name = %workspace.name,
            workspace_root = %path.display(),
            "Created new workspace"
        );

        // Record in known workspaces
        {
            let mut known = self.known_workspaces.write().await;
            known.record_open(&workspace.name, path);
            let _ = known.save();
        }

        Ok(workspace)
    }

    /// Create a new workspace resource and persist it (legacy API).
    ///
    /// Prefer `create_at` for new code.
    pub async fn create_workspace(
        &self,
        name: impl Into<String>,
        robot_id: RobotId,
    ) -> Result<Workspace, RuntimeError> {
        let workspace = Workspace::new(name, robot_id);
        self.workspace_repo
            .save(&workspace)
            .await
            .map_err(|e| RuntimeError::Persistence {
                message: format!("Failed to create workspace: {e}"),
            })?;
        Ok(workspace)
    }

    /// List all persisted workspace resources.
    pub async fn list_workspaces(&self) -> Result<Vec<Workspace>, RuntimeError> {
        self.workspace_repo
            .list()
            .await
            .map_err(|e| RuntimeError::Persistence {
                message: format!("Failed to list workspaces: {e}"),
            })
    }

    /// Retrieve a single workspace resource by its ID.
    pub async fn get_workspace(&self, id: &WorkspaceId) -> Result<Option<Workspace>, RuntimeError> {
        self.workspace_repo
            .get(id)
            .await
            .map_err(|e| RuntimeError::Persistence {
                message: format!("Failed to get workspace: {e}"),
            })
    }

    /// Save or update a workspace resource.
    pub async fn save_workspace(&self, workspace: &Workspace) -> Result<(), RuntimeError> {
        self.workspace_repo
            .save(workspace)
            .await
            .map_err(|e| RuntimeError::Persistence {
                message: format!("Failed to save workspace: {e}"),
            })?;

        // If the saved workspace is the currently active workspace, update active_workspace state as well.
        let mut active = self.active_workspace.write().await;
        if let Some(ref mut active_ws) = *active {
            if active_ws.workspace.id == workspace.id {
                active_ws.workspace = workspace.clone();
            }
        }

        Ok(())
    }

    /// Delete a workspace resource by its ID.
    pub async fn delete_workspace(&self, id: &WorkspaceId) -> Result<(), RuntimeError> {
        self.workspace_repo
            .delete(id)
            .await
            .map_err(|e| RuntimeError::Persistence {
                message: format!("Failed to delete workspace: {e}"),
            })?;

        // If the deleted workspace was active, close active workspace state.
        let mut active = self.active_workspace.write().await;
        if let Some(ref active_ws) = *active {
            if active_ws.workspace.id == *id {
                *active = None;
                *self.active_root.write().await = None;
            }
        }

        Ok(())
    }

    /// Open a workspace by ID: loads the persistent Workspace aggregate, resolves its robot resource,
    /// reconstructs computational state in SceneService, updates active workspace state, and returns OpenedWorkspace.
    ///
    /// Requires that a workspace root has been set (via `open_at` or `create_at`).
    /// If no root is set, falls back to the legacy behavior.
    pub async fn open(&self, id: &WorkspaceId) -> Result<OpenedWorkspace, RuntimeError> {
        let workspace = self
            .workspace_repo
            .get(id)
            .await
            .map_err(|e| RuntimeError::Persistence {
                message: format!("Failed to query workspace: {e}"),
            })?
            .ok_or_else(|| RuntimeError::WorkspaceNotFound {
                id: id.to_string(),
            })?;

        let root = self.active_root.read().await.clone();

        // Orchestrate: load robot into scene to reconstruct computational state
        let snapshot = if let Some(ref root_path) = root {
            self.robot_service
                .load_materialized_robot(&workspace.robot_id.0, root_path, &self.scene_service)
                .await?
        } else {
            self.robot_service
                .load_robot_into_scene(&workspace.robot_id.0, &self.scene_service)
                .await?
        };

        let active_ws = ActiveWorkspace {
            workspace: workspace.clone(),
            active_robot_id: workspace.robot_id.clone(),
        };

        *self.active_workspace.write().await = Some(active_ws);

        tracing::info!(
            workspace_id = %workspace.id,
            workspace_name = %workspace.name,
            robot_id = %workspace.robot_id,
            "Opened workspace and set active workspace context"
        );

        // Record in known workspaces if root is available
        if let Some(ref root) = *self.active_root.read().await {
            let mut known = self.known_workspaces.write().await;
            known.record_open(&workspace.name, root);
            let _ = known.save();
        }

        Ok(OpenedWorkspace {
            active_robot_id: workspace.robot_id.clone(),
            workspace,
            runtime_snapshot: snapshot,
        })
    }

    /// Close the active workspace session, resetting active workspace context.
    pub async fn close(&self) -> Result<(), RuntimeError> {
        let mut active = self.active_workspace.write().await;
        if let Some(ref ws) = *active {
            tracing::info!(
                workspace_id = %ws.workspace.id,
                workspace_name = %ws.workspace.name,
                "Closed active workspace session"
            );
            *active = None;
        }
        *self.active_root.write().await = None;
        Ok(())
    }
}

