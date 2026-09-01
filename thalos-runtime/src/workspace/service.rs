use std::sync::Arc;
use tokio::sync::RwLock;

use crate::error::RuntimeError;
use crate::ports::WorkspaceRepository;
use crate::robot::service::RobotService;
use crate::scene::service::SceneService;
use crate::scene::snapshot::RuntimeSnapshot;
use crate::workspace::aggregate::{ActiveWorkspace, RobotId, Workspace, WorkspaceId};

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
/// Coordinates Workspace persistence with RobotService resource resolution
/// and SceneService computational state reconstruction.
pub struct WorkspaceService {
    workspace_repo: Arc<dyn WorkspaceRepository>,
    robot_service: Arc<RobotService>,
    scene_service: Arc<SceneService>,
    active_workspace: Arc<RwLock<Option<ActiveWorkspace>>>,
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
        }
    }

    /// Retrieve the currently active workspace context, if one is loaded.
    pub async fn active_workspace(&self) -> Option<ActiveWorkspace> {
        self.active_workspace.read().await.clone()
    }

    /// Create a new workspace resource and persist it.
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
            }
        }

        Ok(())
    }

    /// Open a workspace: loads the persistent Workspace aggregate, resolves its robot resource,
    /// reconstructs computational state in SceneService, updates active workspace state, and returns OpenedWorkspace.
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

        // Orchestrate: load robot into scene to reconstruct computational state
        let snapshot = self
            .robot_service
            .load_robot_into_scene(&workspace.robot_id.0, &self.scene_service)
            .await?;

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
        Ok(())
    }
}

