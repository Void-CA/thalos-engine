use crate::ports::robot_repository::Result;
use crate::workspace::aggregate::{Workspace, WorkspaceId};
use async_trait::async_trait;

/// Port interface for Workspace persistence in Layer 1.
///
/// Implementations (such as SqliteWorkspaceRepository in thalos-persistence)
/// allow storing and retrieving Workspace aggregates without leaking infrastructure
/// dependencies into thalos-runtime.
#[async_trait]
pub trait WorkspaceRepository: Send + Sync {
    async fn list(&self) -> Result<Vec<Workspace>>;
    async fn get(&self, id: &WorkspaceId) -> Result<Option<Workspace>>;
    async fn save(&self, workspace: &Workspace) -> Result<()>;
    async fn delete(&self, id: &WorkspaceId) -> Result<()>;
}
