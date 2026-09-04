pub mod aggregate;
pub mod known;
pub mod service;

pub use aggregate::{ActiveWorkspace, RobotId, Workspace, WorkspaceConfiguration, WorkspaceId};
pub use known::{KnownWorkspace, KnownWorkspaces};
pub use service::{OpenedWorkspace, WorkspaceService};

