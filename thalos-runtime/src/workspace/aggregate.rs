use serde::{Deserialize, Serialize};
use std::fmt;
use std::str::FromStr;
use uuid::Uuid;

/// Strongly typed identity for a Layer 1 Robot resource.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RobotId(pub String);

impl RobotId {
    pub fn new(id: impl Into<String>) -> Self {
        Self(id.into())
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for RobotId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl From<String> for RobotId {
    fn from(s: String) -> Self {
        Self(s)
    }
}

impl From<&str> for RobotId {
    fn from(s: &str) -> Self {
        Self(s.to_string())
    }
}

/// Strongly typed identity for a Layer 1 Workspace resource.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct WorkspaceId(pub Uuid);

impl WorkspaceId {
    pub fn new() -> Self {
        Self(Uuid::new_v4())
    }

    pub fn from_uuid(uuid: Uuid) -> Self {
        Self(uuid)
    }
}

impl Default for WorkspaceId {
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Display for WorkspaceId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl FromStr for WorkspaceId {
    type Err = uuid::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Uuid::from_str(s).map(WorkspaceId)
    }
}

/// Persistent configuration parameters for a Workspace.
/// Contains application preferences, scene options, and environment settings.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct WorkspaceConfiguration {
    pub default_units: String,
    pub auto_save: bool,
    pub safety_margin: Option<String>,
    pub custom_metadata: std::collections::HashMap<String, String>,
}

impl Default for WorkspaceConfiguration {
    fn default() -> Self {
        Self {
            default_units: "mm".to_string(),
            auto_save: true,
            safety_margin: None,
            custom_metadata: std::collections::HashMap::new(),
        }
    }
}

/// Layer 1 Workspace aggregate representing a persistent user session/project.
///
/// Note: Computational engine state (e.g. RobotModel, KinematicChain, ExecutionPlan)
/// is NOT stored inside Workspace. It is deterministically reconstructed on load.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Workspace {
    pub id: WorkspaceId,
    pub name: String,
    pub description: Option<String>,
    pub robot_id: RobotId,
    pub active_tcp: Option<String>,
    pub configuration: WorkspaceConfiguration,
    pub created_at: String,
    pub updated_at: String,
}

impl Workspace {
    pub fn new(name: impl Into<String>, robot_id: RobotId) -> Self {
        let now = chrono::Utc::now().to_rfc3339();
        Self {
            id: WorkspaceId::new(),
            name: name.into(),
            description: None,
            robot_id,
            active_tcp: None,
            configuration: WorkspaceConfiguration::default(),
            created_at: now.clone(),
            updated_at: now,
        }
    }
}

/// Application representation of an actively loaded workspace session.
///
/// Distinguishes an active workspace context loaded in memory from a passive persistent Workspace resource.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ActiveWorkspace {
    pub workspace: Workspace,
    pub active_robot_id: RobotId,
}

