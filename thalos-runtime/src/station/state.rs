use serde::{Deserialize, Serialize};
use thalos_engine::prelude::*;

/// StationRuntimeState (ADR-014)
/// Explicit lifecycle states for the Station Runtime supervisor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StationRuntimeState {
    Created,
    Starting,
    Ready,
    Active,
    Stopping,
    Stopped,
    Faulted,
}

impl StationRuntimeState {
    pub fn is_operational(&self) -> bool {
        matches!(self, Self::Ready | Self::Active)
    }
}

/// ModuleKind (ADR-014)
/// Categorizes module runtimes managed by the supervisor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ModuleKind {
    Acquisition,
    Robotics,
}

/// ModuleRuntimeState (ADR-014)
/// Individual operational state for station sub-modules.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ModuleRuntimeState {
    Disabled,
    Idle,
    Starting,
    Running,
    Stopping,
    Faulted,
}

/// OperationalSession (ADR-014)
/// Active session context instantiated when an operational activity begins.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OperationalSession {
    pub id: OperationalSessionId,
    pub station_id: StationId,
    pub started_at: String,
}

impl OperationalSession {
    pub fn new(id: impl Into<String>, station_id: StationId) -> Self {
        Self {
            id: OperationalSessionId(id.into()),
            station_id,
            started_at: chrono::Utc::now().to_rfc3339(),
        }
    }
}
