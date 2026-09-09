use crate::execution::executor::ExecutionSessionState;

/// UI projection of execution session state.
/// This is NOT an authority — it's a read-only view for the interface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SessionStatus {
    Ready,
    Running,
    Paused,
    Completed,
    Cancelled,
    Failed,
}

impl SessionStatus {
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            SessionStatus::Completed | SessionStatus::Cancelled | SessionStatus::Failed
        )
    }

    pub fn is_active(&self) -> bool {
        matches!(self, SessionStatus::Running | SessionStatus::Paused)
    }
}

/// Convert from the authoritative `ExecutionSessionState` to the UI projection `SessionStatus`.
impl From<ExecutionSessionState> for SessionStatus {
    fn from(state: ExecutionSessionState) -> Self {
        match state {
            ExecutionSessionState::Created
            | ExecutionSessionState::Reserved
            | ExecutionSessionState::Dispatched => SessionStatus::Ready,
            ExecutionSessionState::Running => SessionStatus::Running,
            ExecutionSessionState::Paused => SessionStatus::Paused,
            ExecutionSessionState::Completed => SessionStatus::Completed,
            ExecutionSessionState::Cancelled => SessionStatus::Cancelled,
            ExecutionSessionState::Failed => SessionStatus::Failed,
        }
    }
}
