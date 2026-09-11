use async_trait::async_trait;

use super::robot_repository::{PersistenceError, Result};

pub use crate::execution::session::domain::{ExecutionConfiguration, ExecutionSessionId};
pub use crate::execution::session::domain::ExecutionSession;
pub use crate::execution::session::history::ExecutionHistory;

/// Repository for persisting execution sessions and their history.
///
/// This trait speaks in domain types only — no persistence entities,
/// no SQL, no serialization format. The implementation decides how
/// to map between domain models and storage.
///
/// Two aggregates share the same identity (`ExecutionSessionId`):
///
/// - **`ExecutionSession`** — the mutable aggregate root with lifecycle
///   state machine, configuration, and current runtime state.
/// - **`ExecutionHistory`** — the immutable audit log: lifecycle transitions,
///   tick results, and faults. Append-only after session creation.
#[async_trait]
pub trait ExecutionRepository: Send + Sync {
    // ─── Session aggregate ──────────────────────────────────────

    async fn save_session(&self, session: &ExecutionSession) -> Result<()>;
    async fn get_session(&self, id: &ExecutionSessionId) -> Result<Option<ExecutionSession>>;
    async fn list_sessions(&self) -> Result<Vec<ExecutionSession>>;
    async fn delete_session(&self, id: &ExecutionSessionId) -> Result<()>;

    // ─── History aggregate ──────────────────────────────────────

    async fn save_history(&self, history: &ExecutionHistory) -> Result<()>;
    async fn get_history(&self, id: &ExecutionSessionId) -> Result<Option<ExecutionHistory>>;
}
