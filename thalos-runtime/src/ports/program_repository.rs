use async_trait::async_trait;
use crate::ports::robot_repository::{PersistenceError, Result};

/// Persistence record for a program.
///
/// `source` is the user-authored .thalos DSL text (source of truth).
/// `program_json` is the serialized `RobotProgram` (parsed AST with targets + body).
/// IR and motion plans are derived artifacts, not persisted as primary entities.
///
/// `module_id` links a program to the `RoboticsModule` that owns it. It is
/// `None` for unassigned programs (a module owns `0..N` programs; a program
/// belongs to `0..1` modules).
///
/// `revision` is an optimistic-concurrency token: it starts at `0` on insert
/// and is bumped by one on every successful `save`.
#[derive(Debug, Clone)]
pub struct ProgramRecord {
    pub id: String,
    pub module_id: Option<String>,
    pub name: String,
    pub source: String,
    pub program_json: String,
    pub revision: i32,
    pub created_at: String,
    pub updated_at: String,
}

/// Port interface for Program persistence.
#[async_trait]
pub trait ProgramRepository: Send + Sync {
    async fn list(&self) -> Result<Vec<ProgramRecord>>;

    /// List every program owned by a robotics module.
    async fn list_by_module(&self, module_id: &str) -> Result<Vec<ProgramRecord>>;

    async fn get(&self, id: &str) -> Result<Option<ProgramRecord>>;

    /// Insert a new program (`revision = 0`) or update an existing one.
    ///
    /// On update, `record.revision` is treated as the *expected* current
    /// revision (compare-and-swap). A mismatch yields
    /// [`PersistenceError::Conflict`]. Returns the persisted record with the
    /// resulting revision (`db.revision + 1`).
    async fn save(&self, record: &ProgramRecord) -> Result<ProgramRecord>;

    async fn delete(&self, id: &str) -> Result<()>;
}
