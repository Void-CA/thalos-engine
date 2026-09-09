use async_trait::async_trait;
use crate::ports::robot_repository::{PersistenceError, Result};

/// Persistence record for a program.
///
/// `source` is the user-authored .thalos DSL text (source of truth).
/// `program_json` is the serialized `RobotProgram` (parsed AST with targets + body).
/// IR and motion plans are derived artifacts, not persisted as primary entities.
#[derive(Debug, Clone)]
pub struct ProgramRecord {
    pub id: String,
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
    async fn get(&self, id: &str) -> Result<Option<ProgramRecord>>;
    async fn save(&self, record: &ProgramRecord) -> Result<()>;
    async fn delete(&self, id: &str) -> Result<()>;
}
