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

/// Canonical SHA-256 fingerprint (hex) of a program source.
///
/// This is the single source of truth for "does this source correspond to this
/// program revision?". It MUST stay byte-for-byte consistent with the
/// fingerprint frozen on `ExecutionPlan` provenance at planning time.
pub fn source_fingerprint(source: &str) -> String {
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    hasher.update(source.as_bytes());
    format!("{:x}", hasher.finalize())
}

/// True when an execution snapshot (revision + optional fingerprint) no longer
/// matches the current persisted program, i.e. the program changed since the
/// execution was prepared. A missing fingerprint is treated as "unknown" and
/// does not by itself mark the snapshot stale.
///
/// Reuse this for both `ExecutionPlan` and `ExecutionSession` staleness so the
/// semantics stay identical across the two execution architectures.
pub fn program_snapshot_is_stale(
    snapshot_revision: Option<u64>,
    snapshot_fingerprint: Option<&str>,
    current_revision: u64,
    current_fingerprint: &str,
) -> bool {
    if let Some(rev) = snapshot_revision {
        if rev != current_revision {
            return true;
        }
    }
    if let Some(fp) = snapshot_fingerprint {
        if fp != current_fingerprint {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fingerprint_is_deterministic_and_source_sensitive() {
        let a = source_fingerprint("fn main() { movej(PARK) }");
        assert_eq!(a, source_fingerprint("fn main() { movej(PARK) }"));
        assert_ne!(a, source_fingerprint("fn main() { movel(PARK) }"));
    }

    #[test]
    fn same_revision_and_fingerprint_is_not_stale() {
        let fp = source_fingerprint("src");
        assert!(!program_snapshot_is_stale(Some(1), Some(&fp), 1, &fp));
    }

    #[test]
    fn revision_change_is_stale() {
        let fp = source_fingerprint("src");
        assert!(program_snapshot_is_stale(Some(1), Some(&fp), 2, &fp));
    }

    #[test]
    fn fingerprint_mismatch_at_same_revision_is_stale() {
        let fp1 = source_fingerprint("src-1");
        let fp2 = source_fingerprint("src-2");
        assert!(program_snapshot_is_stale(Some(1), Some(&fp1), 1, &fp2));
    }

    #[test]
    fn unknown_snapshot_fields_are_not_stale_by_themselves() {
        let fp = source_fingerprint("src");
        assert!(!program_snapshot_is_stale(None, None, 7, &fp));
    }
}
