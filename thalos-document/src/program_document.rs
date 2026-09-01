use serde::{Deserialize, Serialize};

use crate::id::ProgramDocumentId;
use crate::scene::SceneContent;
use thalos_engine::semantic::program::SemanticProgram;

// ---------------------------------------------------------------------------
// Metadata — document identity and versioning
// ---------------------------------------------------------------------------

/// Document identity and versioning — describes the document itself.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Metadata {
    /// Human-readable project name.
    pub name: String,
    /// Monotonically increasing document version.
    pub version: u32,
    /// ISO 8601 creation timestamp.
    pub created_at: String,
    /// ISO 8601 last-modified timestamp.
    pub modified_at: String,
}

// ---------------------------------------------------------------------------
// ProgramDocument — the top-level document for task-level programming
// ---------------------------------------------------------------------------

/// A complete task document for task-level programming.
///
/// Contains a unique identity (`ProgramDocumentId`), document metadata
/// (name, version, timestamps), the logical scene model (`SceneContent`),
/// and a semantic program that references scene resources by ID.
///
/// This is the input to the semantic lowering pipeline.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProgramDocument {
    /// Unique document identifier.
    pub id: ProgramDocumentId,
    /// Document metadata (name, version, timestamps).
    pub metadata: Metadata,
    /// The logical scene model (objects, locations, tools, home pose).
    pub scene: SceneContent,
    /// The semantic program referencing scene resources by ID.
    pub program: SemanticProgram,
}
