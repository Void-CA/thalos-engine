//! # thalos-semantic
//!
//! Task-level programming model for Thalos. Defines `SemanticProgram` —
//! a linear sequence of logical operations that represent *what* the robot
//! should achieve, independent of geometry, constraints, or motion planning.
//!
//! ## Pipeline
//!
//! ```text
//! SemanticProgram
//!     │
//!     ├─→ Validation (Level 1: sequence + Level 2: resource resolution)
//!     │
//!     └─→ SemanticLowering → ExecutionProgram (via KnowledgeProvider)
//! ```
//!
//! ## Module Structure
//!
//! - `resource` — Logical resource identifiers (`ObjectId`, `LocationId`, `ToolId`)
//! - `operation` — `SemanticOperation` enum with five variants
//! - `program` — `SemanticProgram` container with ordered operations
//! - `knowledge` — `KnowledgeProvider` trait, `GraspPlan`/`PlacementPlan` types, `LoweringError`, `MockKnowledgeProvider`
//! - `lowering` — `SemanticLowering::lower()` and `LoweringContext`
//! - `validation` — Two-level validation pipeline (Level 1: sequence rules, Level 2: resource resolution)

pub mod knowledge;
pub mod lowering;
pub mod operation;
pub mod program;
pub mod resource;
pub mod script;
pub mod validation;

/// Shared helpers for the canonical semantic scenario (feature `test-support`).
///
/// Single source of truth for the `Pick → Wait → Place → Home` program used by
/// the pipeline regression tests (`ir_properties.rs`, `e2e_canonical_pipeline.rs`,
/// `thalos-runtime/tests/e2e_execution.rs`). Test-only; never referenced by
/// production code.
#[cfg(feature = "test-support")]
pub mod test_support;
