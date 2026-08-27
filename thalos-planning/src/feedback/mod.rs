//! Execution feedback types for the planning feedback loop.
//!
//! This module introduces observation types from execution traces,
//! intention operators for transforming motion segments, and an
//! orchestrator that coordinates the full feedback cycle.
//!
//! ## Layering
//!
//! - `operator` — transformation layer (PR 2): applies intention operators
//! - `materializer` — remediation layer (PR 4d): proposal → plan modifications

pub mod materializer;
pub mod operator;
