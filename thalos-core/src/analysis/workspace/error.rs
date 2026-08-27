//! Error types for the workspace analysis module.
//!
//! All workspace-related operations that can fail return `Result<_, WorkspaceError>`.
//! `Reachability` itself contains ONLY domain outcomes (`Reachable` / `OutOfWorkspace`);
//! validation errors flow through this enum (no duplicated error mechanism).

use thiserror::Error;

#[derive(Error, Debug, PartialEq)]
pub enum WorkspaceError {
    #[error("sample count must be > 0, got {0}")]
    InvalidSampleCount(usize),

    #[error("tolerance must be >= 0, got {0}")]
    InvalidTolerance(f64),

    #[error("point has non-finite coordinate: {0}")]
    InvalidPoint(String),

    #[error("workspace is empty")]
    EmptyWorkspace,
}
