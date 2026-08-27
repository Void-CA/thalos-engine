//! Workspace analysis module.
//!
//! Re-exports the public types for ergonomic use:
//! ```ignore
//! use thalos_core::analysis::workspace::{Workspace, WorkspaceConfig, ...};
//! ```

pub mod config;
pub mod error;
pub mod reachability;
pub mod sampler;
pub mod types;
pub mod workspace;

#[cfg(test)]
pub mod tests;

pub use config::WorkspaceConfig;
pub use error::WorkspaceError;
pub use reachability::Reachability;
pub use sampler::WorkspaceSampler;
pub use types::{BoundingBox, WorkspaceKey, WorkspaceMetrics, WorkspaceSample};
pub use workspace::Workspace;
