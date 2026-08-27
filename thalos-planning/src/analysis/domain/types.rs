//! Region domain types — re-exported from `thalos_core`.
//!
//! The canonical definitions have moved to `thalos_core::analysis::region`.
//! This module re-exports them for backward compatibility.

pub use thalos_core::analysis::region::{
    ProblemRegion, RegionEvidence, RegionId, RegionKind, RegionSeverity,
};
