//! Evaluation metrics — re-exported from `thalos_core`.
//!
//! The canonical definitions have moved to `thalos_core::evaluation::metrics`.
//! This module re-exports them for backward compatibility.

pub use thalos_core::evaluation::{
    CollisionMetrics, JointSafetyMetrics, ManipulabilityMetrics, MetricKind, PlanMetrics,
};
