//! Evaluation domain types — metrics, scoring, and cost functions.
//!
//! This module defines the measurement primitives used across planning
//! and optimization subsystems. Types are robot-agnostic and belong
//! in `thalos-core` to avoid circular dependencies.

pub mod metrics;

pub use metrics::{
    CollisionMetrics, JointSafetyMetrics, ManipulabilityMetrics, MetricKind, PlanMetrics,
};
