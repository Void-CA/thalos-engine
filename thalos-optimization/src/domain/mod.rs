//! Domain types for trajectory optimization.
//!
//! This module contains the core domain model — operator traits,
//! scoring, assessment, context, and reports. It is robot-agnostic
//! and depends only on `thalos_core` types.

pub mod assessment;
pub mod context;
pub mod operator;
pub mod report;
pub mod score;

pub use assessment::{OperatorAssessment, OperatorScore, Reason};
pub use context::{JointLimits, OptimizationContext, PipelineConfig};
pub use operator::{Invariant, OperatorFamily, OptimizationObjective, TrajectoryOperator};
pub use report::{OptimizationReport, OptimizationStep};
