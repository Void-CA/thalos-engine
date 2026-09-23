//! # thalos-analysis
//!
//! Reusable analysis services layered over the [`thalos_core`] analyzers:
//! workspace sampling, singularity, and manipulability.
//!
//! ## Error contract
//!
//! Every fallible operation here fails for exactly one reason class: invalid
//! workspace-sampling input or a sampling failure. The services therefore
//! return [`WorkspaceError`] directly rather than introducing a wrapper type.
//! If the analysis surface later grows independent causes (IK failures,
//! unsupported robots, …), replace this with a dedicated `AnalysisError`.

pub mod comparison;
pub mod manipulability;
pub mod singularity;
pub mod workspace;

pub use comparison::{
    AlignedPair, Alignment, ComparePipeline, ComparePipelineError, ComparePipelineOutput,
    ComparisonMetrics, ConditionOutcome, JointErrorMetrics, PlanExecutionComparison,
    SignalComparison, TickComparison, TracePoint, compare_points, compare_tick,
};
pub use manipulability::ManipulabilityService;
pub use singularity::SingularityService;
pub use workspace::WorkspaceService;

pub use thalos_core::analysis::workspace::WorkspaceError;
