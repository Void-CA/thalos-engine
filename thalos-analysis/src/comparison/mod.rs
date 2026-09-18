//! Plan-vs-execution comparison: mechanism-independent analysis over
//! [`TracePoint`] sequences.
//!
//! Moved here from `thalos-runtime` (C22b): it depends only on `thalos-core`
//! and pure utilities, never on runtime evidence mechanisms
//! (`MotionTrace`, `ExecutionTrace`, telemetry or execution sessions). The
//! runtime keeps only the adapters that build `TracePoint`s from its traces.

pub mod alignment;
pub mod comparison;
pub mod input;
pub mod metrics;
pub mod pipeline;

pub use alignment::{AlignedPair, Alignment};
pub use comparison::{PlanExecutionComparison, compare_points};
pub use input::TracePoint;
pub use metrics::{ComparisonMetrics, JointErrorMetrics, compute_metrics};
pub use pipeline::{ComparePipeline, ComparePipelineError, ComparePipelineOutput};
