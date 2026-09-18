//! # thalos-telemetry
//!
//! Execution evidence and observation semantics extracted from Thalos
//! Industrial (ADR-020, F2.1b).
//!
//! Owns what **represents/transforms evidence independently of how it is
//! obtained**: execution traces and samples, motion traces, the trace analyzer,
//! and the lifecycle event vocabulary. It does **not** own acquisition,
//! recording mechanisms, persistence, or the runtime live state.
//!
//! The acquisition→evidence boundary is `thalos_core::robot::RobotObservation`
//! plus a runtime-provided execution context (ADR-020).

pub mod analyzer;
pub mod event;
pub mod motion;
pub mod trace;

pub use analyzer::{ExecutionStatistics, TraceAnalyzer};
pub use event::TelemetryLifecycleEvent;
pub use motion::trace::{MotionSample, MotionTrace};
pub use trace::{ExecutionSample, ExecutionTrace, TraceMetadata};
