//! Neutral analytical input.
//!
//! The comparison pipeline must not depend on the concrete evidence
//! mechanisms (`MotionTrace` legacy recording, `ExecutionTrace` telemetry).
//! It consumes this minimal, mechanism-independent sample instead; each
//! mechanism provides an adapter from its own representation.

use std::time::Duration;

/// Minimal trajectory point consumed by plan-vs-execution analysis.
///
/// Fields are exactly what alignment and metrics need: the timestamp, the
/// joint positions, the joint velocities and (optionally) the tracking error
/// reported by the execution.
#[derive(Debug, Clone, PartialEq)]
pub struct TracePoint {
    pub timestamp: Duration,
    pub joints: Vec<f64>,
    pub velocities: Vec<f64>,
    /// Error reported by the execution for this point, when available.
    pub tracking_error: Option<f64>,
}

impl TracePoint {
    pub fn new(
        timestamp: Duration,
        joints: Vec<f64>,
        velocities: Vec<f64>,
        tracking_error: Option<f64>,
    ) -> Self {
        Self {
            timestamp,
            joints,
            velocities,
            tracking_error,
        }
    }
}
