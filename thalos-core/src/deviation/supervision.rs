//! Expected-vs-observed **deviation** vocabulary of a supervision tick (A5).
//!
//! Moved here from `thalos-analysis` so the engine's execution-event vocabulary
//! (`ExecutionEvent::DeviationDetected`) can carry deviation evidence directly,
//! without `thalos-core::execution` depending on `thalos-analysis`.
//!
//! This is evidence, not a response: there is no decision, priority, severity,
//! policy or reaction here.
//!
//! The three comparison outcomes are preserved:
//!
//! | Comparison outcome | Deviation |
//! |---|---|
//! | `Satisfied` | none |
//! | `Violated`  | deviation / violation |
//! | `Unknown`   | **none** — the evidence does not demonstrate a violation |
//!
//! Collapsing `Unknown` into a deviation would violate NFR-INT-01 through
//! another route, so it is forbidden by construction.
//!
//! The two semantics are kept distinct under a common abstraction — *an
//! expectation that was not met* — while preserving the specific evidence that
//! showed it (kinematic metrics vs the declared condition + observation).

use serde::{Deserialize, Serialize};

use crate::device::ChannelObservation;
use crate::deviation::EnvelopeStatus;
use crate::execution::SignalCondition;

/// A kinematic expectation not met, with the metrics that showed it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct KinematicViolation {
    pub index: u64,
    pub timestamp_ns: u64,
    pub max_abs_error: f64,
    pub rmse: f64,
    /// Per-joint max absolute error (rad).
    pub per_joint_max_error: Vec<f64>,
    /// Per-joint tolerance used to declare the violation.
    pub per_joint_tolerance: Vec<f64>,
    pub envelope: EnvelopeStatus,
}

/// A DECLARED signal condition that was violated, with its observation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SignalViolation {
    pub index: u64,
    pub timestamp_ns: u64,
    pub condition: SignalCondition,
    pub observation: ChannelObservation,
}

/// An expectation that was not met. The variant preserves the specific
/// evidence; the two semantics are not artificially unified.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Deviation {
    Kinematic(KinematicViolation),
    Signal(SignalViolation),
}

/// Deviations detected for one tick.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TickDeviation {
    pub index: u64,
    pub timestamp_ns: u64,
    pub deviations: Vec<Deviation>,
}

impl TickDeviation {
    /// Whether the evidence demonstrated any unfulfilled expectation.
    pub fn is_empty(&self) -> bool {
        self.deviations.is_empty()
    }
}
