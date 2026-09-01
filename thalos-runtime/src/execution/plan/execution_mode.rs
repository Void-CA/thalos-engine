//! Execution mode of a session (R1): a single run (`Once`, the default) or an
//! automated repeat of the same plan (`Repeat { count }`) used for validation
//! and stress testing.
//!
//! Wire format (externally-tagged serde, lowercase — decision #1 of the
//! apply-progress):
//! - `Once`              → `"once"`
//! - `Repeat { count }`  → `{"repeat":{"count":5}}`

use serde::{Deserialize, Serialize};

/// How many times a session executes its plan (R1).
///
/// `Once` is the default (legacy behavior — an absent `mode` everywhere
/// deserializes to `Once` via `#[serde(default)]`). `Repeat { count }`
/// re-executes the plan `count` times within a single session; the API
/// enforces the `1..=1000` wire bound (R9) before the runtime sees it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ExecutionMode {
    /// A single execution.
    Once,
    /// Re-execute the plan `count` times in one session.
    Repeat { count: u32 },
}

impl Default for ExecutionMode {
    fn default() -> Self {
        ExecutionMode::Once
    }
}

impl ExecutionMode {
    /// Total number of iterations this mode runs — `None` for `Once`
    /// (the iteration badge is hidden without it, EW6).
    pub fn total_iterations(&self) -> Option<u32> {
        match self {
            ExecutionMode::Once => None,
            ExecutionMode::Repeat { count } => Some(*count),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── R1 / S6 — serde round-trip ──

    #[test]
    fn serde_round_trip_once() {
        let mode: ExecutionMode = serde_json::from_str("\"once\"").expect("once must deserialize");
        assert_eq!(mode, ExecutionMode::Once);
        let json = serde_json::to_string(&mode).expect("once must serialize");
        assert_eq!(json, "\"once\"");
    }

    #[test]
    fn serde_round_trip_repeat() {
        let mode: ExecutionMode =
            serde_json::from_str("{\"repeat\":{\"count\":5}}").expect("repeat must deserialize");
        assert_eq!(mode, ExecutionMode::Repeat { count: 5 });
        let json = serde_json::to_string(&mode).expect("repeat must serialize");
        assert_eq!(json, "{\"repeat\":{\"count\":5}}");
    }

    #[test]
    fn serde_deserializes_repeat_count_one() {
        // R2: Repeat { count: 1 } is a legal (if equivalent) mode.
        let mode: ExecutionMode =
            serde_json::from_str("{\"repeat\":{\"count\":1}}").expect("count 1 must deserialize");
        assert_eq!(mode, ExecutionMode::Repeat { count: 1 });
    }

    // ── total_iterations() ──

    #[test]
    fn total_iterations_once_is_none() {
        assert_eq!(ExecutionMode::Once.total_iterations(), None);
    }

    #[test]
    fn total_iterations_repeat_is_count() {
        assert_eq!(
            ExecutionMode::Repeat { count: 5 }.total_iterations(),
            Some(5)
        );
    }

    #[test]
    fn default_is_once() {
        let mode = ExecutionMode::default();
        assert_eq!(mode, ExecutionMode::Once);
    }
}
