//! Derived quality summary of an
//! [`AnalysisReport`](crate::analysis::report::AnalysisReport).
//!
//! [`AnalysisSummary`] is a small **projection** (design C2): a derived view over
//! the report's observations. Changing the scoring policy must never require
//! changing the fundamental observation model — the summary is recomputed by the
//! aggregator, the observations are not.
//!
//! # Invariants
//!
//! - **Single quality measure (I7)**: `quality_index` (range 0..1) is the ONLY
//!   aggregate quality field. No `health_score`, no `summary.score`.
//! - **Grade determinism**: `grade` is a projection of `quality_index` per the
//!   grade mapping (Excellent ≥ 0.9, Good ≥ 0.7, Fair ≥ 0.5, Poor < 0.5). The
//!   mapping itself is defined by the aggregator phase (PR 2a), not here.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::analysis::observation::Severity;

/// Qualitative grade derived from `quality_index` (spec `analysis-score-semantics`).
///
/// `#[non_exhaustive]`: new grades are added without breaking consumers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Grade {
    /// `quality_index >= 0.9`.
    Excellent,
    /// `quality_index >= 0.7`.
    Good,
    /// `quality_index >= 0.5`.
    Fair,
    /// `quality_index < 0.5`.
    Poor,
}

/// Derived quality summary of a report (design C2: small, computed projection).
///
/// # Invariants
///
/// - `quality_index` in 0..=1 is the single aggregate quality measure (I7).
/// - `observation_count` and `severity_distribution` are derived from the
///   report's observations; they are computed by the aggregator, never
///   hand-written by analyzers.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AnalysisSummary {
    /// The single aggregate quality measure, range 0..=1 (I7).
    pub quality_index: f64,
    /// Number of observations in the report.
    pub observation_count: usize,
    /// Count of observations per severity.
    pub severity_distribution: BTreeMap<Severity, usize>,
    /// Qualitative grade derived from `quality_index`.
    pub grade: Grade,
}

#[cfg(test)]
mod tests {
    use super::{AnalysisSummary, Grade};
    use crate::analysis::observation::Severity;
    use serde_json::json;
    use std::collections::BTreeMap;

    fn summary() -> AnalysisSummary {
        AnalysisSummary {
            quality_index: 0.85,
            observation_count: 3,
            severity_distribution: BTreeMap::new(),
            grade: Grade::Good,
        }
    }

    #[test]
    fn summary_serializes_with_expected_shape() {
        // The summary carries the four derived fields; severity keys are the
        // machine-readable Severity enum values (I2, deterministic order).
        let mut s = summary();
        s.severity_distribution.insert(Severity::Error, 1);
        s.severity_distribution.insert(Severity::Warning, 2);
        let value = serde_json::to_value(&s).expect("serialize");
        assert_eq!(value["quality_index"], json!(0.85));
        assert_eq!(value["observation_count"], json!(3));
        assert_eq!(value["grade"], json!("Good"));
        assert_eq!(value["severity_distribution"]["Error"], json!(1));
        assert_eq!(value["severity_distribution"]["Warning"], json!(2));
    }

    #[test]
    fn summary_round_trip() {
        let mut s = summary();
        s.severity_distribution.insert(Severity::Error, 1);
        let json = serde_json::to_string(&s).expect("serialize");
        let back: AnalysisSummary = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(back, s);
    }

    #[test]
    fn summary_has_single_quality_measure() {
        // I7 negative: quality_index is the ONLY aggregate quality field; no
        // health_score, no summary.score.
        let value = serde_json::to_value(summary()).expect("serialize");
        let obj = value.as_object().expect("object");
        for banned in ["health_score", "score"] {
            assert!(
                !obj.contains_key(banned),
                "summary must not carry `{banned}`"
            );
        }
    }

    #[test]
    fn grade_has_four_distinct_variants() {
        // Grade mapping vocabulary from spec analysis-score-semantics: the four
        // grades are distinct projections of quality_index.
        let grades = [Grade::Excellent, Grade::Good, Grade::Fair, Grade::Poor];
        for (i, a) in grades.iter().enumerate() {
            for (j, b) in grades.iter().enumerate() {
                if i != j {
                    assert_ne!(a, b, "grades {i} and {j} must be distinct");
                }
            }
        }
    }
}
