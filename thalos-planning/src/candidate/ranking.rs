//! Ranking types and the DERIVED selection reason (PR2, Phase 3, tasks 3.7 +
//! 3.8; spec candidate-evaluation "CandidateScore — Raw Separated from
//! Normalized" + "SelectionReason — Derived from Metric Differences").
//!
//! # Raw separated from normalized (auditable)
//!
//! [`CandidateScore`] carries the RAW contract values (`risk` from
//! `CandidateAssessment.risk`, `duration`/`manipulability`/`length` from
//! `MotionMetrics`) AND the per-set normalized components (the J
//! contributions) as distinct fields. Raw values are stored verbatim — the
//! evaluator never re-derives a metric from the program.
//!
//! # The reason is DERIVED — never hand-written, never LLM
//!
//! [`derive_selection_reason`] builds [`SelectionReason`] from STRUCTURAL
//! metric differences only: each [`MetricComparison`] carries the fixed
//! component identifier and the two numeric values (selected vs baseline).
//! Direction (`<` / `>`) is derivable from the values by any consumer. The
//! endpoint/task status strings are fixed constants, reflecting that every
//! admissible candidate passed the gate's phase-1 invariants (endpoint ε and
//! task identity) by construction.

use crate::candidate::contract::{Candidate, StrategyTrace};
use crate::candidate::strategy::StrategyKind;

/// "Endpoints: preserved" — fixed phrasing (spec "Reason derived from
/// metrics"). Every admissible candidate passed the gate's phase-1 endpoint-ε
/// invariant against the seed.
pub const ENDPOINTS_PRESERVED: &str = "Endpoints: preserved";

/// "Task: preserved" — fixed phrasing (spec "Reason derived from metrics").
/// Every admissible candidate passed the gate's phase-1 task-identity
/// invariant against the seed.
pub const TASK_PRESERVED: &str = "Task: preserved";

/// The structural no-selection reason (spec "All candidates inadmissible"):
/// no admissible candidate exists, so there is nothing to rank or select.
pub const NO_ADMISSIBLE_CANDIDATE_REASON: &str = "no admissible candidates";

/// Component identifiers for [`MetricComparison`], in comparison order.
pub const COMPONENT_RISK: &str = "risk";
pub const COMPONENT_DURATION: &str = "duration";
pub const COMPONENT_MANIPULABILITY: &str = "manipulability";
pub const COMPONENT_LENGTH: &str = "length";
pub const COMPONENT_COST: &str = "cost";

/// The normalized (per-candidate-set) components of the objective — the J
/// contributions. RELATIVE to the candidate set that produced them
/// (see `objective.rs`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObjectiveComponents {
    /// Normalized risk, `norm(risk)` over the candidate set.
    pub risk: f64,
    /// Normalized duration, `norm(duration)` over the candidate set.
    pub duration: f64,
    /// Normalized LOW-manipulability, `norm(1 − avg_manipulability)` over the
    /// candidate set (M is low-manipulability per the J formula).
    pub manipulability: f64,
    /// Normalized path length, `norm(path_length)` over the candidate set.
    pub length: f64,
}

/// One candidate's score: raw contract values separated from the normalized
/// J contributions (spec "CandidateScore — Raw Separated from Normalized").
#[derive(Debug, Clone, PartialEq)]
pub struct CandidateScore {
    /// The strategy that produced this candidate (projection convenience —
    /// mirrors `Candidate.strategy`).
    pub strategy: StrategyKind,
    /// RAW risk — verbatim from `CandidateAssessment.risk` (the Assessor's
    /// crisp `1 − quality`). The evaluator never re-derives it.
    pub risk: f64,
    /// RAW duration (seconds) — verbatim from `MotionMetrics.duration`.
    pub duration: f64,
    /// RAW average manipulability — verbatim from
    /// `MotionMetrics.avg_manipulability`. Higher is better; the objective
    /// uses its complement (low-manipulability) as the M component.
    pub manipulability: f64,
    /// RAW path length (metres) — verbatim from `MotionMetrics.path_length`.
    pub length: f64,
    /// Normalized components — the J contributions (per-candidate-set).
    pub normalized: ObjectiveComponents,
    /// The objective value `J = Σ w_i · normalized_i` for this candidate.
    pub cost: f64,
}

/// One row of the derived metric comparison: a fixed component identifier and
/// the selected candidate's value vs the baseline `Direct`'s value. Direction
/// (`<` / `>`) is derivable from the values — no narrative text.
#[derive(Debug, Clone, PartialEq)]
pub struct MetricComparison {
    /// Fixed component identifier: `risk` | `duration` | `manipulability` |
    /// `length` | `cost`.
    pub component: String,
    /// The selected candidate's value for this component.
    pub selected_value: f64,
    /// The baseline `Direct` candidate's value for this component.
    pub baseline_value: f64,
}

/// The full ranking outcome: the ranked admissible candidates, the selected
/// candidate (argmin J — the first ranked entry), the derived reason, and the
/// FULL strategy trace (design ADR-3 observability — verify Warning 1 FIX).
///
/// The trace makes the ranking a SELF-CONTAINED unit: every strategy that was
/// applied (`Generated` or `Skipped(reason)`) travels with it, so the
/// consumer (runtime → DTO) can render `Direct → Generated`,
/// `InsertWaypoint → Skipped — UnsupportedSegment` without inventing anything.
#[derive(Debug, Clone, PartialEq)]
pub struct CandidateRanking {
    /// All admissible candidates with their scores, ordered by ascending
    /// cost (stable — ties keep input order, favoring the `Direct` baseline
    /// that the runtime places first).
    pub ranked: Vec<(Candidate, CandidateScore)>,
    /// The selected candidate: `ranked[0]` when any candidate is admissible.
    pub selected: Option<Candidate>,
    /// The derived selection reason.
    pub reason: SelectionReason,
    /// The FULL strategy trace from generation: every strategy applied, with
    /// `Generated` or `Skipped(reason)` — including strategies that produced
    /// no admissible candidate. Carried verbatim from the generator.
    pub strategy_trace: Vec<StrategyTrace>,
}

/// Why the ranking selected what it selected — DERIVED from metric
/// differences, never hand-written text, never LLM output (spec
/// "SelectionReason — Derived from Metric Differences").
#[derive(Debug, Clone, PartialEq)]
pub enum SelectionReason {
    /// A candidate was selected.
    Selected {
        /// The selected strategy.
        strategy: StrategyKind,
        /// Structural metric differences between the selected candidate and
        /// the baseline `Direct` (risk, duration, manipulability, length,
        /// cost). EMPTY when `Direct` was not admissible — there is no
        /// baseline to derive differences against.
        metric_comparison: Vec<MetricComparison>,
        /// Always `"Endpoints: preserved"` — every admissible candidate
        /// passed the phase-1 endpoint-ε invariant.
        endpoints: &'static str,
        /// Always `"Task: preserved"` — every admissible candidate passed the
        /// phase-1 task-identity invariant.
        task: &'static str,
    },
    /// No candidate was admissible (spec "All candidates inadmissible") —
    /// there is no selection and no fallback to the "least bad" candidate.
    NoAdmissibleCandidate {
        /// The structural reason, fixed constant.
        reason: &'static str,
    },
}

/// Derive the [`SelectionReason`] from the ranked list — a PURE function so
/// the derivation is testable without the evaluator.
///
/// The ranked list is expected to be ordered by ascending cost (the
/// evaluator's contract), so the selected candidate is the FIRST entry. The
/// baseline is the entry with `StrategyKind::Direct`; metric differences are
/// computed against it for the fixed component order `[risk, duration,
/// manipulability, length, cost]`.
pub fn derive_selection_reason(ranked: &[(Candidate, CandidateScore)]) -> SelectionReason {
    let Some((_, selected_score)) = ranked.first() else {
        return SelectionReason::NoAdmissibleCandidate {
            reason: NO_ADMISSIBLE_CANDIDATE_REASON,
        };
    };

    let baseline = ranked
        .iter()
        .find(|(candidate, _)| candidate.strategy == StrategyKind::Direct);

    let metric_comparison = match baseline {
        Some((_, baseline_score)) => {
            let pairs = [
                (COMPONENT_RISK, selected_score.risk, baseline_score.risk),
                (
                    COMPONENT_DURATION,
                    selected_score.duration,
                    baseline_score.duration,
                ),
                (
                    COMPONENT_MANIPULABILITY,
                    selected_score.manipulability,
                    baseline_score.manipulability,
                ),
                (
                    COMPONENT_LENGTH,
                    selected_score.length,
                    baseline_score.length,
                ),
                (COMPONENT_COST, selected_score.cost, baseline_score.cost),
            ];
            pairs
                .into_iter()
                .map(
                    |(component, selected_value, baseline_value)| MetricComparison {
                        component: component.to_string(),
                        selected_value,
                        baseline_value,
                    },
                )
                .collect()
        }
        // Direct was not admissible (rejected by the risk policy) — no
        // baseline to derive differences against.
        None => Vec::new(),
    };

    SelectionReason::Selected {
        strategy: selected_score.strategy,
        metric_comparison,
        endpoints: ENDPOINTS_PRESERVED,
        task: TASK_PRESERVED,
    }
}

#[cfg(test)]
mod tests {
    use crate::candidate::contract::{
        Candidate, NoCandidateReason, StrategyOutcome, StrategyTrace,
    };
    use crate::candidate::strategy::StrategyKind;
    use crate::motion::program::PlanningProgram;

    use super::*;

    fn score(
        strategy: StrategyKind,
        risk: f64,
        duration: f64,
        manipulability: f64,
        length: f64,
        cost: f64,
    ) -> (Candidate, CandidateScore) {
        let candidate = Candidate {
            strategy,
            program: PlanningProgram::new(vec![]),
        };
        let score = CandidateScore {
            strategy,
            risk,
            duration,
            manipulability,
            length,
            normalized: ObjectiveComponents {
                risk,
                duration,
                manipulability,
                length,
            },
            cost,
        };
        (candidate, score)
    }

    // ── 3.7 — CandidateScore: raw separated from normalized (auditable) ──

    #[test]
    fn candidate_score_keeps_raw_metrics_separate_from_normalized_components() {
        // Spec "Raw and normalized separated": the score carries the raw
        // contract values AND the normalized components as distinct fields.
        let (_, s) = score(StrategyKind::Direct, 0.557, 3.2, 0.458, 1.8, 0.85);
        assert!(
            (s.risk - 0.557).abs() < 1e-12,
            "raw risk must be stored verbatim"
        );
        assert!((s.duration - 3.2).abs() < 1e-12);
        assert!((s.manipulability - 0.458).abs() < 1e-12);
        assert!((s.length - 1.8).abs() < 1e-12);
        // The normalized components are a SEPARATE field (the J contributions)
        // — they must exist and be distinguishable from the raw values.
        assert!((s.normalized.risk - 0.557).abs() < 1e-12);
        assert!((s.cost - 0.85).abs() < 1e-12);
    }

    #[test]
    fn objective_components_carries_all_four_normalized_values() {
        let components = ObjectiveComponents {
            risk: 0.1,
            duration: 0.4,
            manipulability: 0.7,
            length: 0.9,
        };
        assert!((components.risk - 0.1).abs() < 1e-12);
        assert!((components.duration - 0.4).abs() < 1e-12);
        assert!((components.manipulability - 0.7).abs() < 1e-12);
        assert!((components.length - 0.9).abs() < 1e-12);
    }

    // ── 3.7 — SelectionReason derived from metric differences ────────────

    #[test]
    fn selected_reason_derives_metric_comparison_vs_the_direct_baseline() {
        // Spec "Reason derived from metrics": the reason is DERIVED from the
        // metric differences between the selected candidate and the baseline
        // Direct — risk, duration, manipulability, length, AND cost.
        let ranked = vec![
            score(StrategyKind::AlternateElbow, 0.182, 2.1, 0.7, 1.2, 0.15),
            score(StrategyKind::Direct, 0.557, 3.2, 0.458, 1.8, 0.85),
        ];

        let reason = derive_selection_reason(&ranked);

        match reason {
            SelectionReason::Selected {
                strategy,
                metric_comparison,
                ..
            } => {
                assert_eq!(strategy, StrategyKind::AlternateElbow);
                assert_eq!(
                    metric_comparison.len(),
                    5,
                    "risk+duration+manipulability+length+cost"
                );
                let by_component: std::collections::HashMap<&str, &MetricComparison> =
                    metric_comparison
                        .iter()
                        .map(|m| (m.component.as_str(), m))
                        .collect();
                // Risk: 0.182 < 0.557
                let risk = by_component["risk"];
                assert!((risk.selected_value - 0.182).abs() < 1e-12);
                assert!((risk.baseline_value - 0.557).abs() < 1e-12);
                // Duration: 2.1 < 3.2
                let duration = by_component["duration"];
                assert!((duration.selected_value - 2.1).abs() < 1e-12);
                assert!((duration.baseline_value - 3.2).abs() < 1e-12);
                // Manipulability (raw): 0.7 > 0.458 — direction derived from values
                let manip = by_component["manipulability"];
                assert!((manip.selected_value - 0.7).abs() < 1e-12);
                assert!((manip.baseline_value - 0.458).abs() < 1e-12);
                // Length: 1.2 < 1.8
                let length = by_component["length"];
                assert!((length.selected_value - 1.2).abs() < 1e-12);
                assert!((length.baseline_value - 1.8).abs() < 1e-12);
                // Cost: 0.15 < 0.85 — the objective value is part of the reason
                let cost = by_component["cost"];
                assert!((cost.selected_value - 0.15).abs() < 1e-12);
                assert!((cost.baseline_value - 0.85).abs() < 1e-12);
            }
            other => panic!("expected Selected, got {other:?}"),
        }
    }

    #[test]
    fn selected_reason_includes_endpoints_and_task_preserved_phrasing() {
        // Spec "Reason derived from metrics": the reason SHALL include
        // "Endpoints: preserved" and "Task: preserved".
        let ranked = vec![
            score(StrategyKind::AlternateElbow, 0.182, 2.1, 0.7, 1.2, 0.15),
            score(StrategyKind::Direct, 0.557, 3.2, 0.458, 1.8, 0.85),
        ];

        let reason = derive_selection_reason(&ranked);

        match reason {
            SelectionReason::Selected {
                endpoints, task, ..
            } => {
                assert_eq!(endpoints, "Endpoints: preserved");
                assert_eq!(task, "Task: preserved");
            }
            other => panic!("expected Selected, got {other:?}"),
        }
    }

    #[test]
    fn selected_reason_is_structural_data_not_hand_written_text() {
        // The reason must NOT contain narrative or LLM output — only the
        // structural comparison rows with fixed component identifiers.
        let ranked = vec![
            score(StrategyKind::InsertWaypoint, 0.3, 1.0, 0.8, 0.9, 0.4),
            score(StrategyKind::Direct, 0.557, 3.2, 0.458, 1.8, 0.85),
        ];

        let reason = derive_selection_reason(&ranked);

        match reason {
            SelectionReason::Selected {
                metric_comparison, ..
            } => {
                let components: Vec<&str> = metric_comparison
                    .iter()
                    .map(|m| m.component.as_str())
                    .collect();
                assert_eq!(
                    components,
                    vec!["risk", "duration", "manipulability", "length", "cost"],
                    "comparison components are the fixed, ordered identifiers"
                );
            }
            other => panic!("expected Selected, got {other:?}"),
        }
    }

    #[test]
    fn selected_reason_without_direct_baseline_has_empty_comparison() {
        // When the Direct baseline is NOT admissible (rejected by the risk
        // policy), there is no baseline to derive differences against — the
        // comparison is empty and the reason still identifies the strategy
        // and the preservation constants.
        let ranked = vec![
            score(StrategyKind::AlternateElbow, 0.182, 2.1, 0.7, 1.2, 0.15),
            score(StrategyKind::InsertWaypoint, 0.25, 2.5, 0.6, 1.4, 0.3),
        ];

        let reason = derive_selection_reason(&ranked);

        match reason {
            SelectionReason::Selected {
                strategy,
                metric_comparison,
                endpoints,
                task,
            } => {
                assert_eq!(strategy, StrategyKind::AlternateElbow);
                assert!(metric_comparison.is_empty());
                assert_eq!(endpoints, "Endpoints: preserved");
                assert_eq!(task, "Task: preserved");
            }
            other => panic!("expected Selected, got {other:?}"),
        }
    }

    #[test]
    fn no_admissible_candidate_reason_is_structural() {
        // All candidates inadmissible → the reason must indicate no admissible
        // candidate, with NO selection and NO metric comparison.
        let reason = derive_selection_reason(&[]);

        match reason {
            SelectionReason::NoAdmissibleCandidate { reason } => {
                assert_eq!(reason, "no admissible candidates");
            }
            other => panic!("expected NoAdmissibleCandidate, got {other:?}"),
        }
    }

    // ── 3.7 — CandidateRanking structure ─────────────────────────────────

    #[test]
    fn ranking_holds_ranked_list_selected_candidate_and_reason() {
        let (selected_candidate, selected_score) =
            score(StrategyKind::AlternateElbow, 0.182, 2.1, 0.7, 1.2, 0.15);
        let (baseline_candidate, baseline_score) =
            score(StrategyKind::Direct, 0.557, 3.2, 0.458, 1.8, 0.85);
        let ranked = vec![
            (selected_candidate.clone(), selected_score),
            (baseline_candidate.clone(), baseline_score),
        ];
        let reason = derive_selection_reason(&ranked);
        let ranking = CandidateRanking {
            ranked: ranked.clone(),
            selected: Some(selected_candidate.clone()),
            reason,
            strategy_trace: Vec::new(),
        };

        assert_eq!(ranking.ranked.len(), 2);
        assert_eq!(ranking.ranked[0].0.strategy, StrategyKind::AlternateElbow);
        assert_eq!(ranking.ranked[1].0.strategy, StrategyKind::Direct);
        assert_eq!(ranking.selected, Some(selected_candidate));
        assert!(matches!(ranking.reason, SelectionReason::Selected { .. }));
        assert_eq!(
            ranking.selected.as_ref().unwrap().strategy,
            StrategyKind::AlternateElbow
        );
    }

    // ── REMEDIATION (verify Warning 1 FIX, ADR-3 observability) — the
    //    ranking carries the full strategy trace ────────────────────────────

    #[test]
    fn ranking_carries_the_full_strategy_trace() {
        // The ranking is the SELF-CONTAINED pipeline outcome: the full
        // strategy trace (every strategy → Generated/Skipped) travels with it
        // so the consumer (runtime → DTO) can render `Direct → Generated`,
        // `InsertWaypoint → Skipped — UnsupportedSegment` without inventing
        // anything (design ADR-3 "surfaced in the DTO").
        let (candidate, score) = score(StrategyKind::Direct, 0.557, 7.8, 0.458, 3.885, 1.0);
        let trace = vec![
            StrategyTrace {
                strategy: StrategyKind::Direct,
                outcome: StrategyOutcome::Generated(candidate.clone()),
            },
            StrategyTrace {
                strategy: StrategyKind::InsertWaypoint,
                outcome: StrategyOutcome::Skipped(NoCandidateReason::UnsupportedSegment),
            },
        ];
        let ranking = CandidateRanking {
            ranked: vec![(candidate, score)],
            selected: None,
            reason: SelectionReason::NoAdmissibleCandidate {
                reason: NO_ADMISSIBLE_CANDIDATE_REASON,
            },
            strategy_trace: trace.clone(),
        };

        assert_eq!(
            ranking.strategy_trace, trace,
            "the ranking must carry the trace verbatim"
        );
        assert_eq!(ranking.strategy_trace.len(), 2);
        assert_eq!(ranking.strategy_trace[0].strategy, StrategyKind::Direct);
        assert!(matches!(
            ranking.strategy_trace[0].outcome,
            StrategyOutcome::Generated(_)
        ));
        assert_eq!(
            ranking.strategy_trace[1].strategy,
            StrategyKind::InsertWaypoint
        );
        assert!(matches!(
            ranking.strategy_trace[1].outcome,
            StrategyOutcome::Skipped(NoCandidateReason::UnsupportedSegment)
        ));
    }
}
