//! CandidateEvaluator (PR2, Phase 3, tasks 3.5 + 3.6; design ADR-2/ADR-5,
//! spec candidate-evaluation "Objective Function J(c)" + "Selection — argmin
//! Over Admissible").
//!
//! # The evaluator consumes the NEUTRAL contract — nothing else
//!
//! [`CandidateEvaluator::evaluate`] takes `&[AdmissibleCandidate]` — the
//! OUTPUT of the admissibility gate — and reads ONLY:
//!
//! - `assessment.risk` — the Assessor's crisp `1 − quality`, verbatim. The
//!   evaluator NEVER re-derives risk, NEVER reinterprets it as a probability,
//!   and NEVER applies a numeric threshold (there is no `risk >= 0.75` here —
//!   the categorical `Critical` verdict was already mapped to
//!   [`RiskAdmissibility::Rejected`] by the runtime, and the gate already
//!   excluded those candidates).
//! - `metrics.duration` / `metrics.avg_manipulability` /
//!   `metrics.path_length` — verbatim, from the analyzed trajectory + report.
//!
//! The evaluator NEVER computes a metric from the program: the only
//! transformation applied is the OBJECTIVE's orientation — `M =
//! 1 − avg_manipulability` (low-manipulability) per the J formula — followed
//! by per-candidate-set min-max normalization.
//!
//! # argmin over admissible only
//!
//! The input IS the admissible set (gate output); ranking and selection are
//! argmin J over exactly that set. A rejected candidate is structurally
//! unable to be selected — it is not in the input.
//!
//! # J is RELATIVE to the candidate set (see `objective.rs`)
//!
//! Normalization is per-candidate-set: adding a candidate shifts the norms.
//! J answers "best fit within THIS alternative set", never an absolute score.

use crate::candidate::contract::{AdmissibleCandidate, Candidate, StrategyTrace};
use crate::candidate::objective::{ObjectiveProfile, normalize_min_max};
use crate::candidate::ranking::{
    CandidateRanking, CandidateScore, ObjectiveComponents, derive_selection_reason,
};

/// The evaluation entry point: compute J for every admissible candidate,
/// rank by ascending cost, select argmin J, and derive the selection reason.
pub struct CandidateEvaluator;

impl CandidateEvaluator {
    /// Evaluate the admissible candidates under `profile`.
    ///
    /// - Extracts the raw components from the neutral contract (risk,
    ///   duration, low-manipulability `1 − avg_manipulability`, length).
    /// - Normalizes each component per-candidate-set (min-max, tie → 0.5).
    /// - Computes `J = Σ w_i · norm_i` with the profile's weights.
    /// - Ranks by ascending J (STABLE — equal costs keep input order, so the
    ///   `Direct` baseline that the runtime places first wins ties).
    /// - Selects `ranked[0]` and derives the reason vs the baseline `Direct`.
    /// - Carries the FULL strategy trace (every strategy → Generated/Skipped)
    ///   through to the ranking — the evaluator is PURE: it stores the trace
    ///   verbatim and never interprets it (ADR-3 observability).
    ///
    /// An empty input (all candidates inadmissible) yields
    /// `SelectionReason::NoAdmissibleCandidate` — never a "least bad" pick.
    pub fn evaluate(
        candidates: &[AdmissibleCandidate],
        profile: ObjectiveProfile,
        strategy_trace: Vec<StrategyTrace>,
    ) -> CandidateRanking {
        let mut scored = build_scored(candidates, profile);
        // Ascending J, stable: equal costs keep the input order (the Direct
        // baseline is candidate 0 in the runtime flow → ties favor it).
        scored.sort_by(|(_, a), (_, b)| a.cost.total_cmp(&b.cost));

        let selected = scored.first().map(|(candidate, _)| candidate.clone());
        let reason = derive_selection_reason(&scored);

        CandidateRanking {
            ranked: scored,
            selected,
            reason,
            strategy_trace,
        }
    }
}

/// Extract raw components, normalize per-candidate-set, and compute J.
fn build_scored(
    candidates: &[AdmissibleCandidate],
    profile: ObjectiveProfile,
) -> Vec<(Candidate, CandidateScore)> {
    if candidates.is_empty() {
        return Vec::new();
    }

    // Raw components per candidate: [risk, duration, low_manip, length].
    // SOURCES: risk → CandidateAssessment (the Assessor's crisp output,
    // verbatim); duration / avg_manipulability / path_length → MotionMetrics
    // (the analyzed trajectory + report). The evaluator never computes a
    // metric from the program; `low_manip = 1 − avg_manipulability` is the
    // OBJECTIVE's orientation (M in the J formula), not a new metric.
    let raw: Vec<[f64; 4]> = candidates
        .iter()
        .map(|ac| {
            [
                ac.assessment.risk,
                ac.metrics.duration,
                1.0 - ac.metrics.avg_manipulability,
                ac.metrics.path_length,
            ]
        })
        .collect();

    // Per-candidate-set min-max normalization, one component at a time
    // (tie → 0.5, ADR-2). J is RELATIVE to this set.
    let weights = profile.weights();
    let mut normalized = vec![[0.0_f64; 4]; candidates.len()];
    for component in 0..4 {
        let column: Vec<f64> = raw.iter().map(|r| r[component]).collect();
        let norms = normalize_min_max(&column);
        for (i, norm) in norms.into_iter().enumerate() {
            normalized[i][component] = norm;
        }
    }

    candidates
        .iter()
        .zip(raw.iter())
        .zip(normalized.iter())
        .map(|((ac, raw_values), norms)| {
            let cost = weights[0] * norms[0]
                + weights[1] * norms[1]
                + weights[2] * norms[2]
                + weights[3] * norms[3];
            let candidate = ac.candidate.clone();
            let score = CandidateScore {
                strategy: candidate.strategy,
                // RAW contract values, verbatim (auditable):
                risk: raw_values[0],
                duration: raw_values[1],
                manipulability: ac.metrics.avg_manipulability,
                length: raw_values[3],
                // Normalized J contributions (per-candidate-set):
                normalized: ObjectiveComponents {
                    risk: norms[0],
                    duration: norms[1],
                    manipulability: norms[2],
                    length: norms[3],
                },
                cost,
            };
            (candidate, score)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use crate::candidate::admissibility::{AdmissibilityGate, GateCandidate, JointBounds};
    use crate::candidate::contract::{
        AdmissibleCandidate, Candidate, CandidateAssessment, MotionMetrics, NoCandidateReason,
        RiskAdmissibility, StrategyOutcome, StrategyTrace,
    };
    use crate::candidate::objective::ObjectiveProfile;
    use crate::candidate::ranking::{CandidateRanking, SelectionReason};
    use crate::candidate::strategy::StrategyKind;
    use crate::motion::program::PlanningProgram;
    use thalos_core::ids::OperationId;
    use thalos_core::motion::segment::MotionSegment;

    use super::*;

    fn movej(origin: &str, target: Vec<f64>) -> MotionSegment {
        MotionSegment::MoveJ {
            origin: OperationId(origin.to_string()),
            target,
            max_velocity: None,
            max_acceleration: None,
        }
    }

    fn seed_program() -> PlanningProgram {
        PlanningProgram::new(vec![
            movej("op-start", vec![0.0, 0.0]),
            movej("op-goal", vec![0.5, 0.4]),
        ])
    }

    fn admissible(
        strategy: StrategyKind,
        risk: f64,
        duration: f64,
        avg_manipulability: f64,
        path_length: f64,
    ) -> AdmissibleCandidate {
        AdmissibleCandidate {
            candidate: Candidate {
                strategy,
                program: PlanningProgram::new(vec![]),
            },
            assessment: CandidateAssessment {
                risk,
                admissibility: RiskAdmissibility::Accepted,
            },
            metrics: MotionMetrics {
                duration,
                avg_manipulability,
                path_length,
            },
        }
    }

    /// Cost of the candidate with the given strategy inside the ranking.
    fn cost_of(ranking: &CandidateRanking, strategy: StrategyKind) -> f64 {
        ranking
            .ranked
            .iter()
            .find(|(c, _)| c.strategy == strategy)
            .map(|(_, s)| s.cost)
            .expect("strategy must be ranked")
    }

    // ── 3.5 — J = Σ w_i · norm_i, exact hand-computed values ─────────────

    #[test]
    fn evaluate_computes_j_exactly_for_a_hand_computed_set() {
        // Three candidates, each monotone in ALL components:
        //   A: risk 0.1, dur 2.0, manip 0.8 (M=0.2), len 1.0
        //   B: risk 0.3, dur 4.0, manip 0.6 (M=0.4), len 2.0
        //   C: risk 0.5, dur 6.0, manip 0.4 (M=0.6), len 3.0
        // Per-component min-max: all four components are [0, 0.5, 1].
        //   J(A) = 0.5·0 + 0.2·0 + 0.2·0 + 0.1·0 = 0
        //   J(B) = 0.5·0.5 + 0.2·0.5 + 0.2·0.5 + 0.1·0.5 = 0.5
        //   J(C) = 0.5·1 + 0.2·1 + 0.2·1 + 0.1·1 = 1
        let set = vec![
            admissible(StrategyKind::Direct, 0.1, 2.0, 0.8, 1.0),
            admissible(StrategyKind::InsertWaypoint, 0.3, 4.0, 0.6, 2.0),
            admissible(StrategyKind::AlternateElbow, 0.5, 6.0, 0.4, 3.0),
        ];

        let ranking = CandidateEvaluator::evaluate(&set, ObjectiveProfile::SafetyFirst, Vec::new());

        assert_eq!(ranking.ranked.len(), 3);
        assert!((cost_of(&ranking, StrategyKind::Direct) - 0.0).abs() < 1e-9);
        assert!((cost_of(&ranking, StrategyKind::InsertWaypoint) - 0.5).abs() < 1e-9);
        assert!((cost_of(&ranking, StrategyKind::AlternateElbow) - 1.0).abs() < 1e-9);
        assert_eq!(
            ranking.selected.as_ref().unwrap().strategy,
            StrategyKind::Direct,
            "argmin J selects the Direct candidate"
        );
        assert!(matches!(ranking.reason, SelectionReason::Selected { .. }));
    }

    #[test]
    fn evaluate_ranks_by_ascending_cost() {
        let set = vec![
            admissible(StrategyKind::AlternateElbow, 0.5, 6.0, 0.4, 3.0),
            admissible(StrategyKind::Direct, 0.1, 2.0, 0.8, 1.0),
            admissible(StrategyKind::InsertWaypoint, 0.3, 4.0, 0.6, 2.0),
        ];

        let ranking = CandidateEvaluator::evaluate(&set, ObjectiveProfile::SafetyFirst, Vec::new());

        let strategies: Vec<StrategyKind> =
            ranking.ranked.iter().map(|(c, _)| c.strategy).collect();
        assert_eq!(
            strategies,
            vec![
                StrategyKind::Direct,
                StrategyKind::InsertWaypoint,
                StrategyKind::AlternateElbow
            ],
            "ranked must be sorted by ascending J"
        );
        assert_eq!(
            ranking.selected.as_ref().unwrap().strategy,
            StrategyKind::Direct
        );
    }

    #[test]
    fn evaluate_separates_raw_metrics_from_normalized_components() {
        // Spec "Raw and normalized separated": CandidateScore carries the RAW
        // contract values AND the normalized components. For candidate A the
        // raw risk is 0.1 while its normalized contribution is 0.0 (it IS the
        // set minimum) — both visible, distinct, auditable.
        let set = vec![
            admissible(StrategyKind::Direct, 0.1, 2.0, 0.8, 1.0),
            admissible(StrategyKind::InsertWaypoint, 0.3, 4.0, 0.6, 2.0),
            admissible(StrategyKind::AlternateElbow, 0.5, 6.0, 0.4, 3.0),
        ];

        let ranking = CandidateEvaluator::evaluate(&set, ObjectiveProfile::SafetyFirst, Vec::new());

        let score = &ranking
            .ranked
            .iter()
            .find(|(c, _)| c.strategy == StrategyKind::Direct)
            .expect("Direct ranked")
            .1;
        assert!((score.risk - 0.1).abs() < 1e-12, "raw risk stored verbatim");
        assert!((score.duration - 2.0).abs() < 1e-12);
        assert!((score.manipulability - 0.8).abs() < 1e-12);
        assert!((score.length - 1.0).abs() < 1e-12);
        assert!((score.normalized.risk - 0.0).abs() < 1e-12);
        assert!((score.normalized.duration - 0.0).abs() < 1e-12);
        // M is LOW-manipulability: 1 − 0.8 = 0.2 is the set minimum → 0.0.
        assert!((score.normalized.manipulability - 0.0).abs() < 1e-12);
        assert!((score.normalized.length - 0.0).abs() < 1e-12);
        assert!((score.cost - 0.0).abs() < 1e-9);
    }

    // ── 3.5 — argmin over ADMISSIBLE candidates only ─────────────────────

    #[test]
    fn evaluate_selects_argmin_over_admissible_only_via_the_gate() {
        // Pipeline proof: a Critical candidate with the LOWEST raw metrics
        // (would win any J comparison) is excluded by the gate's risk policy.
        // The evaluator receives ONLY admissible candidates — the Critical
        // candidate can never be selected despite its low J.
        let seed = seed_program();
        let critical = GateCandidate {
            candidate: Candidate {
                strategy: StrategyKind::AlternateElbow,
                program: PlanningProgram::new(vec![
                    movej("op-start", vec![0.0, 0.0]),
                    movej("op-goal", vec![0.5, 0.4]),
                ]),
            },
            compile_ok: true,
            assessment: Some(CandidateAssessment {
                risk: 0.95,
                admissibility: RiskAdmissibility::Rejected,
            }),
            metrics: Some(MotionMetrics {
                duration: 0.5,
                avg_manipulability: 0.95,
                path_length: 0.3,
            }),
        };
        let healthy = GateCandidate {
            candidate: Candidate {
                strategy: StrategyKind::Direct,
                program: seed.clone(),
            },
            compile_ok: true,
            assessment: Some(CandidateAssessment {
                risk: 0.557,
                admissibility: RiskAdmissibility::Accepted,
            }),
            metrics: Some(MotionMetrics {
                duration: 3.2,
                avg_manipulability: 0.458,
                path_length: 1.8,
            }),
        };
        let limits: Vec<JointBounds> = vec![
            JointBounds {
                lower: -1.0,
                upper: 1.0,
            },
            JointBounds {
                lower: -1.0,
                upper: 1.0,
            },
        ];
        let report = AdmissibilityGate.filter(&seed, &[critical, healthy], Some(&limits));

        assert_eq!(
            report.admissible.len(),
            1,
            "only the healthy candidate passes"
        );
        assert_eq!(
            report.rejected[0].reason,
            crate::candidate::admissibility::RejectionReason::RiskRejected
        );

        let ranking = CandidateEvaluator::evaluate(
            &report.admissible,
            ObjectiveProfile::SafetyFirst,
            Vec::new(),
        );

        assert_eq!(
            ranking.selected.as_ref().unwrap().strategy,
            StrategyKind::Direct,
            "the Critical candidate must NOT be selected even though its raw metrics are the best"
        );
        assert_eq!(ranking.ranked.len(), 1);
    }

    #[test]
    fn evaluate_with_no_admissible_candidates_reports_no_selection() {
        // Spec "All candidates inadmissible": empty admissible input → no
        // selection, structural reason, no fallback.
        let ranking = CandidateEvaluator::evaluate(&[], ObjectiveProfile::SafetyFirst, Vec::new());

        assert!(ranking.ranked.is_empty());
        assert!(ranking.selected.is_none());
        match ranking.reason {
            SelectionReason::NoAdmissibleCandidate { reason } => {
                assert_eq!(reason, "no admissible candidates");
            }
            other => panic!("expected NoAdmissibleCandidate, got {other:?}"),
        }
    }

    // ── REMEDIATION (verify Warning 1 FIX, ADR-3 observability) — the
    //    strategy trace is carried through to the ranking ───────────────────

    #[test]
    fn evaluate_carries_the_strategy_trace_to_the_ranking() {
        // ADR-3: the trace produced by the generator (every strategy →
        // Generated/Skipped) must reach the ranking so the runtime can surface
        // it in the DTO. The evaluator receives it as INPUT (kept pure — it
        // never interprets the trace) and carries it verbatim.
        let set = vec![
            admissible(StrategyKind::Direct, 0.557, 7.8, 0.458, 3.885),
            admissible(StrategyKind::AlternateElbow, 0.1625, 5.2, 0.6314, 2.14),
        ];
        let trace = vec![
            StrategyTrace {
                strategy: StrategyKind::Direct,
                outcome: StrategyOutcome::Generated(Candidate {
                    strategy: StrategyKind::Direct,
                    program: PlanningProgram::new(vec![]),
                }),
            },
            StrategyTrace {
                strategy: StrategyKind::InsertWaypoint,
                outcome: StrategyOutcome::Skipped(NoCandidateReason::UnsupportedSegment),
            },
            StrategyTrace {
                strategy: StrategyKind::AlternateElbow,
                outcome: StrategyOutcome::Generated(Candidate {
                    strategy: StrategyKind::AlternateElbow,
                    program: PlanningProgram::new(vec![]),
                }),
            },
        ];

        let ranking =
            CandidateEvaluator::evaluate(&set, ObjectiveProfile::SafetyFirst, trace.clone());

        assert_eq!(
            ranking.strategy_trace, trace,
            "the ranking must carry the full strategy trace verbatim"
        );
        assert_eq!(ranking.strategy_trace.len(), 3);
        assert_eq!(ranking.strategy_trace[0].strategy, StrategyKind::Direct);
        assert!(matches!(
            ranking.strategy_trace[1].outcome,
            StrategyOutcome::Skipped(NoCandidateReason::UnsupportedSegment)
        ));
        assert_eq!(
            ranking.strategy_trace[2].strategy,
            StrategyKind::AlternateElbow
        );
        assert!(matches!(
            ranking.strategy_trace[2].outcome,
            StrategyOutcome::Generated(_)
        ));
    }

    // ── 3.5 — proptest: J monotonic per component (all else constant) ────

    proptest! {
        /// risk↑ → J↑, all else constant (spec "Monotonicity — risk increase").
        #[test]
        fn cost_is_strictly_increasing_in_risk(
            low in 0.0f64..1.0,
            high in 0.0f64..1.0,
            duration in 0.0f64..10.0,
            manip in 0.0f64..1.0,
            length in 0.0f64..5.0,
            extra_risk in 0.0f64..1.0,
            extra_duration in 0.0f64..10.0,
            extra_manip in 0.0f64..1.0,
            extra_length in 0.0f64..5.0,
        ) {
            let (low, high) = if low < high { (low, high) } else { (high, low) };
            prop_assume!(high - low > 1e-3);
            let set = vec![
                admissible(StrategyKind::Direct, low, duration, manip, length),
                admissible(StrategyKind::AlternateElbow, high, duration, manip, length),
                admissible(StrategyKind::InsertWaypoint, extra_risk, extra_duration, extra_manip, extra_length),
            ];
            let ranking = CandidateEvaluator::evaluate(&set, ObjectiveProfile::SafetyFirst, Vec::new());
            let cost_low = cost_of(&ranking, StrategyKind::Direct);
            let cost_high = cost_of(&ranking, StrategyKind::AlternateElbow);
            prop_assert!(
                cost_high > cost_low + 1e-12,
                "risk↑ must strictly raise J (all else constant): {cost_low} vs {cost_high}"
            );
        }

        /// duration↑ → J↑, all else constant (spec "Monotonicity — duration increase").
        #[test]
        fn cost_is_strictly_increasing_in_duration(
            risk in 0.0f64..1.0,
            low in 0.0f64..10.0,
            high in 0.0f64..10.0,
            manip in 0.0f64..1.0,
            length in 0.0f64..5.0,
            extra_risk in 0.0f64..1.0,
            extra_duration in 0.0f64..10.0,
            extra_manip in 0.0f64..1.0,
            extra_length in 0.0f64..5.0,
        ) {
            let (low, high) = if low < high { (low, high) } else { (high, low) };
            prop_assume!(high - low > 1e-3);
            let set = vec![
                admissible(StrategyKind::Direct, risk, low, manip, length),
                admissible(StrategyKind::AlternateElbow, risk, high, manip, length),
                admissible(StrategyKind::InsertWaypoint, extra_risk, extra_duration, extra_manip, extra_length),
            ];
            let ranking = CandidateEvaluator::evaluate(&set, ObjectiveProfile::SafetyFirst, Vec::new());
            let cost_low = cost_of(&ranking, StrategyKind::Direct);
            let cost_high = cost_of(&ranking, StrategyKind::AlternateElbow);
            prop_assert!(
                cost_high > cost_low + 1e-12,
                "duration↑ must strictly raise J (all else constant): {cost_low} vs {cost_high}"
            );
        }

        /// low-manipulability↑ (manipulability↓) → J↑, all else constant
        /// (spec "Monotonicity — low manipulability increase").
        #[test]
        fn cost_is_strictly_increasing_in_low_manipulability(
            risk in 0.0f64..1.0,
            duration in 0.0f64..10.0,
            high_manip in 0.0f64..1.0,
            low_manip in 0.0f64..1.0,
            length in 0.0f64..5.0,
            extra_risk in 0.0f64..1.0,
            extra_duration in 0.0f64..10.0,
            extra_manip in 0.0f64..1.0,
            extra_length in 0.0f64..5.0,
        ) {
            // high_manip > low_manip → M(high) = 1−high < M(low) = 1−low.
            let (high_manip, low_manip) = if high_manip > low_manip {
                (high_manip, low_manip)
            } else {
                (low_manip, high_manip)
            };
            prop_assume!(high_manip - low_manip > 1e-3);
            let set = vec![
                admissible(StrategyKind::Direct, risk, duration, high_manip, length),
                admissible(StrategyKind::AlternateElbow, risk, duration, low_manip, length),
                admissible(StrategyKind::InsertWaypoint, extra_risk, extra_duration, extra_manip, extra_length),
            ];
            let ranking = CandidateEvaluator::evaluate(&set, ObjectiveProfile::SafetyFirst, Vec::new());
            let cost_high_manip = cost_of(&ranking, StrategyKind::Direct);
            let cost_low_manip = cost_of(&ranking, StrategyKind::AlternateElbow);
            prop_assert!(
                cost_high_manip < cost_low_manip + 1e-12,
                "manipulability↓ (low-manip↑) must strictly raise J (all else constant): {cost_high_manip} vs {cost_low_manip}"
            );
        }

        /// length↑ → J↑, all else constant (spec "Monotonicity — length increase").
        #[test]
        fn cost_is_strictly_increasing_in_length(
            risk in 0.0f64..1.0,
            duration in 0.0f64..10.0,
            manip in 0.0f64..1.0,
            low in 0.0f64..5.0,
            high in 0.0f64..5.0,
            extra_risk in 0.0f64..1.0,
            extra_duration in 0.0f64..10.0,
            extra_manip in 0.0f64..1.0,
            extra_length in 0.0f64..5.0,
        ) {
            let (low, high) = if low < high { (low, high) } else { (high, low) };
            prop_assume!(high - low > 1e-3);
            let set = vec![
                admissible(StrategyKind::Direct, risk, duration, manip, low),
                admissible(StrategyKind::AlternateElbow, risk, duration, manip, high),
                admissible(StrategyKind::InsertWaypoint, extra_risk, extra_duration, extra_manip, extra_length),
            ];
            let ranking = CandidateEvaluator::evaluate(&set, ObjectiveProfile::SafetyFirst, Vec::new());
            let cost_low = cost_of(&ranking, StrategyKind::Direct);
            let cost_high = cost_of(&ranking, StrategyKind::AlternateElbow);
            prop_assert!(
                cost_high > cost_low + 1e-12,
                "length↑ must strictly raise J (all else constant): {cost_low} vs {cost_high}"
            );
        }
    }
}
