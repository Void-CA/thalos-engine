//! Neutral contract types for the candidate layer (design ADR-3, ADR-5).
//!
//! PLANNING-OWNED and neutral: nothing in this module depends on the
//! intelligence crate. The runtime (composition root) maps the frozen
//! `Assessor`'s output into [`CandidateAssessment`] — this module never
//! re-derives risk.
//!
//! Note on placement: [`Candidate`] lives here (contract) rather than in
//! `generator.rs` so the contract types are self-contained — `StrategyOutcome`
//! and `AdmissibleCandidate` both carry it. The generator is the *producer*,
//! not the owner, of the type.

use crate::candidate::strategy::StrategyKind;
use crate::motion::program::PlanningProgram;

/// Endpoint equality tolerance ε (ADR-1): absolute, per-joint, 1e-4 rad
/// (~0.006°) — defined ONCE in the contract, never per-strategy.
///
/// Rationale: the DLS IK solver converges at `1e-4`, so candidates produced
/// by IK re-solve naturally satisfy this bound; tighter would reject valid
/// alternatives, looser would mask drift. Equality is on the joint goal, NOT
/// the TCP pose (spec candidate-generation "Endpoint identity").
pub const ENDPOINT_TOLERANCE: f64 = 1e-4;

/// A candidate realization of the seed: one strategy applied to the seed
/// `PlanningProgram`. Produced by a [`MotionStrategy`](crate::candidate::strategy::MotionStrategy);
/// `Direct` is the baseline (the seed itself, always candidate 0).
#[derive(Debug, Clone, PartialEq)]
pub struct Candidate {
    /// The strategy that produced this realization.
    pub strategy: StrategyKind,
    /// The full alternative program (same task, same endpoints — only the
    /// geometric realization varies).
    pub program: PlanningProgram,
}

/// Why a strategy produced no candidate (spec candidate-generation "Bounded
/// Strategy Library", design ADR-3).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NoCandidateReason {
    /// Inverse kinematics failed to converge while materializing the edit.
    IkFailed,
    /// The target segment type cannot be transformed by this strategy.
    UnsupportedSegment,
    /// A hard invariant was violated (e.g. target segment out of bounds).
    InvariantViolation {
        /// The invariant that failed, human-readable.
        invariant: String,
    },
}

/// The outcome of applying one strategy to the seed (design ADR-3).
#[derive(Debug, Clone, PartialEq)]
pub enum StrategyOutcome {
    /// The strategy produced a candidate.
    Generated(Candidate),
    /// The strategy produced none, with a recorded reason.
    Skipped(NoCandidateReason),
}

/// One row of the strategy trace: which strategy was applied and what it
/// produced. `CandidateGenerator::generate` returns the FULL trace — every
/// strategy, including skipped ones (spec candidate-generation "No generating
/// strategy produces a candidate").
#[derive(Debug, Clone, PartialEq)]
pub struct StrategyTrace {
    /// The strategy that was applied.
    pub strategy: StrategyKind,
    /// What it produced (or why it produced nothing).
    pub outcome: StrategyOutcome,
}

/// Risk admissibility — a POLICY on the Assessor's categorical verdict,
/// mapped by the runtime (design ADR-5). `Rejected` corresponds to the
/// runtime mapping `Assessment.risk == Critical → Rejected`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RiskAdmissibility {
    /// The candidate's assessed risk is acceptable.
    Accepted,
    /// The candidate's assessed risk is unacceptable (Critical).
    Rejected,
}

/// Neutral assessment — the MINIMUM the candidate layer needs from the frozen
/// Assessor's output (design ADR-5). This is NOT a second risk system: the
/// runtime maps the authoritative `Assessment` into it.
#[derive(Debug, Clone, PartialEq)]
pub struct CandidateAssessment {
    /// Crisp fuzzy risk (`1 − quality`) produced by the authoritative
    /// Assessor. Range [0, 1].
    pub risk: f64,
    /// Risk admissibility, mapped by the runtime from `Assessment.risk`.
    pub admissibility: RiskAdmissibility,
}

/// Motion metrics extracted from the `AnalysisReport` by the runtime.
///
/// Sources (design ADR-5): risk → Assessor (single authority); duration /
/// avg_manipulability / path_length → the analyzed trajectory + report. The
/// evaluator NEVER computes a metric from the program (Analyzer → metrics,
/// Evaluator → objective).
#[derive(Debug, Clone, PartialEq)]
pub struct MotionMetrics {
    /// Total trajectory duration (seconds).
    pub duration: f64,
    /// Average manipulability over the trajectory.
    pub avg_manipulability: f64,
    /// Total path length (metres).
    pub path_length: f64,
}

/// A candidate that passed the admissibility gate: candidate + neutral
/// assessment + motion metrics (design ADR-5 data flow).
#[derive(Debug, Clone, PartialEq)]
pub struct AdmissibleCandidate {
    /// The candidate program.
    pub candidate: Candidate,
    /// The neutral assessment mapped from the Assessor's output.
    pub assessment: CandidateAssessment,
    /// Motion metrics extracted from the analyzed trajectory + report.
    pub metrics: MotionMetrics,
}

/// Resolved context for generation: WHICH segment to transform. Segment
/// selection is a SEPARATE policy from the strategy — detection/selection
/// never mix with synthesis (design "Interfaces / Contracts").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CandidateGenerationContext {
    /// Index of the seed segment the strategies transform.
    pub target_segment: usize,
}

#[cfg(test)]
mod tests {
    use thalos_core::ids::OperationId;
    use thalos_core::motion::segment::MotionSegment;

    use crate::candidate::strategy::StrategyKind;
    use crate::motion::program::PlanningProgram;

    use super::*;

    fn movej(target: Vec<f64>) -> MotionSegment {
        MotionSegment::MoveJ {
            origin: OperationId("op-j".to_string()),
            target,
            max_velocity: None,
            max_acceleration: None,
        }
    }

    // ── 1.1 — ε boundary (ADR-1) ─────────────────────────────────────────

    // Compile-time enforcement: the ADR-1 bound is a contract invariant, so
    // violating it is a build error, not a test failure.
    const _: () = assert!(
        ENDPOINT_TOLERANCE <= 1e-4,
        "ε must not exceed the ADR-1 bound"
    );
    const _: () = assert!(ENDPOINT_TOLERANCE > 0.0, "ε must be a positive tolerance");

    #[test]
    fn endpoint_tolerance_is_the_adr1_per_joint_boundary() {
        // ADR-1: the endpoint equality bound is 1e-4 rad per joint, defined
        // ONCE in the contract — never left to each strategy.
        assert_eq!(
            ENDPOINT_TOLERANCE, 1e-4,
            "ε must equal the ADR-1 value exactly (1e-4 rad per joint)"
        );
    }

    // ── 1.2 — no-candidate reasons ───────────────────────────────────────

    #[test]
    fn no_candidate_reason_records_ik_failure() {
        assert!(matches!(
            NoCandidateReason::IkFailed,
            NoCandidateReason::IkFailed
        ));
    }

    #[test]
    fn no_candidate_reason_records_unsupported_segment() {
        assert!(matches!(
            NoCandidateReason::UnsupportedSegment,
            NoCandidateReason::UnsupportedSegment
        ));
    }

    #[test]
    fn no_candidate_reason_records_invariant_violation_with_message() {
        let reason = NoCandidateReason::InvariantViolation {
            invariant: "target segment out of bounds".to_string(),
        };
        match reason {
            NoCandidateReason::InvariantViolation { invariant } => {
                assert_eq!(invariant, "target segment out of bounds")
            }
            other => panic!("expected InvariantViolation, got {other:?}"),
        }
    }

    // ── 1.2 — strategy outcomes ──────────────────────────────────────────

    #[test]
    fn strategy_outcome_generated_carries_the_candidate() {
        let program = PlanningProgram::new(vec![movej(vec![0.1, 0.2])]);
        let candidate = Candidate {
            strategy: StrategyKind::InsertWaypoint,
            program: program.clone(),
        };
        match StrategyOutcome::Generated(candidate.clone()) {
            StrategyOutcome::Generated(c) => {
                assert_eq!(c, candidate, "Generated must carry the candidate verbatim");
            }
            StrategyOutcome::Skipped(reason) => {
                panic!("expected Generated, got Skipped({reason:?})")
            }
        }
    }

    #[test]
    fn strategy_outcome_skipped_records_the_reason() {
        match StrategyOutcome::Skipped(NoCandidateReason::UnsupportedSegment) {
            StrategyOutcome::Skipped(NoCandidateReason::UnsupportedSegment) => {}
            other => panic!("expected Skipped(UnsupportedSegment), got {other:?}"),
        }
    }

    // ── 1.2 — assessment + metrics (neutral contract, ADR-5) ─────────────

    #[test]
    fn risk_admissibility_distinguishes_accepted_and_rejected() {
        let accepted = RiskAdmissibility::Accepted;
        let rejected = RiskAdmissibility::Rejected;
        assert!(
            !matches!(accepted, RiskAdmissibility::Rejected),
            "Accepted must not match Rejected"
        );
        assert!(
            !matches!(rejected, RiskAdmissibility::Accepted),
            "Rejected must not match Accepted"
        );
    }

    #[test]
    fn candidate_assessment_carries_risk_and_admissibility() {
        let assessment = CandidateAssessment {
            risk: 0.557,
            admissibility: RiskAdmissibility::Accepted,
        };
        assert!(
            (assessment.risk - 0.557).abs() < 1e-12,
            "risk must be stored verbatim"
        );
        assert!(matches!(
            assessment.admissibility,
            RiskAdmissibility::Accepted
        ));
    }

    #[test]
    fn motion_metrics_carry_analyzed_trajectory_quantities() {
        // Sources (design ADR-5): risk → Assessor; duration / avg_manipulability
        // / path_length → the analyzed trajectory + report. The evaluator NEVER
        // computes a metric from the program.
        let metrics = MotionMetrics {
            duration: 3.2,
            avg_manipulability: 0.458,
            path_length: 1.8,
        };
        assert!((metrics.duration - 3.2).abs() < 1e-12);
        assert!((metrics.avg_manipulability - 0.458).abs() < 1e-12);
        assert!((metrics.path_length - 1.8).abs() < 1e-12);
    }

    #[test]
    fn admissible_candidate_bundles_candidate_assessment_and_metrics() {
        let candidate = Candidate {
            strategy: StrategyKind::Direct,
            program: PlanningProgram::new(vec![movej(vec![0.0, 0.0])]),
        };
        let admissible = AdmissibleCandidate {
            candidate: candidate.clone(),
            assessment: CandidateAssessment {
                risk: 0.1,
                admissibility: RiskAdmissibility::Accepted,
            },
            metrics: MotionMetrics {
                duration: 1.0,
                avg_manipulability: 0.9,
                path_length: 0.5,
            },
        };
        assert_eq!(admissible.candidate, candidate);
        assert!((admissible.assessment.risk - 0.1).abs() < 1e-12);
        assert!((admissible.metrics.duration - 1.0).abs() < 1e-12);
    }

    // ── 1.2 — generation context + trace ─────────────────────────────────

    #[test]
    fn generation_context_names_the_target_segment() {
        let ctx = CandidateGenerationContext { target_segment: 2 };
        assert_eq!(
            ctx.target_segment, 2,
            "context must carry the resolved segment"
        );
    }

    #[test]
    fn strategy_trace_records_strategy_and_outcome() {
        let trace = StrategyTrace {
            strategy: StrategyKind::AlternateElbow,
            outcome: StrategyOutcome::Skipped(NoCandidateReason::IkFailed),
        };
        assert_eq!(trace.strategy, StrategyKind::AlternateElbow);
        assert!(matches!(
            trace.outcome,
            StrategyOutcome::Skipped(NoCandidateReason::IkFailed)
        ));
    }
}
