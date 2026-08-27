//! Candidate synthesis and evaluation layer (PR1: contracts + strategies +
//! generator; PR2: evaluation pipeline — objective, admissibility, evaluator,
//! ranking).
//!
//! This module owns the neutral contracts, the bounded strategy library that
//! produces alternative realizations of the SAME task intent (same endpoints,
//! same task sequence — only the geometric realization varies), and the
//! evaluation pipeline (admissibility gate → J ranking → derived reason). It
//! SHALL NOT depend on the intelligence crate: generation produces geometries
//! only, evaluation consumes the neutral [`contract::CandidateAssessment`]
//! that the runtime maps from the frozen Assessor's output (ADR-5).

pub mod admissibility;
pub mod contract;
pub mod evaluator;
pub mod generator;
pub mod objective;
pub mod ranking;
pub mod strategies;
pub mod strategy;

pub use admissibility::{
    AdmissibilityGate, AdmissibilityReport, GateCandidate, JointBounds, RejectedCandidate,
    RejectionPhase, RejectionReason,
};
pub use contract::{
    AdmissibleCandidate, Candidate, CandidateAssessment, CandidateGenerationContext,
    ENDPOINT_TOLERANCE, MotionMetrics, NoCandidateReason, RiskAdmissibility, StrategyOutcome,
    StrategyTrace,
};
pub use evaluator::CandidateEvaluator;
pub use generator::CandidateGenerator;
pub use objective::{ObjectiveProfile, SAFETY_FIRST_WEIGHTS, normalize_min_max};
pub use ranking::{
    COMPONENT_COST, COMPONENT_DURATION, COMPONENT_LENGTH, COMPONENT_MANIPULABILITY, COMPONENT_RISK,
    CandidateRanking, CandidateScore, ENDPOINTS_PRESERVED, MetricComparison,
    NO_ADMISSIBLE_CANDIDATE_REASON, ObjectiveComponents, SelectionReason, TASK_PRESERVED,
    derive_selection_reason,
};
pub use strategy::{MotionStrategy, StrategyKind};

#[cfg(test)]
mod tests {
    use super::*;

    /// 1.4 — the module root re-exports the public contract surface so
    /// consumers (and the runtime composition root, PR3) import from
    /// `candidate::*` instead of the internal file paths.
    #[test]
    fn module_reexports_phase_one_contract_names() {
        // Contract constant re-exported at the module root.
        assert_eq!(ENDPOINT_TOLERANCE, contract::ENDPOINT_TOLERANCE);
        // Strategy surface re-exported at the module root.
        assert_eq!(StrategyKind::Direct, strategy::StrategyKind::Direct);
        // The trait is nameable from the module root (trait-object construction).
        fn _takes_strategy(_s: &dyn MotionStrategy) {}
        // The neutral assessment contract is nameable from the module root.
        let _assessment = RiskAdmissibility::Accepted;
        let _ = _assessment;
    }

    /// 1.4 — the generator and candidate types are re-exported at the module
    /// root for the runtime composition root (PR3).
    #[test]
    fn module_reexports_generator_and_candidate_names() {
        fn _takes_generator(_g: &CandidateGenerator) {}
        fn _takes_candidate(_c: &Candidate) {}
        let _ = (_takes_generator, _takes_candidate);
    }

    /// 3.8 — the evaluation pipeline surface is re-exported at the module
    /// root for the runtime composition root (PR3) and the DTO layer.
    #[test]
    fn module_reexports_phase_three_evaluation_names() {
        // The gate surface.
        fn _takes_gate(_g: &AdmissibilityGate) {}
        fn _takes_report(_r: &AdmissibilityReport) {}
        let _ = (_takes_gate, _takes_report);
        // The evaluator + profile.
        fn _takes_evaluator(_e: &CandidateEvaluator) {}
        let _profile = ObjectiveProfile::SafetyFirst;
        let _ = (_takes_evaluator, _profile);
        // The ranking surface.
        fn _takes_ranking(_r: &CandidateRanking) {}
        fn _takes_score(_s: &CandidateScore) {}
        let _ = (_takes_ranking, _takes_score);
        // The derived-reason constants (spec phrasing).
        assert_eq!(ENDPOINTS_PRESERVED, "Endpoints: preserved");
        assert_eq!(TASK_PRESERVED, "Task: preserved");
        assert_eq!(NO_ADMISSIBLE_CANDIDATE_REASON, "no admissible candidates");
        assert_eq!(COMPONENT_COST, "cost");
        // Normalization helper reachable from the root.
        let norms = normalize_min_max(&[1.0, 3.0]);
        assert!((norms[1] - 1.0).abs() < 1e-12);
        // Derivation reachable from the root.
        let reason = derive_selection_reason(&[]);
        assert!(matches!(
            reason,
            SelectionReason::NoAdmissibleCandidate { .. }
        ));
    }
}
