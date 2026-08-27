//! The strategy contract: what a strategy IS and how it produces a candidate
//! from the seed (design ADR-4, "Interfaces / Contracts").
//!
//! A [`MotionStrategy`] applies ONE geometric transformation to the seed at
//! `ctx.target_segment` and returns at most one candidate (`StrategyOutcome`
//! is `Generated` or `Skipped`). Strategies are geometry-only: they know
//! nothing about risk, assessment, or the Assessor (spec candidate-generation
//! "Strategy Isolation").

use thalos_core::kinematics::inverse::IKSolver;

use crate::candidate::contract::{CandidateGenerationContext, StrategyOutcome};
use crate::motion::program::PlanningProgram;

/// A bounded strategy that produces (at most) one alternative realization of
/// the seed program. `Send + Sync` so the generator can hold strategies
/// behind `Box<dyn MotionStrategy>`.
pub trait MotionStrategy: Send + Sync {
    /// The strategy kind (identity for traces, ranking, and DTO projection).
    fn kind(&self) -> StrategyKind;

    /// Apply the strategy to the seed at `ctx.target_segment`, returning the
    /// candidate or a documented no-candidate reason.
    ///
    /// `ik_solver` is the kinematic context for strategies that re-solve IK
    /// (e.g. `AlternateElbow`); strategies that do not touch IK may ignore it.
    fn apply(
        &self,
        seed: &PlanningProgram,
        ctx: &CandidateGenerationContext,
        ik_solver: &dyn IKSolver,
    ) -> StrategyOutcome;
}

/// The bounded strategy library (spec candidate-generation "Bounded Strategy
/// Library", proposal scope).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StrategyKind {
    /// The seed itself — the baseline, NOT a generating strategy. Always
    /// candidate 0, never skipped.
    Direct,
    /// Split the target segment by inserting an intermediate waypoint
    /// (wraps `InsertWaypointMaterializer`).
    InsertWaypoint,
    /// Re-solve the target segment to the same-side elbow posture
    /// (wraps `SingularityResolveMaterializer`).
    AlternateElbow,
    /// Preserve the alternate elbow state and re-resolve the semantic suffix.
    ReplannedAlternate,
}

#[cfg(test)]
mod tests {
    use thalos_core::ids::OperationId;
    use thalos_core::kinematics::inverse::{IKGoal, IKResult, IKSolver, IkError};
    use thalos_core::motion::segment::MotionSegment;

    use crate::candidate::contract::{
        Candidate, CandidateGenerationContext, NoCandidateReason, StrategyOutcome,
    };
    use crate::motion::program::PlanningProgram;

    use super::*;

    struct NoopIKSolver;

    impl IKSolver for NoopIKSolver {
        fn solve(&self, q0: &[f64], _goal: IKGoal) -> Result<IKResult, IkError> {
            Ok(IKResult::converged(q0.to_vec(), 1, 0.0, None))
        }
    }

    fn seed_program() -> PlanningProgram {
        PlanningProgram::new(vec![MotionSegment::MoveJ {
            origin: OperationId("op-j".to_string()),
            target: vec![0.1, 0.2],
            max_velocity: None,
            max_acceleration: None,
        }])
    }

    /// Test strategy that ALWAYS generates — exercises the trait contract
    /// without materializer machinery.
    struct AlwaysGenerates;

    impl MotionStrategy for AlwaysGenerates {
        fn kind(&self) -> StrategyKind {
            StrategyKind::InsertWaypoint
        }

        fn apply(
            &self,
            seed: &PlanningProgram,
            _ctx: &CandidateGenerationContext,
            _ik_solver: &dyn IKSolver,
        ) -> StrategyOutcome {
            StrategyOutcome::Generated(Candidate {
                strategy: StrategyKind::InsertWaypoint,
                program: seed.clone(),
            })
        }
    }

    /// Test strategy that ALWAYS skips — proves `Skipped` is an expressible
    /// outcome through the trait.
    struct AlwaysSkips;

    impl MotionStrategy for AlwaysSkips {
        fn kind(&self) -> StrategyKind {
            StrategyKind::AlternateElbow
        }

        fn apply(
            &self,
            _seed: &PlanningProgram,
            _ctx: &CandidateGenerationContext,
            _ik_solver: &dyn IKSolver,
        ) -> StrategyOutcome {
            StrategyOutcome::Skipped(NoCandidateReason::IkFailed)
        }
    }

    // ── 1.3 — StrategyKind ───────────────────────────────────────────────

    #[test]
    fn strategy_kind_is_comparable_and_copyable() {
        assert_eq!(StrategyKind::Direct, StrategyKind::Direct);
        assert_ne!(StrategyKind::Direct, StrategyKind::InsertWaypoint);
        let kinds = [
            StrategyKind::Direct,
            StrategyKind::InsertWaypoint,
            StrategyKind::AlternateElbow,
        ];
        let direct = kinds[0]; // Copy — not moved
        assert_eq!(direct, StrategyKind::Direct);
    }

    // ── 1.3 — MotionStrategy trait contract ──────────────────────────────

    #[test]
    fn strategy_reports_its_kind() {
        let s = AlwaysGenerates;
        assert_eq!(s.kind(), StrategyKind::InsertWaypoint);
        let s = AlwaysSkips;
        assert_eq!(s.kind(), StrategyKind::AlternateElbow);
    }

    #[test]
    fn strategy_apply_produces_generated_outcome() {
        let seed = seed_program();
        let solver = NoopIKSolver;
        let strategy = AlwaysGenerates;
        match strategy.apply(
            &seed,
            &CandidateGenerationContext { target_segment: 0 },
            &solver,
        ) {
            StrategyOutcome::Generated(candidate) => {
                assert_eq!(candidate.strategy, StrategyKind::InsertWaypoint);
                assert_eq!(candidate.program, seed);
            }
            other => panic!("expected Generated, got {other:?}"),
        }
    }

    #[test]
    fn strategy_apply_produces_skipped_outcome_with_reason() {
        let seed = seed_program();
        let solver = NoopIKSolver;
        let strategy = AlwaysSkips;
        assert!(matches!(
            strategy.apply(
                &seed,
                &CandidateGenerationContext { target_segment: 0 },
                &solver
            ),
            StrategyOutcome::Skipped(NoCandidateReason::IkFailed)
        ));
    }

    #[test]
    fn strategies_are_send_and_sync_for_trait_objects() {
        // The generator holds strategies behind `Box<dyn MotionStrategy>` —
        // the trait must be object-safe and Send + Sync.
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<AlwaysGenerates>();
        assert_send_sync::<AlwaysSkips>();
    }
}
