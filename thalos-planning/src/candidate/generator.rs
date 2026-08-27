//! CandidateGenerator (PR1, Phase 2, tasks 2.1 + 2.4).
//!
//! Bounded synthesis of motion alternatives from a seed [`PlanningProgram`]
//! (spec candidate-generation). Each candidate is a different geometric
//! realization of the SAME task intent — the generator knows nothing about
//! risk (strategy isolation, spec "Strategy Isolation").
//!
//! ## Baseline contract
//!
//! `Direct` is the IMMUTABLE baseline: the seed is ALWAYS candidate 0 as
//! `Candidate { strategy: Direct, program: seed.clone() }` and is NEVER
//! skipped. Only the generating strategies ([`InsertWaypoint`],
//! [`AlternateElbow`]) may produce `Skipped` outcomes. `generate` returns the
//! successful candidates AND the FULL strategy trace (every strategy,
//! including skipped ones — design ADR-3).

use thalos_core::kinematics::inverse::IKSolver;

use crate::candidate::contract::{
    Candidate, CandidateGenerationContext, StrategyOutcome, StrategyTrace,
};
use crate::candidate::strategies::{AlternateElbow, InsertWaypoint};
use crate::candidate::strategy::{MotionStrategy, StrategyKind};
use crate::motion::program::PlanningProgram;

/// Applies the bounded strategy library to a seed program.
pub struct CandidateGenerator {
    /// The generating strategies, applied in order. `Direct` is NOT included
    /// here: the generator synthesizes the baseline itself so it can NEVER be
    /// displaced or duplicated by the strategy list.
    strategies: Vec<Box<dyn MotionStrategy>>,
}

impl CandidateGenerator {
    /// Creates a generator with the given generating strategies.
    ///
    /// `Direct` must not be added — it is managed internally as the immutable
    /// baseline (always candidate 0).
    pub fn new(strategies: Vec<Box<dyn MotionStrategy>>) -> Self {
        Self { strategies }
    }

    /// Generates candidates from the seed and the full strategy trace.
    ///
    /// The seed is ALWAYS candidate 0 (`Direct`); generating strategies are
    /// applied to the seed in list order, each producing at most one
    /// candidate. Returns the successful candidates (≥1) plus the trace of
    /// every strategy applied, including skipped ones.
    pub fn generate(
        &self,
        seed: &PlanningProgram,
        ctx: &CandidateGenerationContext,
        ik_solver: &dyn IKSolver,
    ) -> (Vec<Candidate>, Vec<StrategyTrace>) {
        // Immutable baseline: the seed itself, never skipped.
        let baseline = Candidate {
            strategy: StrategyKind::Direct,
            program: seed.clone(),
        };
        let mut candidates = vec![baseline.clone()];
        let mut traces = vec![StrategyTrace {
            strategy: StrategyKind::Direct,
            outcome: StrategyOutcome::Generated(baseline),
        }];

        for strategy in &self.strategies {
            let outcome = strategy.apply(seed, ctx, ik_solver);
            if let StrategyOutcome::Generated(candidate) = &outcome {
                candidates.push(candidate.clone());
            }
            traces.push(StrategyTrace {
                strategy: strategy.kind(),
                outcome,
            });
        }

        (candidates, traces)
    }
}

impl Default for CandidateGenerator {
    /// The MVP strategy library: `InsertWaypoint` then `AlternateElbow`
    /// (proposal scope — bounded, each ≤1 candidate).
    fn default() -> Self {
        Self::new(vec![
            Box::new(InsertWaypoint::new()),
            Box::new(AlternateElbow::new()),
        ])
    }
}

#[cfg(test)]
mod tests {
    use thalos_core::ids::OperationId;
    use thalos_core::kinematics::forward::ForwardKinematics;
    use thalos_core::kinematics::inverse::{
        DampedLeastSquaresSolver, IKGoal, IKResult, IKSolver, IkError,
    };
    use thalos_core::models::{RobotModel, RobotRegistry};
    use thalos_core::motion::segment::MotionSegment;
    use thalos_core::robot::serial_chain::SerialChain;
    use thalos_core::spatial::frame::FrameId;
    use thalos_core::spatial::pose::Pose;
    use thalos_math::{Transform3D, Vector3};

    use crate::candidate::contract::{
        CandidateGenerationContext, ENDPOINT_TOLERANCE, NoCandidateReason, StrategyOutcome,
    };
    use crate::candidate::strategies::InsertWaypoint;
    use crate::candidate::{CandidateGenerator, StrategyKind};
    use crate::motion::program::PlanningProgram;

    /// Mock solver WITHOUT a robot chain (`robot() -> None`) — lets
    /// `AlternateElbow` reach its `UnsupportedProposal` path so the generator
    /// skip-tracing is testable without real geometry.
    struct ChainlessIKSolver;

    impl IKSolver for ChainlessIKSolver {
        fn solve(&self, q0: &[f64], _goal: IKGoal) -> Result<IKResult, IkError> {
            Ok(IKResult::converged(q0.to_vec(), 1, 0.0, None))
        }
    }

    fn chain(model: RobotModel) -> SerialChain {
        RobotRegistry::create_default(model)
    }

    fn real_solver(chain: &SerialChain) -> DampedLeastSquaresSolver {
        let fk = ForwardKinematics::new(chain.clone());
        DampedLeastSquaresSolver::new(fk, *chain.end_effector(), 500, 1e-6, 0.1)
    }

    fn movej(origin: &str, target: Vec<f64>) -> MotionSegment {
        MotionSegment::MoveJ {
            origin: OperationId(origin.to_string()),
            target,
            max_velocity: None,
            max_acceleration: None,
        }
    }

    fn pose_at(x: f64, y: f64, z: f64) -> Pose {
        Pose::new(
            FrameId::World,
            FrameId::Id(1),
            Transform3D::from_translation(Vector3::new(x, y, z)),
        )
    }

    fn movel(origin: &str, target_pose: Pose) -> MotionSegment {
        MotionSegment::MoveL {
            origin: OperationId(origin.to_string()),
            frame: FrameId::World,
            target_pose,
            max_velocity: None,
        }
    }

    /// Extract the terminal joints of a program: the LAST explicit MoveJ
    /// target (the joint goal — NOT a TCP pose, per spec Q2).
    fn goal_joints(program: &PlanningProgram) -> Option<Vec<f64>> {
        program.segments.iter().rev().find_map(|s| match s {
            MotionSegment::MoveJ { target, .. } => Some(target.clone()),
            _ => None,
        })
    }

    /// The FIRST explicit joint configuration the program commands.
    fn first_commanded_joints(program: &PlanningProgram) -> Option<Vec<f64>> {
        match program.segments.first()? {
            MotionSegment::MoveJ { target, .. } => Some(target.clone()),
            _ => None,
        }
    }

    // ── 2.1 — seed baseline always present ───────────────────────────────

    #[test]
    fn seed_is_always_candidate_zero_with_direct_strategy() {
        // Spec "Seed is candidate 0": for ANY seed, generate() returns ≥1
        // candidate and index 0 is Direct with the seed program verbatim.
        let seed = PlanningProgram::new(vec![movej("op-a", vec![0.1, 0.2])]);
        let ctx = CandidateGenerationContext { target_segment: 0 };
        let solver = ChainlessIKSolver;
        let generator = CandidateGenerator::default();

        let (candidates, _traces) = generator.generate(&seed, &ctx, &solver);

        assert!(
            !candidates.is_empty(),
            "the seed baseline must ALWAYS be present"
        );
        assert_eq!(candidates[0].strategy, StrategyKind::Direct);
        assert_eq!(candidates[0].program, seed);
    }

    #[test]
    fn all_generating_strategies_skipping_leaves_only_the_seed() {
        // Spec "No generating strategy produces a candidate": both generating
        // strategies skip (InsertWaypoint on a MoveJ target → UnsupportedSegment;
        // AlternateElbow on a MoveJ target with a chain-less solver →
        // InvariantViolation) → exactly 1 candidate (the seed) and EVERY
        // no-candidate reason recorded in the trace.
        let seed = PlanningProgram::new(vec![
            movej("op-a", vec![0.1, 0.2]),
            movej("op-b", vec![0.5, 0.6]),
        ]);
        let ctx = CandidateGenerationContext { target_segment: 1 };
        let solver = ChainlessIKSolver;
        let generator = CandidateGenerator::default();

        let (candidates, traces) = generator.generate(&seed, &ctx, &solver);

        assert_eq!(candidates.len(), 1, "only the seed baseline must survive");
        assert_eq!(candidates[0].strategy, StrategyKind::Direct);
        assert_eq!(candidates[0].program, seed);
        assert_eq!(traces.len(), 3, "Direct + 2 generating strategies");

        assert_eq!(traces[0].strategy, StrategyKind::Direct);
        assert!(matches!(traces[0].outcome, StrategyOutcome::Generated(_)));
        assert_eq!(traces[1].strategy, StrategyKind::InsertWaypoint);
        assert!(matches!(
            traces[1].outcome,
            StrategyOutcome::Skipped(NoCandidateReason::UnsupportedSegment)
        ));
        assert_eq!(traces[2].strategy, StrategyKind::AlternateElbow);
        assert!(matches!(
            traces[2].outcome,
            StrategyOutcome::Skipped(NoCandidateReason::InvariantViolation { .. })
        ));
    }

    // ── 2.4 — generate returns candidates + full trace ───────────────────

    #[test]
    fn generate_returns_candidates_and_full_trace_with_skips() {
        // Happy path with REAL geometry: seed [MoveJ, MoveL, MoveJ], target
        // the MoveL → InsertWaypoint generates (2 MoveL halves); AlternateElbow
        // skips (MoveL target → UnsupportedSegment). Candidates = Direct +
        // InsertWaypoint; trace = all three strategies.
        let robot = chain(RobotModel::Scara);
        let seed = PlanningProgram::new(vec![
            movej("op-start", vec![0.0, -1.31, -0.1, 0.0]),
            movel("op-mid", pose_at(0.3, 0.4, -0.12)),
            movej("op-home", vec![0.0, -1.31, -0.1, 0.0]),
        ]);
        let ctx = CandidateGenerationContext { target_segment: 1 };
        let solver = real_solver(&robot);
        let generator = CandidateGenerator::default();

        let (candidates, traces) = generator.generate(&seed, &ctx, &solver);

        assert_eq!(candidates.len(), 2);
        assert_eq!(candidates[0].strategy, StrategyKind::Direct);
        assert_eq!(candidates[1].strategy, StrategyKind::InsertWaypoint);
        assert_eq!(traces.len(), 3, "the full trace covers every strategy");
        assert_eq!(traces[1].strategy, StrategyKind::InsertWaypoint);
        assert!(matches!(traces[1].outcome, StrategyOutcome::Generated(_)));
        assert_eq!(traces[2].strategy, StrategyKind::AlternateElbow);
        assert!(matches!(
            traces[2].outcome,
            StrategyOutcome::Skipped(NoCandidateReason::UnsupportedSegment)
        ));
    }

    #[test]
    fn generate_respects_injected_strategy_list() {
        // The strategy list is injectable: a generator with ONLY InsertWaypoint
        // produces Direct + one generating candidate and a 2-row trace.
        let robot = chain(RobotModel::Scara);
        let seed = PlanningProgram::new(vec![
            movej("op-start", vec![0.0, -1.31, -0.1, 0.0]),
            movel("op-mid", pose_at(0.3, 0.4, -0.12)),
        ]);
        let ctx = CandidateGenerationContext { target_segment: 1 };
        let solver = real_solver(&robot);
        let generator = CandidateGenerator::new(vec![Box::new(InsertWaypoint::new())]);

        let (candidates, traces) = generator.generate(&seed, &ctx, &solver);

        assert_eq!(candidates.len(), 2, "Direct + InsertWaypoint");
        assert_eq!(candidates[1].strategy, StrategyKind::InsertWaypoint);
        assert_eq!(traces.len(), 2, "Direct + the injected strategy only");
        assert_eq!(traces[1].strategy, StrategyKind::InsertWaypoint);
    }

    // ── 2.5 — equivalence: task sequence + endpoints within ε ────────────

    #[test]
    fn candidates_preserve_task_sequence_and_endpoints_within_epsilon() {
        // Spec "Equivalence Class — Task-Preserving AND Endpoint-Preserving":
        // seed task seq [Pick(A), Wait, Place(B), Home]; the candidate's task
        // sequence (operation kinds + origins) is unchanged, and endpoints are
        // within ε per joint — the joint goal, NOT the TCP pose.
        let seed = PlanningProgram::new(vec![
            movej("op-pick-a", vec![0.0, -1.31, -0.1, 0.0]),
            movel("op-wait", pose_at(0.3, 0.4, -0.12)),
            movej("op-place-b", vec![0.5, -0.4, -0.15, 0.0]),
            movej("op-home", vec![0.0, -1.31, -0.1, 0.0]),
        ]);
        let ctx = CandidateGenerationContext { target_segment: 1 };
        let robot = chain(RobotModel::Scara);
        let solver = real_solver(&robot);
        let generator = CandidateGenerator::default();

        let (candidates, _traces) = generator.generate(&seed, &ctx, &solver);

        let waypoint = candidates
            .iter()
            .find(|c| c.strategy == StrategyKind::InsertWaypoint)
            .expect("InsertWaypoint must generate a candidate for the Wait MoveL");

        // Task-sequence identity: non-target segments byte-identical.
        assert_eq!(waypoint.program.segments[0], seed.segments[0]);
        assert_eq!(
            waypoint.program.segments[0].origin(),
            &OperationId("op-pick-a".to_string())
        );
        // The Wait segment (index 1) splits into two MoveL halves — BOTH keep
        // the Wait origin (operation kind + target unchanged).
        match (&waypoint.program.segments[1], &waypoint.program.segments[2]) {
            (
                MotionSegment::MoveL {
                    origin: o1,
                    target_pose: first,
                    ..
                },
                MotionSegment::MoveL {
                    origin: o2,
                    target_pose: second,
                    ..
                },
            ) => {
                assert_eq!(o1, &OperationId("op-wait".to_string()));
                assert_eq!(o2, &OperationId("op-wait".to_string()));
                // C0: the shared waypoint.
                assert!(
                    (first.translation().z - (-0.06)).abs() < 1e-9
                        && (second.translation().z - (-0.12)).abs() < 1e-9
                );
            }
            other => panic!("expected two MoveL halves, got {other:?}"),
        }
        assert_eq!(waypoint.program.segments[3], seed.segments[2]);
        assert_eq!(waypoint.program.segments[4], seed.segments[3]);

        // Endpoint identity — joint goal per joint within ε (NOT TCP pose).
        let seed_goal = goal_joints(&seed).expect("seed ends in a MoveJ goal");
        let cand_goal = goal_joints(&waypoint.program).expect("candidate ends in a MoveJ goal");
        assert_eq!(seed_goal.len(), cand_goal.len());
        for (qc, qs) in cand_goal.iter().zip(&seed_goal) {
            assert!(
                (qc - qs).abs() <= ENDPOINT_TOLERANCE,
                "goal joint drift: |{qc} - {qs}| > ε = {ENDPOINT_TOLERANCE}"
            );
        }

        // Endpoint identity — the first commanded configuration per joint
        // within ε (guards strategies against touching the head segment).
        let seed_first = first_commanded_joints(&seed).expect("seed starts with a MoveJ");
        let cand_first =
            first_commanded_joints(&waypoint.program).expect("candidate starts with a MoveJ");
        assert_eq!(seed_first.len(), cand_first.len());
        for (qc, qs) in cand_first.iter().zip(&seed_first) {
            assert!(
                (qc - qs).abs() <= ENDPOINT_TOLERANCE,
                "start joint drift: |{qc} - {qs}| > ε = {ENDPOINT_TOLERANCE}"
            );
        }
    }
}
