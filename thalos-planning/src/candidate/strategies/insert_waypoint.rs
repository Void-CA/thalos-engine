//! InsertWaypoint strategy (PR1, Phase 2, task 2.2).
//!
//! Wraps [`InsertWaypointMaterializer`](crate::feedback::materializer::InsertWaypointMaterializer)
//! (design ADR-4): the materializer emits a `Vec<MotionSegment>` PATCH; the
//! strategy splices it back into the seed at `ctx.target_segment`, building a
//! full alternative [`PlanningProgram`] — the seed's structure and every other
//! segment are preserved. The strategy produces at most one candidate and
//! records a documented no-candidate reason otherwise (spec
//! candidate-generation "Bounded Strategy Library").

use std::collections::BTreeMap;

use thalos_core::analysis::action::{ActionImpact, ActionKind, ActionPriority};
use thalos_core::analysis::attribute_value::AttributeValue;
use thalos_core::analysis::observation::ObservationId;
use thalos_core::kinematics::inverse::IKSolver;

use crate::candidate::contract::{
    Candidate, CandidateGenerationContext, NoCandidateReason, StrategyOutcome,
};
use crate::candidate::strategy::{MotionStrategy, StrategyKind};
use crate::feedback::materializer::{
    InsertWaypointMaterializer, MaterializationError, ProposalMaterializer,
};
use crate::feedback::operator::ActionProposal;
use crate::motion::program::PlanningProgram;

/// The `InsertWaypoint` strategy: split the target `MoveL` by inserting an
/// intermediate waypoint at half the straight path (C0 continuous).
///
/// Zero-sized by design: the wrapped materializer needs no state, so nothing
/// is borrowed across calls.
#[derive(Debug, Clone, Copy, Default)]
pub struct InsertWaypoint;

impl InsertWaypoint {
    /// Creates the strategy.
    pub fn new() -> Self {
        Self
    }

    /// The split fraction passed to the materializer (default 0.5).
    pub const DEFAULT_FRACTION: f64 = InsertWaypointMaterializer::DEFAULT_FRACTION;

    /// The `ActionProposal` this strategy always materializes. Generation is
    /// NOT remediation: the proposal is a fixed shape (`Waypoint` at the
    /// default fraction) and the observation id is a placeholder — the
    /// strategy never invents parameters (the materializer stays
    /// parameter-blind, contract C4).
    fn proposal() -> ActionProposal {
        ActionProposal {
            kind: ActionKind::Waypoint,
            target_observation: ObservationId(0),
            priority: ActionPriority::High,
            impact: ActionImpact::High,
            parameters: BTreeMap::from([(
                "fraction".to_string(),
                AttributeValue::Number(Self::DEFAULT_FRACTION),
            )]),
        }
    }
}

impl MotionStrategy for InsertWaypoint {
    fn kind(&self) -> StrategyKind {
        StrategyKind::InsertWaypoint
    }

    fn apply(
        &self,
        seed: &PlanningProgram,
        ctx: &CandidateGenerationContext,
        _ik_solver: &dyn IKSolver,
    ) -> StrategyOutcome {
        let Some(target) = seed.segments.get(ctx.target_segment) else {
            return StrategyOutcome::Skipped(NoCandidateReason::InvariantViolation {
                invariant: format!(
                    "target_segment {} out of bounds (program has {} segments)",
                    ctx.target_segment,
                    seed.segments.len()
                ),
            });
        };

        match InsertWaypointMaterializer::new().materialize(&Self::proposal(), target) {
            Ok(patch) => {
                // ADR-4 splice: seed[..target] + patch + seed[target+1..].
                let mut segments = seed.segments.clone();
                let _ = segments.splice(ctx.target_segment..=ctx.target_segment, patch);
                StrategyOutcome::Generated(Candidate {
                    strategy: StrategyKind::InsertWaypoint,
                    program: PlanningProgram::new(segments),
                })
            }
            Err(MaterializationError::IkFailure) => {
                StrategyOutcome::Skipped(NoCandidateReason::IkFailed)
            }
            Err(MaterializationError::UnsupportedSegment) => {
                StrategyOutcome::Skipped(NoCandidateReason::UnsupportedSegment)
            }
            Err(MaterializationError::UnsupportedProposal { kind }) => {
                StrategyOutcome::Skipped(NoCandidateReason::InvariantViolation {
                    invariant: format!("materializer rejected proposal kind {kind:?}"),
                })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use thalos_core::ids::OperationId;
    use thalos_core::kinematics::inverse::{IKGoal, IKResult, IKSolver, IkError};
    use thalos_core::motion::segment::MotionSegment;
    use thalos_core::spatial::frame::FrameId;
    use thalos_core::spatial::pose::Pose;
    use thalos_math::{Transform3D, Vector3};

    use crate::candidate::contract::{
        CandidateGenerationContext, NoCandidateReason, StrategyOutcome,
    };
    use crate::candidate::strategy::{MotionStrategy, StrategyKind};
    use crate::motion::program::PlanningProgram;

    use super::InsertWaypoint;

    struct NoopIKSolver;

    impl IKSolver for NoopIKSolver {
        fn solve(&self, q0: &[f64], _goal: IKGoal) -> Result<IKResult, IkError> {
            Ok(IKResult::converged(q0.to_vec(), 1, 0.0, None))
        }
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

    /// Seed `[MoveJ(start), MoveL(mid), MoveJ(home)]` — a Cartesian target at
    /// the middle segment, joint targets around it.
    fn three_segment_seed() -> PlanningProgram {
        PlanningProgram::new(vec![
            movej("op-start", vec![0.0, -1.31, -0.1, 0.0]),
            movel("op-mid", pose_at(0.3, 0.4, -0.12)),
            movej("op-home", vec![0.0, -1.31, -0.1, 0.0]),
        ])
    }

    // ── 2.2 — InsertWaypoint generates a C0-split candidate ───────────────

    #[test]
    fn insert_waypoint_splits_target_segment_preserving_c0() {
        // Spec candidate-generation "Bounded Strategy Library": the strategy
        // produces ≤1 candidate by splitting the target MoveL at fraction 0.5
        // — the first half ends exactly where the second starts (C0).
        let seed = three_segment_seed();
        let ctx = CandidateGenerationContext { target_segment: 1 };
        let solver = NoopIKSolver;
        let strategy = InsertWaypoint::new();

        match strategy.apply(&seed, &ctx, &solver) {
            StrategyOutcome::Generated(candidate) => {
                assert_eq!(candidate.strategy, StrategyKind::InsertWaypoint);
                assert_eq!(
                    candidate.program.segments.len(),
                    4,
                    "one segment becomes two"
                );
                // Head and tail are preserved verbatim (ADR-4 splice).
                assert_eq!(candidate.program.segments[0], seed.segments[0]);
                assert_eq!(candidate.program.segments[3], seed.segments[2]);
                // The two halves keep the target's origin (task identity) and
                // share the interpolated waypoint (C0).
                match (
                    &candidate.program.segments[1],
                    &candidate.program.segments[2],
                ) {
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
                        assert_eq!(o1, &OperationId("op-mid".to_string()));
                        assert_eq!(o2, &OperationId("op-mid".to_string()));
                        assert!(
                            (first.translation().z - (-0.06)).abs() < 1e-9,
                            "first half must end at the halfway waypoint"
                        );
                        assert!(
                            (second.translation().z - (-0.12)).abs() < 1e-9,
                            "second half must keep the original target"
                        );
                    }
                    other => panic!("expected two MoveL halves, got {other:?}"),
                }
            }
            StrategyOutcome::Skipped(reason) => {
                panic!("expected Generated, got Skipped({reason:?})")
            }
        }
    }

    #[test]
    fn insert_waypoint_preserves_seed_structure() {
        // ADR-4 mismatch resolution: the materializer emits a PATCH; the
        // strategy splices it into a full program — seed structure and all
        // non-target segments preserved.
        let seed = three_segment_seed();
        let ctx = CandidateGenerationContext { target_segment: 1 };
        let solver = NoopIKSolver;
        let strategy = InsertWaypoint::new();

        let StrategyOutcome::Generated(candidate) = strategy.apply(&seed, &ctx, &solver) else {
            panic!("must generate");
        };
        assert_eq!(candidate.program.segments[0], seed.segments[0]);
        assert_eq!(candidate.program.segments[3], seed.segments[2]);
    }

    // ── 2.2 — documented no-candidate reasons ────────────────────────────

    #[test]
    fn insert_waypoint_skips_joint_space_target() {
        // The materializer cannot insert a Cartesian waypoint into a MoveJ —
        // documented no-candidate reason, never a silent drop.
        let seed = PlanningProgram::new(vec![
            movej("op-a", vec![0.1, 0.2]),
            movej("op-b", vec![0.5, 0.6]),
        ]);
        let ctx = CandidateGenerationContext { target_segment: 1 };
        let solver = NoopIKSolver;
        let strategy = InsertWaypoint::new();

        assert!(matches!(
            strategy.apply(&seed, &ctx, &solver),
            StrategyOutcome::Skipped(NoCandidateReason::UnsupportedSegment)
        ));
    }

    #[test]
    fn insert_waypoint_skips_out_of_bounds_target() {
        let seed = three_segment_seed();
        let ctx = CandidateGenerationContext { target_segment: 9 };
        let solver = NoopIKSolver;
        let strategy = InsertWaypoint::new();

        match strategy.apply(&seed, &ctx, &solver) {
            StrategyOutcome::Skipped(NoCandidateReason::InvariantViolation { invariant }) => {
                assert!(
                    invariant.contains("out of bounds"),
                    "invariant message must explain the violation: {invariant}"
                );
            }
            other => panic!("expected Skipped(InvariantViolation), got {other:?}"),
        }
    }
}
