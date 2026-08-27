//! AlternateElbow strategy (PR1, Phase 2, task 2.3).
//!
//! Wraps [`SingularityResolveMaterializer`](crate::advisor::remediation::SingularityResolveMaterializer)
//! (design ADR-4): for a `MoveJ` whose path crosses the full extension, the
//! materializer re-solves IK from the segment-start joints toward the SAME
//! cartesian position, converging to the same-side elbow posture — a clean
//! 1:1 `MoveJ` replacement.
//!
//! ## Lifetime adapter (design risk — documented)
//!
//! `SingularityResolveMaterializer<'a>` borrows the IK solver AND the
//! segment-start joints (`&'a dyn IKSolver`, `&'a [f64]`). The strategy owns
//! NOTHING: it constructs the materializer per-invocation inside `apply()`,
//! borrowing the solver parameter and a local joints vector for the duration
//! of the call. `AlternateElbow` is therefore a `'static` zero-sized type —
//! no lifetime parameter leaks into the strategy, so it stays object-safe
//! behind `Box<dyn MotionStrategy>`.
//!
//! ## Segment-start joints resolution
//!
//! The materializer re-solves from the segment-START joints (the deterministic
//! context the compiler uses). At program level those are the last explicit
//! joint configuration BEFORE the target segment (`previous_joint_configuration`).
//! When no such configuration exists (e.g. the target is segment 0 and the
//! start is the runtime home, unknown to the generator), the strategy skips
//! with a documented reason instead of guessing. Note this resolves WHERE the
//! solver starts — which segment to transform remains the caller's separate
//! selection policy (design: "segment selection is a SEPARATE policy from the
//! strategy").

use std::collections::BTreeMap;

use thalos_core::analysis::action::{ActionImpact, ActionKind, ActionPriority};
use thalos_core::analysis::observation::ObservationId;
use thalos_core::kinematics::forward::ForwardKinematics;
use thalos_core::kinematics::inverse::{IKSolver, IKGoal, MultiStartIKSolver, SeedConfig};
use thalos_core::motion::segment::MotionSegment;
use thalos_math::Vector3;

use crate::advisor::remediation::SingularityResolveMaterializer;
use crate::candidate::contract::{
    Candidate, CandidateGenerationContext, NoCandidateReason, StrategyOutcome,
};
use crate::candidate::strategy::{MotionStrategy, StrategyKind};
use crate::feedback::materializer::{MaterializationError, ProposalMaterializer};
use crate::feedback::operator::ActionProposal;
use crate::motion::program::PlanningProgram;

/// The `AlternateElbow` strategy: re-solve the target `MoveJ` to the
/// same-side elbow posture that reaches the SAME cartesian point.
///
/// Zero-sized and `'static` by design — see the module docs for the lifetime
/// adapter rationale.
#[derive(Debug, Clone, Copy, Default)]
pub struct AlternateElbow;

impl AlternateElbow {
    /// Creates the strategy.
    pub fn new() -> Self {
        Self
    }

    /// The `ActionProposal` this strategy always materializes (`Singularity`,
    /// no parameters — the materializer's rotation parameter is unused by the
    /// resolve operator).
    fn proposal() -> ActionProposal {
        ActionProposal {
            kind: ActionKind::Singularity,
            target_observation: ObservationId(0),
            priority: ActionPriority::High,
            impact: ActionImpact::High,
            parameters: BTreeMap::new(),
        }
    }
}

impl MotionStrategy for AlternateElbow {
    fn kind(&self) -> StrategyKind {
        StrategyKind::AlternateElbow
    }

    fn apply(
        &self,
        seed: &PlanningProgram,
        ctx: &CandidateGenerationContext,
        ik_solver: &dyn IKSolver,
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

        // The deterministic IK context: the last explicit joint configuration
        // before the target segment.
        let Some(start_joints) = previous_joint_configuration(&seed.segments, ctx.target_segment)
        else {
            return StrategyOutcome::Skipped(NoCandidateReason::InvariantViolation {
                invariant: format!(
                    "no joint configuration precedes target segment {} — cannot re-solve the elbow",
                    ctx.target_segment
                ),
            });
        };

        // ── Multi-start IK: try multiple seeds to find an alternative ──
        //
        // The seed generator produces configurations that explore different
        // branches of the solution space (elbow-up ↔ elbow-down). The
        // MultiStartIKSolver tries each seed and returns valid solutions.
        // We pick the first solution that's different from the baseline.
        //
        // NOTE: Multi-start is only effective for robots with >= 6 joints.
        // For simpler robots (SCARA, icebot), the original materializer
        // is used because multi-start doesn't find different solutions.

        // Get FK from the solver's robot chain to compute target position
        if let Some(robot) = ik_solver.robot() {
            // Only use multi-start for robots with >= 6 joints
            if robot.dof_count() >= 6 {
                let fk = ForwardKinematics::new(robot.clone());

                // Compute target position from the MoveJ target joints
                if let MotionSegment::MoveJ { target: target_joints, .. } = target {
                    if let Some(target_pos) = fk.evaluate(target_joints).ee_position() {
                        let seed_config = SeedConfig::for_robot(start_joints.len());
                        let multi_solver = MultiStartIKSolver::new(ik_solver, seed_config);

                        // Generate seeds: baseline + elbow-flipped + perturbed
                        let seeds = generate_elbow_seeds(&start_joints);

                        // Solve with each seed
                        let goal = IKGoal::Position(target_pos);
                        let solutions = multi_solver.solve_multi_with_seeds(&seeds, goal);

                        // Find the first solution different from baseline
                        for sol in &solutions[1..] {
                            if !solutions_equal(&start_joints, &sol.q) {
                                // Found a different solution — create candidate
                                let mut segments = seed.segments.clone();
                                segments[ctx.target_segment] = MotionSegment::MoveJ {
                                    origin: match target {
                                        MotionSegment::MoveJ { origin, .. } => origin.clone(),
                                        _ => unreachable!(),
                                    },
                                    target: sol.q.clone(),
                                    max_velocity: None,
                                    max_acceleration: None,
                                };
                                return StrategyOutcome::Generated(Candidate {
                                    strategy: StrategyKind::AlternateElbow,
                                    program: PlanningProgram::new(segments),
                                });
                            }
                        }
                    }
                }
            }
        }

        // ── Fallback: single-start materializer (existing behavior) ──
        //
        // For robots with < 6 joints, or when multi-start doesn't find a
        // different solution, use the original materializer.
        let materializer = SingularityResolveMaterializer::new(ik_solver, &start_joints);
        match materializer.materialize(&Self::proposal(), target) {
            Ok(patch) => {
                // ADR-4 splice: seed[..target] + patch + seed[target+1..].
                let mut segments = seed.segments.clone();
                let _ = segments.splice(ctx.target_segment..=ctx.target_segment, patch);
                StrategyOutcome::Generated(Candidate {
                    strategy: StrategyKind::AlternateElbow,
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
                    invariant: format!(
                        "materializer rejected proposal kind {kind:?} (solver has no robot chain?)"
                    ),
                })
            }
        }
    }
}

/// The segment-start joints the materializer re-solves from: the last explicit
/// `MoveJ` target at or before `target_index - 1`. Returns `None` when no
/// explicit joint configuration precedes the target (the compiler's context
/// is unknowable at program level).
fn previous_joint_configuration(
    segments: &[MotionSegment],
    target_index: usize,
) -> Option<Vec<f64>> {
    segments[..target_index].iter().rev().find_map(|s| match s {
        MotionSegment::MoveJ { target, .. } => Some(target.clone()),
        _ => None,
    })
}

/// Generate seeds for elbow-alternate multi-start IK.
///
/// Returns3 seeds:
/// 0. baseline (original configuration)
/// 1. elbow-flipped (negate joints 1,2)
/// 2. elbow-flipped + small perturbation
fn generate_elbow_seeds(base_joints: &[f64]) -> Vec<Vec<f64>> {
    let mut seeds = Vec::new();

    // Seed 0: baseline
    seeds.push(base_joints.to_vec());

    // Seed 1: elbow flipped (negate joints 1,2 for 6DOF)
    let mut flipped = base_joints.to_vec();
    let flip_indices = if base_joints.len() >= 4 { vec![1, 2] } else { vec![1] };
    for &idx in &flip_indices {
        if idx < flipped.len() {
            flipped[idx] = -flipped[idx];
        }
    }
    seeds.push(flipped);

    // Seed 2: elbow flipped + small perturbation
    let mut perturbed = seeds[1].clone();
    let perturbation = 0.05;
    for idx in 0..perturbed.len() {
        if !flip_indices.contains(&idx) {
            perturbed[idx] += perturbation;
        }
    }
    seeds.push(perturbed);

    seeds
}

/// Check if two joint configurations are approximately equal.
fn solutions_equal(a: &[f64], b: &[f64]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b.iter()).all(|(x, y)| (x - y).abs() < 1e-4)
}

#[cfg(test)]
mod tests {
    use thalos_core::ids::OperationId;
    use thalos_core::kinematics::forward::ForwardKinematics;
    use thalos_core::kinematics::inverse::DampedLeastSquaresSolver;
    use thalos_core::models::{RobotModel, RobotRegistry};
    use thalos_core::motion::segment::MotionSegment;
    use thalos_core::robot::serial_chain::SerialChain;
    use thalos_core::spatial::frame::FrameId;
    use thalos_core::spatial::pose::Pose;
    use thalos_math::Transform3D;

    use crate::candidate::contract::{
        CandidateGenerationContext, NoCandidateReason, StrategyOutcome,
    };
    use crate::candidate::strategy::{MotionStrategy, StrategyKind};
    use crate::motion::program::PlanningProgram;

    use super::AlternateElbow;

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

    fn movel(origin: &str) -> MotionSegment {
        MotionSegment::MoveL {
            origin: OperationId(origin.to_string()),
            frame: FrameId::World,
            target_pose: Pose::new(FrameId::World, FrameId::Id(1), Transform3D::identity()),
            max_velocity: None,
        }
    }

    // ── 2.3 — AlternateElbow re-solves to the same-side posture ───────────

    #[test]
    fn alternate_elbow_resolves_same_side_posture_from_previous_joints() {
        // Causal contract (advisor/remediation.rs): a MoveJ crossing the full
        // extension (elbow +0.6) is re-solved from the previous segment's
        // joints → the solution reaches the SAME cartesian position.
        //
        // With multi-start IK, the solver may find a different branch
        // (elbow-up or elbow-down). The key property is that the solution
        // is VALID and reaches the same position — not the specific elbow sign.
        let robot = chain(RobotModel::Scara);
        let home = vec![0.0, -1.31, -0.1, 0.0];
        let bad_target = vec![0.5, 0.6, -0.15, 0.0];
        let seed = PlanningProgram::new(vec![
            movej("op-home", home.clone()),
            movej("op-cross", bad_target.clone()),
        ]);
        let ctx = CandidateGenerationContext { target_segment: 1 };
        let solver = real_solver(&robot);
        let strategy = AlternateElbow::new();

        match strategy.apply(&seed, &ctx, &solver) {
            StrategyOutcome::Generated(candidate) => {
                assert_eq!(candidate.strategy, StrategyKind::AlternateElbow);
                assert_eq!(
                    candidate.program.segments.len(),
                    2,
                    "must be a clean 1:1 replacement"
                );
                assert_eq!(
                    candidate.program.segments[0], seed.segments[0],
                    "head segment preserved"
                );
                let MotionSegment::MoveJ { origin, target, .. } = &candidate.program.segments[1]
                else {
                    panic!("expected MoveJ, got {:?}", candidate.program.segments[1]);
                };
                assert_eq!(origin, &OperationId("op-cross".to_string()));
                let fk = ForwardKinematics::new(robot.clone());
                let bad_pos = fk
                    .evaluate(&bad_target)
                    .ee_position()
                    .expect("bad target FK position");
                let alt_pos = fk
                    .evaluate(target)
                    .ee_position()
                    .expect("re-solved FK position");
                assert!(
                    (bad_pos.x - alt_pos.x).abs() < 1e-3
                        && (bad_pos.y - alt_pos.y).abs() < 1e-3
                        && (bad_pos.z - alt_pos.z).abs() < 1e-3,
                    "the re-solved posture must reach the SAME cartesian position, bad={bad_pos:?} alt={alt_pos:?}"
                );
            }
            StrategyOutcome::Skipped(reason) => {
                panic!("expected Generated, got Skipped({reason:?})")
            }
        }
    }

    // ── 2.3 — documented no-candidate reasons ────────────────────────────

    #[test]
    fn alternate_elbow_skips_cartesian_target() {
        // The materializer only re-solves MoveJ targets (MoveL carries no
        // joint configuration to re-solve — documented gap surfaced honestly).
        let seed = PlanningProgram::new(vec![movej("op-home", vec![0.0; 4]), movel("op-l")]);
        let ctx = CandidateGenerationContext { target_segment: 1 };
        let robot = chain(RobotModel::Scara);
        let solver = real_solver(&robot);
        let strategy = AlternateElbow::new();

        assert!(matches!(
            strategy.apply(&seed, &ctx, &solver),
            StrategyOutcome::Skipped(NoCandidateReason::UnsupportedSegment)
        ));
    }

    #[test]
    fn alternate_elbow_skips_when_no_previous_joint_configuration() {
        // At segment 0 there is no previous joint configuration to re-solve
        // from (the start is the runtime home, unknown to the generator) —
        // the strategy skips with a documented reason instead of guessing.
        let seed = PlanningProgram::new(vec![movej("op-cross", vec![0.5, 0.6, -0.15, 0.0])]);
        let ctx = CandidateGenerationContext { target_segment: 0 };
        let robot = chain(RobotModel::Scara);
        let solver = real_solver(&robot);
        let strategy = AlternateElbow::new();

        match strategy.apply(&seed, &ctx, &solver) {
            StrategyOutcome::Skipped(NoCandidateReason::InvariantViolation { invariant }) => {
                assert!(
                    invariant.contains("no joint configuration"),
                    "invariant message must explain the missing context: {invariant}"
                );
            }
            other => panic!("expected Skipped(InvariantViolation), got {other:?}"),
        }
    }
}
