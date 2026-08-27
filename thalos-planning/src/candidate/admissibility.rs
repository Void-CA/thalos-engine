//! Admissibility gate (PR2, Phase 3, tasks 3.3 + 3.4; design ADR-5, spec
//! candidate-evaluation "Admissibility Gate Before Ranking").
//!
//! One module, TWO phases — conceptually distinct (design ADR-5):
//!
//! - **Phase 1 — geometric invariants** (planning-owned, properties of the
//!   candidate program): compile OK, endpoint `|q_cand − q_seed| ≤ ε` per
//!   joint (ADR-1), joint limits, reachability, task identity.
//! - **Phase 2 — risk policy** (a POLICY on the Assessor's categorical
//!   verdict): `RiskAdmissibility::Rejected` (the runtime's mapping of
//!   `Assessment.risk == Critical`) → inadmissible.
//!
//! # Precedence (explicit)
//!
//! 1. Compile failure → **NO Assessment** exists → inadmissible in phase 1.
//!    A compile failure is rejected even if an assessment were (incorrectly)
//!    attached — a non-compiling candidate NEVER reaches the risk policy.
//! 2. Valid geometry → the Assessor ran → `Critical` → the assessment EXISTS
//!    and is KEPT for trace ([`RejectedCandidate::assessment`]) but the
//!    candidate cannot enter ranking.
//! 3. All candidates inadmissible → the report's admissible list is empty →
//!    the evaluator reports [`SelectionReason::NoAdmissibleCandidate`]
//!    (never a "least bad" fallback).
//!
//! # What each invariant proves (and where it is proven)
//!
//! The runtime pipeline (PR3) compiles each candidate BEFORE the gate runs.
//! Compilation is the single authority on **reachability** (IK convergence —
//! `GoalResolver` fails on `MaxIterations`) and **C0 continuity** (the
//! compiler concatenates trajectories with continuous joint positions) and
//! validates **joint limits** for resolved goals (`GoalResolverConfig`
//! defaults `check_joint_limits: true`). The gate consumes `compile_ok` as
//! those verdicts and re-checks the invariants that are decidable from the
//! programs themselves: endpoint ε, joint limits of commanded `MoveJ`
//! targets (defense-in-depth), and task identity.
//!
//! Task identity compares the COMPACTED semantic sequence `(kind, origin)` of
//! seed vs candidate: consecutive segments sharing kind AND origin are one
//! run. This tolerates the bounded strategies' splits (InsertWaypoint splits
//! ONE segment into two halves with the same origin) while rejecting
//! reordering, deletion, kind changes, or new origins. Origins ARE the task
//! target identity (spec candidate-generation "Generator changes Pick/Place
//! target"); intermediate joint values/poses are "geometric realization" and
//! MAY vary.

use crate::candidate::contract::{
    AdmissibleCandidate, Candidate, CandidateAssessment, ENDPOINT_TOLERANCE, MotionMetrics,
    RiskAdmissibility,
};
use crate::motion::program::PlanningProgram;

/// Closed joint bounds `[lower, upper]` for one joint, supplied by the
/// runtime from the robot chain. Closed interval: `lower ≤ q ≤ upper`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointBounds {
    /// Minimum joint value (rad).
    pub lower: f64,
    /// Maximum joint value (rad).
    pub upper: f64,
}

/// A candidate entering the gate with the runtime's pre-gate verdicts.
///
/// The runtime (PR3) compiles, analyzes, and assesses each generated
/// candidate, then maps the frozen `Assessment` into the neutral
/// [`CandidateAssessment`]. A candidate whose compile failed has
/// `compile_ok == false` and — by precedence — NO assessment.
#[derive(Debug, Clone, PartialEq)]
pub struct GateCandidate {
    /// The candidate program under evaluation.
    pub candidate: Candidate,
    /// Whether the runtime compiler accepted the program. `false` proves
    /// reachability (IK convergence), joint-limit validation, and C0
    /// continuity were NOT established.
    pub compile_ok: bool,
    /// The neutral assessment — `Some` only for compiled candidates (valid
    /// geometry → the Assessor ran). `None` when compile failed or the
    /// pipeline contract was violated.
    pub assessment: Option<CandidateAssessment>,
    /// Motion metrics extracted from the analyzed trajectory — `Some` only
    /// for compiled candidates.
    pub metrics: Option<MotionMetrics>,
}

/// Why a candidate was rejected. Every variant is STRUCTURAL — the gate never
/// emits narrative text.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectionReason {
    /// Phase 1 — the runtime compiler rejected the program. Carries the
    /// reachability verdict (IK did not converge), compile-time joint-limit
    /// validation, and C0 continuity. No assessment exists.
    CompileFailure,
    /// Phase 1 — `|q_cand − q_seed| > ε` on some joint (first commanded
    /// configuration or the joint goal, ADR-1).
    EndpointDrift,
    /// Phase 1 — a commanded `MoveJ` joint lies outside `[lower, upper]`.
    JointLimitViolation,
    /// Phase 1 — the semantic task sequence `(kind, origin)` differs from
    /// the seed after compaction.
    TaskMismatch,
    /// Phase 2 — compiled but no neutral assessment was provided (broken
    /// pipeline contract) → the risk policy cannot be evaluated, fail closed.
    MissingAssessment,
    /// Phase 2 — `RiskAdmissibility::Rejected` (the runtime's mapping of
    /// `Assessment.risk == Critical`). The assessment EXISTS and is kept.
    RiskRejected,
}

/// The gate phase that rejected the candidate (design ADR-5: Critical is NOT
/// a geometric property — the phases are conceptually distinct).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectionPhase {
    /// Phase 1 — geometric invariants (planning-owned).
    Geometric,
    /// Phase 2 — risk policy (consumes the neutral assessment).
    Risk,
}

/// A candidate rejected by the gate, kept for trace.
#[derive(Debug, Clone, PartialEq)]
pub struct RejectedCandidate {
    /// The rejected candidate.
    pub candidate: Candidate,
    /// Which gate phase rejected it.
    pub phase: RejectionPhase,
    /// The structural reason.
    pub reason: RejectionReason,
    /// The neutral assessment when one exists — `Some` for risk-rejected
    /// (Critical) candidates, whose assessment MUST be kept for trace even
    /// though they cannot enter ranking.
    pub assessment: Option<CandidateAssessment>,
    /// The motion metrics when analysis ran.
    pub metrics: Option<MotionMetrics>,
}

/// The full gate outcome: everything that passed both phases, plus every
/// rejection (traceable).
#[derive(Debug, Clone, PartialEq)]
pub struct AdmissibilityReport {
    /// Candidates that passed both phases — the evaluator's input.
    pub admissible: Vec<AdmissibleCandidate>,
    /// Candidates rejected by either phase, with reasons and (for
    /// risk-rejected ones) their kept assessments.
    pub rejected: Vec<RejectedCandidate>,
}

/// Two-phase admissibility gate (design ADR-5).
pub struct AdmissibilityGate;

impl AdmissibilityGate {
    /// Filter the gate-entry candidates against the seed.
    ///
    /// `seed` is the reference for the endpoint-ε and task-identity
    /// invariants (the equivalence class the candidates belong to). The
    /// baseline `Direct` candidate IS the seed, so it always passes phase 1
    /// against itself.
    ///
    /// `joint_limits` — the robot chain's per-joint bounds; `None` skips the
    /// joint-limit re-check (it is always validated at compile time by the
    /// default `GoalResolverConfig`). The check covers commanded `MoveJ`
    /// targets; `MoveL`/`MoveLPosition` resolved configurations are
    /// compile-proven (the gate never re-runs IK).
    pub fn filter(
        &self,
        seed: &PlanningProgram,
        candidates: &[GateCandidate],
        joint_limits: Option<&[JointBounds]>,
    ) -> AdmissibilityReport {
        let mut admissible = Vec::new();
        let mut rejected = Vec::new();

        for row in candidates {
            match gate_candidate(seed, row, joint_limits) {
                GateVerdict::Admissible => {
                    let assessment = row
                        .assessment
                        .clone()
                        .expect("admissible rows always carry an assessment (gate invariant)");
                    let metrics = row
                        .metrics
                        .clone()
                        .expect("admissible rows always carry metrics (gate invariant)");
                    admissible.push(AdmissibleCandidate {
                        candidate: row.candidate.clone(),
                        assessment,
                        metrics,
                    });
                }
                GateVerdict::Rejected(phase, reason) => rejected.push(RejectedCandidate {
                    candidate: row.candidate.clone(),
                    phase,
                    reason,
                    assessment: row.assessment.clone(),
                    metrics: row.metrics.clone(),
                }),
            }
        }

        AdmissibilityReport {
            admissible,
            rejected,
        }
    }
}

/// The per-candidate verdict: passes both phases, or which phase + reason
/// rejected it.
enum GateVerdict {
    Admissible,
    Rejected(RejectionPhase, RejectionReason),
}

/// Two-phase gate with EXPLICIT precedence: compile failure (phase 1) is
/// checked FIRST and rejects with no assessment; the risk policy (phase 2)
/// only ever sees valid geometry.
fn gate_candidate(
    seed: &PlanningProgram,
    row: &GateCandidate,
    joint_limits: Option<&[JointBounds]>,
) -> GateVerdict {
    // ── Phase 1 — geometric invariants ───────────────────────────────────
    if !row.compile_ok {
        return GateVerdict::Rejected(RejectionPhase::Geometric, RejectionReason::CompileFailure);
    }
    if !endpoints_within_epsilon(seed, &row.candidate.program) {
        return GateVerdict::Rejected(RejectionPhase::Geometric, RejectionReason::EndpointDrift);
    }
    if !commanded_joints_within_limits(&row.candidate.program, joint_limits) {
        return GateVerdict::Rejected(
            RejectionPhase::Geometric,
            RejectionReason::JointLimitViolation,
        );
    }
    if !task_sequence_identical(seed, &row.candidate.program) {
        return GateVerdict::Rejected(RejectionPhase::Geometric, RejectionReason::TaskMismatch);
    }

    // ── Phase 2 — risk policy (only valid geometry reaches this point) ───
    match &row.assessment {
        Some(assessment) => {
            if assessment.admissibility == RiskAdmissibility::Rejected {
                // Critical — the Assessor's categorical verdict, mapped by
                // the runtime. The assessment is kept by the caller for trace.
                GateVerdict::Rejected(RejectionPhase::Risk, RejectionReason::RiskRejected)
            } else {
                GateVerdict::Admissible
            }
        }
        None => GateVerdict::Rejected(RejectionPhase::Risk, RejectionReason::MissingAssessment),
    }
}

/// Phase 1 endpoint invariant (spec candidate-generation "Endpoint identity",
/// ADR-1): the first commanded `MoveJ` configuration and the joint goal (last
/// `MoveJ` target) of the candidate must each be within `ε` of the seed's,
/// per joint.
///
/// Both programs lacking a joint goal (or a first `MoveJ`) compare equal by
/// absence (the equivalence class has no joint endpoint to preserve); a
/// mismatch — one has an endpoint the other lacks — is a drift.
fn endpoints_within_epsilon(seed: &PlanningProgram, candidate_program: &PlanningProgram) -> bool {
    let (seed_first, seed_goal) = commanded_endpoints(seed);
    let (cand_first, cand_goal) = commanded_endpoints(candidate_program);
    within_epsilon(seed_first.as_deref(), cand_first.as_deref())
        && within_epsilon(seed_goal.as_deref(), cand_goal.as_deref())
}

/// `(first_commanded_joints, joint_goal)` — the first and last explicit
/// `MoveJ` targets of a program (the joint goal, NOT the TCP pose).
fn commanded_endpoints(program: &PlanningProgram) -> (Option<Vec<f64>>, Option<Vec<f64>>) {
    let first = program.segments.iter().find_map(|s| match s {
        thalos_core::motion::segment::MotionSegment::MoveJ { target, .. } => Some(target.clone()),
        _ => None,
    });
    let goal = program.segments.iter().rev().find_map(|s| match s {
        thalos_core::motion::segment::MotionSegment::MoveJ { target, .. } => Some(target.clone()),
        _ => None,
    });
    (first, goal)
}

/// Per-joint `|q_cand − q_seed| ≤ ε`. Both `None` → equal by absence; one
/// `None` → drift; mismatched lengths → drift (cannot verify per joint).
fn within_epsilon(seed: Option<&[f64]>, candidate: Option<&[f64]>) -> bool {
    match (seed, candidate) {
        (None, None) => true,
        (Some(s), Some(c)) => {
            s.len() == c.len()
                && s.iter()
                    .zip(c.iter())
                    .all(|(qs, qc)| (qc - qs).abs() <= ENDPOINT_TOLERANCE)
        }
        _ => false,
    }
}

/// Phase 1 joint-limit invariant: every commanded `MoveJ` target value must
/// lie within `[lower, upper]` (closed interval). `None` limits skip the
/// re-check (compile-time validation already ran with the default resolver
/// config).
fn commanded_joints_within_limits(
    program: &PlanningProgram,
    joint_limits: Option<&[JointBounds]>,
) -> bool {
    let Some(limits) = joint_limits else {
        return true;
    };
    program.segments.iter().all(|s| match s {
        thalos_core::motion::segment::MotionSegment::MoveJ { target, .. } => target
            .iter()
            .zip(limits.iter())
            .all(|(q, bound)| *q >= bound.lower && *q <= bound.upper),
        // MoveL / MoveLPosition configurations are compile-proven (resolved
        // by IK under GoalResolver's joint-limit validation) — the gate never
        // re-runs IK.
        _ => true,
    })
}

/// A segment's geometric kind for task-identity comparison.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SegmentKind {
    MoveJ,
    MoveL,
    MoveLPosition,
}

impl SegmentKind {
    fn of(segment: &thalos_core::motion::segment::MotionSegment) -> Self {
        match segment {
            thalos_core::motion::segment::MotionSegment::MoveJ { .. } => SegmentKind::MoveJ,
            thalos_core::motion::segment::MotionSegment::MoveL { .. } => SegmentKind::MoveL,
            thalos_core::motion::segment::MotionSegment::MoveLPosition { .. } => {
                SegmentKind::MoveLPosition
            }
        }
    }
}

/// Phase 1 task-identity invariant: the COMPACTED semantic sequence
/// `(kind, origin)` must be identical to the seed's. Consecutive segments
/// sharing kind AND origin collapse into one run — this is exactly what the
/// bounded strategies produce when they split a segment (InsertWaypoint
/// splits ONE segment into two halves with the same origin, PR1 test 2.2),
/// while reordering, deletion, kind changes, or new origins break identity.
fn task_sequence_identical(seed: &PlanningProgram, candidate_program: &PlanningProgram) -> bool {
    compact(seed) == compact(candidate_program)
}

/// Collapse consecutive segments with equal `(kind, origin)` into runs.
fn compact(program: &PlanningProgram) -> Vec<(SegmentKind, String)> {
    let mut runs: Vec<(SegmentKind, String)> = Vec::new();
    for segment in &program.segments {
        let key = (SegmentKind::of(segment), segment.origin().0.clone());
        match runs.last_mut() {
            Some(last) if *last == key => {}
            _ => runs.push(key),
        }
    }
    runs
}

#[cfg(test)]
mod tests {
    use thalos_core::ids::OperationId;
    use thalos_core::motion::segment::MotionSegment;
    use thalos_core::spatial::frame::FrameId;
    use thalos_core::spatial::pose::Pose;
    use thalos_math::{Transform3D, Vector3};

    use crate::candidate::contract::{
        Candidate, CandidateAssessment, ENDPOINT_TOLERANCE, MotionMetrics, RiskAdmissibility,
    };
    use crate::candidate::strategy::StrategyKind;
    use crate::motion::program::PlanningProgram;

    use super::*;

    fn movej(origin: &str, target: Vec<f64>) -> MotionSegment {
        MotionSegment::MoveJ {
            origin: OperationId(origin.to_string()),
            target,
            max_velocity: None,
            max_acceleration: None,
        }
    }

    fn movel(origin: &str, x: f64, y: f64, z: f64) -> MotionSegment {
        MotionSegment::MoveL {
            origin: OperationId(origin.to_string()),
            frame: FrameId::World,
            target_pose: Pose::new(
                FrameId::World,
                FrameId::Id(1),
                Transform3D::from_translation(Vector3::new(x, y, z)),
            ),
            max_velocity: None,
        }
    }

    fn candidate(strategy: StrategyKind, program: PlanningProgram) -> Candidate {
        Candidate { strategy, program }
    }

    fn accepted(risk: f64) -> CandidateAssessment {
        CandidateAssessment {
            risk,
            admissibility: RiskAdmissibility::Accepted,
        }
    }

    fn critical(risk: f64) -> CandidateAssessment {
        CandidateAssessment {
            risk,
            admissibility: RiskAdmissibility::Rejected,
        }
    }

    fn metrics(duration: f64, avg_manipulability: f64, path_length: f64) -> MotionMetrics {
        MotionMetrics {
            duration,
            avg_manipulability,
            path_length,
        }
    }

    fn gate_row(
        candidate: Candidate,
        compile_ok: bool,
        assessment: Option<CandidateAssessment>,
        metrics: Option<MotionMetrics>,
    ) -> GateCandidate {
        GateCandidate {
            candidate,
            compile_ok,
            assessment,
            metrics,
        }
    }

    fn limits(n: usize, lower: f64, upper: f64) -> Vec<JointBounds> {
        (0..n).map(|_| JointBounds { lower, upper }).collect()
    }

    /// Seed `[MoveJ(op-start, [0,0]), MoveJ(op-goal, [0.5,0.4])]` — the
    /// canonical two-joint program used by most gate fixtures.
    fn seed_program() -> PlanningProgram {
        PlanningProgram::new(vec![
            movej("op-start", vec![0.0, 0.0]),
            movej("op-goal", vec![0.5, 0.4]),
        ])
    }

    // ── 3.3 — phase 1: compile failure → NO Assessment → inadmissible ────

    #[test]
    fn compile_failure_is_inadmissible_with_no_assessment() {
        let seed = seed_program();
        let failing = gate_row(
            candidate(
                StrategyKind::AlternateElbow,
                PlanningProgram::new(vec![movej("op-start", vec![0.0, 0.0])]),
            ),
            false,
            None,
            None,
        );

        let report = AdmissibilityGate.filter(
            &seed,
            std::slice::from_ref(&failing),
            Some(&limits(2, -1.0, 1.0)),
        );

        assert!(report.admissible.is_empty());
        assert_eq!(report.rejected.len(), 1);
        let row = &report.rejected[0];
        assert_eq!(row.reason, RejectionReason::CompileFailure);
        assert_eq!(row.phase, RejectionPhase::Geometric);
        assert!(
            row.assessment.is_none(),
            "compile failure → NO Assessment → nothing to keep for trace"
        );
        assert_eq!(row.candidate, failing.candidate);
    }

    #[test]
    fn reachability_failure_surfaces_as_compile_failure() {
        // Reachability (IK convergence) is decided by the runtime compiler:
        // a candidate whose waypoints fail IK never compiles. The gate treats
        // `compile_ok == false` as the reachability verdict — it never
        // re-runs IK (that would duplicate the compile step).
        let seed = seed_program();
        let row = gate_row(
            candidate(StrategyKind::InsertWaypoint, seed.clone()),
            false,
            None,
            None,
        );

        let report = AdmissibilityGate.filter(&seed, &[row], Some(&limits(2, -1.0, 1.0)));

        assert_eq!(report.rejected.len(), 1);
        assert_eq!(report.rejected[0].reason, RejectionReason::CompileFailure);
        assert_eq!(report.rejected[0].phase, RejectionPhase::Geometric);
    }

    #[test]
    fn compile_failure_takes_precedence_over_any_attached_assessment() {
        // Precedence is EXPLICIT: a compile failure is inadmissible in phase 1
        // even if an assessment were (incorrectly) attached — the gate never
        // lets a non-compiling candidate reach the risk policy.
        let seed = seed_program();
        let row = gate_row(
            candidate(
                StrategyKind::AlternateElbow,
                PlanningProgram::new(vec![movej("op-start", vec![0.0, 0.0])]),
            ),
            false,
            Some(accepted(0.1)),
            Some(metrics(1.0, 0.9, 0.5)),
        );

        let report = AdmissibilityGate.filter(&seed, &[row], Some(&limits(2, -1.0, 1.0)));

        assert_eq!(report.rejected.len(), 1);
        assert_eq!(report.rejected[0].reason, RejectionReason::CompileFailure);
        assert_eq!(
            report.rejected[0].phase,
            RejectionPhase::Geometric,
            "a compile failure must be rejected by phase 1, never by the risk policy"
        );
    }

    // ── 3.3 — phase 1: endpoint ε boundary (ADR-1) ───────────────────────

    #[test]
    fn endpoint_drift_beyond_epsilon_is_inadmissible() {
        let seed = seed_program();
        // Goal joint drifts by 2ε on the second joint.
        let drifting = candidate(
            StrategyKind::InsertWaypoint,
            PlanningProgram::new(vec![
                movej("op-start", vec![0.0, 0.0]),
                movej("op-goal", vec![0.5, 0.4 + 2.0 * ENDPOINT_TOLERANCE]),
            ]),
        );
        let row = gate_row(
            drifting,
            true,
            Some(accepted(0.2)),
            Some(metrics(2.0, 0.7, 1.0)),
        );

        let report = AdmissibilityGate.filter(&seed, &[row], Some(&limits(2, -1.0, 1.0)));

        assert!(report.admissible.is_empty());
        assert_eq!(report.rejected[0].reason, RejectionReason::EndpointDrift);
        assert_eq!(report.rejected[0].phase, RejectionPhase::Geometric);
    }

    #[test]
    fn endpoint_drift_exactly_at_epsilon_is_admissible() {
        // Boundary: |q_cand − q_seed| == ε per joint is WITHIN tolerance (≤ ε).
        let seed = seed_program();
        let at_boundary = candidate(
            StrategyKind::InsertWaypoint,
            PlanningProgram::new(vec![
                movej("op-start", vec![0.0, 0.0]),
                movej("op-goal", vec![0.5 + ENDPOINT_TOLERANCE, 0.4]),
            ]),
        );
        let row = gate_row(
            at_boundary,
            true,
            Some(accepted(0.2)),
            Some(metrics(2.0, 0.7, 1.0)),
        );

        let report = AdmissibilityGate.filter(&seed, &[row], Some(&limits(2, -1.0, 1.0)));

        assert_eq!(
            report.admissible.len(),
            1,
            "drift exactly at ε must pass the endpoint invariant (≤ ε)"
        );
        assert!(report.rejected.is_empty());
    }

    #[test]
    fn start_joint_drift_beyond_epsilon_is_inadmissible() {
        let seed = seed_program();
        // The FIRST commanded joint configuration is also an endpoint of the
        // equivalence class — drifting it is an invariant violation.
        let drifting = candidate(
            StrategyKind::AlternateElbow,
            PlanningProgram::new(vec![
                movej("op-start", vec![ENDPOINT_TOLERANCE * 3.0, 0.0]),
                movej("op-goal", vec![0.5, 0.4]),
            ]),
        );
        let row = gate_row(
            drifting,
            true,
            Some(accepted(0.2)),
            Some(metrics(2.0, 0.7, 1.0)),
        );

        let report = AdmissibilityGate.filter(&seed, &[row], Some(&limits(2, -1.0, 1.0)));

        assert!(report.admissible.is_empty());
        assert_eq!(report.rejected[0].reason, RejectionReason::EndpointDrift);
    }

    // ── 3.3 — phase 1: joint limits ──────────────────────────────────────

    #[test]
    fn joint_limit_violation_is_inadmissible() {
        let seed = seed_program();
        // An INTERMEDIATE commanded joint of 2.0 lies outside [−1.0, 1.0].
        // Endpoints and task identity are preserved (the out-of-limit value
        // is not a start/goal joint and reuses the goal origin), so the
        // JointLimitViolation invariant is exercised in isolation.
        let out_of_limits = candidate(
            StrategyKind::AlternateElbow,
            PlanningProgram::new(vec![
                movej("op-start", vec![0.0, 0.0]),
                movej("op-goal", vec![2.0, 0.4]),
                movej("op-goal", vec![0.5, 0.4]),
            ]),
        );
        let row = gate_row(
            out_of_limits,
            true,
            Some(accepted(0.2)),
            Some(metrics(2.0, 0.7, 1.0)),
        );

        let report = AdmissibilityGate.filter(&seed, &[row], Some(&limits(2, -1.0, 1.0)));

        assert!(report.admissible.is_empty());
        assert_eq!(
            report.rejected[0].reason,
            RejectionReason::JointLimitViolation
        );
        assert_eq!(report.rejected[0].phase, RejectionPhase::Geometric);
    }

    #[test]
    fn joints_within_limits_pass_the_limit_invariant() {
        let seed = seed_program();
        // Intermediate commanded joints near the limit EDGE but inside
        // [−1.0, 1.0] (0.99 / −0.99): endpoints preserved, identity preserved,
        // limits respected → admissible. Proves the limit check evaluates
        // every commanded joint, not just the endpoints.
        let within = candidate(
            StrategyKind::InsertWaypoint,
            PlanningProgram::new(vec![
                movej("op-start", vec![0.0, 0.0]),
                movej("op-goal", vec![0.99, 0.99]),
                movej("op-goal", vec![0.5, 0.4]),
            ]),
        );
        let row = gate_row(
            within,
            true,
            Some(accepted(0.2)),
            Some(metrics(2.0, 0.7, 1.0)),
        );

        let report = AdmissibilityGate.filter(&seed, &[row], Some(&limits(2, -1.0, 1.0)));

        assert_eq!(report.admissible.len(), 1);
        assert!(report.rejected.is_empty());
    }

    // ── 3.3 — phase 1: task identity ─────────────────────────────────────

    #[test]
    fn task_identity_mismatch_is_inadmissible() {
        // A candidate that adds a segment with a DIFFERENT origin breaks the
        // semantic task sequence [op-start, op-goal] → TaskMismatch (spec
        // candidate-generation "Generator changes Pick/Place target").
        let seed = seed_program();
        let tampered = candidate(
            StrategyKind::InsertWaypoint,
            PlanningProgram::new(vec![
                movej("op-start", vec![0.0, 0.0]),
                movej("op-sneaky", vec![0.7, 0.7]),
                movej("op-goal", vec![0.5, 0.4]),
            ]),
        );
        let row = gate_row(
            tampered,
            true,
            Some(accepted(0.2)),
            Some(metrics(2.0, 0.7, 1.0)),
        );

        let report = AdmissibilityGate.filter(&seed, &[row], Some(&limits(2, -1.0, 1.0)));

        assert!(report.admissible.is_empty());
        assert_eq!(report.rejected[0].reason, RejectionReason::TaskMismatch);
        assert_eq!(report.rejected[0].phase, RejectionPhase::Geometric);
    }

    #[test]
    fn task_identity_tolerates_strategy_splits() {
        // InsertWaypoint splits ONE segment into two halves with the SAME
        // kind + origin (the Wait MoveL → two MoveL halves, PR1 test 2.2).
        // Task identity compares the COMPACTED (kind, origin) sequence, so a
        // legitimate split must NOT be rejected.
        let seed = PlanningProgram::new(vec![
            movej("op-pick-a", vec![0.0, 0.0]),
            movel("op-wait", 0.3, 0.4, -0.12),
            movej("op-home", vec![0.5, 0.4]),
        ]);
        let split = candidate(
            StrategyKind::InsertWaypoint,
            PlanningProgram::new(vec![
                movej("op-pick-a", vec![0.0, 0.0]),
                movel("op-wait", 0.3, 0.4, -0.06),
                movel("op-wait", 0.3, 0.4, -0.12),
                movej("op-home", vec![0.5, 0.4]),
            ]),
        );
        let row = gate_row(
            split,
            true,
            Some(accepted(0.2)),
            Some(metrics(2.0, 0.7, 1.0)),
        );

        let report = AdmissibilityGate.filter(&seed, &[row], Some(&limits(2, -1.0, 1.0)));

        assert_eq!(
            report.admissible.len(),
            1,
            "a same-kind/same-origin split must preserve task identity"
        );
        assert!(report.rejected.is_empty());
    }

    // ── 3.3 — phase 2: risk policy ───────────────────────────────────────

    #[test]
    fn critical_risk_is_inadmissible_but_assessment_is_kept_for_trace() {
        let seed = seed_program();
        let critical_candidate = candidate(
            StrategyKind::AlternateElbow,
            PlanningProgram::new(vec![
                movej("op-start", vec![0.0, 0.0]),
                movej("op-goal", vec![0.5, 0.4]),
            ]),
        );
        let row = gate_row(
            critical_candidate.clone(),
            true,
            Some(critical(0.95)),
            Some(metrics(3.0, 0.4, 2.0)),
        );

        let report = AdmissibilityGate.filter(&seed, &[row], Some(&limits(2, -1.0, 1.0)));

        assert!(
            report.admissible.is_empty(),
            "Critical must not enter ranking — not even with the lowest J"
        );
        assert_eq!(report.rejected.len(), 1);
        let rejected = &report.rejected[0];
        assert_eq!(rejected.reason, RejectionReason::RiskRejected);
        assert_eq!(rejected.phase, RejectionPhase::Risk);
        // The assessment EXISTS (the Assessor ran on valid geometry) and is
        // kept for trace — it must not be discarded by the gate.
        let kept = rejected
            .assessment
            .as_ref()
            .expect("assessment kept for trace");
        assert!((kept.risk - 0.95).abs() < 1e-12);
        assert!(matches!(kept.admissibility, RiskAdmissibility::Rejected));
        assert_eq!(rejected.metrics, Some(metrics(3.0, 0.4, 2.0)));
        assert_eq!(rejected.candidate, critical_candidate);
    }

    #[test]
    fn compiled_but_missing_assessment_fails_closed() {
        // Compile OK but no neutral assessment = a broken pipeline contract.
        // The gate fails closed: it cannot evaluate the risk policy, so the
        // candidate is inadmissible.
        let seed = seed_program();
        let row = gate_row(
            candidate(
                StrategyKind::Direct,
                PlanningProgram::new(vec![
                    movej("op-start", vec![0.0, 0.0]),
                    movej("op-goal", vec![0.5, 0.4]),
                ]),
            ),
            true,
            None,
            None,
        );

        let report = AdmissibilityGate.filter(&seed, &[row], Some(&limits(2, -1.0, 1.0)));

        assert!(report.admissible.is_empty());
        assert_eq!(
            report.rejected[0].reason,
            RejectionReason::MissingAssessment
        );
    }

    // ── 3.3 — combined gate behavior ─────────────────────────────────────

    #[test]
    fn accepted_candidates_pass_both_phases() {
        let seed = seed_program();
        let baseline = gate_row(
            candidate(StrategyKind::Direct, seed.clone()),
            true,
            Some(accepted(0.557)),
            Some(metrics(3.2, 0.458, 1.8)),
        );
        let alternative = gate_row(
            candidate(
                StrategyKind::AlternateElbow,
                PlanningProgram::new(vec![
                    movej("op-start", vec![0.0, 0.0]),
                    movej("op-goal", vec![0.5, 0.4]),
                ]),
            ),
            true,
            Some(accepted(0.182)),
            Some(metrics(2.1, 0.7, 1.2)),
        );

        let report =
            AdmissibilityGate.filter(&seed, &[baseline, alternative], Some(&limits(2, -1.0, 1.0)));

        assert_eq!(
            report.admissible.len(),
            2,
            "both candidates pass both phases"
        );
        assert!(report.rejected.is_empty());
        assert_eq!(
            report.admissible[0].candidate.strategy,
            StrategyKind::Direct
        );
        assert_eq!(
            report.admissible[1].candidate.strategy,
            StrategyKind::AlternateElbow
        );
    }

    #[test]
    fn all_inadmissible_leaves_no_admissible_candidates() {
        let seed = seed_program();
        let critical_row = gate_row(
            candidate(StrategyKind::Direct, seed.clone()),
            true,
            Some(critical(0.95)),
            Some(metrics(3.2, 0.458, 1.8)),
        );
        let compile_fail_row = gate_row(
            candidate(
                StrategyKind::InsertWaypoint,
                PlanningProgram::new(vec![movej("op-start", vec![0.0, 0.0])]),
            ),
            false,
            None,
            None,
        );

        let report = AdmissibilityGate.filter(
            &seed,
            &[critical_row, compile_fail_row],
            Some(&limits(2, -1.0, 1.0)),
        );

        assert!(
            report.admissible.is_empty(),
            "no admissible candidate → the evaluator must report NoAdmissibleCandidate"
        );
        assert_eq!(report.rejected.len(), 2);
        assert_eq!(report.rejected[0].reason, RejectionReason::RiskRejected);
        assert_eq!(report.rejected[1].reason, RejectionReason::CompileFailure);
    }

    #[test]
    fn seed_baseline_passes_phase_one_against_itself() {
        // The Direct baseline is the seed: endpoints vs itself are 0 ≤ ε,
        // task identity vs itself holds, compile is true by construction. It
        // can only ever be rejected by the risk policy (phase 2).
        let seed = seed_program();
        let baseline = gate_row(
            candidate(StrategyKind::Direct, seed.clone()),
            true,
            Some(accepted(0.557)),
            Some(metrics(3.2, 0.458, 1.8)),
        );

        let report = AdmissibilityGate.filter(&seed, &[baseline], Some(&limits(2, -1.0, 1.0)));

        assert_eq!(
            report.admissible.len(),
            1,
            "the seed baseline must always pass phase 1"
        );
        assert!(report.rejected.is_empty());
    }
}
