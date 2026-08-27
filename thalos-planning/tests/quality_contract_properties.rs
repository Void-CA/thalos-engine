//! Property suite — the intelligence quality contract as CI-enforced laws
//! (design ADR-6, spec `quality-scoring-contract` + `recommendation-model`,
//! tasks T15/M4).
//!
//! Four properties lock the M1→M3 contracts against regressions:
//!
//! 1. **availability ⇒ executable** — for EVERY recommendation the advisor
//!    marks `Available`, its edited program MUST compile with the REAL IK
//!    solver and re-analyze. This is the D8-gate honesty law (M2): a
//!    recommendation the user can click must never point at an edit that
//!    cannot be materialized into a runnable plan.
//!
//! 2. **remediation ⇒ observation-count decreases** — for every `Available`
//!    Singularity recommendation, applying the edit and re-analyzing MUST
//!    strictly decrease the Singularity observation count (unless the plan
//!    already has none). This is the causal-remediation law (M3): an
//!    available repair must actually repair.
//!
//! 3. **score monotonicity** — removing observations without introducing
//!    worse ones MUST NOT decrease `quality_index` (spec
//!    "Monotonic Improvement Under Penalty Removal"), asserted through the
//!    real aggregation path (`DefaultAggregator::aggregate_with_metrics`).
//!
//! 4. **score domain** — `quality_index` ∈ [0, 1], finite, NaN-free and
//!    deterministic (exact `f64` equality on rerun), through the aggregator.
//!
//! The pipeline properties (1–2) run against the SAME real harness as the
//! permanent usability suite (`usability_intelligence.rs`): real IK solver,
//! real planner dispatcher, real analyzer, `NaiveCollisionChecker` enabled.
//! Generation is bounded to a family of near-reach / singular-start programs
//! (Planar3R + Scara) so every case exercises the advisory path; the per-case
//! cost of a full compile→analyze→recommend→recompile→reanalyze loop keeps
//! the case count small by design.
//!
//! Property 2 applies ONLY to `Available` Singularity recommendations —
//! `Unavailable{Unsupported}` (e.g. interior MoveL singularity, documented M3
//! gap) is deliberately NOT tested here: an unsupported remediation is not
//! available, so no repair obligation exists (spec recommendation-model
//! "Unsupported operation produces unavailable with reason").

use proptest::prelude::*;
use std::collections::BTreeMap;
use thalos_collision::NaiveCollisionChecker;
use thalos_core::{
    analysis::{
        Aggregator,
        aggregator::DefaultAggregator,
        action::ActionKind,
        location::Location,
        observation::{ArtifactRef, Observation, ObservationId, ObservationKind, Severity},
        scoring::DefaultScoringPolicy,
    },
    collision::CollisionMatrix,
    ids::{MotionPlanId, OperationId},
    kinematics::{forward::ForwardKinematics, inverse::DampedLeastSquaresSolver},
    models::{RobotModel, RobotRegistry},
    motion::segment::MotionSegment,
    prelude::RobotState,
    robot::serial_chain::SerialChain,
    trajectory::Trajectory,
};
use thalos_planning::{
    advisor::PlanAdvisor,
    analysis::TrajectoryAnalyzer,
    motion::{
        compiler::{DefaultPlannerDispatcher, PlanCompiler},
        planner::SegmentPlanningContext,
        program::{CompiledPlan, PlanningProgram},
    },
    program_edit::ProgramEdit,
    recommendation::{Recommendation, RecommendationStatus},
};

// ────────────────────────────────────────────────────────────────────────────
// Real-pipeline harness (mirrors usability_intelligence.rs — the permanent
// gate's harness, kept verbatim in shape so the properties measure the SAME
// pipeline the usability scenarios measure).
// ────────────────────────────────────────────────────────────────────────────

fn chain(model: RobotModel) -> SerialChain {
    RobotRegistry::create_default(model)
}

fn real_solver(chain: &SerialChain) -> DampedLeastSquaresSolver {
    let fk = ForwardKinematics::new(chain.clone());
    DampedLeastSquaresSolver::new(fk, chain.end_effector().clone(), 500, 1e-6, 0.1)
}

/// The runtime's canonical analysis (mirrors `PlanAnalysisService`).
fn analyze(
    chain: &SerialChain,
    trajectory: &Trajectory,
) -> thalos_core::analysis::report::AnalysisReport {
    let checker = NaiveCollisionChecker;
    let matrix = CollisionMatrix::new();
    let analyzer = TrajectoryAnalyzer::new(chain, None).with_collision_checker(&checker, &matrix);
    let artifact = ArtifactRef::MotionPlan(MotionPlanId("quality-contract".to_string()));
    let (analysis, observations) = analyzer
        .analyze_with_observations(artifact.clone(), trajectory)
        .expect("real analysis must succeed");
    DefaultAggregator::new(DefaultScoringPolicy).aggregate_with_metrics(
        artifact,
        observations,
        analysis.metrics.to_btree_map(),
    )
}

/// Compile a program with the REAL IK solver from a given start configuration.
fn compile(
    chain: &SerialChain,
    start: &[f64],
    program: &PlanningProgram,
) -> Result<thalos_core::trajectory::Trajectory, String> {
    let solver = real_solver(chain);
    let state = RobotState::new(start.to_vec());
    let ctx = SegmentPlanningContext {
        robot: chain,
        current_state: &state,
        ik_solver: &solver,
        tcp: None,
    };
    let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
    compiler
        .compile(program, &ctx)
        .map(|p| p.merged_trajectory)
        .map_err(|e| e.to_string())
}

/// Compile a program with the REAL IK solver and return the full
/// [`CompiledPlan`] — the whole-region projection needs the per-segment
/// `waypoint_range` of the recompiled trajectory (which the trajectory-only
/// `compile` helper hides).
fn compile_plan(
    chain: &SerialChain,
    start: &[f64],
    program: &PlanningProgram,
) -> Result<CompiledPlan, String> {
    let solver = real_solver(chain);
    let state = RobotState::new(start.to_vec());
    let ctx = SegmentPlanningContext {
        robot: chain,
        current_state: &state,
        ik_solver: &solver,
        tcp: None,
    };
    let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));
    compiler.compile(program, &ctx).map_err(|e| e.to_string())
}

fn movej(target: Vec<f64>) -> MotionSegment {
    MotionSegment::MoveJ {
        origin: OperationId("op-j".to_string()),
        target,
        max_velocity: None,
        max_acceleration: None,
    }
}

/// Program families that reliably exercise the advisory path (design ADR-6).
///
/// - **Scara interior-crossing**: the user's real case — a MoveJ that departs
///   from a NON-singular home (elbow bent negative) to a target whose elbow is
///   POSITIVE, so the straight-line path CROSSES the full extension (elbow = 0)
///   mid-segment. The re-solve materializer re-solves IK from the home (same
///   side) to the alternate elbow posture that reaches the same cartesian point
///   without crossing the extension.
///
/// Perturbations are bounded around the known-singular target so every
/// generated program stays inside the workspace (compilability is filtered
/// per-case via `prop_assume!`).
fn pipeline_case_strategy() -> impl Strategy<Value = (SerialChain, Vec<f64>, PlanningProgram)> {
    // Scara interior-crossing family: elbow ∈ [0.3, 0.9] (positive → crosses
    // the extension from the negative-elbow home), base and prismatic perturbed
    // in a small box around the verified case [0.5, 0.6, -0.15].
    let scara = (0.3f64..0.9, -0.3f64..0.1, -0.3f64..-0.05).prop_map(|(elbow, dbase, dz)| {
        let chain = chain(RobotModel::Scara);
        let program = PlanningProgram::new(vec![
            movej(vec![0.5 + dbase, elbow, dz, 0.0]),
        ]);
        // Home: non-singular — elbow bent to the negative side, base ~0.
        (chain, vec![0.0, -1.31, -0.1, 0.0], program)
    });
    scara
}

fn available_of(recommendations: &[Recommendation]) -> Vec<Recommendation> {
    recommendations
        .iter()
        .filter(|r| r.status == Some(RecommendationStatus::Available))
        .cloned()
        .collect()
}

// ────────────────────────────────────────────────────────────────────────────
// PROPERTY 1 — availability ⇒ executable (D8-gate honesty, M2)
// ────────────────────────────────────────────────────────────────────────────
//
// Spec recommendation-availability-contract: a recommendation is `Available`
// ONLY if its edit materializes, the edited program compiles and re-analyzes.
// The property re-runs that verification INDEPENDENTLY of the advisor on the
// real pipeline, so any future weakening of the gate fails here in CI.

proptest! {
    #![proptest_config(proptest::test_runner::Config { failure_persistence: None, ..proptest::test_runner::Config::with_cases(12) })]

    #[test]
    fn available_recommendations_always_recompile_and_reanalyze(
        (chain, start, program) in pipeline_case_strategy(),
    ) {
        let trajectory = match compile(&chain, &start, &program) {
            Ok(trajectory) => trajectory,
            Err(_) => {
                prop_assume!(false, "precondition: generated program must compile");
                unreachable!("prop_assume! must reject the case");
            }
        };
        let report = analyze(&chain, &trajectory);
        let recommendations =
            PlanAdvisor.recommend(&report.observations, &program, &real_solver(&chain), &start);
        let available = available_of(&recommendations);
        prop_assume!(
            !available.is_empty(),
            "precondition: the generated program must yield an available recommendation"
        );

        for rec in &available {
            // (a) The edit must apply to a clone of the original program.
            let edited = rec.edit.apply(&program).unwrap_or_else(|e| {
                panic!(
                    "AVAILABILITY CONTRACT VIOLATED: recommendation {} (kind {:?}) is Available \
                     but its edit does not apply to the program: {e}",
                    rec.id.0, rec.action.kind
                )
            });
            // (b) The edited program must compile with the real IK solver.
            let edited_trajectory = compile(&chain, &start, &edited).unwrap_or_else(|e| {
                panic!(
                    "AVAILABILITY CONTRACT VIOLATED: recommendation {} (kind {:?}) is Available \
                     but its edited program does not recompile: {e}",
                    rec.id.0, rec.action.kind
                )
            });
            // (c) Re-analysis must succeed (the D8 gate's honesty includes it).
            analyze(&chain, &edited_trajectory);
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// PROPERTY 2 — remediation ⇒ the re-analyzed target region is FREE of
// Singularity observations (R3-1 P0 fix, M3)
// ────────────────────────────────────────────────────────────────────────────
//
// Spec causal-remediation "Whole-Region Availability Guarantee": `Available`
// for a Singularity recommendation SHALL mean the re-analyzed target region
// (all waypoints in `ProblemRegion.waypoint_range`, projected onto the
// recompiled trajectory via the owning segment) is free of Singularity
// observations — NOT merely fewer than before. A partial reduction that leaves
// the region singular (24→23) MUST never be `Available`.
//
// The ONLY exclusion is the irreducible fixed-start waypoint (merged
// trajectory waypoint 0 of the plan's first segment): the remediation edits
// trajectory segments only, never the fixed initial configuration, so a
// singular starting state cannot be repaired by any segment edit (the M3
// scenarios keep 24→1 and 17→0 passing under this criterion).
//
// The property re-runs the whole-region check INDEPENDENTLY of the advisor
// (mirroring production's projection: the recompiled owning segment's
// waypoint_range), so any future weakening of the gate fails here in CI.
// `Unavailable{Unsupported}` recommendations (e.g. interior MoveL, documented
// M3 gap) carry no repair obligation and are not generated as `Available`.

/// Singularity observations inside the projected range, excluding the
/// irreducible fixed-start waypoint (mirror of the production gate).
fn singular_in_range(
    observations: &[Observation],
    range: &std::ops::Range<usize>,
    exclude_fixed_start: bool,
) -> usize {
    observations
        .iter()
        .filter(|o| {
            o.kind == ObservationKind::Singularity
                && matches!(o.location, Location::Waypoint(wp) if range.contains(&wp))
                && !(exclude_fixed_start && matches!(o.location, Location::Waypoint(0)))
        })
        .count()
}

proptest! {
    #![proptest_config(proptest::test_runner::Config { failure_persistence: None, ..proptest::test_runner::Config::with_cases(12) })]

    #[test]
    fn available_singularity_remediation_clears_the_whole_target_region(
        (chain, start, program) in pipeline_case_strategy(),
    ) {
        let trajectory = match compile(&chain, &start, &program) {
            Ok(trajectory) => trajectory,
            Err(_) => {
                prop_assume!(false, "precondition: generated program must compile");
                unreachable!("prop_assume! must reject the case");
            }
        };
        let report = analyze(&chain, &trajectory);
        let recommendations =
            PlanAdvisor.recommend(&report.observations, &program, &real_solver(&chain), &start);

        let singular_available: Vec<Recommendation> = recommendations
            .iter()
            .filter(|r| {
                r.action.kind == ActionKind::Singularity
                    && r.status == Some(RecommendationStatus::Available)
            })
            .cloned()
            .collect();
        prop_assume!(
            !singular_available.is_empty(),
            "precondition: the generated program must yield an Available Singularity recommendation"
        );

        for rec in &singular_available {
            let ProgramEdit::ReplaceSegment { index, .. } = rec.edit else {
                panic!("a singularity edit must be ReplaceSegment");
            };
            let edited = rec
                .edit
                .apply(&program)
                .expect("an available edit must apply to the program clone");
            let edited_plan = compile_plan(&chain, &start, &edited)
                .expect("availability contract guarantees the edited program compiles");
            // The projected region: the recompiled owning segment's waypoint
            // range — the deterministic projection of the original
            // ProblemRegion onto the recompiled trajectory (the edit replaces
            // exactly that segment; waypoint indices diverge, the segment
            // correspondence is exact).
            let projected_range = edited_plan.segments[index].waypoint_range.clone();
            let healed = analyze(&chain, &edited_plan.merged_trajectory);
            // The irreducible fixed-start waypoint (merged waypoint 0 of the
            // plan's first segment) is excluded from the guarantee.
            let relevant = singular_in_range(&healed.observations, &projected_range, index == 0);
            prop_assert_eq!(
                relevant, 0,
                "REMEDIATION CONTRACT VIOLATED: recommendation {} (kind {:?}) is Available but the \
                 re-analyzed target region still contains {} Singularity observation(s) \
                 inside the projected range {:?} (fixed-start excluded: {}) — the whole region must \
                 be free, not merely reduced (24→23 must never be Available) — program: {:?}",
                rec.id.0,
                rec.action.kind,
                relevant,
                projected_range,
                index == 0,
                program.segments.iter().map(|s| format!("{s:?}")).collect::<Vec<_>>(),
            );
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// PROPERTIES 3 & 4 — score monotonicity + domain through the REAL aggregation
// path (M1, design ADR-1). These complement the unit proptests in
// `scoring.rs` by locking the AGGREGATOR wiring (T2: `build_summary` passes
// `report.metrics` into scoring) — the exact path the acceptance gate uses.
// ────────────────────────────────────────────────────────────────────────────

fn synthetic_observation_strategy() -> impl Strategy<Value = Observation> {
    prop_oneof![
        Just(Severity::Info),
        Just(Severity::Warning),
        Just(Severity::Error),
    ]
    .prop_map(|severity| Observation {
        id: ObservationId(0),
        kind: ObservationKind::ResidualError,
        severity,
        artifact: ArtifactRef::MotionPlan(MotionPlanId("prop".to_string())),
        location: Location::Waypoint(0),
        attributes: BTreeMap::new(),
        causes: Vec::new(),
        related: Vec::new(),
    })
}

/// Arbitrary metric values including the specials — NaN MUST be treated as
/// neutral, ±∞ MUST clamp (the aggregator forwards the map verbatim).
fn metric_value_strategy() -> impl Strategy<Value = f64> {
    prop_oneof![
        -1000.0..1000.0,
        -1e9f64..1e9,
        Just(f64::NAN),
        Just(f64::INFINITY),
        Just(f64::NEG_INFINITY),
    ]
}

fn metrics_strategy() -> impl Strategy<Value = BTreeMap<String, f64>> {
    let keys = prop::sample::select(vec![
        "avg_manipulability".to_string(),
        "smoothness".to_string(),
        "min_collision_distance".to_string(),
        "joint_safety.min_margin".to_string(),
        "orientation_change".to_string(),
        "unknown-key".to_string(),
    ]);
    prop::collection::btree_map(keys, metric_value_strategy(), 0..8)
}

fn aggregate(
    observations: &[Observation],
    metrics: &BTreeMap<String, f64>,
) -> f64 {
    let artifact = ArtifactRef::MotionPlan(MotionPlanId("prop".to_string()));
    let report = DefaultAggregator::new(DefaultScoringPolicy)
        .aggregate_with_metrics(artifact, observations.to_vec(), metrics.clone());
    report.summary.quality_index
}

proptest! {
    #![proptest_config(proptest::test_runner::Config { failure_persistence: None, ..proptest::test_runner::Config::with_cases(128) })]

    #[test]
    fn removing_observations_never_decreases_aggregate_quality(
        base in prop::collection::vec(synthetic_observation_strategy(), 0..20),
        keep in prop::collection::vec(proptest::bool::ANY, 0..20),
        metrics in metrics_strategy(),
    ) {
        // Spec "Monotonic Improvement Under Penalty Removal" through the real
        // aggregation path: subsetting observations (removing without
        // introducing worse) MUST NOT decrease the report's quality_index.
        let full = aggregate(&base, &metrics);
        let subset: Vec<Observation> = base
            .iter()
            .zip(keep.iter())
            .filter(|(_, keep)| **keep)
            .map(|(obs, _)| obs.clone())
            .collect();
        let reduced = aggregate(&subset, &metrics);
        prop_assert!(
            reduced >= full,
            "removing observations must not lower the aggregate quality_index: \
             {reduced} < {full}"
        );
    }

    #[test]
    fn aggregate_quality_stays_in_unit_interval_finite_and_deterministic(
        observations in prop::collection::vec(synthetic_observation_strategy(), 0..30),
        metrics in metrics_strategy(),
    ) {
        // Spec "Score Domain" through the aggregator: [0, 1], finite,
        // NaN-free and exactly deterministic (same input → same f64).
        let quality = aggregate(&observations, &metrics);
        prop_assert!(
            (0.0..=1.0).contains(&quality),
            "aggregate quality {quality} must be in [0, 1]"
        );
        prop_assert!(
            quality.is_finite() && !quality.is_nan(),
            "aggregate quality {quality} must be finite and NaN-free"
        );
        let rerun = aggregate(&observations, &metrics);
        prop_assert_eq!(
            quality, rerun,
            "aggregate quality must be deterministic (exact f64 equality)"
        );
    }
}
