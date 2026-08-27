//! Demo end-to-end reproducible: a BAD plan is built on purpose (a MoveJ whose
//! target CROSSES the full extension), the REAL pipeline diagnoses it, the
//! advisor recommends the root-cause repair (IK re-solve to the same-side elbow
//! posture), and applying it removes the singularities and improves the score —
//! printed to stdout.
//!
//! Run it and watch the flow:
//!
//! ```text
//! cargo test -p thalos-planning --test demo_end_to_end -- --nocapture
//! ```
//!
//! This is the SAME real pipeline as `usability_intelligence.rs`: real IK
//! solver, real planner dispatcher, real analyzer, `NaiveCollisionChecker`.
//! No mocks. Every assertion is a real regression guard — if any step stops
//! working, this test fails with the actual before/after numbers.

use thalos_collision::NaiveCollisionChecker;
use thalos_core::{
    analysis::{
        Aggregator,
        action::ActionKind,
        aggregator::DefaultAggregator,
        observation::{ArtifactRef, Observation, ObservationKind},
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
        program::PlanningProgram,
    },
    program_edit::ProgramEdit,
    recommendation::{Recommendation, RecommendationStatus},
};

// ── Real-pipeline harness (same shape as usability_intelligence.rs) ─────────

fn chain(model: RobotModel) -> SerialChain {
    RobotRegistry::create_default(model)
}

fn real_solver(chain: &SerialChain) -> DampedLeastSquaresSolver {
    let fk = ForwardKinematics::new(chain.clone());
    DampedLeastSquaresSolver::new(fk, chain.end_effector().clone(), 500, 1e-6, 0.1)
}

fn analyze(
    chain: &SerialChain,
    trajectory: &Trajectory,
) -> thalos_core::analysis::report::AnalysisReport {
    let checker = NaiveCollisionChecker;
    let matrix = CollisionMatrix::new();
    let analyzer = TrajectoryAnalyzer::new(chain, None).with_collision_checker(&checker, &matrix);
    let artifact = ArtifactRef::MotionPlan(MotionPlanId("demo-end-to-end".to_string()));
    let (analysis, observations) = analyzer
        .analyze_with_observations(artifact.clone(), trajectory)
        .expect("real analysis must succeed");
    DefaultAggregator::new(DefaultScoringPolicy).aggregate_with_metrics(
        artifact,
        observations,
        analysis.metrics.to_btree_map(),
    )
}

fn compile(
    chain: &SerialChain,
    start: &[f64],
    program: &PlanningProgram,
) -> Result<Trajectory, String> {
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

fn singular_errors(observations: &[Observation]) -> usize {
    observations
        .iter()
        .filter(|o| o.kind == ObservationKind::Singularity)
        .count()
}

fn movej(target: Vec<f64>) -> MotionSegment {
    MotionSegment::MoveJ {
        origin: OperationId("op-j".to_string()),
        target,
        max_velocity: None,
        max_acceleration: None,
    }
}

/// Continuity = timestamps are monotonic non-decreasing over the merged
/// trajectory (the shared boundary waypoint between segments may repeat its
/// timestamp — that is NOT a gap).
fn trajectory_continuity(traj: &Trajectory) -> bool {
    let wps = traj.waypoints();
    !wps.is_empty() && wps.windows(2).all(|w| w[1].timestamp() >= w[0].timestamp())
}

fn fmt_joints(joints: &[f64]) -> String {
    joints
        .iter()
        .map(|j| format!("{j:.3}"))
        .collect::<Vec<_>>()
        .join(", ")
}

/// The elbow joint (index 1) of a MoveJ target — the joint whose sign
/// distinguishes the elbow-up / elbow-down posture.
fn movej_elbow(segment: &MotionSegment) -> Option<f64> {
    match segment {
        MotionSegment::MoveJ { target, .. } if target.len() > 1 => Some(target[1]),
        _ => None,
    }
}

// ── The demo ────────────────────────────────────────────────────────────────

#[test]
fn demo_scara_crossing_extension_diagnosed_and_repaired() {
    let robot = chain(RobotModel::Scara);
    // A BAD plan on purpose: depart from a BENT home posture toward a target
    // whose elbow (+0.6) is on the OPPOSITE side — the straight-line MoveJ
    // path crosses the full extension mid-segment (interior singularity).
    let home = vec![0.0, -1.31, -0.1, 0.0];
    let bad_target = vec![0.5, 0.6, -0.15, 0.0];
    let program = PlanningProgram::new(vec![movej(bad_target.clone())]);

    println!("\n═══════════════════════════════════════════════════════════");
    println!("END-TO-END DEMO — SCARA: cruce de extensión → diagnóstico → reparación");
    println!("═══════════════════════════════════════════════════════════");
    println!("Robot: SCARA (R-R-P-R)");
    println!(
        "home  = [{}]  (elbow -1.31, codo doblado, NO singular)",
        fmt_joints(&home)
    );
    println!(
        "target = [{}]  (elbow +0.6, CRUZA la extensión)",
        fmt_joints(&bad_target)
    );

    // 1. Compile + diagnose the bad plan.
    let trajectory = compile(&robot, &home, &program).expect("original plan must compile");
    let report = analyze(&robot, &trajectory);
    let errors_before = singular_errors(&report.observations);
    println!("\n[1] DIAGNÓSTICO (antes de reparar)");
    println!(
        "    health        = {:.2}  ({:?})",
        report.summary.quality_index, report.summary.grade
    );
    println!("    singularities = {errors_before}");
    println!("    waypoints     = {}", trajectory.len());

    // 2. Ask the advisor.
    let recommendations =
        PlanAdvisor.recommend(&report.observations, &program, &real_solver(&robot), &home);
    println!(
        "\n[2] RECOMENDACIÓN DEL ADVISOR ({})",
        recommendations.len()
    );
    for rec in &recommendations {
        print_recommendation(rec);
    }
    let available: Vec<&Recommendation> = recommendations
        .iter()
        .filter(|r| r.status == Some(RecommendationStatus::Available))
        .collect();
    assert!(
        !available.is_empty(),
        "precondition: a bad crossing MoveJ MUST produce an available Singularity recommendation"
    );

    // The Singularity edit must be a single-segment ReplaceSegment whose target
    // is re-solved to the same-side (NEGATIVE) elbow.
    let singularity = available
        .iter()
        .find(|r| r.action.kind == ActionKind::Singularity)
        .expect("a Singularity recommendation must be available");
    match &singularity.edit {
        ProgramEdit::ReplaceSegment { replacement, .. } => {
            assert_eq!(
                replacement.len(),
                1,
                "the re-solve must be a clean 1:1 replacement, not a split"
            );
            let re_solved_elbow = movej_elbow(&replacement[0])
                .expect("the replacement must be a MoveJ with an elbow joint");
            assert!(
                re_solved_elbow < 0.0,
                "the re-solved elbow must be NEGATIVE (same side as home), got {re_solved_elbow:.3}"
            );
        }
        other => panic!("a singularity edit must be ReplaceSegment, got {other:?}"),
    }

    // 3. Apply every available recommendation, recompile, re-analyze.
    println!("\n[3] APLICANDO la(s) recomendación(es) disponible(s)...");
    let mut edited = program.clone();
    for rec in &available {
        edited = rec
            .edit
            .apply(&edited)
            .expect("an available edit must apply");
    }
    let healed_trajectory = compile(&robot, &home, &edited)
        .unwrap_or_else(|e| panic!("the repaired program must recompile: {e}"));
    let healed = analyze(&robot, &healed_trajectory);
    let errors_after = singular_errors(&healed.observations);
    let continuous = trajectory_continuity(&healed_trajectory);

    println!("\n[4] DESPUÉS DE REPARAR");
    println!(
        "    health        = {:.2}  ({:?})",
        healed.summary.quality_index, healed.summary.grade
    );
    println!("    singularities = {errors_after}");
    println!("    waypoints     = {}", healed_trajectory.len());
    println!("    continuidad   = {continuous}");

    println!("\n[5] VEREDICTO");
    println!("    singularities: {errors_before} → {errors_after}");
    println!(
        "    health:        {:.2} → {:.2}",
        report.summary.quality_index, healed.summary.quality_index
    );
    println!("    continuidad:   {continuous}");
    println!("═══════════════════════════════════════════════════════════\n");

    assert!(
        errors_after == 0,
        "the root-cause fix must drive singularities to ZERO: {errors_before} -> {errors_after}"
    );
    assert!(
        healed.summary.quality_index > report.summary.quality_index,
        "health must STRICTLY improve: {:.2} -> {:.2}",
        report.summary.quality_index,
        healed.summary.quality_index
    );
    assert!(continuous, "the repaired trajectory must be continuous");
}

fn print_recommendation(rec: &Recommendation) {
    let kind = rec.action.kind;
    let status = match rec.status {
        Some(RecommendationStatus::Available) => "Available".to_string(),
        Some(RecommendationStatus::Unavailable) => {
            format!("Unavailable ({:?})", rec.reason)
        }
        None => "not-evaluated".to_string(),
    };
    println!("    - kind={kind:?}  status={status}");
    if kind == ActionKind::Singularity {
        if let ProgramEdit::ReplaceSegment {
            index,
            replacement,
            original,
        } = &rec.edit
        {
            println!(
                "      edit = ReplaceSegment {{ index: {index}, replacement: {n} segment(s) }}",
                n = replacement.len()
            );
            let original_elbow = original
                .as_ref()
                .and_then(|segs| segs.first())
                .and_then(movej_elbow);
            for (i, seg) in replacement.iter().enumerate() {
                if let MotionSegment::MoveJ { target, .. } = seg {
                    println!(
                        "        replacement[{i}] = MoveJ target=[{}]  elbow={:.3}",
                        fmt_joints(target),
                        target.get(1).copied().unwrap_or(f64::NAN)
                    );
                }
            }
            match original_elbow {
                Some(original) => {
                    let re_solved = replacement
                        .first()
                        .and_then(movej_elbow)
                        .unwrap_or(f64::NAN);
                    println!(
                        "      codo: original={original:+.3} → re-suelto={re_solved:+.3}  (mismo lado, sin cruzar)"
                    );
                }
                None => println!("      codo: (target original no articular)"),
            }
        }
    }
}
