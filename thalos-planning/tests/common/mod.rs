//! Shared real-pipeline harness for the candidate suite (extracted from the
//! duplicated `tests/candidate_feasibility.rs` + `tests/candidate_counterfactual.rs`
//! — the demo-scenarios change; `tests/assessment_demo.rs` keeps its own
//! harness, it does not run the candidate pipeline).
//!
//! The full chain runs with REAL components, no mocks:
//!
//! ```text
//! CandidateGenerator::generate → per candidate (PlanCompiler → TrajectoryAnalyzer
//! → DefaultAggregator → Assessor::assess) → runtime adapter mapping (replicated
//! HERE because the planning crate must stay free of thalos-intelligence as a
//! dependency — the runtime owns the mapping in production) → AdmissibilityGate
//! → CandidateEvaluator → CandidateRanking
//! ```
//!
//! The adapter mapping replicated in this harness is EXACTLY the runtime's
//! (design ADR-5): `risk = 1 − quality` and the CATEGORICAL verdict
//! `Assessment.risk == Critical → RiskAdmissibility::Rejected` — never a
//! numeric threshold in planning.
//!
//! ## Pipeline completion contract
//!
//! [`run_pipeline`] returns `Result` so SEED / stage errors PROPAGATE instead
//! of silently degrading into a categorical result: an un-compilable seed (IK
//! failure, joint-limit violation) or an analysis failure FAILS the pipeline
//! caller — the demo-scenarios "pipeline completion guard" depends on this.
//! Per-candidate compile failures remain `GateCandidate.compile_ok == false`
//! rows (the gate's designed input — a generated alternative may legitimately
//! fail to compile and is rejected with `CompileFailure`).

use thalos_collision::NaiveCollisionChecker;
use thalos_core::{
    analysis::{
        Aggregator, aggregator::DefaultAggregator, observation::ArtifactRef,
        report::AnalysisReport, scoring::DefaultScoringPolicy,
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
use thalos_intelligence::{Assessor, Risk};
use thalos_planning::{
    analysis::TrajectoryAnalyzer,
    candidate::{
        AdmissibilityGate, AdmissibleCandidate, Candidate, CandidateAssessment, CandidateEvaluator,
        CandidateGenerationContext, CandidateGenerator, CandidateRanking, CandidateScore,
        GateCandidate, JointBounds, MotionMetrics, ObjectiveProfile, RejectedCandidate,
        RiskAdmissibility, SelectionReason, StrategyKind, StrategyTrace,
    },
    motion::{
        compiler::{DefaultPlannerDispatcher, PlanCompiler},
        planner::SegmentPlanningContext,
        program::PlanningProgram,
    },
};

/// The gate's endpoint tolerance (ADR-1), re-exported for the consumers that
/// reference it in assertion messages.
pub use thalos_planning::candidate::ENDPOINT_TOLERANCE;

// ── Real-pipeline harness ────────────────────────────────────────────────────

pub fn chain(model: RobotModel) -> SerialChain {
    RobotRegistry::create_default(model)
}

pub fn real_solver(chain: &SerialChain) -> DampedLeastSquaresSolver {
    let fk = ForwardKinematics::new(chain.clone());
    DampedLeastSquaresSolver::new(fk, *chain.end_effector(), 500, 1e-6, 0.1)
}

pub fn compile(
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

/// Analyze + aggregate the trajectory. Returns `Err` on a REAL stage failure
/// (the pipeline-completion guard: analysis errors propagate, they never
/// silently degrade a scenario into a categorical pass). The artifact label is
/// cosmetic — it never influences assessment or metrics.
pub fn analyze(chain: &SerialChain, trajectory: &Trajectory) -> Result<AnalysisReport, String> {
    let checker = NaiveCollisionChecker;
    let matrix = CollisionMatrix::new();
    let analyzer = TrajectoryAnalyzer::new(chain, None).with_collision_checker(&checker, &matrix);
    let artifact = ArtifactRef::MotionPlan(MotionPlanId("demo-scenarios".to_string()));
    let (analysis, observations) = analyzer
        .analyze_with_observations(artifact.clone(), trajectory)
        .map_err(|e| e.to_string())?;
    Ok(
        DefaultAggregator::new(DefaultScoringPolicy).aggregate_with_metrics(
            artifact,
            observations,
            analysis.metrics.to_btree_map(),
        ),
    )
}

pub fn movej(origin: &str, target: Vec<f64>) -> MotionSegment {
    MotionSegment::MoveJ {
        origin: OperationId(origin.to_string()),
        target,
        max_velocity: None,
        max_acceleration: None,
    }
}

// ── THE RUNTIME ADAPTER — replicated verbatim (design ADR-5) ────────────────

/// The runtime's mapping `Assessment → CandidateAssessment` (design ADR-5):
/// crisp risk = `1 − quality` and the CATEGORICAL verdict
/// `Risk::Critical → Rejected`, everything else Accepted. NO numeric
/// threshold — the Assessor is the single authority on "Critical".
pub fn map_assessment(assessment: &thalos_intelligence::Assessment) -> CandidateAssessment {
    CandidateAssessment {
        risk: 1.0 - assessment.quality,
        admissibility: match assessment.risk {
            Risk::Critical => RiskAdmissibility::Rejected,
            _ => RiskAdmissibility::Accepted,
        },
    }
}

/// The runtime's `MotionMetrics` extraction: duration / path_length from the
/// analyzed trajectory, avg_manipulability from the technical analysis
/// (design ADR-5 — the evaluator never computes a metric from the program).
pub fn extract_metrics(
    trajectory: &Trajectory,
    analysis: &thalos_planning::analysis::PlanAnalysis,
) -> MotionMetrics {
    MotionMetrics {
        duration: trajectory.duration(),
        avg_manipulability: analysis.metrics.avg_manipulability.unwrap_or(0.0),
        path_length: trajectory
            .waypoints()
            .windows(2)
            .map(|w| {
                w[1].joints()
                    .iter()
                    .zip(w[0].joints().iter())
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt()
            })
            .sum(),
    }
}

/// Joint bounds for the gate, from the chain's actuated joints (same source
/// the runtime uses — limits.enabled gates the closed interval).
pub fn joint_bounds(chain: &SerialChain) -> Vec<JointBounds> {
    chain
        .segments
        .iter()
        .filter(|s| s.joint.dof() > 0)
        .map(|s| {
            let limits = s.joint.limits();
            if limits.enabled {
                JointBounds {
                    lower: limits.min,
                    upper: limits.max,
                }
            } else {
                JointBounds {
                    lower: -std::f64::consts::PI,
                    upper: std::f64::consts::PI,
                }
            }
        })
        .collect()
}

/// The compacted task sequence `(kind, origin)` — the gate's task-identity
/// invariant, re-verified independently by consumers.
pub fn compact_task(program: &PlanningProgram) -> Vec<(&'static str, String)> {
    let mut runs: Vec<(&'static str, String)> = Vec::new();
    for segment in &program.segments {
        let kind = match segment {
            MotionSegment::MoveJ { .. } => "MoveJ",
            MotionSegment::MoveL { .. } => "MoveL",
            MotionSegment::MoveLPosition { .. } => "MoveLPosition",
        };
        let key = (kind, segment.origin().0.clone());
        match runs.last() {
            Some(last) if *last == key => {}
            _ => runs.push(key),
        }
    }
    runs
}

/// The first commanded MoveJ and the joint goal (last MoveJ target) of a
/// program — the gate's endpoint pair (ADR-1), re-verified independently.
pub fn commanded_endpoints(program: &PlanningProgram) -> (Option<Vec<f64>>, Option<Vec<f64>>) {
    let first = program.segments.iter().find_map(|s| match s {
        MotionSegment::MoveJ { target, .. } => Some(target.clone()),
        _ => None,
    });
    let goal = program.segments.iter().rev().find_map(|s| match s {
        MotionSegment::MoveJ { target, .. } => Some(target.clone()),
        _ => None,
    });
    (first, goal)
}

pub fn endpoints_within_epsilon(seed: &PlanningProgram, candidate: &PlanningProgram) -> bool {
    let (s_first, s_goal) = commanded_endpoints(seed);
    let (c_first, c_goal) = commanded_endpoints(candidate);
    let within = |seed: Option<&[f64]>, cand: Option<&[f64]>| match (seed, cand) {
        (None, None) => true,
        (Some(s), Some(c)) => {
            s.len() == c.len()
                && s.iter()
                    .zip(c.iter())
                    .all(|(qs, qc)| (qc - qs).abs() <= ENDPOINT_TOLERANCE)
        }
        _ => false,
    };
    within(s_first.as_deref(), c_first.as_deref()) && within(s_goal.as_deref(), c_goal.as_deref())
}

// ── The composed pipeline ────────────────────────────────────────────────────

/// The full outcome of the candidate pipeline. Carries the union of what the
/// feasibility / counterfactual / demo-scenarios consumers need: the seed's
/// own assessment + report + trajectory, the ranking, the gate outcome, the
/// generated candidates, the FULL strategy trace (ADR-3 observability) and
/// the per-candidate compiled trajectories / analysis reports.
pub struct PipelineOutcome {
    pub seed_assessment: thalos_intelligence::Assessment,
    pub seed_report: AnalysisReport,
    /// The seed's compiled trajectory (the plain path's trace).
    pub seed_trajectory: Trajectory,
    pub ranking: CandidateRanking,
    pub admissible: Vec<AdmissibleCandidate>,
    pub rejected: Vec<RejectedCandidate>,
    pub candidates: Vec<Candidate>,
    /// The FULL strategy trace (every strategy → Generated/Skipped) — the
    /// ADR-3 observability data the ranking now carries.
    pub traces: Vec<StrategyTrace>,
    /// Per-candidate compiled trajectory (index-aligned with `candidates`).
    pub trajectories: Vec<Option<Trajectory>>,
    /// Per-candidate analysis report (index-aligned with `candidates`).
    pub reports: Vec<Option<AnalysisReport>>,
}

/// Run the full real pipeline: generate → compile → analyze → assess → map →
/// gate → rank.
///
/// **Pipeline-completion contract**: returns `Err(stage error)` when the seed
/// fails to compile (e.g. IK failure, joint-limit violation) or an analysis
/// stage fails — the caller's scenario FAILS with the stage error instead of
/// silently degrading into a categorical result. Per-candidate compile
/// failures are NOT errors: they are the gate's designed input
/// (`compile_ok == false` → `CompileFailure` rejection).
pub fn run_pipeline(
    seed: &PlanningProgram,
    home: &[f64],
    target_segment: usize,
) -> Result<PipelineOutcome, String> {
    let robot = chain(RobotModel::Scara);
    let solver = real_solver(&robot);
    let generator = CandidateGenerator::default();
    let ctx = CandidateGenerationContext { target_segment };

    // 1. Generate (Direct is always candidate 0 — the seed itself). The FULL
    //    strategy trace is carried into the ranking (ADR-3), never dropped.
    let (candidates, traces) = generator.generate(seed, &ctx, &solver);

    // 2. Per candidate: compile → analyze → assess → map (the runtime adapter).
    let mut gate_rows: Vec<GateCandidate> = Vec::new();
    let mut trajectories: Vec<Option<Trajectory>> = Vec::new();
    let mut reports: Vec<Option<AnalysisReport>> = Vec::new();
    for candidate in &candidates {
        match compile(&robot, home, &candidate.program) {
            Ok(trajectory) => {
                let report = analyze(&robot, &trajectory)?;
                let assessment = Assessor::assess(&report);
                let neutral = map_assessment(&assessment);
                let (analysis, _obs) = {
                    let checker = NaiveCollisionChecker;
                    let matrix = CollisionMatrix::new();
                    let analyzer = TrajectoryAnalyzer::new(&robot, None)
                        .with_collision_checker(&checker, &matrix);
                    let artifact =
                        ArtifactRef::MotionPlan(MotionPlanId("demo-scenarios".to_string()));
                    analyzer
                        .analyze_with_observations(artifact, &trajectory)
                        .map_err(|e| e.to_string())?
                };
                gate_rows.push(GateCandidate {
                    candidate: candidate.clone(),
                    compile_ok: true,
                    assessment: Some(neutral),
                    metrics: Some(extract_metrics(&trajectory, &analysis)),
                });
                trajectories.push(Some(trajectory));
                reports.push(Some(report));
            }
            Err(_) => {
                gate_rows.push(GateCandidate {
                    candidate: candidate.clone(),
                    compile_ok: false,
                    assessment: None,
                    metrics: None,
                });
                trajectories.push(None);
                reports.push(None);
            }
        }
    }

    // The seed's OWN path (the plain assessment): compile → analyze → assess.
    // A seed that cannot compile is a REAL stage failure — it propagates.
    let seed_trajectory = compile(&robot, home, seed)?;
    let seed_report = analyze(&robot, &seed_trajectory)?;
    let seed_assessment = Assessor::assess(&seed_report);

    // 3. Gate (two phases: geometric invariants, then the risk policy).
    let bounds = joint_bounds(&robot);
    let report = AdmissibilityGate.filter(seed, &gate_rows, Some(&bounds));

    // 4. Rank (argmin J over the admissible set only). The ranking carries
    //    the full strategy trace (ADR-3 observability).
    let ranking = CandidateEvaluator::evaluate(
        &report.admissible,
        ObjectiveProfile::SafetyFirst,
        traces.clone(),
    );

    Ok(PipelineOutcome {
        seed_assessment,
        seed_report,
        seed_trajectory,
        ranking,
        admissible: report.admissible,
        rejected: report.rejected,
        candidates,
        traces,
        trajectories,
        reports,
    })
}

pub fn score_of<'a>(
    ranking: &'a CandidateRanking,
    candidate: &Candidate,
) -> Option<&'a CandidateScore> {
    ranking
        .ranked
        .iter()
        .find(|(c, _)| c == candidate)
        .map(|(_, s)| s)
}

// ── The ranked table — the demo's PRIMARY output (design demo table shape) ──

pub fn print_ranked_table(outcome: &PipelineOutcome, title: &str) {
    println!("\n{:=^90}", format!(" {} ", title));
    let seed = &outcome.seed_assessment;
    println!(
        "SEED (Direct)   verdict={:?}  crisp_risk={:.4}  quality={:.4}  singular={}  near_singular={}",
        seed.risk,
        1.0 - seed.quality,
        seed.quality,
        outcome
            .seed_report
            .metrics
            .get("singular_count")
            .copied()
            .unwrap_or(0.0),
        outcome
            .seed_report
            .metrics
            .get("near_singular_count")
            .copied()
            .unwrap_or(0.0),
    );
    println!(
        "{:<18} {:>8} {:>8} {:>10} {:>9} {:>13} {:>8}   status",
        "strategy", "risk", "quality", "singular", "dur(s)", "manip", "cost"
    );
    for (i, row) in outcome.candidates.iter().enumerate() {
        let admissible = outcome.admissible.iter().find(|a| a.candidate == *row);
        let rejected = outcome.rejected.iter().find(|r| r.candidate == *row);
        let singular = outcome.reports[i]
            .as_ref()
            .and_then(|r| r.metrics.get("singular_count"))
            .copied()
            .unwrap_or(0.0);
        let (risk, quality, duration, manip, cost, status) = match (admissible, rejected) {
            (Some(a), _) => (
                a.assessment.risk,
                1.0 - a.assessment.risk,
                a.metrics.duration,
                a.metrics.avg_manipulability,
                score_of(&outcome.ranking, &a.candidate)
                    .map(|s| s.cost)
                    .unwrap_or(f64::NAN),
                "admissible".to_string(),
            ),
            (None, Some(r)) => (
                r.assessment.as_ref().map(|a| a.risk).unwrap_or(f64::NAN),
                r.assessment
                    .as_ref()
                    .map(|a| 1.0 - a.risk)
                    .unwrap_or(f64::NAN),
                0.0,
                0.0,
                f64::NAN,
                format!("rejected: {:?}", r.reason),
            ),
            (None, None) => (
                f64::NAN,
                f64::NAN,
                0.0,
                0.0,
                f64::NAN,
                "no verdict".to_string(),
            ),
        };
        println!(
            "{:<18} {:>8.4} {:>8.4} {:>10} {:>9.3} {:>13.4} {:>8.4}   {}",
            format!("{:?}", row.strategy),
            risk,
            quality,
            singular,
            duration,
            manip,
            cost,
            status
        );
    }
    match &outcome.ranking.reason {
        SelectionReason::Selected {
            strategy,
            metric_comparison,
            ..
        } => {
            let direct_risk = outcome
                .admissible
                .iter()
                .find(|a| a.candidate.strategy == StrategyKind::Direct)
                .map(|a| a.assessment.risk)
                .unwrap_or(f64::NAN);
            let selected_risk = outcome
                .admissible
                .iter()
                .find(|a| a.candidate.strategy == *strategy)
                .map(|a| a.assessment.risk)
                .unwrap_or(f64::NAN);
            println!(
                "SELECTED: {:?} — risk {:.4} vs {:.4} | endpoints/task preserved | reason derived",
                strategy, selected_risk, direct_risk
            );
            println!(
                "  derived reason: {}",
                metric_comparison
                    .iter()
                    .map(|m| format!(
                        "{}: {:.4} vs {:.4}",
                        m.component, m.selected_value, m.baseline_value
                    ))
                    .collect::<Vec<_>>()
                    .join(" | ")
            );
        }
        SelectionReason::NoAdmissibleCandidate { reason } => {
            println!("SELECTED: none — {reason}");
        }
    }
    println!("{:=^90}", "");
}
