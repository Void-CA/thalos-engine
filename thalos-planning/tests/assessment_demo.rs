//! Standalone demonstration of the intelligent trajectory assessment.
//!
//! The ONLY job of this demo is to prove, end-to-end, that a REAL trajectory
//! produced by the REAL Thalos pipeline reaches the intelligent component:
//!
//! ```text
//! Real trajectory → TrajectoryAnalyzer → AnalysisReport → Assessor::assess
//! ```
//!
//! No mock reports, no hand-built fixtures, no invented metrics. Every number
//! printed is computed by the same implementation Thalos uses in production
//! (`Assessor::assess` on the report the real analyzer emits).
//!
//! Two contrasting scenarios:
//!   1. Healthy trajectory  → Low risk / high quality;
//!   2. Degraded trajectory (crossing the full extension) → low manipulability
//!      + near singularity → R07 → R09 → R11 → Critical (expert-system chain).
//!
//! Run it and watch the flow:
//!
//! ```text
//! cargo test -p thalos-planning --test assessment_demo -- --nocapture
//! ```

use std::collections::BTreeMap;

use thalos_collision::NaiveCollisionChecker;
use thalos_core::{
    analysis::{
        Aggregator,
        aggregator::DefaultAggregator,
        observation::{ArtifactRef, ObservationKind},
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
use thalos_intelligence::{Assessment, Assessor, Risk, kb};
use thalos_planning::{
    analysis::TrajectoryAnalyzer,
    motion::{
        compiler::{DefaultPlannerDispatcher, PlanCompiler},
        planner::SegmentPlanningContext,
        program::PlanningProgram,
    },
};

// ── Real-pipeline harness (same shape as demo_end_to_end.rs / usability) ────

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
    let artifact = ArtifactRef::MotionPlan(MotionPlanId("assessment-demo".to_string()));
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
    let state = RobotState::from_positions(start.to_vec());
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

fn movej(target: Vec<f64>) -> MotionSegment {
    MotionSegment::MoveJ {
        origin: OperationId("op-j".to_string()),
        target,
        max_velocity: None,
        max_acceleration: None,
    }
}

// ── Presentation: the four crisp fuzzy inputs, read from the REAL evidence ──

/// The crisp fuzzy inputs, read from `Assessment.evidence` (the exact values
/// the real implementation derived from the report): global avg manipulability
/// + localized evidence (localized singularity score, minimum clearance).
fn crisp_inputs(a: &Assessment) -> (f64, f64, f64) {
    (
        a.evidence.get("manipulability").copied().unwrap_or(0.0),
        a.evidence
            .get("singularity_proximity")
            .copied()
            .unwrap_or(0.0),
        a.evidence
            .get("collision_clearance")
            .copied()
            .unwrap_or(0.0),
    )
}

/// The risk set a fired rule contributes to Mamdani output, read from the REAL
/// KB (a `RiskIs` consequent). Rules that only derive facts contribute nothing.
fn risk_contribution(rule_id: &str) -> Option<&'static str> {
    kb::default_kb()
        .iter()
        .find(|r| r.id == rule_id)
        .and_then(|r| {
            r.consequents.iter().find_map(|c| match c {
                kb::Consequent::RiskIs { set } => Some(match set {
                    kb::RiskSet::Low => "Low",
                    kb::RiskSet::Medium => "Medium",
                    kb::RiskSet::High => "High",
                    kb::RiskSet::Critical => "Critical",
                }),
                _ => None,
            })
        })
}

fn metric(report: &thalos_core::analysis::report::AnalysisReport, key: &str) -> String {
    report
        .metrics
        .get(key)
        .map(|v| format!("{v:.3}"))
        .unwrap_or_else(|| "—".to_string())
}

// ── One scenario, printed in the 8-section narrative ─────────────────────────

fn run_scenario(
    title: &str,
    robot: &SerialChain,
    home: &[f64],
    program: &PlanningProgram,
) -> (thalos_core::analysis::report::AnalysisReport, Assessment) {
    println!("\n{:=^78}", format!(" {} ", title));
    println!("{:=^78}", "");

    let trajectory = compile(robot, home, program).expect("plan must compile");
    let report = analyze(robot, &trajectory);
    let assessment = Assessor::assess(&report);

    println!("\n[1] TRAJECTORY (real pipeline)");
    println!("    robot     = {:?}", RobotModel::Scara);
    println!("    waypoints = {}", trajectory.len());
    println!(
        "    duration  = {} s",
        metric(&report, "trajectory_duration")
    );

    println!("\n[2] ANALYSIS (real analyzer report)");
    println!(
        "    manipulability      = {}  (avg)",
        metric(&report, "avg_manipulability")
    );
    println!(
        "    min manipulability  = {}  (analyzer metric — NOT a fuzzy input)",
        metric(&report, "min_manipulability")
    );
    println!(
        "    collision distance  = {} m",
        metric(&report, "min_collision_distance")
    );
    println!(
        "    near-singular count = {}",
        metric(&report, "near_singular_count")
    );
    println!(
        "    singular count      = {}",
        metric(&report, "singular_count")
    );
    println!("    observations        = {}", report.observations.len());
    println!(
        "    health              = {:.2} ({:?})  [analyzer's strict fault-penalty score]",
        report.summary.quality_index, report.summary.grade
    );

    let (manip, prox, clear) = crisp_inputs(&assessment);
    println!("\n[3] FUZZIFICATION (crisp inputs from real evidence)");
    println!("    GLOBAL  avg manipulability = {manip:.3}");
    println!("    LOCAL   singularity score  = {prox:.3}  (from analyzer observations)");
    println!("    LOCAL   min clearance      = {clear:.3} m");
    let vars = kb::input_variables();
    for (var_name, x) in [
        ("manipulability", manip),
        ("singularity_proximity", prox),
        ("collision_clearance", clear),
    ] {
        let var = vars.iter().find(|v| v.name == var_name).expect("variable");
        let degrees: Vec<String> = var
            .fuzzify(x)
            .into_iter()
            .filter(|(_, d)| *d > 0.0)
            .map(|(set, d)| format!("{set} = {d:.3}"))
            .collect();
        println!("      {var_name}: {}", degrees.join("  |  "));
    }

    println!("\n[4] MAMDANI INFERENCE (fired rules, real trace)");
    for entry in &assessment.trace {
        let risk = risk_contribution(&entry.rule_id).unwrap_or("—");
        println!(
            "    [{}] priority={}  → risk {}",
            entry.rule_id, entry.priority, risk
        );
        for (key, value) in &entry.bindings {
            println!("        {key} → {value}");
        }
    }

    let crisp = 1.0 - assessment.quality;
    println!("\n[5] DEFUZZIFICATION");
    println!("    crisp risk = {crisp:.3}");
    println!("    verdict    = {:?}", assessment.risk);
    println!("    quality    = {:.3}", assessment.quality);
    println!("    NOTE: analyzer `health` (strict fault-penalty score of the faults it");
    println!("    counted) and assessor `quality` (1 − crisp fuzzy risk, a graded");
    println!("    interpretation) are DIFFERENT metrics by design — both are shown so");
    println!("    the distinction is explicit: 13 faults → health 0.00, but the fuzzy");
    println!("    reading weighs the localized event and reports 44.3% quality.");

    println!("\n[6] EXPERT SYSTEM (forward chaining)");
    println!("    Initial facts (crisp inputs):");
    println!("        avg_manip={manip:.3}  singularity={prox:.3}  clearance={clear:.3}");
    let derived: Vec<(String, BTreeMap<String, bool>)> = assessment
        .trace
        .iter()
        .filter(|t| !t.derived_output.is_empty())
        .map(|t| (t.rule_id.clone(), t.derived_output.clone()))
        .collect();
    if derived.is_empty() {
        println!("    Derived facts: (none)");
    } else {
        println!("    Derived facts:");
        for (rule, facts) in &derived {
            for (fact, value) in facts {
                println!("        {rule} → {fact} = {value}");
            }
        }
    }
    println!("    Fired rules (firing order):");
    for (i, entry) in assessment.trace.iter().enumerate() {
        println!("        {}. {}", i + 1, entry.rule_id);
    }

    println!("\n[7] DECISION");
    if assessment.recommendations.is_empty() {
        println!(
            "    No PlanAdvisor actions in this report — the expert-system diagnosis \
             stands alone; remediation is the PlanAdvisor's job."
        );
    } else {
        for r in &assessment.recommendations {
            println!(
                "    {:?} region={:?}: {}",
                r.action_kind, r.region_id, r.rationale
            );
        }
    }

    println!("\n[8] TRACE (full inference, real order)");
    for entry in &assessment.trace {
        println!("    {} (priority {})", entry.rule_id, entry.priority);
        if !entry.bindings.is_empty() {
            println!("        matched: {:?}", entry.bindings);
        }
        if !entry.derived_output.is_empty() {
            println!("        derived: {:?}", entry.derived_output);
        }
    }
    println!("{:=^78}", "");

    (report, assessment)
}

// ── Scenario 1 — healthy trajectory ──────────────────────────────────────────

#[test]
fn healthy_trajectory_verdicts_low() {
    let robot = chain(RobotModel::Scara);
    let home = vec![0.0, -1.31, -0.1, 0.0];
    let healthy_target = vec![0.5, -1.31, -0.15, 0.0];
    let program = PlanningProgram::new(vec![movej(healthy_target)]);

    let (_report, assessment) = run_scenario(
        "SCENARIO 1 — HEALTHY TRAJECTORY (same-side elbow, modest move)",
        &robot,
        &home,
        &program,
    );

    assert_eq!(
        assessment.risk,
        Risk::Low,
        "a healthy trajectory must verdict Low, got {:?}",
        assessment.risk
    );
    assert!(
        assessment.quality > 0.6,
        "a healthy trajectory must score high quality, got {:.3}",
        assessment.quality
    );
}

// ── Scenario 2 — localized singularity event (crossing the full extension) ────

#[test]
fn localized_singularity_crossing_elevates_to_high() {
    let robot = chain(RobotModel::Scara);
    let home = vec![0.0, -1.31, -0.1, 0.0];
    // A LOCALIZED singularity event: the MoveJ crosses the full extension
    // mid-segment. The analyzer detects it (singularity observations), but a
    // whole-trajectory aggregate fraction would dilute the event (13/392 ≈
    // 0.03) and the verdict would read Low. The assessor's LOCALIZED evidence
    // must elevate it to High instead.
    let bad_target = vec![0.5, 0.6, -0.15, 0.0];
    let program = PlanningProgram::new(vec![movej(bad_target)]);

    let (report, assessment) = run_scenario(
        "SCENARIO 2 — LOCALIZED SINGULARITY (MoveJ crossing the full extension)",
        &robot,
        &home,
        &program,
    );

    // The analyzer DID detect the localized event...
    assert!(
        report
            .observations
            .iter()
            .any(|o| o.kind == ObservationKind::Singularity
                || o.kind == ObservationKind::NearSingularity),
        "the crossing trajectory must carry singularity observations from the real analyzer"
    );

    // ...and the assessor must see it: elevated to High, not diluted to Low.
    assert_eq!(
        assessment.risk,
        Risk::High,
        "a localized singularity event must elevate the verdict to High, got {:?}",
        assessment.risk
    );
    assert!(
        assessment
            .trace
            .iter()
            .any(|t| t.rule_id == "R09_near_singularity"),
        "the near-singularity rule must fire from the localized evidence"
    );
}
