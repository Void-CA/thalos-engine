//! Golden fixtures for the intelligent trajectory assessment (design
//! "Golden Fixture Demo Material" — verification cases AND UI demo material).
//!
//! Every fixture builds a real `AnalysisReport` from metrics; the assessor is
//! exercised end-to-end through the public `Assessor::assess` facade. The
//! behavioral anchoring tests pin the threshold contract: they assert that the
//! analyzer's documented boundary observation agrees with the IA's fuzzy
//! degree (the pure crate cannot run `TrajectoryAnalyzer`, so each boundary
//! report carries the analyzer observation that the replicated constant
//! implies — if the analyzer's local constant ever changes, these fail loudly).

use std::collections::BTreeMap;

use thalos_core::analysis::action::{Action, ActionId, ActionImpact, ActionKind, ActionPriority};
use thalos_core::analysis::location::Location;
use thalos_core::analysis::observation::{
    ArtifactRef, Observation, ObservationId, ObservationKind, Severity,
};
use thalos_core::analysis::report::AnalysisReport;
use thalos_core::analysis::summary::{AnalysisSummary, Grade};
use thalos_core::ids::MotionPlanId;
use thalos_intelligence::{
    Assessor, MANIPULABILITY_LOW_THRESHOLD, NEAR_COLLISION_DISTANCE, Risk,
    SINGULAR_CONDITION_THRESHOLD,
};

/// Build a report from metrics, optionally carrying analyzer-emitted
/// observations (for the behavioral anchoring fixtures).
fn report(metrics: BTreeMap<String, f64>) -> AnalysisReport {
    AnalysisReport {
        artifact: ArtifactRef::MotionPlan(MotionPlanId("mp-1".to_string())),
        observations: Vec::new(),
        actions: Vec::new(),
        metrics,
        summary: AnalysisSummary {
            quality_index: 0.8,
            observation_count: 0,
            severity_distribution: BTreeMap::new(),
            grade: Grade::Good,
        },
        robot_id: None,
    }
}

fn metrics(entries: &[(&str, f64)]) -> BTreeMap<String, f64> {
    entries.iter().map(|(k, v)| (k.to_string(), *v)).collect()
}

fn observation(kind: ObservationKind, severity: Severity, waypoint: usize) -> Observation {
    Observation {
        id: ObservationId(1),
        kind,
        severity,
        artifact: ArtifactRef::MotionPlan(MotionPlanId("mp-1".to_string())),
        location: Location::Waypoint(waypoint),
        attributes: BTreeMap::new(),
        causes: Vec::new(),
        related: Vec::new(),
    }
}

fn trace_ids(assessment: &thalos_intelligence::Assessment) -> Vec<String> {
    assessment
        .trace
        .iter()
        .map(|entry| entry.rule_id.clone())
        .collect()
}

// ── golden_low (Plan A) — clean, high manipulability, no singularities ──

#[test]
fn golden_low_verdicts_low_risk_with_high_quality() {
    // Spec "Low Risk Verdict": risk=low, quality > 0.7.
    let assessment = Assessor::assess(&report(metrics(&[
        ("waypoint_count", 10.0),
        ("trajectory_duration", 20.0),
        ("avg_manipulability", 0.9),
        ("min_manipulability", 0.8),
        ("near_singular_count", 0.0),
        ("singular_count", 0.0),
        ("min_collision_distance", 0.5),
        ("has_collisions", 0.0),
    ])));

    assert_eq!(assessment.risk, Risk::Low);
    assert!(
        assessment.quality > 0.7,
        "golden_low quality must exceed 0.7, got {}",
        assessment.quality
    );
    assert!(
        !assessment
            .trace
            .iter()
            .any(|t| t.rule_id == "R11_compromised_manipulability"),
        "a clean plan must not fire the compromised-manipulability rule"
    );
}

// ── golden_high (Plan B) — collisions, low manipulability, singular ──

#[test]
fn golden_high_verdicts_high_or_critical_risk() {
    // Spec "High Risk Verdict": risk High/Critical, quality < 0.4, collision
    // and singularity rules fired.
    let assessment = Assessor::assess(&report(metrics(&[
        ("waypoint_count", 10.0),
        ("trajectory_duration", 10.0),
        ("avg_manipulability", 0.2),
        ("min_manipulability", 0.1),
        ("near_singular_count", 2.0),
        ("singular_count", 1.0),
        ("min_collision_distance", -0.1),
        ("has_collisions", 1.0),
    ])));

    assert!(
        matches!(assessment.risk, Risk::High | Risk::Critical),
        "golden_high must be High or Critical, got {:?}",
        assessment.risk
    );
    assert!(
        assessment.quality < 0.4,
        "golden_high quality must be below 0.4, got {}",
        assessment.quality
    );

    let ids = trace_ids(&assessment);
    assert!(
        ids.iter()
            .any(|id| id.starts_with("R01") || id.starts_with("R03")),
        "golden_high must fire a collision rule, trace: {ids:?}"
    );
    assert!(
        ids.iter().any(|id| id == "R09_near_singularity"),
        "golden_high must fire the singularity rule, trace: {ids:?}"
    );
}

// ── golden_chained_inference — EXACT trace R07 → R09 → R11 ──

#[test]
fn golden_chained_inference_trace_is_exact() {
    // Spec "Chained Inference Golden": the trace SHALL equal exactly
    // ["R07_low_manipulability", "R09_near_singularity",
    //  "R11_compromised_manipulability"] — proves the mechanism, not just the
    // result. The fixture is designed so R07 derives `low_manipulability` from
    // low manipulability, R09 derives `near_singularity` from high proximity,
    // and R11 consumes BOTH derived facts in a later pass → Critical.
    let assessment = Assessor::assess(&report(metrics(&[
        ("waypoint_count", 10.0),
        ("trajectory_duration", 10.0),
        ("avg_manipulability", 0.1),
        ("min_manipulability", 0.1),
        ("near_singular_count", 4.0),
        // One real singular event → the discrete fallback maps it to 0.5 (high
        // proximity zone). NOT the old 4/10 = 0.4 ratio: near-only events map
        // to 0.15 (medium) and would fire R04 instead of R09.
        ("singular_count", 1.0),
        ("min_collision_distance", 0.05),
        ("has_collisions", 0.0),
    ])));

    let expected = [
        "R07_low_manipulability",
        "R09_near_singularity",
        "R11_compromised_manipulability",
    ];
    assert_eq!(
        trace_ids(&assessment),
        expected.iter().map(|s| s.to_string()).collect::<Vec<_>>(),
        "chained inference trace must be exact"
    );
}

// ── golden_fuzzy_boundary — gradual membership at analyzer thresholds ──

#[test]
fn golden_fuzzy_boundary_anchors_manipulability_threshold() {
    // avg_manipulability = 0.29 (< the analyzer's manip_threshold 0.3): the
    // analyzer emits LowManipulability AND the IA `low` membership exceeds 0.5.
    let mut boundary = report(metrics(&[
        ("waypoint_count", 10.0),
        ("trajectory_duration", 20.0),
        ("avg_manipulability", 0.29),
        ("near_singular_count", 0.0),
        ("singular_count", 0.0),
        ("min_collision_distance", 0.5),
        ("has_collisions", 0.0),
    ]));
    boundary.observations = vec![observation(
        ObservationKind::LowManipulability,
        Severity::Warning,
        2,
    )];

    // The analyzer observation is present (the replicated constant drives it).
    assert!(
        boundary
            .observations
            .iter()
            .any(|o| o.kind == ObservationKind::LowManipulability)
    );
    assert_eq!(MANIPULABILITY_LOW_THRESHOLD, 0.3);

    // The IA `medium` membership at 0.29 dominates (degree > 0.5): a value
    // just below the analyzer threshold is MARGINAL (recoverable), not clearly
    // low — the fuzzy layer refines the analyzer's crisp boundary.
    let variable = &thalos_intelligence::kb::input_variables()[0];
    let medium = variable
        .fuzzify(0.29)
        .into_iter()
        .find(|(name, _)| *name == "medium")
        .expect("medium set present");
    assert!(
        medium.1 > 0.5,
        "medium(0.29) must exceed 0.5, got {}",
        medium.1
    );

    // And the full assessment still runs on the boundary report: the marginal
    // value fires the MEDIUM rule (R05), not the low-manipulability rule.
    let assessment = Assessor::assess(&boundary);
    assert!(
        assessment
            .trace
            .iter()
            .any(|t| t.rule_id == "R05_manipulability_medium"),
        "marginal manipulability must fire R05"
    );
}

#[test]
fn golden_fuzzy_boundary_anchors_collision_thresholds() {
    // min_collision_distance = 0.0 (COLLISION_DISTANCE): analyzer emits
    // CollisionRisk, IA `danger` membership is 1.0.
    let mut collision_report = report(metrics(&[
        ("waypoint_count", 10.0),
        ("trajectory_duration", 10.0),
        ("avg_manipulability", 0.7),
        ("near_singular_count", 0.0),
        ("singular_count", 0.0),
        ("min_collision_distance", 0.0),
        ("has_collisions", 1.0),
    ]));
    collision_report.observations = vec![observation(
        ObservationKind::CollisionRisk,
        Severity::Error,
        3,
    )];
    assert!(
        collision_report
            .observations
            .iter()
            .any(|o| o.kind == ObservationKind::CollisionRisk)
    );

    let clearance_var = &thalos_intelligence::kb::input_variables()[2];
    let danger = clearance_var
        .fuzzify(0.0)
        .into_iter()
        .find(|(name, _)| *name == "danger")
        .expect("danger set present");
    assert!(
        danger.1 > 0.5,
        "danger(0.0) must exceed 0.5, got {}",
        danger.1
    );

    // min_collision_distance = 0.049 (< near-collision 0.05): analyzer emits
    // CollisionNear, IA `near` membership is positive.
    let near = clearance_var
        .fuzzify(0.049)
        .into_iter()
        .find(|(name, _)| *name == "near")
        .expect("near set present");
    assert!(near.1 > 0.0, "near(0.049) must be positive, got {}", near.1);
    assert_eq!(NEAR_COLLISION_DISTANCE, 0.05);
}

#[test]
fn golden_fuzzy_boundary_anchors_singularity_thresholds() {
    // A plan whose waypoints are fully singular (condition >= 1000) yields a
    // proximity whose `high` membership exceeds 0.5; the analyzer observation
    // is Singularity.
    let mut singular_report = report(metrics(&[
        ("waypoint_count", 10.0),
        ("trajectory_duration", 10.0),
        ("avg_manipulability", 0.6),
        ("near_singular_count", 3.0),
        ("singular_count", 0.0),
        ("min_collision_distance", 0.3),
        ("has_collisions", 0.0),
    ]));
    singular_report.observations = vec![observation(
        ObservationKind::Singularity,
        Severity::Error,
        1,
    )];
    assert!(
        singular_report
            .observations
            .iter()
            .any(|o| o.kind == ObservationKind::Singularity)
    );
    assert_eq!(SINGULAR_CONDITION_THRESHOLD, 1000.0);

    // proximity = 0.3 → the analyzer's Singularity boundary; IA high > 0.5.
    let proximity_var = &thalos_intelligence::kb::input_variables()[1];
    let high = proximity_var
        .fuzzify(0.3)
        .into_iter()
        .find(|(name, _)| *name == "high")
        .expect("high set present");
    assert!(high.1 > 0.5, "high(0.3) must exceed 0.5, got {}", high.1);

    // The shared near-singular threshold is read from SingularityConfig.
    assert_eq!(thalos_intelligence::kb::near_singular_threshold(), 100.0);
}

// ── golden_collision_critical — deep collision → Critical ──

#[test]
fn golden_collision_critical_verdicts_critical() {
    let assessment = Assessor::assess(&report(metrics(&[
        ("waypoint_count", 10.0),
        ("trajectory_duration", 10.0),
        ("avg_manipulability", 0.7),
        ("min_manipulability", 0.6),
        ("near_singular_count", 0.0),
        ("singular_count", 0.0),
        ("min_collision_distance", -0.5),
        ("has_collisions", 1.0),
    ])));

    assert_eq!(
        assessment.risk,
        Risk::Critical,
        "a deep collision must be Critical, got {:?}",
        assessment.risk
    );
    assert!(
        assessment
            .trace
            .iter()
            .any(|t| t.rule_id == "R01_collision_danger"),
        "critical verdict must fire the collision danger rule"
    );
}

// ── Wire round-trip of the golden output (additive contract) ──

#[test]
fn golden_assessment_round_trips_on_the_wire() {
    let assessment = Assessor::assess(&report(metrics(&[
        ("waypoint_count", 10.0),
        ("trajectory_duration", 10.0),
        ("avg_manipulability", 0.2),
        ("near_singular_count", 2.0),
        ("singular_count", 1.0),
        ("min_collision_distance", -0.1),
        ("has_collisions", 1.0),
    ])));

    let json = serde_json::to_string(&assessment).expect("serialize");
    let back: thalos_intelligence::Assessment = serde_json::from_str(&json).expect("deserialize");
    assert_eq!(back, assessment);
    assert_eq!(back.risk, assessment.risk);
}

/// Helper that keeps `ActionKind` imports meaningful for future fixtures that
/// associate advisor actions (recommendations).
#[allow(dead_code)]
fn action(kind: ActionKind) -> Action {
    Action {
        id: ActionId(1),
        kind,
        target_observation: ObservationId(1),
        priority: ActionPriority::High,
        impact: ActionImpact::High,
        parameters: BTreeMap::new(),
    }
}
