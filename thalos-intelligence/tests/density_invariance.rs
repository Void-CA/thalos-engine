//! Density-invariance and same-scale regression tests for the singularity
//! fallback (design "Two-path singularity semantics", ADR-004 "Derived Feature
//! Scale Contract for the Intelligence KB").
//!
//! The fallback maps aggregated singularity counts to the DISCRETE
//! presence/severity scale {0.0, 0.15, 0.5} — the same scale as the
//! observations path. These tests pin that mapping:
//!   - `fallback_is_density_invariant`: the score depends on PRESENCE, not on
//!     how densely the trajectory was discretized. If someone reintroduces
//!     `events / waypoint_count`, this test breaks (1/10 != 1/100).
//!   - `fallback_remaps_*`: each event category maps to its discrete score.
//!   - `same_scale_observations_vs_fallback`: both paths produce the same
//!     score for the same semantic event.
//!   - `healthy_dense_trajectory_stays_low`: behavior guard — a healthy dense
//!     trajectory is never promoted by a residual complexity-like signal.

use std::collections::BTreeMap;

use thalos_core::analysis::location::Location;
use thalos_core::analysis::observation::{
    ArtifactRef, Observation, ObservationId, ObservationKind, Severity,
};
use thalos_core::analysis::report::AnalysisReport;
use thalos_core::analysis::summary::{AnalysisSummary, Grade};
use thalos_core::ids::MotionPlanId;
use thalos_intelligence::{Assessor, Risk};

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

fn singularity_proximity(assessment: &thalos_intelligence::Assessment) -> f64 {
    assessment.evidence["singularity_proximity"]
}

/// A metric-only report (no observations → fallback path) whose trajectory
/// carries a single singular event, discretized at `waypoints` waypoints.
fn singular_report(waypoints: f64) -> AnalysisReport {
    report(BTreeMap::from([
        ("waypoint_count".to_string(), waypoints),
        ("trajectory_duration".to_string(), 20.0),
        ("avg_manipulability".to_string(), 0.9),
        ("near_singular_count".to_string(), 0.0),
        ("singular_count".to_string(), 1.0),
        ("min_collision_distance".to_string(), 0.5),
    ]))
}

#[test]
fn fallback_is_density_invariant() {
    // Spec "Density-invariance holds": the SAME semantic trajectory (one
    // singular event) discretized at 10 vs 100 waypoints must produce the SAME
    // fallback score. The old `(near + singular) / waypoint_count` gives
    // 1/10 = 0.1 vs 1/100 = 0.01 — unequal — so a future regression that
    // divides by waypoint_count MUST break this test.
    let sparse = Assessor::assess(&singular_report(10.0));
    let dense = Assessor::assess(&singular_report(100.0));

    assert_eq!(singularity_proximity(&sparse), 0.5);
    assert_eq!(singularity_proximity(&dense), 0.5);
    assert_eq!(
        singularity_proximity(&sparse),
        singularity_proximity(&dense),
        "fallback must be invariant to trajectory discretization density"
    );
}

#[test]
fn fallback_remaps_singular_event_to_point_five() {
    // Spec "Fallback path — singular count present": singular_count=1,
    // waypoint_count=100 → 0.5 (NOT 1/100 = 0.01).
    let assessment = Assessor::assess(&singular_report(100.0));
    assert_eq!(
        singularity_proximity(&assessment),
        0.5,
        "a singular event must map to 0.5 regardless of waypoint density"
    );
}

#[test]
fn fallback_remaps_near_event_to_point_fifteen() {
    // Spec "Fallback path — near-singular only": near=5, singular=0,
    // waypoint_count=50 → 0.15 (NOT 5/50 = 0.10).
    let assessment = Assessor::assess(&report(BTreeMap::from([
        ("waypoint_count".to_string(), 50.0),
        ("trajectory_duration".to_string(), 20.0),
        ("avg_manipulability".to_string(), 0.9),
        ("near_singular_count".to_string(), 5.0),
        ("singular_count".to_string(), 0.0),
        ("min_collision_distance".to_string(), 0.5),
    ])));
    assert_eq!(
        singularity_proximity(&assessment),
        0.15,
        "near-singular events only must map to 0.15, not a waypoint fraction"
    );
}

#[test]
fn fallback_remaps_absent_events_to_zero() {
    // Spec "Fallback path — no singularity metrics": zero counts → 0.0.
    let assessment = Assessor::assess(&report(BTreeMap::from([
        ("waypoint_count".to_string(), 50.0),
        ("trajectory_duration".to_string(), 20.0),
        ("avg_manipulability".to_string(), 0.9),
        ("near_singular_count".to_string(), 0.0),
        ("singular_count".to_string(), 0.0),
        ("min_collision_distance".to_string(), 0.5),
    ])));
    assert_eq!(singularity_proximity(&assessment), 0.0);
}

#[test]
fn same_scale_observations_vs_fallback() {
    // Spec "Same-scale holds for near-singular event": one NearSingularity
    // OBSERVATION (observations path) and near_singular_count=1 with NO
    // observations (fallback path) must produce the SAME score, 0.15.
    let mut via_observation = report(BTreeMap::from([
        ("waypoint_count".to_string(), 10.0),
        ("trajectory_duration".to_string(), 20.0),
        ("avg_manipulability".to_string(), 0.9),
        ("near_singular_count".to_string(), 0.0),
        ("singular_count".to_string(), 0.0),
        ("min_collision_distance".to_string(), 0.5),
    ]));
    via_observation.observations = vec![Observation {
        id: ObservationId(1),
        kind: ObservationKind::NearSingularity,
        severity: Severity::Warning,
        artifact: via_observation.artifact.clone(),
        location: Location::Waypoint(3),
        attributes: BTreeMap::new(),
        causes: Vec::new(),
        related: Vec::new(),
    }];

    let via_fallback = report(BTreeMap::from([
        ("waypoint_count".to_string(), 10.0),
        ("trajectory_duration".to_string(), 20.0),
        ("avg_manipulability".to_string(), 0.9),
        ("near_singular_count".to_string(), 1.0),
        ("singular_count".to_string(), 0.0),
        ("min_collision_distance".to_string(), 0.5),
    ]));

    let obs_score = singularity_proximity(&Assessor::assess(&via_observation));
    let fallback_score = singularity_proximity(&Assessor::assess(&via_fallback));
    assert_eq!(obs_score, 0.15);
    assert_eq!(fallback_score, 0.15);
    assert_eq!(
        obs_score, fallback_score,
        "observations and fallback paths must share one presence/severity scale"
    );
}

#[test]
fn healthy_dense_trajectory_stays_low() {
    // Spec "Behavior Test — Healthy Dense Trajectory Not Promoted": a healthy
    // dense trajectory (high manipulability, safe clearance, ~100 waypoints per
    // second — the real-pipeline density that fired R06, e.g. 392/3.9, no
    // observations) must stay Risk::Low — the removed R06 / complexity signal
    // cannot promote it, and no complexity/trajectory rule fires.
    let assessment = Assessor::assess(&report(BTreeMap::from([
        ("waypoint_count".to_string(), 1000.0),
        ("trajectory_duration".to_string(), 10.0),
        ("avg_manipulability".to_string(), 0.9),
        ("near_singular_count".to_string(), 0.0),
        ("singular_count".to_string(), 0.0),
        ("min_collision_distance".to_string(), 0.5),
    ])));

    assert_eq!(
        assessment.risk,
        Risk::Low,
        "a healthy dense trajectory must stay Low, got {:?}",
        assessment.risk
    );
    for entry in &assessment.trace {
        assert!(
            !entry.rule_id.contains("complexity") && !entry.rule_id.contains("trajectory"),
            "trace must not contain a complexity/trajectory rule, got {}",
            entry.rule_id
        );
    }
}
