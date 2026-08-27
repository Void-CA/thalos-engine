//! Acceptance scenarios for the trajectory assessor's knowledge base.
//!
//! This file FREEZES the current behavior of `Assessor::assess` as executable
//! evidence: every scenario is printed with its real crisp risk/quality, and
//! the target expectation is asserted. Before KB calibration the target
//! assertions FAIL (showing the current, miscalibrated verdict); after the KB
//! is corrected they PASS — turning this file into a permanent regression
//! guard.
//!
//! Each scenario names its three crisp fuzzy inputs directly:
//!   manip     = avg_manipulability
//!   prox      = singularity presence/severity, the DISCRETE scale
//!               {0.0, 0.15, 0.5} — 0.0 absent, 0.15 near-singular only,
//!               0.5 singular event (same scale as the fallback in
//!               `extract_inputs`, lib.rs — NOT a waypoint fraction)
//!   clearance = min_collision_distance (m)
//!
//! Run to observe the frozen behavior:
//!   cargo test -p thalos-intelligence --test acceptance_scenarios -- --nocapture

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

/// Build a report whose derived crisp inputs equal (manip, prox, clearance).
/// `waypoint_count = 10` is fixed. `prox` is the DISCRETE singularity
/// presence/severity score in {0.0, 0.15, 0.5} — the same scale the assessor's
/// fallback produces from counts (see `extract_inputs` in lib.rs). It is
/// inverted back into the counts the fallback consumes: 0.0 → none,
/// 0.15 → near-singular only, 0.5 → one singular event.
fn scenario(manip: f64, prox: f64, clearance: f64) -> AnalysisReport {
    let waypoints = 10.0;
    let (near, singular) = if prox >= 0.5 {
        (0.0, 1.0)
    } else if prox > 0.0 {
        (2.0, 0.0)
    } else {
        (0.0, 0.0)
    };
    let mut metrics = BTreeMap::new();
    metrics.insert("waypoint_count".to_string(), waypoints);
    metrics.insert("avg_manipulability".to_string(), manip);
    metrics.insert("near_singular_count".to_string(), near);
    metrics.insert("singular_count".to_string(), singular);
    metrics.insert("min_collision_distance".to_string(), clearance);
    metrics.insert("trajectory_duration".to_string(), 100.0);
    report(metrics)
}

#[test]
fn acceptance_scenarios_match_expectation() {
    // (name, manip, prox, clearance, expected verdict). `prox` is the discrete
    // presence/severity score: near-singular only reads MEDIUM (0.15 → R04),
    // a true singular event reads HIGH (0.5 → R09). The old waypoint-fraction
    // fixture had near-only read high (0.9 → R09); the discrete remap is the
    // representation correction that makes both paths agree.
    let cases: &[(&str, f64, f64, f64, Risk)] = &[
        ("healthy", 0.9, 0.0, 0.5, Risk::Low),
        ("marginal_manip", 0.29, 0.0, 0.5, Risk::Medium),
        ("clearly_low_manip", 0.1, 0.0, 0.5, Risk::High),
        ("near_singular_only", 0.9, 0.15, 0.5, Risk::Medium),
        // Clearly-low manip + near-only: the R07 High signal (0.667) and the
        // R04 Medium signal (0.75) blend just under the 0.5 boundary → Medium.
        // The compromised Critical path (R11) requires a TRUE singular event
        // (R09 → near_singularity fact) — see `low_manip_singular`.
        ("low_manip_near_singular", 0.1, 0.15, 0.5, Risk::Medium),
        ("low_manip_singular", 0.1, 0.5, 0.5, Risk::Critical),
        ("critical_clearance", 0.9, 0.0, -0.1, Risk::Critical),
        ("triple_degraded", 0.1, 0.5, 0.02, Risk::Critical),
    ];

    let mut failures = Vec::new();
    println!("\n=== ACCEPTANCE SCENARIOS (frozen current behavior) ===");
    for (name, manip, prox, clearance, expected) in cases {
        let a = Assessor::assess(&scenario(*manip, *prox, *clearance));
        let crisp = 1.0 - a.quality;
        println!(
            "{name:<24} manip={manip:.2} prox={prox:.2} clear={clearance:.2} \
             -> risk={:?} quality={:.3} crisp={crisp:.3} (expected {:?})",
            a.risk, a.quality, expected
        );
        if a.risk != *expected {
            failures.push(format!(
                "{name}: got {:?} (crisp {crisp:.3}), expected {:?}",
                a.risk, expected
            ));
        }
    }
    assert!(
        failures.is_empty(),
        "\nAcceptance failures:\n  {}",
        failures.join("\n  ")
    );
}

#[test]
fn boundaries_are_stable_and_monotonic() {
    // Acceptance requirement: no hidden discontinuities around the relevant
    // boundaries. Neighboring crisp inputs must yield the SAME verdict within
    // each semantic zone (stability), so a marginal manipulability of 0.28 and
    // 0.31 are not treated differently by a knife-edge. The singularity input
    // is a discrete presence/severity score {0.0, 0.15, 0.5} — each score
    // lands in its own zone (low / medium / high) with no knife-edge.
    let cases: &[(&str, f64, f64, f64, Risk)] = &[
        // Marginal manipulability zone — all stable at Medium.
        ("manip_0.28", 0.28, 0.0, 0.5, Risk::Medium),
        ("manip_0.29", 0.29, 0.0, 0.5, Risk::Medium),
        ("manip_0.30", 0.30, 0.0, 0.5, Risk::Medium),
        ("manip_0.31", 0.31, 0.0, 0.5, Risk::Medium),
        // Clearly-low manipulability zone — all stable at High.
        ("manip_0.05", 0.05, 0.0, 0.5, Risk::High),
        ("manip_0.10", 0.10, 0.0, 0.5, Risk::High),
        ("manip_0.15", 0.15, 0.0, 0.5, Risk::High),
        // Discrete singularity zones — absent → Low, near-only → Medium,
        // singular event → High.
        ("prox_absent_0.00", 0.9, 0.0, 0.5, Risk::Low),
        ("prox_near_0.15", 0.9, 0.15, 0.5, Risk::Medium),
        ("prox_singular_0.50", 0.9, 0.5, 0.5, Risk::High),
    ];

    let mut failures = Vec::new();
    println!("\n=== BOUNDARY NEIGHBORS (stability check) ===");
    for (name, manip, prox, clearance, expected) in cases {
        let a = Assessor::assess(&scenario(*manip, *prox, *clearance));
        let crisp = 1.0 - a.quality;
        println!(
            "{name:<14} manip={manip:.2} prox={prox:.2} clear={clearance:.2} \
             -> risk={:?} quality={:.3} crisp={crisp:.3} (expected {:?})",
            a.risk, a.quality, expected
        );
        if a.risk != *expected {
            failures.push(format!(
                "{name}: got {:?} (crisp {crisp:.3}), expected {:?}",
                a.risk, expected
            ));
        }
    }
    assert!(
        failures.is_empty(),
        "\nBoundary failures:\n  {}",
        failures.join("\n  ")
    );
}

#[test]
fn localized_singularity_observation_elevates_otherwise_healthy_trajectory() {
    // The canonical LOCALIZED path: the analyzer emits a Singularity observation
    // on an otherwise-healthy trajectory (high avg manipulability, no collision).
    // A whole-trajectory aggregate fraction would dilute the event (13/392 ≈
    // 0.03); the observation must elevate the verdict instead.
    let base = scenario(0.9, 0.0, 0.5);

    // Without the observation: healthy metrics alone stay Low.
    let without = Assessor::assess(&base);
    assert_eq!(
        without.risk,
        Risk::Low,
        "healthy metrics alone must stay Low, got {:?}",
        without.risk
    );

    // With a Singularity observation: the localized event is SEEN → High.
    let mut with_obs = base;
    with_obs.observations = vec![Observation {
        id: ObservationId(1),
        kind: ObservationKind::Singularity,
        severity: Severity::Error,
        artifact: with_obs.artifact.clone(),
        location: Location::Waypoint(200),
        attributes: BTreeMap::new(),
        causes: Vec::new(),
        related: Vec::new(),
    }];
    let with = Assessor::assess(&with_obs);
    assert_eq!(
        with.risk,
        Risk::High,
        "a localized singularity observation must elevate the verdict to High, got {:?} (crisp {:.3})",
        with.risk,
        1.0 - with.quality
    );
    assert!(
        with.trace
            .iter()
            .any(|t| t.rule_id == "R09_near_singularity"),
        "the near-singularity rule must fire on the localized event"
    );
}
