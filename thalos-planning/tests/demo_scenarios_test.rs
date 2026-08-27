//! Demo-scenarios integration suite (demo-scenarios change): each scenario
//! runs the REAL candidate pipeline (generate → compile → analyze → assess →
//! gate → rank) and asserts its INVARIANT CONTRACT — never exact numbers
//! (evidence lives in docs/execution/demos/demo-scenarios.md, reference only).
//!
//! Pipeline-completion guard: every scenario unwraps `run_pipeline` — a stage
//! failure FAILS the scenario with the stage error; it can never silently
//! degrade into a categorical result (see the dedicated guard test).

mod common;
mod demo_scenarios;

use common::*;
use demo_scenarios::crossing::crossing_pick_place_home;
use demo_scenarios::healthy::healthy_pick_place_home;
use demo_scenarios::single_segment::single_segment_crossing;
use demo_scenarios::{DirectRiskCategory, assert_invariants};
use thalos_planning::candidate::{StrategyKind, StrategyOutcome};
use thalos_planning::motion::program::PlanningProgram;

/// The analyzer's plan-level singular waypoint count for candidate index `idx`.
fn singular_count(outcome: &PipelineOutcome, idx: usize) -> f64 {
    outcome.reports[idx]
        .as_ref()
        .and_then(|r| r.metrics.get("singular_count"))
        .copied()
        .unwrap_or(0.0)
}

// ── 1. crossing-pick-place-home — the validated counterfactual star ──────────

#[test]
fn crossing_pick_place_home_holds_the_six_row_contract() {
    let scenario = crossing_pick_place_home();
    let outcome = run_pipeline(&scenario.task, &scenario.home, scenario.target_segment)
        .expect("the crossing scenario must complete every pipeline stage");

    print_ranked_table(
        &outcome,
        "DEMO — CROSSING-PICK-PLACE-HOME (target_segment = 1)",
    );

    // Row 1 — Direct risk: seed_risk > 0.5 (High), STRICTLY (not == 0.5571).
    let direct_crisp = 1.0 - outcome.seed_assessment.quality;
    assert!(
        direct_crisp > 0.5,
        "Direct must assess High (crisp > 0.5), got {direct_crisp:.4}"
    );
    assert_eq!(
        DirectRiskCategory::from_crisp(direct_crisp),
        scenario.expected_behavior.direct_risk_category
    );

    // Row 2 — alternative exists: ≥1 admissible non-Direct with risk < seed.
    let better: Vec<_> = outcome
        .admissible
        .iter()
        .filter(|a| {
            a.candidate.strategy != StrategyKind::Direct && a.assessment.risk + 1e-12 < direct_crisp
        })
        .collect();
    assert!(
        !better.is_empty(),
        "≥1 admissible generated alternative must be strictly better than Direct"
    );

    // Row 3 — selected ≠ Direct, J_selected ≤ J_direct.
    let selected = outcome
        .ranking
        .selected
        .as_ref()
        .expect("a selection must exist");
    assert_ne!(
        selected.strategy,
        StrategyKind::Direct,
        "the counterfactual must select an alternative, not the seed"
    );
    let direct = outcome
        .admissible
        .iter()
        .find(|a| a.candidate.strategy == StrategyKind::Direct)
        .expect("Direct admissible");
    let selected_score = score_of(&outcome.ranking, selected).expect("selected is ranked");
    let direct_score = score_of(&outcome.ranking, &direct.candidate).expect("Direct is ranked");
    assert!(
        selected_score.cost <= direct_score.cost + 1e-9,
        "J_selected {:.4} ≤ J_direct {:.4}",
        selected_score.cost,
        direct_score.cost
    );

    // Row 4 — singularities: singular_selected < singular_direct.
    let direct_singular = singular_count(&outcome, 0);
    let selected_idx = outcome
        .candidates
        .iter()
        .position(|c| c == selected)
        .expect("the selected candidate must be one of the generated rows");
    let selected_singular = singular_count(&outcome, selected_idx);
    assert!(
        selected_singular < direct_singular,
        "singular_selected ({selected_singular}) must be < singular_direct ({direct_singular})"
    );

    // Rows 5 + 6 — endpoints ≤ ε per joint and compacted (kind, origin) task
    // identical, plus the categorical contract rows (alternative_exists,
    // selected_strategy, strategy_outcomes set).
    assert_invariants(&scenario, &outcome);

    // The pipeline carries the full observability surface (ADR-3): the seed's
    // own compiled trajectory and the per-candidate compiled trajectories.
    assert!(
        !outcome.seed_trajectory.waypoints().is_empty(),
        "the seed's trajectory trace must exist"
    );
    assert!(
        outcome.trajectories[0].is_some(),
        "Direct must compile and carry a trajectory"
    );
}

// ── 2. healthy-pick-place-home — selectivity ────────────────────────────────

#[test]
fn healthy_pick_place_home_holds_the_three_row_contract() {
    let scenario = healthy_pick_place_home();
    let outcome = run_pipeline(&scenario.task, &scenario.home, scenario.target_segment)
        .expect("the healthy scenario must complete every pipeline stage");

    print_ranked_table(
        &outcome,
        "DEMO — HEALTHY-PICK-PLACE-HOME (target_segment = 1)",
    );

    // Row 1 — Direct risk: seed_risk < 0.25 (Low), STRICTLY.
    let direct_crisp = 1.0 - outcome.seed_assessment.quality;
    assert!(
        direct_crisp < 0.25,
        "Direct must assess Low (crisp < 0.25), got {direct_crisp:.4}"
    );

    // Row 2 — no better alternative: zero admissible non-Direct with
    // risk < seed_risk.
    let better = outcome
        .admissible
        .iter()
        .filter(|a| {
            a.candidate.strategy != StrategyKind::Direct && a.assessment.risk + 1e-12 < direct_crisp
        })
        .count();
    assert_eq!(
        better, 0,
        "no admissible alternative may be strictly better than Direct"
    );

    // Row 3 — Direct selected.
    let selected = outcome
        .ranking
        .selected
        .as_ref()
        .expect("a selection must exist");
    assert_eq!(
        selected.strategy,
        StrategyKind::Direct,
        "selectivity: Direct is already the best — the system must keep it"
    );

    // The categorical contract rows.
    assert_invariants(&scenario, &outcome);
}

// ── 3. single-segment-crossing — boundedness / honesty ──────────────────────

#[test]
fn single_segment_crossing_holds_the_three_row_contract() {
    let scenario = single_segment_crossing();
    let outcome = run_pipeline(&scenario.task, &scenario.home, scenario.target_segment)
        .expect("the single-segment scenario must complete every pipeline stage");

    print_ranked_table(
        &outcome,
        "DEMO — SINGLE-SEGMENT-CROSSING (target_segment = 0)",
    );

    // Row 1 — no eligible segment: no alternative candidate was generated
    // (observable behavior — the strategies cannot meaningfully operate on a
    // single-segment program; NO dependency on `select_candidate_target_segment`
    // and NO reason-string assertions).
    assert_eq!(
        outcome.candidates.len(),
        1,
        "only Direct may be generated on a single segment"
    );
    assert_eq!(
        outcome.candidates[0].strategy,
        StrategyKind::Direct,
        "the only candidate is Direct (the seed)"
    );

    // Row 2 — both generating strategies Skipped (CATEGORY only; the concrete
    // reasons — UnsupportedSegment / InvariantViolation — are diagnostic, in
    // docs/execution/demos/demo-scenarios.md, never asserted here).
    let generating: Vec<_> = outcome
        .traces
        .iter()
        .filter(|t| t.strategy != StrategyKind::Direct)
        .collect();
    assert_eq!(
        generating.len(),
        2,
        "InsertWaypoint + AlternateElbow must both appear in the trace"
    );
    for trace in &generating {
        assert!(
            matches!(trace.outcome, StrategyOutcome::Skipped(_)),
            "{:?} must be Skipped on a single segment (category only)",
            trace.strategy
        );
    }

    // Row 3 — Direct selected.
    let selected = outcome
        .ranking
        .selected
        .as_ref()
        .expect("a selection must exist");
    assert_eq!(
        selected.strategy,
        StrategyKind::Direct,
        "boundedness: with no applicable strategy, Direct is kept"
    );

    // The categorical contract rows.
    assert_invariants(&scenario, &outcome);
}

// ── 4. Pipeline-completion guard (spec "Pipeline Completion (honesty guard)")

#[test]
fn pipeline_stage_failure_fails_the_scenario_never_a_categorical_pass() {
    // A seed that cannot compile (joint targets far outside the Scara's joint
    // limits → the planner rejects the program) MUST fail the scenario with
    // the stage error — it MUST NOT degrade into a categorical
    // "no better alternative" pass (a broken pipeline must never look like a
    // valid semantic outcome).
    let broken = PlanningProgram::new(vec![movej("op-broken", vec![99.0, 99.0, 99.0, 99.0])]);
    let err = match run_pipeline(&broken, &[0.0, -1.31, -0.1, 0.0], 0) {
        Ok(_) => panic!(
            "an un-compilable seed MUST fail the pipeline — a categorical outcome \
             would mask a broken stage as a valid semantic result"
        ),
        Err(e) => e,
    };
    assert!(
        !err.is_empty(),
        "the propagated stage error must carry a message, got {err:?}"
    );
    println!("GUARD: broken-stage scenario failed with stage error: {err}");
}
