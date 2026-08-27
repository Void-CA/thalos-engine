//! DemoScenario representation (demo-scenarios change) — the CONSUMER-AGNOSTIC
//! scenario contract. Both the test harness (this crate) and the future UI
//! loader (a TS mirror in `web/`) consume the same behavior: a stable `id`, a
//! real `PlanningProgram`, and an `ExpectedBehavior` of CATEGORICAL invariants
//! — never exact numbers, never metrics (spec "MUST NOT contain f64 fields").
//!
//! Evidence numbers (e.g. crisp risk 0.5571) live in `docs/execution/demos/demo-scenarios.md`
//! as REFERENCE ONLY; the contract asserts categories and relative
//! comparisons, so a recalibration that shifts numbers by epsilon cannot break
//! the suite.

pub mod crossing;
pub mod healthy;
pub mod single_segment;

pub use crate::common::movej;
use crate::common::{PipelineOutcome, compact_task, endpoints_within_epsilon, run_pipeline};
use thalos_core::models::RobotModel;
use thalos_planning::candidate::{StrategyKind, StrategyOutcome};
use thalos_planning::motion::program::PlanningProgram;

/// A reproducible scenario demonstrating an intelligent motion capability.
/// Both test harness and future UI loader consume the same contract.
pub struct DemoScenario {
    /// Stable URL-safe slug (e.g. `crossing-pick-place-home`).
    pub id: &'static str,
    /// Human-readable name.
    pub name: &'static str,
    /// Human-readable description of what the scenario shows.
    pub description: &'static str,
    /// What the scenario demonstrates (capability tag).
    pub demonstrates: &'static str,
    /// Robot model (Scara for all three scenarios).
    pub robot: RobotModel,
    /// The seed planning program.
    pub task: PlanningProgram,
    /// Starting joint configuration.
    pub home: Vec<f64>,
    /// Target segment for candidate generation (0 for single-segment).
    pub target_segment: usize,
    /// Categorical expected behavior (no f64).
    pub expected_behavior: ExpectedBehavior,
}

/// Categorical expected behavior — NO f64 fields (spec mandate).
/// Evidence numbers documented in docs/execution/demos/demo-scenarios.md (reference only).
pub struct ExpectedBehavior {
    /// Direct risk category (Low/Med/High).
    pub direct_risk_category: DirectRiskCategory,
    /// At least one alternative that is Generated AND Admissible AND strictly
    /// better than Direct (spec `alternative_exists` semantic).
    pub alternative_exists: bool,
    /// The selected strategy.
    pub selected_strategy: StrategyKind,
    /// Per-strategy categorical outcomes (Generated/Skipped/Admissible/
    /// Rejected/Selected) — the spec's set of categorical states.
    pub strategy_outcomes: Vec<DemoStrategyOutcome>,
    /// Endpoints preserved within ε.
    pub endpoints_preserved: bool,
    /// Task sequence preserved.
    pub task_preserved: bool,
}

/// Categorical risk category — no f64 thresholds in the contract (design:
/// Low < 0.25, Medium < 0.5, High ≥ 0.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DirectRiskCategory {
    Low,
    Medium,
    High,
}

impl DirectRiskCategory {
    /// Map the Assessor's crisp risk (`1 − quality`) to the categorical
    /// bucket. The category is the CONTRACT; the crisp number is evidence.
    pub fn from_crisp(crisp: f64) -> DirectRiskCategory {
        if crisp < 0.25 {
            DirectRiskCategory::Low
        } else if crisp < 0.5 {
            DirectRiskCategory::Medium
        } else {
            DirectRiskCategory::High
        }
    }
}

/// Categorical strategy outcome — no metrics, no reason text in the contract.
/// EXACTLY ONE type (the categorical projection of the runtime's real
/// `StrategyOutcome`); the concrete reasons (UnsupportedSegment /
/// InvariantViolation) are DIAGNOSTIC, documented in the docs, never asserted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum DemoStrategyOutcome {
    Generated,
    Admissible,
    Rejected,
    Skipped,
    Selected,
}

/// Project the real pipeline outcome onto the categorical state SET (spec:
/// "a set of CATEGORICAL states, never metrics"). Reasons and metrics are
/// dropped at this boundary — a refactor that renames a reason string cannot
/// break the demo contract.
pub fn strategy_outcomes_of(outcome: &PipelineOutcome) -> Vec<DemoStrategyOutcome> {
    let mut states: Vec<DemoStrategyOutcome> = Vec::new();
    let mut push = |state: DemoStrategyOutcome| {
        if !states.contains(&state) {
            states.push(state);
        }
    };
    for trace in &outcome.traces {
        match &trace.outcome {
            StrategyOutcome::Generated(_) => push(DemoStrategyOutcome::Generated),
            StrategyOutcome::Skipped(_) => push(DemoStrategyOutcome::Skipped),
        }
    }
    for _ in &outcome.admissible {
        push(DemoStrategyOutcome::Admissible);
    }
    for _ in &outcome.rejected {
        push(DemoStrategyOutcome::Rejected);
    }
    if outcome.ranking.selected.is_some() {
        push(DemoStrategyOutcome::Selected);
    }
    states
}

/// The spec's `alternative_exists` semantic: at least one alternative that is
/// (a) GENERATED (every non-Direct candidate comes from `generate`), (b)
/// ADMISSIBLE (passed both gate phases), and (c) STRICTLY better than Direct
/// on the objective (`risk < direct_risk`). A case of `generated > 0` with
/// `admissible == 0` (or admissible without a better score) resolves false.
pub fn alternative_exists_of(outcome: &PipelineOutcome) -> bool {
    let Some(direct) = outcome
        .admissible
        .iter()
        .find(|a| a.candidate.strategy == StrategyKind::Direct)
    else {
        return false;
    };
    outcome.admissible.iter().any(|a| {
        a.candidate.strategy != StrategyKind::Direct
            && a.assessment.risk + 1e-12 < direct.assessment.risk
    })
}

/// Assert the scenario's INVARIANT CONTRACT against the real pipeline outcome.
/// Every row is categorical or relative — never an exact number. The strict
/// crisp thresholds (`> 0.5` / `< 0.25`) are asserted by the per-scenario
/// tests where the spec demands them; this fn enforces the categorical
/// projection.
pub fn assert_invariants(scenario: &DemoScenario, outcome: &PipelineOutcome) {
    let expected = &scenario.expected_behavior;
    let direct_crisp = 1.0 - outcome.seed_assessment.quality;

    // Row: Direct risk category (recalibration-resilient).
    assert_eq!(
        DirectRiskCategory::from_crisp(direct_crisp),
        expected.direct_risk_category,
        "scenario {}: Direct crisp risk {direct_crisp:.4} maps to the wrong category",
        scenario.id
    );

    // Row: alternative_exists — Generated AND Admissible AND strictly better.
    assert_eq!(
        alternative_exists_of(outcome),
        expected.alternative_exists,
        "scenario {}: alternative_exists semantic mismatch (admissible AND strictly \
         better than Direct, never merely 'generated')",
        scenario.id
    );

    // Row: selected strategy.
    let selected = outcome
        .ranking
        .selected
        .as_ref()
        .expect("a selection must exist")
        .strategy;
    assert_eq!(
        selected, expected.selected_strategy,
        "scenario {}: selected strategy mismatch",
        scenario.id
    );

    // Row: strategy_outcomes — the categorical SET, no metrics, no reasons.
    let mut actual = strategy_outcomes_of(outcome);
    let mut expected_set = expected.strategy_outcomes.clone();
    actual.sort();
    expected_set.sort();
    assert_eq!(
        actual, expected_set,
        "scenario {}: strategy outcome set mismatch",
        scenario.id
    );

    // Row: endpoints preserved (every admissible candidate ≤ ε per joint).
    let endpoints_preserved = outcome
        .admissible
        .iter()
        .all(|a| endpoints_within_epsilon(&scenario.task, &a.candidate.program));
    assert_eq!(
        endpoints_preserved, expected.endpoints_preserved,
        "scenario {}: endpoint preservation mismatch",
        scenario.id
    );

    // Row: task preserved (compacted (kind, origin) runs identical).
    let task_preserved = outcome
        .admissible
        .iter()
        .all(|a| compact_task(&scenario.task) == compact_task(&a.candidate.program));
    assert_eq!(
        task_preserved, expected.task_preserved,
        "scenario {}: task preservation mismatch",
        scenario.id
    );
}

// ── Unit tests (task 2.1 — RED first, GREEN now) ─────────────────────────────

#[test]
fn direct_risk_category_buckets_follow_the_crisp_ranges() {
    assert_eq!(DirectRiskCategory::from_crisp(0.0), DirectRiskCategory::Low);
    assert_eq!(
        DirectRiskCategory::from_crisp(0.24),
        DirectRiskCategory::Low
    );
    assert_eq!(
        DirectRiskCategory::from_crisp(0.25),
        DirectRiskCategory::Medium
    );
    assert_eq!(
        DirectRiskCategory::from_crisp(0.49),
        DirectRiskCategory::Medium
    );
    assert_eq!(
        DirectRiskCategory::from_crisp(0.5),
        DirectRiskCategory::High
    );
    assert_eq!(
        DirectRiskCategory::from_crisp(0.75),
        DirectRiskCategory::High
    );
}

#[test]
fn expected_behavior_contract_fields_are_categorical_only() {
    // Exhaustive destructure: adding ANY field to `ExpectedBehavior` (e.g.
    // `risk: f64`) makes this pattern non-exhaustive and FAILS TO COMPILE —
    // the spec's "MUST NOT contain f64 fields" is frozen at the type level.
    let eb = ExpectedBehavior {
        direct_risk_category: DirectRiskCategory::Low,
        alternative_exists: false,
        selected_strategy: StrategyKind::Direct,
        strategy_outcomes: vec![DemoStrategyOutcome::Generated],
        endpoints_preserved: true,
        task_preserved: true,
    };
    let ExpectedBehavior {
        direct_risk_category,
        alternative_exists,
        selected_strategy,
        strategy_outcomes,
        endpoints_preserved,
        task_preserved,
    } = &eb;
    assert_eq!(*direct_risk_category, DirectRiskCategory::Low);
    assert!(!alternative_exists);
    assert_eq!(*selected_strategy, StrategyKind::Direct);
    assert_eq!(strategy_outcomes.len(), 1);
    assert!(*endpoints_preserved);
    assert!(*task_preserved);
}

// ── Mapping tests (task 2.3 — the categorical projection runs on the REAL
//    pipeline; triangulated on crossing / single-segment / healthy) ──────────

#[test]
fn demo_scenario_representation_is_fully_populated_for_every_fixture() {
    // The representation is CONSUMER-AGNOSTIC: the future UI loader consumes
    // id/name/description/demonstrates/robot/task/home/target_segment/
    // expected_behavior unchanged — every field must be populated, and the
    // id must be a stable URL-safe slug.
    let scenarios = [
        crossing::crossing_pick_place_home(),
        healthy::healthy_pick_place_home(),
        single_segment::single_segment_crossing(),
    ];
    for s in &scenarios {
        assert!(!s.id.is_empty(), "id must be set");
        assert!(!s.name.is_empty(), "name must be set");
        assert!(!s.description.is_empty(), "description must be set");
        assert!(!s.demonstrates.is_empty(), "demonstrates must be set");
        assert_eq!(s.robot, RobotModel::Scara, "all scenarios run the Scara");
        assert!(!s.task.segments.is_empty(), "task must be a real program");
        assert_eq!(s.home.len(), 4, "home must be a 4-DOF joint config");
        assert!(
            s.id.chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
                && !s.id.starts_with('-')
                && !s.id.ends_with('-'),
            "id must be a stable URL-safe slug, got {:?}",
            s.id
        );
        assert!(
            !s.expected_behavior.strategy_outcomes.is_empty(),
            "strategy_outcomes must be populated"
        );
    }
    assert_ne!(scenarios[0].id, scenarios[1].id);
    assert_ne!(scenarios[1].id, scenarios[2].id);
    assert_ne!(scenarios[0].id, scenarios[2].id);
}

#[test]
fn strategy_outcomes_of_projects_the_crossing_pipeline_categorically() {
    let scenario = crossing::crossing_pick_place_home();
    let outcome = run_pipeline(&scenario.task, &scenario.home, scenario.target_segment)
        .expect("real pipeline must complete");
    let mut set = strategy_outcomes_of(&outcome);
    let mut expected = vec![
        DemoStrategyOutcome::Generated,
        DemoStrategyOutcome::Skipped,
        DemoStrategyOutcome::Admissible,
        DemoStrategyOutcome::Selected,
    ];
    set.sort();
    expected.sort();
    assert_eq!(
        set, expected,
        "the crossing projection must be the full categorical set — no metrics, \
         no reason text (the bare enum cannot carry either)"
    );
    assert!(
        alternative_exists_of(&outcome),
        "crossing: a generated + admissible + strictly-better alternative exists"
    );
}

#[test]
fn strategy_outcomes_of_projects_the_single_segment_pipeline_categorically() {
    let scenario = single_segment::single_segment_crossing();
    let outcome = run_pipeline(&scenario.task, &scenario.home, scenario.target_segment)
        .expect("real pipeline must complete");
    let set = strategy_outcomes_of(&outcome);
    assert!(
        set.contains(&DemoStrategyOutcome::Skipped),
        "both strategies skip on a single segment (category only)"
    );
    assert!(set.contains(&DemoStrategyOutcome::Selected));
    assert!(
        !alternative_exists_of(&outcome),
        "single-segment: no generated alternative can be strictly better (none is generated)"
    );
}

#[test]
fn alternative_exists_resolves_false_when_admissible_but_not_strictly_better() {
    // The spec's distinct situation "admissible > 0 with better_than_direct == 0":
    // the healthy alternate is ADMISSIBLE but its risk EQUALS Direct's — the
    // semantic must resolve FALSE, never the looser "generated > 0" reading.
    let scenario = healthy::healthy_pick_place_home();
    let outcome = run_pipeline(&scenario.task, &scenario.home, scenario.target_segment)
        .expect("real pipeline must complete");
    let alt_count = outcome
        .admissible
        .iter()
        .filter(|a| a.candidate.strategy != StrategyKind::Direct)
        .count();
    println!("admissible non-Direct = {alt_count} (degenerate same-side re-solve, equal risk)");
    assert!(
        !alternative_exists_of(&outcome),
        "admissible-but-equal-risk MUST NOT count as an existing better alternative"
    );
}
