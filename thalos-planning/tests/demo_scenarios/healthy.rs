//! `healthy-pick-place-home` fixture (spec Requirement
//! "healthy-pick-place-home"). Small-joint movements in a well-conditioned
//! region → Direct assesses Low and NO alternative is strictly better, so
//! Direct is selected (selectivity: do NOT invent an alternative when the
//! seed is already the best).
//!
//! ## Geometry verification (design open question — RESOLVED)
//!
//! The design's PROPOSED geometry (`[0.05, -1.25, -0.1, 0.0]`) was run through
//! the REAL pipeline first: Direct assessed Low (0.1470) and no alternative
//! was strictly better, BUT the evaluator selected `AlternateElbow` — the
//! same-side elbow re-solve is degenerate in a well-conditioned region
//! (sub-ε joint drift, identical risk 0.1470), and its tiny path-length
//! perturbation wins the min-max J tie-break. The contract row "Selected =
//! Direct" did NOT hold, so per the spec ("If a scenario cannot pass with
//! real geometry, it SHALL NOT be included" → change the fixture, NEVER relax
//! the contract) the geometry below was chosen and VERIFIED:
//!
//! ```text
//! [MoveJ home (0.0, -1.31, -0.1, 0.0)  →  MoveJ shift (0.2, -1.31, -0.1, 0.0)
//!   →  MoveJ goal (0.0, -1.31, -0.1, 0.0)],  target_segment = 1
//! ```
//!
//! A radial out-and-back at constant q2: Direct crisp risk 0.1470 (Low),
//! AlternateElbow admissible with EQUAL risk (0.1470) and a slightly LONGER
//! path → `alternative_exists == false` (admissible but not strictly better)
//! AND the evaluator keeps Direct (J 0.45 vs 0.55). All three contract rows
//! hold on real geometry. The sub-epsilon tie-break is documented as evidence
//! in docs/execution/demos/demo-scenarios.md (reference only, not asserted).

use super::*;
use thalos_core::models::RobotModel;
use thalos_planning::candidate::StrategyKind;
use thalos_planning::motion::program::PlanningProgram;

pub fn healthy_pick_place_home() -> DemoScenario {
    DemoScenario {
        id: "healthy-pick-place-home",
        name: "Healthy Pick-Place Home",
        description: "Small joint movements in a well-conditioned region; the system \
                      correctly decides NO alternative is needed and keeps Direct.",
        demonstrates: "selectivity",
        robot: RobotModel::Scara,
        task: PlanningProgram::new(vec![
            movej("op-home", vec![0.0, -1.31, -0.1, 0.0]),
            movej("op-shift", vec![0.2, -1.31, -0.1, 0.0]),
            movej("op-goal", vec![0.0, -1.31, -0.1, 0.0]),
        ]),
        home: vec![0.0, -1.31, -0.1, 0.0],
        target_segment: 1,
        expected_behavior: ExpectedBehavior {
            direct_risk_category: DirectRiskCategory::Low,
            alternative_exists: false,
            selected_strategy: StrategyKind::Direct,
            strategy_outcomes: vec![
                DemoStrategyOutcome::Generated,
                DemoStrategyOutcome::Skipped,
                DemoStrategyOutcome::Admissible,
                DemoStrategyOutcome::Selected,
            ],
            endpoints_preserved: true,
            task_preserved: true,
        },
    }
}
