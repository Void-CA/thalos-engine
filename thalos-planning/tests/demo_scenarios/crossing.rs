//! `crossing-pick-place-home` fixture — the validated counterfactual star
//! (spec Requirement "crossing-pick-place-home"; same scenario the PR3/PR4
//! feasibility + counterfactual tests proved).
//!
//! Three-segment MoveJ, middle-segment crossing (target_segment = 1). Direct
//! assesses High; `AlternateElbow` re-solves the crossing to the same-side
//! elbow posture (same cartesian path, q1 stays negative → no full-extension
//! crossing) and is selected. Evidence numbers (0.5571 / 0.1625) are
//! REFERENCE ONLY — see docs/execution/demos/demo-scenarios.md.

use super::*;
use thalos_core::models::RobotModel;
use thalos_planning::candidate::StrategyKind;
use thalos_planning::motion::program::PlanningProgram;

pub fn crossing_pick_place_home() -> DemoScenario {
    DemoScenario {
        id: "crossing-pick-place-home",
        name: "Crossing Pick-Place Home",
        description: "A three-segment pick-and-place program whose middle segment crosses \
                      the full-extension singularity; counterfactual reasoning selects a \
                      safer realization of the same task.",
        demonstrates: "counterfactual-reasoning",
        robot: RobotModel::Scara,
        task: PlanningProgram::new(vec![
            movej("op-home", vec![0.0, -1.31, -0.1, 0.0]),
            movej("op-cross", vec![0.5, 0.6, -0.15, 0.0]),
            movej("op-goal", vec![0.5, -1.31, -0.15, 0.0]),
        ]),
        home: vec![0.0, -1.31, -0.1, 0.0],
        target_segment: 1,
        expected_behavior: ExpectedBehavior {
            direct_risk_category: DirectRiskCategory::High,
            alternative_exists: true,
            selected_strategy: StrategyKind::AlternateElbow,
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
