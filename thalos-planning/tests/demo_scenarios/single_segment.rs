//! `single-segment-crossing` fixture (spec Requirement
//! "single-segment-crossing"). A single-segment program: no alternative
//! strategy can meaningfully operate on it (observable behavior — both
//! InsertWaypoint and AlternateElbow are `Skipped`), so Direct is selected
//! (boundedness / honesty: the generator is honest when no transformation is
//! possible).
//!
//! The concrete skip reasons (UnsupportedSegment / InvariantViolation) are
//! DIAGNOSTIC — documented in docs/execution/demos/demo-scenarios.md, never asserted.

use super::*;
use thalos_core::models::RobotModel;
use thalos_planning::candidate::StrategyKind;
use thalos_planning::motion::program::PlanningProgram;

pub fn single_segment_crossing() -> DemoScenario {
    DemoScenario {
        id: "single-segment-crossing",
        name: "Single-Segment Crossing",
        description: "A single-segment program on which no alternative strategy can \
                      meaningfully operate; both strategies skip and Direct is kept \
                      (boundedness / honesty).",
        demonstrates: "boundedness",
        robot: RobotModel::Scara,
        task: PlanningProgram::new(vec![movej("op-cross", vec![0.5, 0.6, -0.15, 0.0])]),
        home: vec![0.0, -1.31, -0.1, 0.0],
        target_segment: 0,
        expected_behavior: ExpectedBehavior {
            // The spec table for this scenario carries NO Direct-risk row; the
            // category below is DESCRIPTIVE (observed on the real pipeline).
            direct_risk_category: DirectRiskCategory::High,
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
