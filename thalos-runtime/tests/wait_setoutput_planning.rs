//! E2E: `wait` and `set_output` MUST survive into the ExecutionPlan.
//!
//! Regression for the silent drop: the THLS → plan path used to discard every
//! non-motion statement. `wait` must appear as an explicit `Delay` (and advance
//! the trajectory time); `set_output` must appear as an explicit instruction
//! without contributing geometry.

use thalos_engine::core::execution::plan::PlanInstruction;
use thalos_engine::core::models::{factory::RobotRegistry, RobotModel};
use thalos_runtime::planning::service::{PlanResult, PlanningService, RobotPlanningContext};

fn planar_ctx() -> RobotPlanningContext {
    let chain = RobotRegistry::create_default(RobotModel::Planar2R);
    RobotPlanningContext {
        robot_id: "robot_planar_2r".into(),
        chain,
        initial_positions: vec![0.0, 0.0],
        tcp: None,
    }
}

const SOURCE: &str = r#"
    target A = joints(0deg, 0deg)
    target B = joints(30deg, 30deg)
    fn main() {
        movej(A)
        wait(250ms)
        set_output(GRIPPER, true)
        movej(B)
    }
"#;

#[test]
fn wait_and_set_output_survive_into_the_plan() {
    let ctx = planar_ctx();
    let plan = match PlanningService::plan_thls_source_with_context(SOURCE, "prog_ws", 1, &ctx) {
        PlanResult::Planned(plan) => plan,
        PlanResult::Diagnostics(d) => panic!("expected Planned, got diagnostics: {d:?}"),
    };

    // movej, Delay, SetOutput, movej — nothing dropped.
    assert_eq!(plan.segments.len(), 4, "expected 4 segments: {:#?}", plan.segments);

    // `wait` → explicit Delay with the right duration.
    assert!(
        plan.segments.iter().any(|s| matches!(
            &s.instruction,
            PlanInstruction::Delay { seconds } if (seconds - 0.25).abs() < 1e-9
        )),
        "wait(250ms) must be an explicit Delay: {:#?}",
        plan.segments
    );

    // `set_output` → explicit instruction, no geometry.
    let set_output = plan
        .segments
        .iter()
        .find(|s| matches!(&s.instruction, PlanInstruction::SetOutput { .. }))
        .expect("set_output must be an explicit plan instruction");
    if let PlanInstruction::SetOutput { channel, value } = &set_output.instruction {
        assert_eq!(channel, "GRIPPER");
        assert!(*value);
    }
    assert!(
        set_output.waypoint_range.start == set_output.waypoint_range.end,
        "set_output must not contribute waypoints"
    );

    // `wait` advances the trajectory time while holding joints.
    let held = plan.waypoints.windows(2).any(|w| {
        w[0].joints == w[1].joints && (w[1].timestamp - w[0].timestamp - 0.25).abs() < 1e-6
    });
    assert!(held, "wait(250ms) must advance the trajectory time while holding");

    // ... and the total plan duration includes the wait.
    assert!(
        plan.duration >= 0.25,
        "plan duration must include the wait; got {}",
        plan.duration
    );
}
