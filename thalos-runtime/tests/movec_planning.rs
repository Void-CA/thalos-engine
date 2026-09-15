//! End-to-end `movec` (circular move) planning tests.
//!
//! Covers the full THLS → parse → compile → resolve → plan path for circular
//! moves, plus the rejection cases (collinear arc, non-cartesian via/target).

use thalos_engine::core::kinematics::forward::ForwardKinematics;
use thalos_engine::core::models::{factory::RobotRegistry, RobotModel};
use thalos_engine::math::Vector3;
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

/// A reachable, non-collinear arc: start (2.0,0,0) → via (1.7,0.7,0) → end (1.4,1.0,0).
const VALID_MOVEC: &str = r#"
    target VIA = position([1700mm, 700mm, 0mm])
    target END = position([1400mm, 1000mm, 0mm])
    fn main() {
        movec(VIA, END)
    }
"#;

#[test]
fn movec_plans_a_circular_arc() {
    let ctx = planar_ctx();
    let result = PlanningService::plan_thls_source_with_context(VALID_MOVEC, "prog_movec", 1, &ctx);

    let plan = match result {
        PlanResult::Planned(plan) => plan,
        PlanResult::Diagnostics(d) => panic!("expected Planned, got diagnostics: {d:?}"),
    };

    assert!(!plan.waypoints.is_empty(), "plan must have waypoints");
    assert_eq!(plan.segments.len(), 1);
    assert_eq!(
        plan.segments[0].instruction,
        thalos_engine::core::execution::plan::PlanInstruction::MoveC,
        "the single segment must be a MoveC"
    );

    // The planned path must actually pass near the via point. A straight line
    // between start and end is ~0.10 m away from the via, so a min distance
    // well under that proves the arc bulges through the via.
    let fk = ForwardKinematics::new(ctx.chain.clone());
    let via = Vector3::new(1.7, 0.7, 0.0);
    let mut min_distance = f64::MAX;
    for wp in &plan.waypoints {
        let res = fk.evaluate(&wp.joints);
        if let Some(pose) = res.ee_pose() {
            let p = pose.translation();
            min_distance = min_distance.min((p - via).magnitude());
        }
    }
    assert!(
        min_distance < 0.05,
        "circular arc should pass near the via point; min distance was {min_distance}"
    );
}

#[test]
fn movec_collinear_points_are_rejected() {
    // via lies exactly on the segment start(2,0) → end(1.4,1.0): no unique arc.
    let source = r#"
        target VIA = position([1700mm, 500mm, 0mm])
        target END = position([1400mm, 1000mm, 0mm])
        fn main() {
            movec(VIA, END)
        }
    "#;
    let ctx = planar_ctx();
    let result = PlanningService::plan_thls_source_with_context(source, "prog_mc", 1, &ctx);

    match result {
        PlanResult::Diagnostics(diags) => {
            assert!(
                diags.iter().any(|d| d.message.contains("collinear")),
                "expected a collinear diagnostic, got: {diags:?}"
            );
        }
        PlanResult::Planned(_) => panic!("collinear movec must NOT plan"),
    }
}

#[test]
fn movec_joint_target_is_rejected() {
    let source = r#"
        target VIA = position([1700mm, 700mm, 0mm])
        target END = joints(0deg, 0deg)
        fn main() {
            movec(VIA, END)
        }
    "#;
    let ctx = planar_ctx();
    let result = PlanningService::plan_thls_source_with_context(source, "prog_mc", 1, &ctx);

    match result {
        PlanResult::Diagnostics(diags) => {
            assert!(
                diags.iter().any(|d| d.code.as_deref() == Some("THL_MOVEC_INVALID")),
                "expected THL_MOVEC_INVALID, got: {diags:?}"
            );
        }
        PlanResult::Planned(_) => panic!("movec with a joint target must NOT plan"),
    }
}

#[test]
fn movec_joint_via_is_rejected() {
    let source = r#"
        target VIA = joints(0deg, 0deg)
        target END = position([1400mm, 1000mm, 0mm])
        fn main() {
            movec(VIA, END)
        }
    "#;
    let ctx = planar_ctx();
    let result = PlanningService::plan_thls_source_with_context(source, "prog_mc", 1, &ctx);

    match result {
        PlanResult::Diagnostics(diags) => {
            assert!(
                diags.iter().any(|d| d.code.as_deref() == Some("THL_MOVEC_INVALID")),
                "expected THL_MOVEC_INVALID, got: {diags:?}"
            );
        }
        PlanResult::Planned(_) => panic!("movec with a joint via must NOT plan"),
    }
}
