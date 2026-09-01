use std::sync::Arc;
use tokio::sync::RwLock;

use thalos_engine::core::{
    models::{factory::RobotRegistry, RobotModel},
    robot::tool_frame::ToolFrame,
};
use thalos_engine::math::Vector3;
use thalos_runtime::{
    backends::{controller::simulation::SimulationController, manager::BackendManager},
    planning::service::{PlanResult, PlanningService, RobotPlanningContext},
    scene::service::SceneService,
    RobotController,
};

async fn make_test_service(model: RobotModel) -> SceneService {
    let controller = Arc::new(RwLock::new(SimulationController::new(model.metadata().dof)))
        as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    SceneService::new(manager, model)
}

#[tokio::test]
async fn test_f2_01_single_source_compatible_robots() {
    let source = r#"
        target p1 = joints(45deg, 30deg)
        fn main() {
            movej(p1)
        }
    "#;

    let chain2 = RobotRegistry::create_default(RobotModel::Planar2R);
    let ctx_planar = RobotPlanningContext {
        robot_id: "robot_planar_2r".into(),
        chain: chain2.clone(),
        initial_positions: vec![0.0; 2],
        tcp: None,
    };

    let result_planar = PlanningService::plan_thls_source_with_context(source, "prog_01", 1, &ctx_planar);

    match result_planar {
        PlanResult::Planned(plan) => {
            assert_eq!(plan.program_id.as_deref(), Some("prog_01"));
            assert_eq!(plan.program_revision, Some(1));
            assert_eq!(plan.robot_id.as_deref(), Some("robot_planar_2r"));
            assert!(plan.source_fingerprint.is_some());
            assert!(!plan.waypoints.is_empty());
        }
        PlanResult::Diagnostics(diags) => {
            panic!("Expected Planned for Planar2R, got diagnostics: {:?}", diags);
        }
    }
}

#[tokio::test]
async fn test_f2_02_dof_mismatch_diagnostic() {
    let source = r#"
        target p1 = joints(0deg, 0deg, 0deg, 0deg, 0deg, 0deg)
        fn main() {
            movej(p1)
        }
    "#;

    let chain2 = RobotRegistry::create_default(RobotModel::Planar2R);
    let ctx2 = RobotPlanningContext {
        robot_id: "planar_2dof".into(),
        chain: chain2,
        initial_positions: vec![0.0; 2],
        tcp: None,
    };

    let result = PlanningService::plan_thls_source_with_context(source, "prog_02", 1, &ctx2);

    match result {
        PlanResult::Diagnostics(diags) => {
            assert!(!diags.is_empty());
            let dof_diag = diags.iter().find(|d| d.code.as_deref() == Some("THL_DOF_MISMATCH"));
            assert!(dof_diag.is_some(), "Expected THL_DOF_MISMATCH diagnostic, got: {:?}", diags);
        }
        PlanResult::Planned(_) => panic!("Expected THL_DOF_MISMATCH error, got valid plan"),
    }
}

#[tokio::test]
async fn test_f2_03_unreachable_target_kinematic_diagnostic() {
    let source = r#"
        target far_away = position([100000mm, 100000mm, 100000mm])
        fn main() {
            movel(far_away)
        }
    "#;

    let chain2 = RobotRegistry::create_default(RobotModel::Planar2R);
    let ctx = RobotPlanningContext {
        robot_id: "planar_2dof".into(),
        chain: chain2,
        initial_positions: vec![0.0; 2],
        tcp: None,
    };

    let result = PlanningService::plan_thls_source_with_context(source, "prog_03", 1, &ctx);

    match result {
        PlanResult::Diagnostics(diags) => {
            assert!(!diags.is_empty());
            let unreachable = diags.iter().find(|d| d.code.as_deref() == Some("THL_UNREACHABLE_TARGET"));
            assert!(unreachable.is_some(), "Expected THL_UNREACHABLE_TARGET diagnostic, got: {:?}", diags);
        }
        PlanResult::Planned(_) => panic!("Expected THL_UNREACHABLE_TARGET error, got valid plan"),
    }
}

#[tokio::test]
async fn test_f2_04_source_immutability() {
    let raw_source = r#"
        const VEL = 50%
        target home = position([300mm, 100mm, 200mm])
        fn main() {
            movej(home)
        }
    "#;
    let source_copy = raw_source.to_string();

    let chain = RobotRegistry::create_default(RobotModel::Planar2R);
    let ctx = RobotPlanningContext {
        robot_id: "planar_2dof".into(),
        chain,
        initial_positions: vec![0.0; 2],
        tcp: None,
    };

    let _ = PlanningService::plan_thls_source_with_context(raw_source, "prog_04", 1, &ctx);

    assert_eq!(raw_source, source_copy, "Source code was modified during planning!");
}

#[tokio::test]
async fn test_f2_05_recontextualization_multi_robot() {
    let source = r#"
        target p1 = joints(45deg, 30deg)
        fn main() {
            movej(p1)
        }
    "#;

    let chain_r1 = RobotRegistry::create_default(RobotModel::Planar2R);
    let ctx_r1 = RobotPlanningContext {
        robot_id: "cell_robot_1".into(),
        chain: chain_r1.clone(),
        initial_positions: vec![0.0; 2],
        tcp: None,
    };

    let transform = thalos_engine::math::Transform3D::from_translation(Vector3::new(0.0, 0.0, 0.05));
    let tcp2 = ToolFrame::with_offset(thalos_engine::core::spatial::frame::FrameId::World, transform);
    let ctx_r2 = RobotPlanningContext {
        robot_id: "cell_robot_2".into(),
        chain: chain_r1,
        initial_positions: vec![0.0; 2],
        tcp: Some(tcp2),
    };

    let res1 = PlanningService::plan_thls_source_with_context(source, "shared_prog", 1, &ctx_r1);
    let res2 = PlanningService::plan_thls_source_with_context(source, "shared_prog", 1, &ctx_r2);

    match (res1, res2) {
        (PlanResult::Planned(plan1), PlanResult::Planned(plan2)) => {
            assert_eq!(plan1.robot_id.as_deref(), Some("cell_robot_1"));
            assert_eq!(plan2.robot_id.as_deref(), Some("cell_robot_2"));
            assert_eq!(plan1.source_fingerprint, plan2.source_fingerprint);
            assert_eq!(plan1.program_id, plan2.program_id);
        }
        (r1, r2) => panic!("Recontextualization failed: r1={:?}, r2={:?}", r1, r2),
    }
}

#[tokio::test]
async fn test_f2_06_no_partial_stale_plan_on_diagnostics() {
    let scene = Arc::new(make_test_service(RobotModel::Planar2R).await);
    let planner = PlanningService::new(scene.clone());

    let invalid_source = r#"
        target far = position([999999mm, 999999mm, 999999mm])
        fn main() {
            movel(far)
        }
    "#;

    let (res, snapshot) = planner
        .preview_thls_source(invalid_source, "prog_invalid", 1)
        .await
        .expect("Preview call should succeed with diagnostic result");

    match res {
        PlanResult::Diagnostics(diags) => {
            assert!(!diags.is_empty());
            assert!(snapshot.active_plan.is_none(), "Active plan must remain None when planning fails");
        }
        PlanResult::Planned(_) => panic!("Expected planning failure"),
    }
}

#[tokio::test]
async fn test_f2_07_diagnostic_span_traceability() {
    let source = r#"
        target unreachable = position([100000mm, 0mm, 0mm])
        fn main() {
            movel(unreachable)
        }
    "#;

    let chain = RobotRegistry::create_default(RobotModel::Planar2R);
    let ctx = RobotPlanningContext {
        robot_id: "planar_2dof".into(),
        chain,
        initial_positions: vec![0.0; 2],
        tcp: None,
    };

    let res = PlanningService::plan_thls_source_with_context(source, "prog_span", 1, &ctx);

    match res {
        PlanResult::Diagnostics(diags) => {
            let diag = diags.first().expect("Expected at least one diagnostic");
            assert!(diag.span.end > diag.span.start, "Span must be valid range");
        }
        PlanResult::Planned(_) => panic!("Expected diagnostic for unreachable target"),
    }
}
