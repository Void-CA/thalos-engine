//! E01–E05 — Plan Provenance, Preview Non-Activation, Explicit Activation, Execution, and Stale Mutation Lifecycle.
//!
//! E01: Plan is built with explicit source revision & fingerprint provenance.
//! E02: Preview loads a compiled plan for visualization without making it active (active_plan == None).
//! E03: Explicit activation transitions preview plan to active_plan.
//! E04: Execution requires an active plan; succeeds when valid.
//! E05: Source mutation increments program revision/fingerprint, causing stale plan rejection.

use std::sync::Arc;
use sha2::{Digest, Sha256};
use tokio::sync::RwLock;

use thalos_engine::core::{
    models::RobotModel,
    prelude::Trajectory,
    trajectory::TrajectoryPoint,
};
use thalos_engine::planning::motion::program::CompiledPlan;

use thalos_runtime::{
    backends::{
        controller::simulation::SimulationController, manager::BackendManager,
    },
    error::RuntimeError,
    scene::service::SceneService,
    RobotController,
};

fn compute_fingerprint(source: &str) -> String {
    let mut hasher = Sha256::new();
    hasher.update(source.as_bytes());
    format!("{:x}", hasher.finalize())
}

fn sample_compiled_plan() -> CompiledPlan {
    let points = vec![
        TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
        TrajectoryPoint::new(vec![0.5, 0.5], 0.5),
        TrajectoryPoint::new(vec![1.0, 1.0], 1.0),
    ];
    CompiledPlan::new(Trajectory::new(points), vec![])
}

async fn make_test_service(model: RobotModel) -> SceneService {
    let controller = Arc::new(RwLock::new(SimulationController::new(model.metadata().dof)))
        as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    SceneService::new(manager, model)
}

#[tokio::test]
async fn test_e01_to_e05_plan_provenance_and_activation_lifecycle() {
    let service = make_test_service(RobotModel::Planar2R).await;

    let source_v12 = "movej(joints(0deg, 0deg)) ; movej(joints(45deg, 45deg))";
    let source_v13 = "movej(joints(0deg, 0deg)) ; movej(joints(90deg, 90deg))";

    let fp_v12 = compute_fingerprint(source_v12);
    let fp_v13 = compute_fingerprint(source_v13);

    // E01 — Provenance initialization
    service.set_program_provenance("program-demo", 12, &fp_v12).await;

    let compiled = sample_compiled_plan();

    // E02 — Preview loads trajectory for rendering, but leaves active_plan == None
    let snapshot_preview = service.preview_plan(compiled.clone()).await.unwrap();
    assert!(
        snapshot_preview.active_plan.is_none(),
        "E02: Preview MUST leave active_plan as None"
    );

    // Attempting to execute in preview mode must be rejected with NoActivePlan
    let err_preview_exec = service.start_execution().await.unwrap_err();
    assert!(
        matches!(err_preview_exec, RuntimeError::NoActivePlan),
        "E02: Executing in preview mode must fail with NoActivePlan, got {:?}",
        err_preview_exec
    );

    // E03 — Explicit activation sets active_plan
    let snapshot_activated = service.activate_plan().await.unwrap();
    assert!(
        snapshot_activated.active_plan.is_some(),
        "E03: Explicit activation must populate active_plan"
    );

    // E04 — Execution of an active and valid plan succeeds
    let snapshot_exec = service.start_execution().await.unwrap();
    assert!(
        snapshot_exec.execution.is_some(),
        "E04: Execution of active plan must produce an ExecutionSession"
    );

    // E05 — Source mutation (revision 12 -> 13) causes execution to be rejected as stale
    service.set_program_provenance("program-demo", 13, &fp_v13).await;

    let err_stale = service.start_execution().await.unwrap_err();
    assert!(
        err_stale.is_stale(),
        "E05: Mutated source MUST cause err.is_stale() to evaluate to true"
    );

    match err_stale {
        RuntimeError::StalePlanRevision { expected, actual } => {
            assert_eq!(expected, 13);
            assert_eq!(actual, 12);
        }
        other => panic!("E05: Expected StalePlanRevision error, got {:?}", other),
    }
}
