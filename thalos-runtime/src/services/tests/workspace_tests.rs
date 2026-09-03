use std::sync::Arc;

use tokio::sync::RwLock;

use thalos_engine::core::analysis::workspace::{Workspace, WorkspaceConfig, WorkspaceError};
use thalos_engine::core::models::RobotModel;
use thalos_engine::math::Vector3;

use crate::backends::{
    controller::simulation::SimulationController, manager::BackendManager,
};
use crate::error::RuntimeError;
use crate::services::workspace::WorkspaceService;
use crate::{RobotController, SceneService};

// ─── 4.2: sample returns Arc<Workspace> ────────────────────────────────

#[test]
fn sample_returns_arc_workspace() {
    let ws = WorkspaceService::sample(RobotModel::Scara, WorkspaceConfig::default()).unwrap();

    // Must be Arc<Workspace>
    let _: Arc<Workspace> = ws;
}

#[test]
fn sample_produces_valid_metrics() {
    let ws = WorkspaceService::sample(
        RobotModel::Scara,
        WorkspaceConfig {
            samples: 100,
            seed: 0,
            tolerance: 1e-3,
        },
    )
    .unwrap();

    assert_eq!(ws.metrics().sample_count, 100);
    assert!(ws.metrics().max_reach > 0.0);
    // Scara has 3D workspace (z via prismatic joint) → AABB has volume
    assert!(ws.metrics().bounding_volume > 0.0);
}

// ─── 4.3: query delegates to is_reachable ───────────────────────────────

#[test]
fn query_returns_reachable_for_center() {
    let ws = WorkspaceService::sample(
        RobotModel::Scara,
        WorkspaceConfig {
            samples: 500,
            seed: 0,
            tolerance: 1e-3,
        },
    )
    .unwrap();

    // Point well within canonical SCARA workspace (r_min ≈ 0.50)
    let result = WorkspaceService::query(&ws, &Vector3::new(0.7, 0.5, 0.25), 0.5).unwrap();
    assert!(matches!(
        result,
        thalos_engine::core::analysis::workspace::Reachability::Reachable
    ));
}

#[test]
fn query_returns_out_of_workspace_for_distant_point() {
    let ws = WorkspaceService::sample(
        RobotModel::Scara,
        WorkspaceConfig {
            samples: 500,
            seed: 0,
            tolerance: 1e-3,
        },
    )
    .unwrap();

    let result = WorkspaceService::query(&ws, &Vector3::new(100.0, 0.0, 0.0), 0.1).unwrap();
    assert!(matches!(
        result,
        thalos_engine::core::analysis::workspace::Reachability::OutOfWorkspace { .. }
    ));
}

#[test]
fn query_validates_nan_point() {
    let ws = WorkspaceService::sample(
        RobotModel::Scara,
        WorkspaceConfig {
            samples: 10,
            seed: 0,
            tolerance: 1e-3,
        },
    )
    .unwrap();

    let result = WorkspaceService::query(&ws, &Vector3::new(f64::NAN, 0.0, 0.0), 0.1);
    assert!(result.is_err());
}

// ─── 4.5: determinism at service level ──────────────────────────────────

#[test]
fn same_seed_produces_identical_workspaces() {
    let config = WorkspaceConfig {
        samples: 200,
        seed: 42,
        tolerance: 1e-3,
    };

    let ws_a = WorkspaceService::sample(RobotModel::Scara, config).unwrap();
    let ws_b = WorkspaceService::sample(RobotModel::Scara, config).unwrap();

    assert_eq!(ws_a.metrics(), ws_b.metrics());
    assert_eq!(ws_a.bounds(), ws_b.bounds());
    for (a, b) in ws_a.samples().iter().zip(ws_b.samples().iter()) {
        assert_eq!(a.q, b.q);
    }
}

// ─── 4.4: sample does NOT mutate SceneService state ─────────────────────

#[tokio::test]
async fn sample_does_not_mutate_scene_service_state() {
    let controller = Arc::new(RwLock::new(SimulationController::new(
        RobotModel::Scara.metadata().dof,
    ))) as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let scene = SceneService::new(manager, RobotModel::Scara);
    let snap_before = scene.snapshot().await.unwrap();

    let _ws = WorkspaceService::sample(
        RobotModel::Scara,
        WorkspaceConfig {
            samples: 100,
            seed: 0,
            tolerance: 1e-3,
        },
    )
    .unwrap();

    let snap_after = scene.snapshot().await.unwrap();

    // Joints must be identical
    assert_eq!(snap_before.joints, snap_after.joints);
    // Robot model must be identical
    assert_eq!(snap_before.robot, snap_after.robot);
}

// ─── error propagation ─────────────────────────────────────────────────

#[test]
fn sample_rejects_zero_samples() {
    let config = WorkspaceConfig {
        samples: 0,
        seed: 0,
        tolerance: 1e-3,
    };
    let result = WorkspaceService::sample(RobotModel::Scara, config);
    assert!(result.is_err());
    // Must be RuntimeError wrapping WorkspaceError
    match result.unwrap_err() {
        RuntimeError::Workspace(WorkspaceError::InvalidSampleCount(0)) => {} // ok
        other => panic!(
            "expected RuntimeError::Workspace(InvalidSampleCount), got {:?}",
            other
        ),
    }
}

// NOTE: NaN tolerance is validated by Workspace::is_reachable at query time,
// not at sampling time. No test needed at service level for this.

// ─── Workspace with TCP ─────────────────────────────────────────────────

#[test]
fn sample_with_tcp_uses_tcp_position() {
    use thalos_engine::core::models::RobotRegistry;
    use thalos_engine::core::robot::tool_frame::ToolFrame;
    use thalos_engine::math::Transform3D;

    let chain = RobotRegistry::create_default(RobotModel::Scara);
    let config = WorkspaceConfig {
        samples: 50,
        seed: 0,
        tolerance: 1e-3,
    };

    // Sample without TCP (flange)
    let ws_flange = WorkspaceService::sample_from_chain(&chain, config).unwrap();

    // Sample with TCP (12cm offset below flange)
    let tcp = ToolFrame::with_offset(
        *chain.end_effector(),
        Transform3D::from_translation(Vector3::new(0.0, 0.0, -0.12)),
    );
    let ws_tcp = WorkspaceService::sample_from_chain_with_tcp(&chain, config, Some(&tcp)).unwrap();

    // Both should have the same number of samples
    assert_eq!(ws_flange.samples().len(), ws_tcp.samples().len());

    // TCP workspace should be systematically lower in Z
    // (all samples should have Z reduced by 0.12)
    for (flange_sample, tcp_sample) in ws_flange.samples().iter().zip(ws_tcp.samples().iter()) {
        let z_diff = flange_sample.position.z - tcp_sample.position.z;
        assert!(
            (z_diff - 0.12).abs() < 1e-6,
            "Z difference should be 0.12, got {}",
            z_diff
        );
        // X and Y should be identical (offset is only in Z)
        assert!((flange_sample.position.x - tcp_sample.position.x).abs() < 1e-6);
        assert!((flange_sample.position.y - tcp_sample.position.y).abs() < 1e-6);
    }
}
