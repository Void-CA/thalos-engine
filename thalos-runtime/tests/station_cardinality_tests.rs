use std::sync::Arc;

use thalos_engine::core::models::RobotModel;
use thalos_engine::prelude::StationId;
use thalos_persistence::{SqliteRobotRepository, SqliteStationRepository, SqliteWorkspaceRepository, SqliteEquipmentModuleRepository};
use thalos_runtime::backends::manager::BackendManager;
use thalos_runtime::ports::{RobotRepository, StationRepository, EquipmentModuleRepository};
use thalos_runtime::station::{
    EquipmentModuleId, Station, StationService, StationError,
};
use thalos_runtime::{RobotService, SceneService, WorkspaceService};

const URDF_SIMPLE: &str = r#"<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <link name="link_1"/>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
</robot>"#;

struct TestContext {
    robot_service: Arc<RobotService>,
    station_service: StationService,
    module_repo: Arc<SqliteEquipmentModuleRepository>,
    _dir: tempfile::TempDir,
}

async fn setup() -> TestContext {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("robots")).unwrap();
    let db_path = dir.path().join("workspace.db");

    let robot_repo = Arc::new(
        SqliteRobotRepository::new(db_path.to_str().unwrap()).await.unwrap(),
    );
    let workspace_repo = Arc::new(
        SqliteWorkspaceRepository::new(db_path.to_str().unwrap()).await.unwrap(),
    );

    let station_repo = Arc::new(
        SqliteStationRepository::from_pool(workspace_repo.pool().clone()).await.unwrap(),
    );
    let module_repo = Arc::new(
        SqliteEquipmentModuleRepository::new(workspace_repo.pool().clone()),
    );

    let robot_service = Arc::new(RobotService::new(Some(robot_repo.clone())));
    let station_service = StationService::new(
        station_repo.clone(),
        module_repo.clone(),
        robot_repo.clone() as Arc<dyn RobotRepository>,
    );

    TestContext { robot_service, station_service, module_repo, _dir: dir }
}

// ═══════════════════════════════════════════════════════════════════════════
// Cardinality tests
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn create_empty_station() {
    let ctx = setup().await;

    let station = ctx.station_service
        .create_station("empty_cell", "Empty Cell")
        .await
        .expect("create station");

    assert_eq!(station.id.0, "empty_cell");
    assert_eq!(station.name, "Empty Cell");

    // Verify modules are empty
    let modules = ctx.station_service.get_station_modules(&station.id).await.unwrap();
    assert_eq!(modules.len(), 0);
}

#[tokio::test]
async fn create_station_with_acquisition_only() {
    let ctx = setup().await;

    let station = ctx.station_service
        .create_station("monitoring", "Monitoring Station")
        .await
        .expect("create station");

    let module = ctx.station_service
        .add_interconnection_module(&station.id, "Temperature")
        .await
        .expect("add acquisition module");

    let modules = ctx.station_service.get_station_modules(&station.id).await.unwrap();
    assert_eq!(modules.len(), 1);
    assert_eq!(modules[0].name, "Temperature");
}

#[tokio::test]
async fn create_station_with_robotics_only() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let record = ctx.robot_service
        .import_urdf(URDF_SIMPLE)
        .await
        .expect("import robot");

    let station = ctx.station_service
        .create_station("assembly", "Assembly Station")
        .await
        .expect("create station");

    let module = ctx.station_service
        .add_robotics_module(&station.id, &record.id, "Primary Arm", "{}")
        .await
        .expect("add robotics module");

    let modules = ctx.station_service.get_station_modules(&station.id).await.unwrap();
    assert_eq!(modules.len(), 1);
    assert_eq!(modules[0].name, "Primary Arm");
}

#[tokio::test]
async fn create_station_with_both() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let record = ctx.robot_service
        .import_urdf(URDF_SIMPLE)
        .await
        .expect("import robot");

    let station = ctx.station_service
        .create_station("hybrid", "Hybrid Station")
        .await
        .expect("create station");

    ctx.station_service
        .add_robotics_module(&station.id, &record.id, "Arm", "{}")
        .await
        .expect("add robotics");

    ctx.station_service
        .add_interconnection_module(&station.id, "Vision")
        .await
        .expect("add acquisition");

    let modules = ctx.station_service.get_station_modules(&station.id).await.unwrap();
    assert_eq!(modules.len(), 2);
}

#[tokio::test]
async fn robot_can_exist_without_station() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let record = ctx.robot_service
        .import_urdf(URDF_SIMPLE)
        .await
        .expect("import robot");

    // Station exists but does NOT reference the robot
    ctx.station_service
        .create_station("unrelated", "Unrelated Station")
        .await
        .expect("create station");

    // Robot exists independently
    let repo = ctx.robot_service.repo().unwrap();
    let fetched = repo.get(&record.id).await.unwrap();
    assert!(fetched.is_some(), "robot must exist independently of station");
}

#[tokio::test]
async fn station_survives_reopen_with_zero_modules() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("robots")).unwrap();
    let db_path = dir.path().join("workspace.db");

    // Phase 1: Create station
    {
        let robot_repo = Arc::new(SqliteRobotRepository::new(db_path.to_str().unwrap()).await.unwrap());
        let workspace_repo = Arc::new(SqliteWorkspaceRepository::new(db_path.to_str().unwrap()).await.unwrap());
        let station_repo = Arc::new(SqliteStationRepository::from_pool(workspace_repo.pool().clone()).await.unwrap());
        let module_repo = Arc::new(SqliteEquipmentModuleRepository::new(workspace_repo.pool().clone()));

        let station_service = StationService::new(
            station_repo, module_repo, robot_repo as Arc<dyn RobotRepository>,
        );
        station_service.create_station("empty", "Empty Cell").await.unwrap();
    }

    // Phase 2: Reopen
    let robot_repo = Arc::new(SqliteRobotRepository::new(db_path.to_str().unwrap()).await.unwrap());
    let workspace_repo = Arc::new(SqliteWorkspaceRepository::new(db_path.to_str().unwrap()).await.unwrap());
    let station_repo = Arc::new(SqliteStationRepository::from_pool(workspace_repo.pool().clone()).await.unwrap());
    let module_repo = Arc::new(SqliteEquipmentModuleRepository::new(workspace_repo.pool().clone()));

    let station_service = StationService::new(
        station_repo, module_repo, robot_repo as Arc<dyn RobotRepository>,
    );

    let stations = station_service.list_stations().await.unwrap();
    assert_eq!(stations.len(), 1);
    assert_eq!(stations[0].id.0, "empty");

    let modules = station_service.get_station_modules(&stations[0].id).await.unwrap();
    assert_eq!(modules.len(), 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// Reference protection tests
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn delete_unreferenced_robot_succeeds() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let record = ctx.robot_service
        .import_urdf(URDF_SIMPLE)
        .await
        .expect("import robot");

    // Station exists but does NOT reference this robot
    ctx.station_service
        .create_station("unrelated", "Unrelated")
        .await
        .expect("create station");

    // Delete should succeed
    ctx.robot_service
        .delete_robot(&record.id, ctx._dir.path(), ctx.module_repo.as_ref())
        .await
        .expect("delete unreferenced robot must succeed");

    let repo = ctx.robot_service.repo().unwrap();
    let fetched = repo.get(&record.id).await.unwrap();
    assert!(fetched.is_none(), "robot should be deleted");
}

#[tokio::test]
async fn delete_referenced_robot_fails() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let record = ctx.robot_service
        .import_urdf(URDF_SIMPLE)
        .await
        .expect("import robot");

    // Station references this robot
    let station = ctx.station_service
        .create_station("cell_1", "Cell 1")
        .await
        .expect("create station");

    ctx.station_service
        .add_robotics_module(&station.id, &record.id, "Arm", "{}")
        .await
        .expect("add module");

    // Delete should fail
    let result = ctx.robot_service.delete_robot(&record.id, ctx._dir.path(), ctx.module_repo.as_ref()).await;
    assert!(result.is_err(), "delete referenced robot must fail");
    let err_msg = result.unwrap_err().to_string();
    assert!(err_msg.contains("referenced"), "error must mention reference: {err_msg}");

    // Verify robot still exists
    let repo = ctx.robot_service.repo().unwrap();
    let fetched = repo.get(&record.id).await.unwrap();
    assert!(fetched.is_some(), "robot must still exist after failed delete");
}

#[tokio::test]
async fn delete_robotics_module_does_not_delete_robot() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let record = ctx.robot_service
        .import_urdf(URDF_SIMPLE)
        .await
        .expect("import robot");

    let station = ctx.station_service
        .create_station("cell_1", "Cell 1")
        .await
        .expect("create station");

    let module = ctx.station_service
        .add_robotics_module(&station.id, &record.id, "Arm", "{}")
        .await
        .expect("add module");

    // Remove the module
    ctx.station_service.remove_module(&module.id).await.unwrap();

    // Verify module is gone
    let modules = ctx.station_service.get_station_modules(&station.id).await.unwrap();
    assert_eq!(modules.len(), 0);

    // Robot should still exist
    let repo = ctx.robot_service.repo().unwrap();
    let fetched = repo.get(&record.id).await.unwrap();
    assert!(fetched.is_some(), "robot must survive module deletion");
}

#[tokio::test]
async fn find_robot_references_works() {
    let ctx = setup().await;

    // Import a real robot (legacy import — same URDF produces same ID)
    #[allow(deprecated)]
    let record = ctx.robot_service.import_urdf(URDF_SIMPLE).await.expect("import robot");

    let station_a = ctx.station_service
        .create_station("cell_a", "Cell A")
        .await
        .expect("create station");

    ctx.station_service
        .add_robotics_module(&station_a.id, &record.id, "Arm A", "{}")
        .await
        .expect("add module");

    let station_b = ctx.station_service
        .create_station("cell_b", "Cell B")
        .await
        .expect("create station");

    // Station B references the SAME robot (different module)
    ctx.station_service
        .add_robotics_module(&station_b.id, &record.id, "Arm B", "{}")
        .await
        .expect("add module");

    // Find references — robot is referenced by 2 modules in 2 stations
    let references = ctx.module_repo.find_robot_references(&record.id).await.unwrap();
    assert_eq!(references.len(), 2, "robot referenced by 2 modules");

    // No reference to nonexistent robot
    let references = ctx.module_repo.find_robot_references("nonexistent").await.unwrap();
    assert!(references.is_empty());
}

#[tokio::test]
async fn foreign_keys_are_enabled() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let record = ctx.robot_service
        .import_urdf(URDF_SIMPLE)
        .await
        .expect("import robot");

    let station = ctx.station_service
        .create_station("cell_1", "Cell 1")
        .await
        .expect("create station");

    ctx.station_service
        .add_robotics_module(&station.id, &record.id, "Arm", "{}")
        .await
        .expect("add module");

    // Application-level validation blocks deletion
    let result = ctx.robot_service.delete_robot(&record.id, ctx._dir.path(), ctx.module_repo.as_ref()).await;
    assert!(result.is_err());
}
