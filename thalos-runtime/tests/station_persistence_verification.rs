use std::sync::Arc;

use thalos_persistence::{SqliteRobotRepository, SqliteStationRepository, SqliteWorkspaceRepository, SqliteEquipmentModuleRepository};
use thalos_runtime::ports::{RobotRepository, StationRepository, EquipmentModuleRepository};
use thalos_runtime::station::{Station, StationService, StationError};
use thalos_runtime::RobotService;

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

struct Ctx {
    robot_service: Arc<RobotService>,
    station_service: StationService,
    module_repo: Arc<SqliteEquipmentModuleRepository>,
    station_repo: Arc<SqliteStationRepository>,
    _dir: tempfile::TempDir,
}

async fn setup() -> Ctx {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("robots")).unwrap();
    let db = dir.path().join("workspace.db");
    let robot_repo = Arc::new(SqliteRobotRepository::new(db.to_str().unwrap()).await.unwrap());
    let ws_repo = Arc::new(SqliteWorkspaceRepository::new(db.to_str().unwrap()).await.unwrap());
    let station_repo = Arc::new(SqliteStationRepository::from_pool(ws_repo.pool().clone()).await.unwrap());
    let module_repo = Arc::new(SqliteEquipmentModuleRepository::new(ws_repo.pool().clone()));
    let robot_service = Arc::new(RobotService::new(Some(robot_repo.clone())));
    let station_service = StationService::new(station_repo.clone(), module_repo.clone(), robot_repo as Arc<dyn RobotRepository>);
    Ctx { robot_service, station_service, module_repo, station_repo, _dir: dir }
}

// ═══════════════════════════════════════════════════════════════════════════
// 1. Station full lifecycle
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn station_full_lifecycle() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let robot = ctx.robot_service.import_urdf(URDF_SIMPLE).await.unwrap();

    let station = ctx.station_service.create_station("cell_1", "Cell 1").await.unwrap();
    ctx.station_service.add_robotics_module(&station.id, &robot.id, "Arm", "{}").await.unwrap();
    ctx.station_service.add_interconnection_module(&station.id, "Vision").await.unwrap();

    let modules = ctx.station_service.get_station_modules(&station.id).await.unwrap();
    assert_eq!(modules.len(), 2);
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. Cascade delete station
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn cascade_delete_station() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let robot = ctx.robot_service.import_urdf(URDF_SIMPLE).await.unwrap();

    let station = ctx.station_service.create_station("cell_1", "Cell 1").await.unwrap();
    ctx.station_service.add_robotics_module(&station.id, &robot.id, "Arm", "{}").await.unwrap();
    ctx.station_service.add_interconnection_module(&station.id, "Vision").await.unwrap();

    // Delete station
    ctx.station_service.delete_station(&station.id).await.unwrap();

    // Station gone
    assert!(ctx.station_service.get_station(&station.id).await.unwrap().is_none());

    // Modules gone (via FK CASCADE)
    assert!(ctx.module_repo.list_by_station(&station.id.0).await.unwrap().is_empty());

    // Robot still exists
    let repo = ctx.robot_service.repo().unwrap();
    assert!(repo.get(&robot.id).await.unwrap().is_some());
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Cascade delete module
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn cascade_delete_module() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let robot = ctx.robot_service.import_urdf(URDF_SIMPLE).await.unwrap();

    let station = ctx.station_service.create_station("cell_1", "Cell 1").await.unwrap();
    let module = ctx.station_service.add_robotics_module(&station.id, &robot.id, "Arm", "{}").await.unwrap();
    ctx.station_service.add_interconnection_module(&station.id, "Vision").await.unwrap();

    // Delete robotics module
    ctx.station_service.remove_module(&module.id).await.unwrap();

    // Module gone
    let modules = ctx.station_service.get_station_modules(&station.id).await.unwrap();
    assert_eq!(modules.len(), 1);
    assert_eq!(modules[0].name, "Vision");

    // Extension gone
    assert!(ctx.module_repo.get_robotics_extension(&module.id.0).await.unwrap().is_none());

    // Robot still exists
    let repo = ctx.robot_service.repo().unwrap();
    assert!(repo.get(&robot.id).await.unwrap().is_some());
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. Delete robot with reference fails
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn delete_robot_with_reference_fails() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let robot = ctx.robot_service.import_urdf(URDF_SIMPLE).await.unwrap();

    let station = ctx.station_service.create_station("cell_1", "Cell 1").await.unwrap();
    ctx.station_service.add_robotics_module(&station.id, &robot.id, "Arm", "{}").await.unwrap();

    // Delete should fail
    let result = ctx.robot_service.delete_robot(&robot.id, ctx.module_repo.as_ref()).await;
    assert!(result.is_err());
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. Delete robot without reference succeeds
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn delete_robot_without_reference_succeeds() {
    let ctx = setup().await;

    #[allow(deprecated)]
    let robot = ctx.robot_service.import_urdf(URDF_SIMPLE).await.unwrap();

    // No station references this robot
    let result = ctx.robot_service.delete_robot(&robot.id, ctx.module_repo.as_ref()).await;
    assert!(result.is_ok());
    assert!(ctx.robot_service.repo().unwrap().get(&robot.id).await.unwrap().is_none());
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. Materialized robot lifecycle
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn materialized_robot_lifecycle() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("robots")).unwrap();
    let db = dir.path().join("workspace.db");

    let robot_repo = Arc::new(SqliteRobotRepository::new(db.to_str().unwrap()).await.unwrap());
    let ws_repo = Arc::new(SqliteWorkspaceRepository::new(db.to_str().unwrap()).await.unwrap());
    let station_repo = Arc::new(SqliteStationRepository::from_pool(ws_repo.pool().clone()).await.unwrap());
    let module_repo = Arc::new(SqliteEquipmentModuleRepository::new(ws_repo.pool().clone()));
    let robot_service = Arc::new(RobotService::new(Some(robot_repo.clone())));
    let station_service = StationService::new(station_repo, module_repo.clone(), robot_repo.clone() as Arc<dyn RobotRepository>);

    // Create test package with meshes
    let pkg = tempfile::tempdir().unwrap();
    let visual = pkg.path().join("meshes").join("visual");
    let collision = pkg.path().join("meshes").join("collision");
    std::fs::create_dir_all(&visual).unwrap();
    std::fs::create_dir_all(&collision).unwrap();
    let stl = b"solid test\nfacet normal 0 0 0\nendfacet\nendsolid test\n";
    std::fs::write(visual.join("base_link.stl"), stl).unwrap();
    std::fs::write(collision.join("base_link.stl"), stl).unwrap();

    let urdf = r#"<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual><geometry><mesh filename="package://pkg/meshes/visual/base_link.stl"/></geometry></visual>
    <collision><geometry><mesh filename="package://pkg/meshes/collision/base_link.stl"/></geometry></collision>
  </link>
</robot>"#;

    // Import with materialization
    let record = robot_service.import_urdf_materialized(
        dir.path(), urdf, Some("test"), &[pkg.path().to_path_buf()],
    ).await.unwrap();

    // Create station + module
    let station = station_service.create_station("cell_1", "Cell 1").await.unwrap();
    station_service.add_robotics_module(&station.id, &record.id, "Arm", "{}").await.unwrap();

    // Verify on disk
    let robot_dir = dir.path().join("robots").join(&record.id);
    assert!(robot_dir.join("robot.urdf").exists());
    assert!(robot_dir.join("assets").join("visual").join("base_link.stl").exists());
    assert!(robot_dir.join("assets").join("collision").join("base_link.stl").exists());

    // Verify in SQLite
    assert!(module_repo.list_by_station(&station.id.0).await.unwrap().len() == 1);
    assert!(module_repo.find_robot_references(&record.id).await.unwrap().len() == 1);

    // Simulate restart — reinitialize
    drop(station_service);
    drop(robot_service);

    let robot_repo2 = Arc::new(SqliteRobotRepository::new(db.to_str().unwrap()).await.unwrap());
    let ws_repo2 = Arc::new(SqliteWorkspaceRepository::new(db.to_str().unwrap()).await.unwrap());
    let station_repo2 = Arc::new(SqliteStationRepository::from_pool(ws_repo2.pool().clone()).await.unwrap());
    let module_repo2 = Arc::new(SqliteEquipmentModuleRepository::new(ws_repo2.pool().clone()));
    let robot_service2 = Arc::new(RobotService::new(Some(robot_repo2.clone())));
    let station_service2 = StationService::new(station_repo2, module_repo2.clone(), robot_repo2 as Arc<dyn RobotRepository>);

    // Verify everything survived
    let stations = station_service2.list_stations().await.unwrap();
    assert_eq!(stations.len(), 1);

    let modules = station_service2.get_station_modules(&stations[0].id).await.unwrap();
    assert_eq!(modules.len(), 1);

    let references = module_repo2.find_robot_references(&record.id).await.unwrap();
    assert_eq!(references.len(), 1);

    // Robot is still materialized
    assert!(robot_dir.join("robot.urdf").exists());
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. Channel metadata survives reopen
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn channel_metadata_survives_reopen() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("robots")).unwrap();
    let db = dir.path().join("workspace.db");

    let robot_repo = Arc::new(SqliteRobotRepository::new(db.to_str().unwrap()).await.unwrap());
    let ws_repo = Arc::new(SqliteWorkspaceRepository::new(db.to_str().unwrap()).await.unwrap());
    let station_repo = Arc::new(SqliteStationRepository::from_pool(ws_repo.pool().clone()).await.unwrap());
    let module_repo = Arc::new(SqliteEquipmentModuleRepository::new(ws_repo.pool().clone()));
    let robot_service = Arc::new(RobotService::new(Some(robot_repo.clone())));
    let station_service = StationService::new(station_repo, module_repo.clone(), robot_repo as Arc<dyn RobotRepository>);

    // Create station with acquisition module + channels
    let station = station_service.create_station("monitoring", "Monitoring").await.unwrap();
    let acq_module = station_service.add_interconnection_module(&station.id, "Sensors").await.unwrap();

    let ch1 = station_service.add_channel(&acq_module.id, "temperature", "Temperature", "f64", "°C").await.unwrap();
    let ch2 = station_service.add_channel(&acq_module.id, "pressure", "Pressure", "f64", "Pa").await.unwrap();

    // Verify channels persisted
    let channels = station_service.get_module_channels(&acq_module.id).await.unwrap();
    assert_eq!(channels.len(), 2);

    let ch1_found = channels.iter().find(|c| c.symbol == "temperature").unwrap();
    assert_eq!(ch1_found.name, "Temperature");
    assert_eq!(ch1_found.data_type, "f64");
    assert_eq!(ch1_found.unit, "°C");
    assert_eq!(ch1_found.id, ch1.id);

    let ch2_found = channels.iter().find(|c| c.symbol == "pressure").unwrap();
    assert_eq!(ch2_found.name, "Pressure");
    assert_eq!(ch2_found.unit, "Pa");

    // Simulate restart
    drop(station_service);
    drop(robot_service);

    let robot_repo2 = Arc::new(SqliteRobotRepository::new(db.to_str().unwrap()).await.unwrap());
    let ws_repo2 = Arc::new(SqliteWorkspaceRepository::new(db.to_str().unwrap()).await.unwrap());
    let station_repo2 = Arc::new(SqliteStationRepository::from_pool(ws_repo2.pool().clone()).await.unwrap());
    let module_repo2 = Arc::new(SqliteEquipmentModuleRepository::new(ws_repo2.pool().clone()));
    let robot_service2 = Arc::new(RobotService::new(Some(robot_repo2.clone())));
    let station_service2 = StationService::new(station_repo2, module_repo2.clone(), robot_repo2 as Arc<dyn RobotRepository>);

    // Verify channels survived with all metadata
    let channels = station_service2.get_module_channels(&acq_module.id).await.unwrap();
    assert_eq!(channels.len(), 2);

    let ch1_found = channels.iter().find(|c| c.symbol == "temperature").unwrap();
    assert_eq!(ch1_found.name, "Temperature");
    assert_eq!(ch1_found.data_type, "f64");
    assert_eq!(ch1_found.unit, "°C");
    assert_eq!(ch1_found.id, ch1.id);

    let ch2_found = channels.iter().find(|c| c.symbol == "pressure").unwrap();
    assert_eq!(ch2_found.name, "Pressure");
    assert_eq!(ch2_found.unit, "Pa");
    assert_eq!(ch2_found.id, ch2.id);
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Schema verification (indirect — via repository API)
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn schema_verification() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("robots")).unwrap();
    let db = dir.path().join("workspace.db");

    let robot_repo = Arc::new(SqliteRobotRepository::new(db.to_str().unwrap()).await.unwrap());
    let ws_repo = Arc::new(SqliteWorkspaceRepository::new(db.to_str().unwrap()).await.unwrap());
    let station_repo = Arc::new(SqliteStationRepository::from_pool(ws_repo.pool().clone()).await.unwrap());
    let module_repo = Arc::new(SqliteEquipmentModuleRepository::new(ws_repo.pool().clone()));
    let robot_service = Arc::new(RobotService::new(Some(robot_repo.clone())));
    let station_service = StationService::new(station_repo.clone(), module_repo.clone(), robot_repo as Arc<dyn RobotRepository>);

    // Create a station with all module types
    #[allow(deprecated)]
    let robot = robot_service.import_urdf(URDF_SIMPLE).await.unwrap();

    let station = station_service.create_station("test_station", "Test Station").await.unwrap();
    station_service.add_robotics_module(&station.id, &robot.id, "Arm", "{}").await.unwrap();
    let acq = station_service.add_interconnection_module(&station.id, "Sensors").await.unwrap();
    station_service.add_channel(&acq.id, "temp", "Temperature", "f64", "°C").await.unwrap();

    // Station loads from relational tables
    let loaded = station_repo.get(&station.id.0).await.unwrap();
    assert!(loaded.is_some());
    assert_eq!(loaded.unwrap().name, "Test Station");

    // Modules in equipment_modules
    let modules = module_repo.list_by_station(&station.id.0).await.unwrap();
    assert_eq!(modules.len(), 2);

    // Extensions accessible
    assert!(module_repo.get_robotics_extension(&modules.iter().find(|m| m.kind == "robotics").unwrap().id).await.unwrap().is_some());
    assert!(module_repo.get_interconnection_extension(&modules.iter().find(|m| m.kind == "interconnection").unwrap().id).await.unwrap().is_some());

    // Channels with real metadata
    let channels = module_repo.list_channels(&modules.iter().find(|m| m.kind == "interconnection").unwrap().id).await.unwrap();
    assert_eq!(channels.len(), 1);
    assert_eq!(channels[0].symbol, "temp");
    assert_eq!(channels[0].name, "Temperature");
    assert_eq!(channels[0].unit, "°C");

    // Robot reference via JOIN
    assert_eq!(module_repo.find_robot_references(&robot.id).await.unwrap().len(), 1);

    // Empty station
    let empty = station_service.create_station("empty", "Empty").await.unwrap();
    assert!(module_repo.list_by_station(&empty.id.0).await.unwrap().is_empty());
}
