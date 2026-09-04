use std::sync::Arc;

use thalos_engine::core::models::RobotModel;
use thalos_engine::prelude::StationId;
use thalos_persistence::{SqliteRobotRepository, SqliteStationRepository, SqliteWorkspaceRepository};
use thalos_runtime::backends::manager::BackendManager;
use thalos_runtime::ports::StationRepository;
use thalos_runtime::station::{
    AcquisitionModule, AcquisitionModuleId, RoboticsModule, RoboticsModuleId, Station, StationService,
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

async fn setup() -> (Arc<RobotService>, Arc<StationService>, tempfile::TempDir) {
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join("robots")).unwrap();
    let db_path = dir.path().join("workspace.db");

    let robot_repo = Arc::new(
        SqliteRobotRepository::new(db_path.to_str().unwrap()).await.unwrap(),
    );
    let workspace_repo = Arc::new(
        SqliteWorkspaceRepository::new(db_path.to_str().unwrap()).await.unwrap(),
    );

    let robot_service = Arc::new(RobotService::new(Some(robot_repo)));
    let station_repo = Arc::new(workspace_repo.station_repo());
    let station_service = Arc::new(StationService::with_repository(station_repo));

    (robot_service, station_service, dir)
}

// ═══════════════════════════════════════════════════════════════════════════
// Cardinality tests
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn create_empty_station() {
    let (_robot_service, station_service, _dir) = setup().await;

    let station = Station::new("empty_cell", "Empty Cell");
    station_service.register_station(station);

    let stations = station_service.list_stations();
    assert_eq!(stations.len(), 1);
    assert_eq!(stations[0].robotics_modules.len(), 0);
    assert_eq!(stations[0].acquisition_modules.len(), 0);
}

#[tokio::test]
async fn create_station_with_acquisition_only() {
    let (_robot_service, station_service, _dir) = setup().await;

    let mut station = Station::new("monitoring", "Monitoring Station");
    station.add_acquisition_module(AcquisitionModule {
        id: AcquisitionModuleId("temp_sensor".into()),
        station_id: StationId("monitoring".into()),
        name: "Temperature".into(),
        channels: [("temp_c".into(), 0.0)].into(),
    });
    station_service.register_station(station);

    let stations = station_service.list_stations();
    assert_eq!(stations.len(), 1);
    assert_eq!(stations[0].robotics_modules.len(), 0);
    assert_eq!(stations[0].acquisition_modules.len(), 1);

    let module = stations[0].acquisition_modules.get(&AcquisitionModuleId("temp_sensor".into())).unwrap();
    assert_eq!(module.name, "Temperature");
}

#[tokio::test]
async fn create_station_with_robotics_only() {
    let (robot_service, station_service, dir) = setup().await;

    // Import a robot first (legacy import — no materialization needed for cardinality tests)
    #[allow(deprecated)]
    let record = robot_service
        .import_urdf(URDF_SIMPLE)
        .await
        .expect("import robot");

    let mut station = Station::new("assembly", "Assembly Station");
    station.add_robotics_module(RoboticsModule {
        id: RoboticsModuleId("arm_01".into()),
        station_id: StationId("assembly".into()),
        name: "Primary Arm".into(),
        robot_name: "test_robot".into(),
        robot_definition_id: Some(record.id.clone()),
        controller_binding: "simulation".into(),
    });
    station_service.register_station(station);

    let stations = station_service.list_stations();
    assert_eq!(stations.len(), 1);
    assert_eq!(stations[0].robotics_modules.len(), 1);
    assert_eq!(stations[0].acquisition_modules.len(), 0);
}

#[tokio::test]
async fn create_station_with_both() {
    let (robot_service, station_service, dir) = setup().await;

    #[allow(deprecated)]
    let record = robot_service
        .import_urdf(URDF_SIMPLE)
        .await
        .expect("import robot");

    let mut station = Station::new("hybrid", "Hybrid Station");
    station.add_robotics_module(RoboticsModule {
        id: RoboticsModuleId("arm_01".into()),
        station_id: StationId("hybrid".into()),
        name: "Arm".into(),
        robot_name: "test_robot".into(),
        robot_definition_id: Some(record.id.clone()),
        controller_binding: "simulation".into(),
    });
    station.add_acquisition_module(AcquisitionModule {
        id: AcquisitionModuleId("sensor_01".into()),
        station_id: StationId("hybrid".into()),
        name: "Vision".into(),
        channels: HashMap::new(),
    });
    station_service.register_station(station);

    let stations = station_service.list_stations();
    assert_eq!(stations.len(), 1);
    assert_eq!(stations[0].robotics_modules.len(), 1);
    assert_eq!(stations[0].acquisition_modules.len(), 1);
}

#[tokio::test]
async fn robot_can_exist_without_station() {
    let (robot_service, station_service, dir) = setup().await;

    // Import a robot
    #[allow(deprecated)]
    let record = robot_service
        .import_urdf(URDF_SIMPLE)
        .await
        .expect("import robot");

    // Station exists but does NOT reference the robot
    let station = Station::new("unrelated", "Unrelated Station");
    station_service.register_station(station);

    // Robot exists independently
    let stations = station_service.list_stations();
    assert_eq!(stations.len(), 1);
    assert_eq!(stations[0].robotics_modules.len(), 0);

    // Robot record is in the repository
    let repo = robot_service.repo().unwrap();
    let fetched = repo.get(&record.id).await.unwrap();
    assert!(fetched.is_some(), "robot must exist independently of station");
}

#[tokio::test]
async fn station_survives_reopen_with_zero_modules() {
    let (_robot_service, station_service, dir) = setup().await;

    // Create empty station
    let station = Station::new("empty", "Empty Cell");
    station_service.register_station(station);

    // Simulate restart
    drop(station_service);

    let station_repo = Arc::new(
        SqliteWorkspaceRepository::new(dir.path().join("workspace.db").to_str().unwrap())
            .await
            .unwrap()
            .station_repo(),
    );
    let new_station_service = StationService::with_repository(station_repo);
    new_station_service.load_all().await.unwrap();

    let stations = new_station_service.list_stations();
    assert_eq!(stations.len(), 1);
    assert_eq!(stations[0].id.0, "empty");
    assert_eq!(stations[0].robotics_modules.len(), 0);
    assert_eq!(stations[0].acquisition_modules.len(), 0);
}

use std::collections::HashMap;
