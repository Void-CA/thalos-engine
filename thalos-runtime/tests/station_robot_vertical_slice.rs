use std::sync::Arc;

use thalos_engine::core::models::RobotModel;
use thalos_persistence::{SqliteRobotRepository, SqliteStationRepository, SqliteWorkspaceRepository};
use thalos_runtime::backends::manager::BackendManager;
use thalos_runtime::ports::{RobotRepository, StationRepository};
use thalos_runtime::robot::availability::{check_robot_availability, RobotAvailability};
use thalos_runtime::station::{AcquisitionModule, AcquisitionModuleId, RoboticsModule, RoboticsModuleId, Station, StationService};
use thalos_runtime::{RobotService, SceneService, WorkspaceService};
use thalos_engine::prelude::StationId;

const URDF_WITH_MESHES: &str = r#"<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry><mesh filename="package://test_pkg/meshes/visual/base_link.stl"/></geometry>
    </visual>
    <collision>
      <geometry><mesh filename="package://test_pkg/meshes/collision/base_link.stl"/></geometry>
    </collision>
  </link>
  <link name="link_1">
    <visual>
      <geometry><mesh filename="package://test_pkg/meshes/visual/link_1.stl"/></geometry>
    </visual>
    <collision>
      <geometry><mesh filename="package://test_pkg/meshes/collision/link_1.stl"/></geometry>
    </collision>
  </link>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
</robot>"#;

fn create_test_package(dir: &std::path::Path) {
    let visual_dir = dir.join("meshes").join("visual");
    let collision_dir = dir.join("meshes").join("collision");
    std::fs::create_dir_all(&visual_dir).unwrap();
    std::fs::create_dir_all(&collision_dir).unwrap();

    let stl = b"solid test\nfacet normal 0 0 0\nendfacet\nendsolid test\n";
    for name in &["base_link.stl", "link_1.stl"] {
        std::fs::write(visual_dir.join(name), stl).unwrap();
        std::fs::write(collision_dir.join(name), stl).unwrap();
    }
}

/// Setup: creates a workspace directory, initializes DB, and returns all services.
async fn setup_workspace(
    workspace_dir: &std::path::Path,
) -> (Arc<RobotService>, Arc<StationService>, Arc<WorkspaceService>, Arc<SceneService>) {
    std::fs::create_dir_all(workspace_dir.join("robots")).unwrap();

    let db_path = workspace_dir.join("workspace.db");
    let robot_repo = Arc::new(
        SqliteRobotRepository::new(db_path.to_str().unwrap())
            .await
            .expect("init robot repo"),
    );
    let workspace_repo = Arc::new(
        SqliteWorkspaceRepository::new(db_path.to_str().unwrap())
            .await
            .expect("init workspace repo"),
    );

    let robot_service = Arc::new(RobotService::new(Some(robot_repo.clone())));
    let manager = Arc::new(BackendManager::new());
    let scene_service = Arc::new(SceneService::new(manager, RobotModel::Planar2R));

    // Station service with persistence
    let station_repo = Arc::new(workspace_repo.station_repo());
    let station_service = Arc::new(StationService::with_repository(station_repo));

    let workspace_service = Arc::new(WorkspaceService::new(
        workspace_repo.clone(),
        robot_service.clone(),
        scene_service.clone(),
    ));

    (robot_service, station_service, workspace_service, scene_service)
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 1: Full lifecycle — Station → Module → Robot → Materialized Assets
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn station_module_robot_lifecycle() {
    let workspace_dir = tempfile::tempdir().unwrap();
    let (robot_service, station_service, workspace_service, scene_service) =
        setup_workspace(workspace_dir.path()).await;

    // 1. Import robot with materialization
    let package_dir = tempfile::tempdir().unwrap();
    create_test_package(package_dir.path());

    let record = robot_service
        .import_urdf_materialized(
            workspace_dir.path(),
            URDF_WITH_MESHES,
            Some("test_pkg"),
            &[package_dir.path().to_path_buf()],
        )
        .await
        .expect("import robot");

    let robot_id = record.id.clone();

    // 2. Create workspace (robots are independent resources)
    let workspace = workspace_service
        .create_workspace("Assembly Cell")
        .await
        .expect("create workspace");

    // 3. Create station with robotics module referencing the robot
    let mut station = Station::new("assembly_cell", "Assembly Cell");
    station.add_robotics_module(RoboticsModule {
        id: RoboticsModuleId("arm_01".into()),
        station_id: StationId("assembly_cell".into()),
        name: "Primary Arm".into(),
        robot_name: "test_robot".into(),
        robot_definition_id: Some(robot_id.clone()),
        controller_binding: "simulation".into(),
    });
    station.add_acquisition_module(AcquisitionModule {
        id: AcquisitionModuleId("vision_01".into()),
        station_id: StationId("assembly_cell".into()),
        name: "Vision Sensor".into(),
        channels: [("target_x".into(), 0.0), ("target_y".into(), 0.0)].into(),
    });
    station_service.register_station(station);

    // 4. Verify station exists in memory
    let stations = station_service.list_stations();
    assert_eq!(stations.len(), 1);
    let station = &stations[0];
    assert_eq!(station.id.0, "assembly_cell");

    let module = station.robotics_modules.get(&RoboticsModuleId("arm_01".into()))
        .expect("arm_01 module must exist");
    assert_eq!(module.robot_definition_id.as_deref(), Some(robot_id.as_str()));

    // 5. Verify robot is materialized
    let availability = check_robot_availability(&robot_id, workspace_dir.path(), robot_service.repo().unwrap())
        .await;
    assert_eq!(availability, RobotAvailability::Materialized);

    println!("✓ Station → Module → Robot lifecycle complete");
    println!("  station: assembly_cell");
    println!("  module: arm_01 → robot: {robot_id}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 2: Station + Robot survive reopen
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn station_module_robot_survives_reopen() {
    let workspace_dir = tempfile::tempdir().unwrap();
    let robot_id;

    // ─── Phase 1: Create workspace, import robot, create station ───
    {
        let (robot_service, station_service, _ws, _scene) =
            setup_workspace(workspace_dir.path()).await;

        let package_dir = tempfile::tempdir().unwrap();
        create_test_package(package_dir.path());

        let record = robot_service
            .import_urdf_materialized(
                workspace_dir.path(),
                URDF_WITH_MESHES,
                Some("test_pkg"),
                &[package_dir.path().to_path_buf()],
            )
            .await
            .expect("import robot");

        robot_id = record.id.clone();

        let mut station = Station::new("inspection_cell", "Inspection Cell");
        station.add_robotics_module(RoboticsModule {
            id: RoboticsModuleId("arm_01".into()),
            station_id: StationId("inspection_cell".into()),
            name: "Inspector Arm".into(),
            robot_name: "test_robot".into(),
            robot_definition_id: Some(robot_id.clone()),
            controller_binding: "simulation".into(),
        });
        station_service.register_station(station);

        // Verify before close
        assert_eq!(station_service.list_stations().len(), 1);
    }

    // ─── Phase 2: "Restart" — reinitialize everything ───
    let (robot_service2, station_service2, _ws2, scene2) =
        setup_workspace(workspace_dir.path()).await;

    // Load stations from DB
    station_service2.load_all().await.expect("load stations");

    // Verify station survived
    let stations = station_service2.list_stations();
    assert_eq!(stations.len(), 1, "station must survive reopen");
    assert_eq!(stations[0].id.0, "inspection_cell");

    let module = stations[0].robotics_modules.get(&RoboticsModuleId("arm_01".into()))
        .expect("arm_01 must exist");
    assert_eq!(
        module.robot_definition_id.as_deref(),
        Some(robot_id.as_str()),
        "module → robot reference must survive"
    );

    // Verify robot is still materialized
    let availability = check_robot_availability(&robot_id, workspace_dir.path(), robot_service2.repo().unwrap())
        .await;
    assert_eq!(availability, RobotAvailability::Materialized, "robot must survive reopen");

    // Verify robot loads correctly
    let snapshot = robot_service2
        .load_materialized_robot(&robot_id, workspace_dir.path(), &scene2)
        .await
        .expect("load robot after reopen");
    assert_eq!(snapshot.robot_name, "test_robot");
    assert_eq!(snapshot.joints.len(), 1, "must have 1 DOF");

    println!("✓ Station + Robot survive full reopen cycle");
    println!("  station: inspection_cell");
    println!("  module → robot: {robot_id}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 3: Broken reference detection
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn station_module_detects_broken_robot_reference() {
    let workspace_dir = tempfile::tempdir().unwrap();
    let (_robot_service, station_service, _ws, _scene) =
        setup_workspace(workspace_dir.path()).await;

    // Create station with a module referencing a robot that DOESN'T exist
    let mut station = Station::new("ghost_cell", "Ghost Cell");
    station.add_robotics_module(RoboticsModule {
        id: RoboticsModuleId("ghost_arm".into()),
        station_id: StationId("ghost_cell".into()),
        name: "Ghost Arm".into(),
        robot_name: "nonexistent_robot".into(),
        robot_definition_id: Some("robot-does-not-exist".into()),
        controller_binding: "simulation".into(),
    });
    station_service.register_station(station);

    // Give the async persist task time to complete
    tokio::time::sleep(std::time::Duration::from_millis(50)).await;

    // Reload from DB
    station_service.load_all().await.expect("load stations");

    let stations = station_service.list_stations();
    assert_eq!(stations.len(), 1);

    let module = stations[0].robotics_modules.get(&RoboticsModuleId("ghost_arm".into()))
        .expect("ghost_arm must exist");

    // The reference EXISTS in the station...
    let referenced_robot_id = module.robot_definition_id.as_ref()
        .expect("robot_definition_id must be set");

    // ...but the robot does NOT exist in the repository
    let robot_repo = SqliteRobotRepository::in_memory().await.unwrap();
    let robot_exists = robot_repo.get(referenced_robot_id).await.unwrap().is_some();
    assert!(!robot_exists, "referenced robot must not exist");

    // The availability check returns Legacy (no record found)
    let availability = check_robot_availability(
        referenced_robot_id,
        workspace_dir.path(),
        &robot_repo,
    ).await;
    assert!(
        matches!(availability, RobotAvailability::Legacy),
        "broken reference should be detectable via availability check"
    );

    println!("✓ Broken reference detection works");
    println!("  station → module → robot_id: {referenced_robot_id}");
    println!("  robot exists: {robot_exists}");
    println!("  availability: {:?}", availability);
}

// ═══════════════════════════════════════════════════════════════════════════
// Test 4: Multiple stations with different robots
// ═══════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn multiple_stations_with_different_robots() {
    let workspace_dir = tempfile::tempdir().unwrap();
    let (robot_service, station_service, _ws, _scene) =
        setup_workspace(workspace_dir.path()).await;

    // Import two different robots with DIFFERENT content to avoid hash collisions
    let package_a = tempfile::tempdir().unwrap();
    create_test_package(package_a.path());

    let package_b = tempfile::tempdir().unwrap();
    create_test_package(package_b.path());
    // Write different content to package_b to ensure different hashes
    std::fs::write(
        package_b.path().join("meshes").join("visual").join("base_link.stl"),
        b"solid robot_b\nfacet normal 0 0 1\nendfacet\nendsolid robot_b\n",
    ).unwrap();

    let record_a = robot_service
        .import_urdf_materialized(
            workspace_dir.path(),
            URDF_WITH_MESHES,
            Some("robot_a"),
            &[package_a.path().to_path_buf()],
        )
        .await
        .expect("import robot A");

    let record_b = robot_service
        .import_urdf_materialized(
            workspace_dir.path(),
            URDF_WITH_MESHES,
            Some("robot_b"),
            &[package_b.path().to_path_buf()],
        )
        .await
        .expect("import robot B");

    assert_ne!(record_a.id, record_b.id, "robots must have different IDs");

    // Create two stations, each referencing a different robot
    let mut station_a = Station::new("cell_a", "Cell A");
    station_a.add_robotics_module(RoboticsModule {
        id: RoboticsModuleId("arm_a".into()),
        station_id: StationId("cell_a".into()),
        name: "Arm A".into(),
        robot_name: "test_robot".into(),
        robot_definition_id: Some(record_a.id.clone()),
        controller_binding: "simulation".into(),
    });

    let mut station_b = Station::new("cell_b", "Cell B");
    station_b.add_robotics_module(RoboticsModule {
        id: RoboticsModuleId("arm_b".into()),
        station_id: StationId("cell_b".into()),
        name: "Arm B".into(),
        robot_name: "test_robot".into(),
        robot_definition_id: Some(record_b.id.clone()),
        controller_binding: "simulation".into(),
    });

    station_service.register_station(station_a);
    station_service.register_station(station_b);

    // Verify both stations exist with correct references
    let stations = station_service.list_stations();
    assert_eq!(stations.len(), 2);

    let station_a = stations.iter().find(|s| s.id.0 == "cell_a").unwrap();
    let station_b = stations.iter().find(|s| s.id.0 == "cell_b").unwrap();

    let module_a = station_a.robotics_modules.get(&RoboticsModuleId("arm_a".into())).unwrap();
    let module_b = station_b.robotics_modules.get(&RoboticsModuleId("arm_b".into())).unwrap();

    assert_eq!(module_a.robot_definition_id.as_deref(), Some(record_a.id.as_str()));
    assert_eq!(module_b.robot_definition_id.as_deref(), Some(record_b.id.as_str()));

    // Both robots are materialized
    let avail_a = check_robot_availability(&record_a.id, workspace_dir.path(), robot_service.repo().unwrap()).await;
    let avail_b = check_robot_availability(&record_b.id, workspace_dir.path(), robot_service.repo().unwrap()).await;
    assert_eq!(avail_a, RobotAvailability::Materialized);
    assert_eq!(avail_b, RobotAvailability::Materialized);

    println!("✓ Multiple stations with different robots work");
    println!("  cell_a → arm_a → {}", record_a.id);
    println!("  cell_b → arm_b → {}", record_b.id);
}
