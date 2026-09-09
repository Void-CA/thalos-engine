use std::sync::Arc;

use thalos_persistence::{SqliteRobotRepository, SqliteStationRepository, SqliteWorkspaceRepository, SqliteEquipmentModuleRepository};
use thalos_runtime::ports::{RobotRepository, StationRepository, EquipmentModuleRepository};
use thalos_runtime::robot::availability::{check_robot_availability, RobotAvailability};
use thalos_runtime::station::{Station, StationService};
use thalos_runtime::RobotService;

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
    let visual = dir.join("meshes").join("visual");
    let collision = dir.join("meshes").join("collision");
    std::fs::create_dir_all(&visual).unwrap();
    std::fs::create_dir_all(&collision).unwrap();
    let stl = b"solid test\nfacet normal 0 0 0\nendfacet\nendsolid test\n";
    for name in &["base_link.stl", "link_1.stl"] {
        std::fs::write(visual.join(name), stl).unwrap();
        std::fs::write(collision.join(name), stl).unwrap();
    }
}

struct Ctx {
    robot_service: Arc<RobotService>,
    station_service: StationService,
    module_repo: Arc<SqliteEquipmentModuleRepository>,
    workspace: std::path::PathBuf,
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
    let station_service = StationService::new(station_repo, module_repo.clone(), robot_repo as Arc<dyn RobotRepository>);
    Ctx { robot_service, station_service, module_repo, workspace: dir.into_path() }
}

#[tokio::test]
async fn station_module_robot_lifecycle() {
    let ctx = setup().await;
    let pkg = tempfile::tempdir().unwrap();
    create_test_package(pkg.path());

    let record = ctx.robot_service
        .import_urdf_materialized(&ctx.workspace, URDF_WITH_MESHES, Some("test_pkg"), &[pkg.path().to_path_buf()])
        .await.expect("import");
    let robot_id = record.id.clone();

    let station = ctx.station_service.create_station("assembly", "Assembly").await.unwrap();
    ctx.station_service.add_robotics_module(&station.id, &robot_id, "Arm", "{}").await.unwrap();

    assert_eq!(check_robot_availability(&robot_id, &ctx.workspace, ctx.robot_service.repo().unwrap()).await, RobotAvailability::Materialized);
    assert_eq!(ctx.module_repo.find_robot_references(&robot_id).await.unwrap().len(), 1);
}

#[tokio::test]
async fn station_module_robot_survives_reopen() {
    let dir = tempfile::tempdir().unwrap();
    let robot_id;

    {
        let db = dir.path().join("workspace.db");
        std::fs::create_dir_all(dir.path().join("robots")).unwrap();
        let r = Arc::new(SqliteRobotRepository::new(db.to_str().unwrap()).await.unwrap());
        let w = Arc::new(SqliteWorkspaceRepository::new(db.to_str().unwrap()).await.unwrap());
        let s = Arc::new(SqliteStationRepository::from_pool(w.pool().clone()).await.unwrap());
        let m = Arc::new(SqliteEquipmentModuleRepository::new(w.pool().clone()));
        let rs = Arc::new(RobotService::new(Some(r.clone())));
        let ss = StationService::new(s, m, r as Arc<dyn RobotRepository>);

        let pkg = tempfile::tempdir().unwrap();
        create_test_package(pkg.path());
        let rec = rs.import_urdf_materialized(dir.path(), URDF_WITH_MESHES, Some("pkg"), &[pkg.path().to_path_buf()]).await.unwrap();
        robot_id = rec.id.clone();

        let st = ss.create_station("cell", "Cell").await.unwrap();
        ss.add_robotics_module(&st.id, &robot_id, "Arm", "{}").await.unwrap();
    }

    let db = dir.path().join("workspace.db");
    let r = Arc::new(SqliteRobotRepository::new(db.to_str().unwrap()).await.unwrap());
    let w = Arc::new(SqliteWorkspaceRepository::new(db.to_str().unwrap()).await.unwrap());
    let s = Arc::new(SqliteStationRepository::from_pool(w.pool().clone()).await.unwrap());
    let m = Arc::new(SqliteEquipmentModuleRepository::new(w.pool().clone()));
    let rs = Arc::new(RobotService::new(Some(r.clone())));
    let ss = StationService::new(s, m.clone(), r as Arc<dyn RobotRepository>);

    assert_eq!(ss.list_stations().await.unwrap().len(), 1);
    assert_eq!(ss.get_station_modules(&StationId("cell".into())).await.unwrap().len(), 1);
    assert_eq!(m.find_robot_references(&robot_id).await.unwrap().len(), 1);
    assert_eq!(check_robot_availability(&robot_id, &dir.path(), rs.repo().unwrap()).await, RobotAvailability::Materialized);
}

#[tokio::test]
async fn broken_reference_rejected_by_service() {
    let ctx = setup().await;
    let st = ctx.station_service.create_station("ghost", "Ghost").await.unwrap();
    let result = ctx.station_service.add_robotics_module(&st.id, "nonexistent", "Arm", "{}").await;
    assert!(result.is_err());
    assert!(matches!(result.unwrap_err(), thalos_runtime::station::StationError::RobotNotFound(_)));
}

#[tokio::test]
async fn multiple_stations_different_robots() {
    let ctx = setup().await;
    let pkg_a = tempfile::tempdir().unwrap();
    create_test_package(pkg_a.path());
    let pkg_b = tempfile::tempdir().unwrap();
    create_test_package(pkg_b.path());
    std::fs::write(pkg_b.path().join("meshes").join("visual").join("base_link.stl"), b"solid b\n").unwrap();

    let ra = ctx.robot_service.import_urdf_materialized(&ctx.workspace, URDF_WITH_MESHES, Some("a"), &[pkg_a.path().to_path_buf()]).await.unwrap();
    let rb = ctx.robot_service.import_urdf_materialized(&ctx.workspace, URDF_WITH_MESHES, Some("b"), &[pkg_b.path().to_path_buf()]).await.unwrap();
    assert_ne!(ra.id, rb.id);

    let sa = ctx.station_service.create_station("ca", "Cell A").await.unwrap();
    ctx.station_service.add_robotics_module(&sa.id, &ra.id, "Arm A", "{}").await.unwrap();
    let sb = ctx.station_service.create_station("cb", "Cell B").await.unwrap();
    ctx.station_service.add_robotics_module(&sb.id, &rb.id, "Arm B", "{}").await.unwrap();

    assert_eq!(ctx.module_repo.find_robot_references(&ra.id).await.unwrap()[0].station_id, "ca");
    assert_eq!(ctx.module_repo.find_robot_references(&rb.id).await.unwrap()[0].station_id, "cb");
    assert_eq!(check_robot_availability(&ra.id, &ctx.workspace, ctx.robot_service.repo().unwrap()).await, RobotAvailability::Materialized);
    assert_eq!(check_robot_availability(&rb.id, &ctx.workspace, ctx.robot_service.repo().unwrap()).await, RobotAvailability::Materialized);
}

use thalos_engine::prelude::StationId;