use std::sync::Arc;

use thalos_engine::core::kinematics::inverse::IKGoal;
use thalos_engine::core::models::RobotModel;
use thalos_engine::math::Vector3;

use thalos_persistence::SqliteRobotRepository;
use thalos_runtime::backends::manager::BackendManager;
use thalos_runtime::robot::RobotService;
use thalos_runtime::scene::SceneService;

const SAMPLE_2DOF_URDF: &str = r#"<?xml version="1.0"?>
<robot name="custom_planar_2dof">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0"/>
  </joint>
</robot>"#;

#[tokio::test]
async fn imported_robot_survives_full_lifecycle() {
    // 1. Setup in-memory SQLite repository + RobotService + SceneService
    let repo = SqliteRobotRepository::in_memory()
        .await
        .expect("in-memory db initialization failed");
    let repo_arc: Arc<dyn thalos_runtime::ports::RobotRepository> = Arc::new(repo);
    let robot_service = RobotService::new(Some(repo_arc));

    let default_model = RobotModel::Planar2R;
    let manager = Arc::new(BackendManager::new());
    let scene_service = SceneService::new(manager, default_model);

    // 2. Import URDF via RobotService
    let imported_record = robot_service
        .import_urdf(SAMPLE_2DOF_URDF)
        .await
        .expect("URDF import must succeed");

    assert_eq!(imported_record.name, "custom_planar_2dof");
    assert!(imported_record.id.starts_with("urdf-"));

    // 3. Verify record in unified catalog
    let all_robots = robot_service.list_all().await;
    assert!(
        all_robots.iter().any(|r| r.id == imported_record.id),
        "Imported robot must be present in list_all"
    );
    // Canonical robots (scara, puma560, etc.) must exist without duplication
    assert!(
        all_robots.iter().any(|r| r.id == "scara"),
        "Canonical SCARA must be present in unified catalog"
    );

    let retrieved_record = robot_service
        .get_record(&imported_record.id)
        .await
        .expect("get_record must find imported robot");
    assert_eq!(retrieved_record.name, "custom_planar_2dof");

    // 4. Load imported robot into SceneService via RobotService
    let snapshot = robot_service
        .load_robot_into_scene(&imported_record.id, &scene_service)
        .await
        .expect("load_robot_into_scene must succeed");

    assert_eq!(snapshot.robot_name, "custom_planar_2dof");
    assert_eq!(snapshot.joints.len(), 2);
    assert_eq!(snapshot.joints_meta.len(), 2);
    assert_eq!(snapshot.joints_meta[0].name, "joint1");
    assert_eq!(snapshot.joints_meta[1].name, "joint2");

    // 5. Execute real engine kinematics (IK) on the imported robot in SceneService
    let ee_frame = snapshot.chain.end_effector;
    let goal = IKGoal::Position(Vector3::new(1.0, 0.0, 0.0));
    let (joints, ik_res) = scene_service
        .solve_ik(ee_frame, goal)
        .await
        .expect("IK solve must succeed for imported robot");

    println!("IK RESULT: status={:?}, final_error={}, q={:?}", ik_res.status, ik_res.final_error, joints);

    assert!(
        ik_res.status.is_converged(),
        "IK solve must converge for 2-DOF chain (final_error={})", ik_res.final_error
    );
    assert_eq!(joints.len(), 2);
}

#[tokio::test]
async fn persisted_robot_survives_process_restart() {
    let tmp_file = tempfile::NamedTempFile::new().expect("failed to create temp file");
    let db_path = tmp_file.path().to_string_lossy().to_string();

    let imported_id = {
        // Phase 1: Initialize DB, import URDF, verify save, then close/drop everything
        let repo = SqliteRobotRepository::new(&db_path)
            .await
            .expect("sqlite db creation failed");
        let repo_arc: Arc<dyn thalos_runtime::ports::RobotRepository> = Arc::new(repo);
        let robot_service = RobotService::new(Some(repo_arc));

        let record = robot_service
            .import_urdf(SAMPLE_2DOF_URDF)
            .await
            .expect("import urdf in phase 1 must succeed");
        record.id
    };

    // Phase 2: Simulate process restart — re-open SQLite DB at the exact same file path
    let repo_reloaded = SqliteRobotRepository::new(&db_path)
        .await
        .expect("re-opening sqlite db failed");
    let repo_arc: Arc<dyn thalos_runtime::ports::RobotRepository> = Arc::new(repo_reloaded);
    let robot_service_reloaded = RobotService::new(Some(repo_arc));

    let default_model = RobotModel::Planar2R;
    let manager = Arc::new(BackendManager::new());
    let scene_service = SceneService::new(manager, default_model);

    // Verify imported record persists in SQLite across restart
    let record = robot_service_reloaded
        .get_record(&imported_id)
        .await
        .expect("imported record must survive process restart in SQLite");
    assert_eq!(record.name, "custom_planar_2dof");

    // Load persisted robot into new SceneService
    let snapshot = robot_service_reloaded
        .load_robot_into_scene(&imported_id, &scene_service)
        .await
        .expect("load_robot_into_scene after restart must succeed");

    assert_eq!(snapshot.robot_name, "custom_planar_2dof");
    assert_eq!(snapshot.joints.len(), 2);

    // Verify kinematics operations work after restart
    let ee_frame = snapshot.chain.end_effector;
    let goal = IKGoal::Position(Vector3::new(0.0, 1.0, 0.0));
    let (_joints, ik_res) = scene_service
        .solve_ik(ee_frame, goal)
        .await
        .expect("solve_ik after restart must succeed");

    assert!(
        ik_res.status.is_converged(),
        "IK solve after restart must converge (final_error={})", ik_res.final_error
    );
}
