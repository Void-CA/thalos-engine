use std::sync::Arc;

use thalos_persistence::{SqliteRobotRepository, SqliteWorkspaceRepository};
use thalos_runtime::backends::manager::BackendManager;
use thalos_runtime::ports::RobotRepository;
use thalos_runtime::robot::availability::{check_robot_availability, RobotAvailability};
use thalos_runtime::{RobotService, SceneService, WorkspaceService};
use thalos_engine::core::models::RobotModel;

/// URDF with mesh references (visual + collision) — the ABB IRB 140 pattern.
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

/// Create a fake package directory with URDF + mesh files.
fn create_test_package(dir: &std::path::Path) {
    let urdf_dir = dir.join("urdf");
    let visual_dir = dir.join("meshes").join("visual");
    let collision_dir = dir.join("meshes").join("collision");
    std::fs::create_dir_all(&urdf_dir).unwrap();
    std::fs::create_dir_all(&visual_dir).unwrap();
    std::fs::create_dir_all(&collision_dir).unwrap();

    std::fs::write(urdf_dir.join("robot.urdf"), URDF_WITH_MESHES).unwrap();

    // Create minimal STL files (binary header — enough for hash verification)
    let stl_header = b"solid test\nfacet normal 0 0 0\nendfacet\nendsolid test\n";
    for name in &["base_link.stl", "link_1.stl"] {
        std::fs::write(visual_dir.join(name), stl_header).unwrap();
        std::fs::write(collision_dir.join(name), stl_header).unwrap();
    }
}

#[tokio::test]
async fn materialized_robot_survives_full_lifecycle() {
    // ═══════════════════════════════════════════════════════════════
    // PHASE 1: Create workspace, import robot with materialization
    // ═══════════════════════════════════════════════════════════════
    let workspace_dir = tempfile::tempdir().unwrap();
    let workspace_path = workspace_dir.path().to_path_buf();

    // Create workspace directory structure manually
    std::fs::create_dir_all(workspace_path.join("robots")).unwrap();

    // Initialize the real DB inside the workspace directory
    let db_path = workspace_path.join("workspace.db");
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
    let workspace_service = Arc::new(WorkspaceService::new(
        workspace_repo.clone(),
        robot_service.clone(),
        scene_service.clone(),
    ));

    // Import robot FIRST (generates a UUID-based ID)
    let package_dir = tempfile::tempdir().unwrap();
    create_test_package(package_dir.path());

    let record = robot_service
        .import_urdf_materialized(
            &workspace_path,
            URDF_WITH_MESHES,
            Some("test_package"),
            &[package_dir.path().to_path_buf()],
        )
        .await
        .expect("import_urdf_materialized must succeed");

    let robot_id = record.id.clone();
    println!("Imported robot: {robot_id}");

    // Create workspace (robots are independent resources, not owned by workspace)
    let workspace = workspace_service
        .create_workspace("Test Cell")
        .await
        .expect("create_workspace must succeed");

    // Verify files exist on disk
    let robot_dir = workspace_path.join("robots").join(&robot_id);
    assert!(robot_dir.join("robot.urdf").exists(), "robot.urdf must exist");
    assert!(robot_dir.join("assets").join("visual").join("base_link.stl").exists());
    assert!(robot_dir.join("assets").join("visual").join("link_1.stl").exists());
    assert!(robot_dir.join("assets").join("collision").join("base_link.stl").exists());
    assert!(robot_dir.join("assets").join("collision").join("link_1.stl").exists());

    // Verify assets are persisted in SQLite
    let assets = robot_repo.get_assets(&robot_id).await.expect("get_assets");
    assert_eq!(assets.len(), 4, "must have 4 assets (2 visual + 2 collision)");
    assert!(assets.iter().all(|a| !a.sha256.is_empty()), "all assets must have SHA-256");

    // Verify availability is Materialized
    let availability = check_robot_availability(&robot_id, &workspace_path, robot_repo.as_ref())
        .await;
    assert_eq!(availability, RobotAvailability::Materialized);

    // Save the hash for later verification
    let original_hash = assets[0].sha256.clone();
    let original_chain_dof = {
        let snapshot = robot_service
            .load_materialized_robot(&robot_id, &workspace_path, &scene_service)
            .await
            .expect("load_materialized_robot must succeed");
        snapshot.joints.len()
    };

    // ═══════════════════════════════════════════════════════════════
    // PHASE 2: Simulate process restart — drop everything, reinit
    // ═══════════════════════════════════════════════════════════════
    drop(workspace_service);
    drop(scene_service);
    drop(robot_service);
    drop(workspace_repo);
    drop(robot_repo);

    // Reinitialize from the same workspace directory
    let new_robot_repo = Arc::new(
        SqliteRobotRepository::new(db_path.to_str().unwrap())
            .await
            .expect("reopen robot repo"),
    );
    let new_workspace_repo = Arc::new(
        SqliteWorkspaceRepository::new(db_path.to_str().unwrap())
            .await
            .expect("reopen workspace repo"),
    );
    let new_robot_service = Arc::new(RobotService::new(Some(new_robot_repo.clone())));
    let new_manager = Arc::new(BackendManager::new());
    let new_scene_service = Arc::new(SceneService::new(new_manager, RobotModel::Planar2R));
    let new_workspace_service = Arc::new(WorkspaceService::new(
        new_workspace_repo,
        new_robot_service.clone(),
        new_scene_service.clone(),
    ));

    // ═══════════════════════════════════════════════════════════════
    // PHASE 3: Reopen workspace, load robot, verify integrity
    // ═══════════════════════════════════════════════════════════════

    // Load robot from materialized workspace
    let snapshot = new_robot_service
        .load_materialized_robot(&robot_id, &workspace_path, &new_scene_service)
        .await
        .expect("load_materialized_robot after restart must succeed");

    // Verify same robot name
    assert_eq!(snapshot.robot_name, "test_robot");

    // Verify same kinematic chain DOF
    assert_eq!(snapshot.joints.len(), original_chain_dof, "DOF must survive restart");

    // Verify asset hashes match
    let reopened_assets = new_robot_repo.get_assets(&robot_id).await.expect("get_assets after restart");
    assert_eq!(reopened_assets.len(), 4, "must have 4 assets after restart");
    assert_eq!(
        reopened_assets[0].sha256, original_hash,
        "asset hash must survive restart"
    );

    // Verify availability is still Materialized after restart
    let availability = check_robot_availability(&robot_id, &workspace_path, new_robot_repo.as_ref())
        .await;
    assert_eq!(availability, RobotAvailability::Materialized);

    // ═══════════════════════════════════════════════════════════════
    // PHASE 4: Verify workspace service lifecycle
    // ═══════════════════════════════════════════════════════════════
    let active = new_workspace_service.active_workspace().await;
    assert!(active.is_none(), "no active workspace after fresh init");

    let root = new_workspace_service.root().await;
    assert!(root.is_none(), "no root set yet");

    // Open via workspace service
    let opened = new_workspace_service
        .open_at(&workspace_path)
        .await
        .expect("open_at must succeed");

    assert_eq!(opened.workspace.id, workspace.id);
    assert_eq!(opened.workspace.name, "Test Cell");
    // Workspace no longer loads a specific robot — that happens when a station
    // module is selected. open_at loads a default scene.
    assert!(opened.runtime_snapshot.joints.len() >= 2);

    let root_after = new_workspace_service.root().await;
    assert_eq!(root_after, Some(workspace_path.clone()));

    // Close workspace
    new_workspace_service.close().await.expect("close must succeed");
    assert!(new_workspace_service.root().await.is_none());
    assert!(new_workspace_service.active_workspace().await.is_none());

    println!("✓ Full materialization lifecycle passed");
    println!("  workspace: {}", workspace_path.display());
    println!("  robot_id: {robot_id}");
    println!("  assets: 4 (2 visual + 2 collision)");
    println!("  DOF: {original_chain_dof}");
}

#[tokio::test]
async fn legacy_robot_detected_as_requires_reimport() {
    let workspace_dir = tempfile::tempdir().unwrap();
    let workspace_path = workspace_dir.path().to_path_buf();
    let db_path = workspace_path.join("workspace.db");
    std::fs::create_dir_all(workspace_path.join("robots")).unwrap();

    let robot_repo = Arc::new(
        SqliteRobotRepository::new(db_path.to_str().unwrap())
            .await
            .expect("init robot repo"),
    );

    // Simulate a legacy robot: urdf_xml in SQLite, no filesystem artifacts
    #[allow(deprecated)]
    let legacy_record = thalos_runtime::ports::RobotRecord {
        id: "legacy-robot-001".to_string(),
        name: "Legacy Bot".to_string(),
        manufacturer: None,
        model: None,
        source_type: thalos_runtime::ports::RobotSource::ImportedUrdf,
        source_label: None,
        urdf_xml: Some(URDF_WITH_MESHES.to_string()),
        created_at: chrono::Utc::now().to_rfc3339(),
        updated_at: chrono::Utc::now().to_rfc3339(),
    };
    robot_repo.save(&legacy_record).await.expect("save legacy");

    // Check availability — should be Legacy
    let availability = check_robot_availability("legacy-robot-001", &workspace_path, robot_repo.as_ref())
        .await;
    assert_eq!(availability, RobotAvailability::Legacy);

    // Nonexistent robot — also Legacy
    let availability = check_robot_availability("nonexistent", &workspace_path, robot_repo.as_ref())
        .await;
    assert_eq!(availability, RobotAvailability::Legacy);
}

#[tokio::test]
async fn corrupted_robot_detected_as_corrupted() {
    let workspace_dir = tempfile::tempdir().unwrap();
    let workspace_path = workspace_dir.path().to_path_buf();

    // Use a separate DB file to avoid conflicts
    let db_file = tempfile::NamedTempFile::new().unwrap();
    let db_path = db_file.path().to_str().unwrap().to_string();

    let robot_repo = Arc::new(
        SqliteRobotRepository::new(&db_path)
            .await
            .expect("init robot repo"),
    );

    // Import a robot
    let robot_service = RobotService::new(Some(robot_repo.clone()));
    let package_dir = tempfile::tempdir().unwrap();
    create_test_package(package_dir.path());

    let record = robot_service
        .import_urdf_materialized(
            &workspace_path,
            URDF_WITH_MESHES,
            Some("test"),
            &[package_dir.path().to_path_buf()],
        )
        .await
        .expect("import must succeed");

    // Verify initially Materialized
    let availability = check_robot_availability(&record.id, &workspace_path, robot_repo.as_ref())
        .await;
    assert_eq!(availability, RobotAvailability::Materialized);

    // Delete a mesh file to simulate corruption
    let mesh_path = workspace_path
        .join("robots")
        .join(&record.id)
        .join("assets")
        .join("visual")
        .join("base_link.stl");
    std::fs::remove_file(&mesh_path).expect("delete mesh");

    // Should now detect corruption
    let availability = check_robot_availability(&record.id, &workspace_path, robot_repo.as_ref())
        .await;
    match availability {
        RobotAvailability::Corrupted { missing } => {
            assert!(!missing.is_empty(), "must report which assets are corrupted");
            println!("✓ Corrupted detection works: {}", missing[0]);
        }
        other => panic!("expected Corrupted, got {:?}", other),
    }
}
