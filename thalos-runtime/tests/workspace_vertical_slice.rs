use std::sync::Arc;
use tempfile::NamedTempFile;

use thalos_engine::core::models::RobotModel;
use thalos_persistence::{SqliteRobotRepository, SqliteWorkspaceRepository};
use thalos_runtime::backends::manager::BackendManager;
use thalos_runtime::{
    RobotService, SceneService, WorkspaceService,
};

#[tokio::test]
async fn test_workspace_vertical_slice_lifecycle() {
    let temp_db = NamedTempFile::new().expect("temp file failed");
    let db_path = temp_db.path().to_str().expect("db path failed");

    // Phase 1: Initialize repositories & services
    let robot_repo = Arc::new(
        SqliteRobotRepository::new(db_path)
            .await
            .expect("init robot repo failed"),
    );
    let workspace_repo = Arc::new(
        SqliteWorkspaceRepository::new(db_path)
            .await
            .expect("init workspace repo failed"),
    );

    let robot_service = Arc::new(RobotService::new(Some(robot_repo.clone())));
    let manager = Arc::new(BackendManager::new());
    let scene_service = Arc::new(SceneService::new(manager, RobotModel::Scara));
    let workspace_service = WorkspaceService::new(
        workspace_repo.clone(),
        robot_service.clone(),
        scene_service.clone(),
    );

    // Phase 2: Create a workspace
    let created_ws = workspace_service
        .create_workspace("Workstation 1 - Assembly")
        .await
        .expect("create workspace failed");

    assert_eq!(created_ws.name, "Workstation 1 - Assembly");

    // Phase 3: Simulate application process restart (re-opening SQLite database file)
    drop(workspace_service);
    drop(scene_service);
    drop(robot_service);
    drop(workspace_repo);
    drop(robot_repo);

    let new_robot_repo = Arc::new(
        SqliteRobotRepository::new(db_path)
            .await
            .expect("reopen robot repo failed"),
    );
    let new_workspace_repo = Arc::new(
        SqliteWorkspaceRepository::new(db_path)
            .await
            .expect("reopen workspace repo failed"),
    );

    let new_robot_service = Arc::new(RobotService::new(Some(new_robot_repo)));
    let new_manager = Arc::new(BackendManager::new());
    let new_scene_service = Arc::new(SceneService::new(new_manager, RobotModel::Scara));
    let new_workspace_service = WorkspaceService::new(
        new_workspace_repo,
        new_robot_service,
        new_scene_service.clone(),
    );

    // Phase 4: Open Workspace by ID -> Triggers robot resolution & scene reconstruction
    let opened = new_workspace_service
        .open(&created_ws.id)
        .await
        .expect("open workspace failed");

    assert_eq!(opened.workspace.id, created_ws.id);
    assert_eq!(opened.workspace.name, "Workstation 1 - Assembly");

    // Verify SceneService has a default robot loaded
    assert!(opened.runtime_snapshot.joints.len() >= 2);

    // Phase 5: Verify active workspace session tracking & close() lifecycle
    let active = new_workspace_service
        .active_workspace()
        .await
        .expect("active workspace must be present after open");
    assert_eq!(active.workspace.id, created_ws.id);

    new_workspace_service
        .close()
        .await
        .expect("close workspace session must succeed");

    assert!(
        new_workspace_service.active_workspace().await.is_none(),
        "active workspace must be cleared after close"
    );
}


