use std::sync::Arc;

use async_trait::async_trait;
use std::sync::atomic::{AtomicBool, AtomicU32};

use tokio::sync::RwLock;

use crate::backends::controller::tests::MockController;
use crate::backends::controller::BackendCapabilities;
use crate::error::{ControllerError, RuntimeError};
use crate::execution_boundary::ExecutionSample;
use crate::services::scene::RepeatPhase;
use crate::session::{ExecutionSource, SessionManager};
use crate::state::robot_state::{MotionMode, RobotState};
use crate::{
    Command, RobotController, RuntimeSnapshot, SceneService,
    backends::{
        controller::simulation::SimulationController, manager::BackendManager,
    },
    commands::kinematics::KinematicsCommand,
    commands::motion::MotionCommands,
};
use thalos_engine::core::{
    execution::plan::ExecutionPlan,
    models::RobotModel,
    prelude::IKGoal,
    spatial::{frame::FrameId, pose::Pose},
};
use thalos_engine::math::{Transform3D, UnitQuaternion, Vector3};

// ─── Helpers ───────────────────────────────────────────────────────

/// A VALID compiled plan: two waypoints, non-zero duration, target `[t, t]`.
fn compiled_plan(t: f64) -> thalos_engine::planning::motion::program::CompiledPlan {
    let points = vec![
        thalos_engine::core::trajectory::TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
        thalos_engine::core::trajectory::TrajectoryPoint::new(vec![t, t], 1.0),
    ];
    thalos_engine::planning::motion::program::CompiledPlan::new(
        thalos_engine::core::trajectory::Trajectory::new(points),
        vec![],
    )
}

/// Create a SceneService with the given model and a BackendManager (simulation).
async fn make_service(model: RobotModel) -> (SceneService, Arc<BackendManager>) {
    let controller = Arc::new(RwLock::new(SimulationController::new(model.metadata().dof)))
        as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(manager.clone(), model);
    (svc, manager)
}

/// Resolve the end effector frame from a snapshot's chain.
fn ee(snapshot: &RuntimeSnapshot) -> FrameId {
    *snapshot.chain.end_effector()
}

/// Assert that the FK position of the end effector in `snapshot`
/// is within `tol` of the expected `target` position.
fn assert_ee_at(snapshot: &RuntimeSnapshot, target: Vector3, tol: f64) {
    let frame = ee(snapshot);
    let pose = snapshot
        .fk_result
        .pose(&frame)
        .expect("end effector must exist in FK result");
    let pos = pose.translation();
    let error = (target - pos).magnitude();
    assert!(
        error < tol,
        "EE position error: {:.4} (tol {:.4})\n  expected: {:.4?}\n  actual:   {:.4?}",
        error,
        tol,
        target,
        pos,
    );
}

// ─── SetJoints ────────────────────────────────────────────────────

#[tokio::test]
async fn set_joints_updates_state_and_fk() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;
    let joints = vec![0.5, -0.3, 0.1, 0.0];

    let snap = svc
        .execute(Command::SetJoints(joints.clone()))
        .await
        .unwrap();

    assert_eq!(snap.joints, joints);
    assert_eq!(snap.robot, Some(RobotModel::Scara));
    // FK must be valid after setting joints
    let _pose = snap
        .fk_result
        .pose(&ee(&snap))
        .expect("FK result must contain end effector");
}

// NOTE: no hay test de "SetJoints con DOF incorrecto" porque el FK
// evalúa contra la cadena cinemática y paniquea si el tamaño de joints
// no coincide. La validación de DOF es responsabilidad del caller.

// ─── LoadRobot ────────────────────────────────────────────────────

#[tokio::test]
async fn load_robot_changes_model_and_resets_joints() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;
    // Set some joints first
    svc.execute(Command::SetJoints(vec![1.0, 2.0, 3.0, 4.0]))
        .await
        .unwrap();

    let snap = svc
        .execute(Command::LoadRobot(RobotModel::Planar3R))
        .await
        .unwrap();

    assert_eq!(snap.robot, Some(RobotModel::Planar3R));
    assert_eq!(snap.joints.len(), 3, "Planar3R has 3 DOF");
    // Joints reset to zero
    assert!(snap.joints.iter().all(|&j| j == 0.0));
}

#[tokio::test]
async fn load_robot_twice_produces_independent_snapshots() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;

    let snap1 = svc
        .execute(Command::LoadRobot(RobotModel::Planar2R))
        .await
        .unwrap();
    let snap2 = svc
        .execute(Command::LoadRobot(RobotModel::Scara))
        .await
        .unwrap();

    assert_eq!(snap1.robot, Some(RobotModel::Planar2R));
    assert_eq!(snap2.robot, Some(RobotModel::Scara));
    assert_eq!(snap1.joints.len(), 2);
    assert_eq!(snap2.joints.len(), 4);
}

#[tokio::test]
async fn load_robot_clears_active_plan() {
    let (svc, _mgr) = make_service(RobotModel::Planar2R).await;

    // Create a plan for Planar2R
    let snap = svc
        .execute(Command::Motion(MotionCommands::PlanAndMoveJ {
            target: vec![0.5, 0.3],
            max_velocity: None,
            max_acceleration: None,
            time_step: None,
        }))
        .await
        .unwrap();
    assert!(
        snap.active_plan.is_some(),
        "plan must exist after PlanAndMoveJ",
    );

    // Load a different robot — plan must be cleared
    let snap = svc
        .execute(Command::LoadRobot(RobotModel::Scara))
        .await
        .unwrap();
    assert!(
        snap.active_plan.is_none(),
        "active_plan must be None after LoadRobot",
    );
    // New robot has 4 DOF, initialised to zero
    assert_eq!(snap.joints.len(), 4);
    assert!(snap.joints.iter().all(|&j| j == 0.0));
}

// ─── MoveToPosition (IK + FK round-trip) ──────────────────────────

/// Single execution of MoveToPosition on a SCARA: the solver should
/// bring the end effector within tolerance of the target.
#[tokio::test]
async fn move_to_position_converges_scara() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;
    let snap0 = svc.snapshot().await.unwrap();
    let ee = ee(&snap0);
    // Well within SCARA workspace: r_xy = sqrt(0.6²+0.5²) = 0.78 > r_min (0.50)
    let target = Vector3::new(0.6, 0.5, 0.25);

    let snap = svc
        .execute(Command::Kinematics(KinematicsCommand::MoveToPosition {
            frame: ee,
            target,
        }))
        .await
        .unwrap();

    assert_ee_at(&snap, target, 0.01);
}

/// Sequential MoveToPosition commands: each should converge from the
/// previous configuration.
#[tokio::test]
async fn move_to_position_sequential() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;

    // All targets within canonical SCARA workspace (r_min ≈ 0.50, r_max ≈ 1.8)
    let targets = [
        Vector3::new(0.7, 0.5, 0.25),
        Vector3::new(0.3, 0.8, 0.10),
        Vector3::new(0.5, 0.6, 0.00),
    ];

    let mut snap = svc.snapshot().await.unwrap();
    for &target in &targets {
        let ee = ee(&snap);
        snap = svc
            .execute(Command::Kinematics(KinematicsCommand::MoveToPosition {
                frame: ee,
                target,
            }))
            .await
            .unwrap();
        assert_ee_at(&snap, target, 0.01);
    }
}

/// MoveToPosition with a frame that is not the chain's default end
/// effector — verifies the IK solver correctly handles arbitrary frames.
#[tokio::test]
async fn move_to_position_custom_frame() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;
    let snap = svc.snapshot().await.unwrap();

    // Use prismatic_frame (id 3): the first frame whose Z is affected by q3
    let target_frame = FrameId::Id(3);
    let _initial = snap
        .fk_result
        .pose(&target_frame)
        .expect("target frame must exist")
        .translation();

    // Target within canonical SCARA workspace
    let target = Vector3::new(0.7, 0.5, 0.25);

    let snap = svc
        .execute(Command::Kinematics(KinematicsCommand::MoveToPosition {
            frame: target_frame,
            target,
        }))
        .await
        .unwrap();

    let final_pos = snap.fk_result.pose(&target_frame).unwrap().translation();
    let error = (target - final_pos).magnitude();
    assert!(
        error < 0.01,
        "frame position error: {:.4} (target {:.4?}, actual {:.4?})",
        error,
        target,
        final_pos,
    );
}

/// Reachable target close to the initial configuration with a Y offset
/// to avoid the X-axis singularity (full extension at q=[0,0,0,0]).
#[tokio::test]
async fn move_to_position_nearby() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;
    let snap0 = svc.snapshot().await.unwrap();
    let ee = ee(&snap0);

    // Target cerca del EE inicial (1.8, 0, 0.5), bien dentro del workspace
    let target = Vector3::new(1.5, 0.3, 0.4);

    let snap = svc
        .execute(Command::Kinematics(KinematicsCommand::MoveToPosition {
            frame: ee,
            target,
        }))
        .await
        .unwrap();

    assert_ee_at(&snap, target, 0.01);
}

// ─── MoveToPose (IK + FK round-trip with orientation) ─────────────

/// MoveToPose with identity rotation (same as initial SCARA orientation).
/// Since the orientation is already matched, the solver primarily works
/// on position error, but exercises the full 6-DOF IK path.
#[tokio::test]
async fn move_to_pose_converges_with_identity_rotation() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;
    let snap0 = svc.snapshot().await.unwrap();
    let ee_frame = ee(&snap0);

    let target_pos = Vector3::new(0.6, 0.5, 0.25);
    let identity_rot = UnitQuaternion::identity();
    let target_pose = Pose::new(
        FrameId::World,
        ee_frame,
        Transform3D {
            translation: target_pos,
            rotation: identity_rot,
        },
    );

    let snap = svc
        .execute(Command::Kinematics(KinematicsCommand::MoveToPose {
            frame: ee_frame,
            target: target_pose,
        }))
        .await
        .unwrap();

    assert_ee_at(&snap, target_pos, 0.01);
}

/// MoveToPose converges when both position and orientation targets
/// can be satisfied (using a planar 3R arm, targeting identity rot +
/// a reachable position).
#[tokio::test]
async fn move_to_pose_3r_converges() {
    let (svc, _mgr) = make_service(RobotModel::Planar3R).await;
    let snap0 = svc.snapshot().await.unwrap();
    let ee_frame = ee(&snap0);

    let target_pos = Vector3::new(2.5, 0.5, 0.0);
    let identity_rot = UnitQuaternion::identity();
    let target_pose = Pose::new(
        FrameId::World,
        ee_frame,
        Transform3D {
            translation: target_pos,
            rotation: identity_rot,
        },
    );

    let snap = svc
        .execute(Command::Kinematics(KinematicsCommand::MoveToPose {
            frame: ee_frame,
            target: target_pose,
        }))
        .await
        .unwrap();

    assert_ee_at(&snap, target_pos, 0.01);
}

// ─── Snapshot consistency ─────────────────────────────────────────

#[tokio::test]
async fn snapshot_after_ik_differs_from_initial() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;
    let snap0 = svc.snapshot().await.unwrap();
    let ee_frame = ee(&snap0);
    let initial_joints = snap0.joints.clone();

    let target = Vector3::new(0.2, 0.6, 0.0);
    let snap1 = svc
        .execute(Command::Kinematics(KinematicsCommand::MoveToPosition {
            frame: ee_frame,
            target,
        }))
        .await
        .unwrap();

    // Joints must have changed
    assert_ne!(snap1.joints, initial_joints);
    // Timestamps should differ
    assert!(snap1.generated_at > snap0.generated_at);
}

/// After an IK command, the snapshot carries solver metadata.
#[tokio::test]
async fn move_to_position_includes_ik_result() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;
    let snap0 = svc.snapshot().await.unwrap();
    let ee_frame = ee(&snap0);

    // Non-IK snapshot → ik_result is None
    assert!(
        snap0.ik_result.is_none(),
        "snapshot() must not have ik_result"
    );

    let target = Vector3::new(0.6, 0.5, 0.25);
    let snap1 = svc
        .execute(Command::Kinematics(KinematicsCommand::MoveToPosition {
            frame: ee_frame,
            target,
        }))
        .await
        .unwrap();

    let ik = snap1
        .ik_result
        .as_ref()
        .expect("IK command snapshot must have ik_result");
    assert!(
        ik.status.is_converged(),
        "IK should converge: {:?}",
        ik.status
    );
    assert!(ik.iterations > 0, "IK should run at least one iteration");
    assert!(ik.final_error.is_finite(), "final_error must be finite");
}

/// Multiple snapshots without mutations must return consistent joints.
#[tokio::test]
async fn snapshot_is_deterministic() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;

    let snap1 = svc.snapshot().await.unwrap();
    let snap2 = svc.snapshot().await.unwrap();

    assert_eq!(snap1.joints, snap2.joints);
}

// ─── SolveIK (no mutation) ────────────────────────────────────────

#[tokio::test]
async fn solve_ik_returns_joints_without_mutating_state() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;
    let snap0 = svc.snapshot().await.unwrap();
    let ee_frame = ee(&snap0);
    let initial_joints = snap0.joints.clone();

    let target = Vector3::new(0.6, 0.5, 0.25);
    let (solved_joints, ik) = svc
        .solve_ik(ee_frame, IKGoal::Position(target))
        .await
        .unwrap();

    // Must return solved joints distinct from initial
    assert_ne!(
        solved_joints, initial_joints,
        "solve_ik must propose new joints"
    );
    assert!(ik.status.is_converged(), "IK must converge");

    // State must NOT have changed
    let snap1 = svc.snapshot().await.unwrap();
    assert_eq!(
        snap1.joints, initial_joints,
        "solve_ik must NOT mutate runtime state",
    );
}

// ─── Edge cases ───────────────────────────────────────────────────

#[tokio::test]
async fn move_to_position_unreachable_still_produces_valid_fk() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;
    let snap0 = svc.snapshot().await.unwrap();
    let ee_frame = ee(&snap0);

    // Target far outside SCARA workspace
    let target = Vector3::new(10.0, 10.0, 0.0);

    let snap = svc
        .execute(Command::Kinematics(KinematicsCommand::MoveToPosition {
            frame: ee_frame,
            target,
        }))
        .await
        .unwrap();

    // Even if IK fails to converge, the snapshot must have valid FK
    let pose = snap
        .fk_result
        .pose(&ee_frame)
        .expect("end effector must exist after failed IK");
    let _pos = pose.translation();

    // Joints must all be finite (no NaN from failed IK)
    for &j in &snap.joints {
        assert!(j.is_finite(), "joint {} is not finite", j);
    }
}

// ─── PlanAndMoveJ (joint-space trajectory) ─────────────────────────

#[tokio::test]
async fn plan_and_movej_stores_trajectory_in_snapshot() {
    let (svc, _mgr) = make_service(RobotModel::Planar2R).await;
    let initial = svc.snapshot().await.unwrap();
    assert!(
        initial.active_plan.is_none(),
        "initial snapshot must not have active_plan",
    );

    let snap = svc
        .execute(Command::Motion(MotionCommands::PlanAndMoveJ {
            target: vec![1.0, 0.5],
            max_velocity: None,
            max_acceleration: None,
            time_step: None,
        }))
        .await
        .unwrap();

    let plan = snap
        .active_plan
        .as_ref()
        .expect("snapshot must have active_plan after PlanAndMoveJ");
    let traj = &plan.trajectory;
    assert!(
        traj.len() >= 2,
        "trajectory should have at least 2 waypoints, got {}",
        traj.len(),
    );

    let progress = snap
        .trajectory_progress()
        .expect("snapshot must have trajectory_progress");
    assert!(
        (0.0..=1.0).contains(&progress),
        "trajectory_progress must be in [0, 1], got {progress}",
    );
}

#[tokio::test]
async fn plan_and_movej_reaches_target_position() {
    let (svc, _mgr) = make_service(RobotModel::Planar2R).await;
    let target = vec![1.5, -0.8];

    let snap = svc
        .execute(Command::Motion(MotionCommands::PlanAndMoveJ {
            target: target.clone(),
            max_velocity: None,
            max_acceleration: None,
            time_step: None,
        }))
        .await
        .unwrap();

    assert_eq!(
        snap.joints, target,
        "joints must match the target after PlanAndMoveJ"
    );
}

#[tokio::test]
async fn plan_and_movej_trajectory_starts_at_initial_position() {
    let (svc, _mgr) = make_service(RobotModel::Planar2R).await;
    let initial = svc.snapshot().await.unwrap().joints;

    let snap = svc
        .execute(Command::Motion(MotionCommands::PlanAndMoveJ {
            target: vec![1.0, 0.5],
            max_velocity: None,
            max_acceleration: None,
            time_step: None,
        }))
        .await
        .unwrap();

    let plan = snap.active_plan.as_ref().unwrap();
    let first_waypoint = &plan.trajectory.waypoints()[0];

    assert_eq!(
        first_waypoint.joints(),
        &initial,
        "first waypoint must equal initial position",
    );
}

#[tokio::test]
async fn plan_and_movej_with_velocity_param() {
    let (svc, _mgr) = make_service(RobotModel::Planar2R).await;

    let snap = svc
        .execute(Command::Motion(MotionCommands::PlanAndMoveJ {
            target: vec![0.5, -0.3],
            max_velocity: Some(2.0),
            max_acceleration: Some(1.0),
            time_step: None,
        }))
        .await
        .unwrap();

    assert_eq!(snap.joints, vec![0.5, -0.3]);
    assert!(snap.active_plan.is_some());
}

// ─── PlanAndMoveL (cartesian → joint-space trajectory) ─────────────

#[tokio::test]
async fn plan_and_movel_stores_trajectory_in_snapshot() {
    let (svc, _mgr) = make_service(RobotModel::Planar2R).await;
    let snap0 = svc.snapshot().await.unwrap();
    let ee = *snap0.chain.end_effector();

    let target_pos = Vector3::new(0.3, 0.4, 0.0);
    let target_pose = Pose::new(
        FrameId::World,
        ee,
        Transform3D {
            translation: target_pos,
            rotation: UnitQuaternion::identity(),
        },
    );

    let snap = svc
        .execute(Command::Motion(MotionCommands::PlanAndMoveL {
            frame: ee,
            target_pose,
            max_velocity: None,
            max_acceleration: None,
            time_step: None,
            cartesian_step: None,
        }))
        .await
        .unwrap();

    let plan = snap
        .active_plan
        .as_ref()
        .expect("snapshot must have active_plan after PlanAndMoveL");
    let traj = &plan.trajectory;
    assert!(
        traj.len() >= 2,
        "trajectory should have at least 2 waypoints, got {}",
        traj.len(),
    );

    // Joints must be finite
    for &j in &snap.joints {
        assert!(j.is_finite(), "joint {} is not finite", j);
    }
}

#[tokio::test]
async fn snapshot_includes_trajectory_after_plan_command() {
    let (svc, _mgr) = make_service(RobotModel::Planar2R).await;

    let snap1 = svc.snapshot().await.unwrap();
    assert!(snap1.active_plan.is_none());

    svc.execute(Command::Motion(MotionCommands::PlanAndMoveJ {
        target: vec![0.8, -0.4],
        max_velocity: None,
        max_acceleration: None,
        time_step: None,
    }))
    .await
    .unwrap();

    let snap2 = svc.snapshot().await.unwrap();
    assert!(
        snap2.active_plan.is_some(),
        "snapshot() must include active_plan after planning command",
    );

    // Trajectory persists in subsequent snapshots until replaced
    let snap3 = svc.snapshot().await.unwrap();
    assert!(
        snap3.active_plan.is_some(),
        "plan must persist across snapshots",
    );
}

// ─── SelectToolFrame ─────────────────────────────────────────────

#[tokio::test]
async fn select_tool_frame_sets_active_tcp_in_snapshot() {
    use thalos_engine::core::robot::tool_frame::ToolFrame;

    let (svc, _mgr) = make_service(RobotModel::Scara).await;

    // Initial state: active_tcp is None
    let snap1 = svc.snapshot().await.unwrap();
    assert!(
        snap1.active_tcp.is_none(),
        "active_tcp should be None initially"
    );

    // Set a TCP with identity transform
    let tcp = ToolFrame::identity(ee(&snap1));
    let snap2 = svc
        .execute(Command::SelectToolFrame(Some(tcp)))
        .await
        .unwrap();

    assert!(
        snap2.active_tcp.is_some(),
        "active_tcp should be Some after SelectToolFrame"
    );
    let active_tcp = snap2.active_tcp.as_ref().unwrap();
    assert_eq!(
        active_tcp.base_frame,
        ee(&snap2),
        "TCP base_frame should match the requested frame"
    );
    assert!(
        !active_tcp.has_offset(),
        "TCP with identity transform should have no offset"
    );

    // Clear the TCP
    let snap3 = svc.execute(Command::SelectToolFrame(None)).await.unwrap();
    assert!(
        snap3.active_tcp.is_none(),
        "active_tcp should be None after clearing"
    );
}

#[tokio::test]
async fn select_tool_frame_with_offset_propagates_to_tick_delta() {
    use thalos_engine::core::robot::tool_frame::ToolFrame;

    let (svc, _mgr) = make_service(RobotModel::Scara).await;

    // Set a TCP with a 12cm offset below the flange
    let offset = Transform3D::from_translation(Vector3::new(0.0, 0.0, -0.12));
    let tcp = ToolFrame::with_offset(ee(&svc.snapshot().await.unwrap()), offset);
    svc.execute(Command::SelectToolFrame(Some(tcp)))
        .await
        .unwrap();

    // Verify TickDelta includes the active_tcp
    let delta = svc.tick_execution_delta(0.0).await.unwrap();
    assert!(
        delta.active_tcp.is_some(),
        "TickDelta should include active_tcp"
    );
    let active_tcp = delta.active_tcp.as_ref().unwrap();
    assert!(
        active_tcp.has_offset(),
        "TCP with non-identity transform should have offset"
    );
}

#[tokio::test]
async fn select_tool_frame_persists_across_multiple_commands() {
    use thalos_engine::core::robot::tool_frame::ToolFrame;

    let (svc, _mgr) = make_service(RobotModel::Scara).await;

    // Set a TCP
    let tcp = ToolFrame::identity(ee(&svc.snapshot().await.unwrap()));
    svc.execute(Command::SelectToolFrame(Some(tcp)))
        .await
        .unwrap();

    // Execute other commands — TCP should persist
    svc.execute(Command::SetJoints(vec![0.5, -0.3, 0.1, 0.0]))
        .await
        .unwrap();
    let snap = svc.snapshot().await.unwrap();
    assert!(
        snap.active_tcp.is_some(),
        "active_tcp should persist after SetJoints"
    );
}

#[tokio::test]
async fn select_tool_frame_clears_on_robot_change() {
    use thalos_engine::core::robot::tool_frame::ToolFrame;

    let (svc, _mgr) = make_service(RobotModel::Scara).await;

    // Set a TCP
    let tcp = ToolFrame::identity(ee(&svc.snapshot().await.unwrap()));
    svc.execute(Command::SelectToolFrame(Some(tcp)))
        .await
        .unwrap();

    let snap1 = svc.snapshot().await.unwrap();
    assert!(snap1.active_tcp.is_some(), "active_tcp should be set");

    // Load a different robot — TCP should be cleared
    svc.execute(Command::LoadRobot(RobotModel::Planar3R))
        .await
        .unwrap();
    let snap2 = svc.snapshot().await.unwrap();
    assert!(
        snap2.active_tcp.is_none(),
        "active_tcp should be cleared after LoadRobot"
    );

    // Set TCP again and load URDF robot — TCP should be cleared again
    let tcp2 = ToolFrame::identity(ee(&snap2));
    svc.execute(Command::SelectToolFrame(Some(tcp2)))
        .await
        .unwrap();
    let snap3 = svc.snapshot().await.unwrap();
    assert!(snap3.active_tcp.is_some(), "active_tcp should be set again");

    let urdf = include_str!("../../../../thalos-models/tests/fixtures/scara.urdf");
    let robot = thalos_importer::import_urdf(urdf).unwrap();
    let joints_meta: Vec<crate::snapshots::scene::JointMeta> = robot
        .bfs_joints()
        .unwrap_or_default()
        .iter()
        .filter(|j| !j.kind.is_fixed())
        .map(|j| crate::snapshots::scene::JointMeta {
            name: j.name.clone(),
            kind: j.kind.to_string(),
            min: j.limits.map(|l| l.min),
            max: j.limits.map(|l| l.max),
        })
        .collect();
    svc.execute(Command::LoadUrdfRobot {
        name: "urdf_scara".to_string(),
        joints_meta,
        chain: thalos_engine::core::robot::adapter::from_urdf(urdf).unwrap(),
        robot,
        robot_id: "urdf:abcdef123456".to_string(),
    })
    .await
    .unwrap();
    let snap4 = svc.snapshot().await.unwrap();
    assert!(
        snap4.active_tcp.is_none(),
        "active_tcp should be cleared after LoadUrdfRobot"
    );
    // Real metadata (previously `vec![]`) — the DTO mapper derives the
    // "urdf" robot id from non-empty joints_meta; an empty vec masked the
    // fallback to built-in metadata.
    assert_eq!(
        snap4.chain.dof_count(),
        4,
        "loaded chain must keep its 4-DOF kinematics in the snapshot"
    );
    assert_eq!(
        snap4.joints_meta.len(),
        4,
        "URDF joints_meta must carry the 4 actuated joints"
    );
}

/// Spec: unified-kinematics "RobotModel Is Catalog Membership" + "Snapshot
/// provides chain and joints atomically". A URDF-loaded robot must keep its
/// real chain in the snapshot: DOF, joints, and joints_meta all derive from
/// the loaded chain, and `robot` is `None` (identity carried by
/// `robot_name`/`robot_source`/`joints_meta`/`chain`).
#[tokio::test]
async fn load_urdf_keeps_real_chain_in_snapshot() {
    let (svc, _mgr) = make_service(RobotModel::Scara).await;
    let urdf = include_str!("../../../../thalos-models/tests/fixtures/scara.urdf");
    let robot = thalos_importer::import_urdf(urdf).unwrap();
    let joints_meta: Vec<crate::snapshots::scene::JointMeta> = robot
        .bfs_joints()
        .unwrap_or_default()
        .iter()
        .filter(|j| !j.kind.is_fixed())
        .map(|j| crate::snapshots::scene::JointMeta {
            name: j.name.clone(),
            kind: j.kind.to_string(),
            min: j.limits.map(|l| l.min),
            max: j.limits.map(|l| l.max),
        })
        .collect();

    let snap = svc
        .execute(Command::LoadUrdfRobot {
            name: "urdf_scara".to_string(),
            joints_meta: joints_meta.clone(),
            chain: thalos_engine::core::robot::adapter::from_urdf(urdf).unwrap(),
            robot,
            robot_id: "urdf:abcdef123456".to_string(),
        })
        .await
        .unwrap();

    assert_eq!(
        snap.chain.dof_count(),
        4,
        "snapshot must carry the loaded chain DOF"
    );
    assert_eq!(
        snap.joints.len(),
        4,
        "joints must match the loaded chain DOF"
    );
    assert!(
        snap.robot.is_none(),
        "URDF-loaded robot must carry None model (catalog membership), not a fabricated RobotModel"
    );
    assert_eq!(
        snap.joints_meta.len(),
        4,
        "joints_meta must carry the real actuated metadata"
    );
}

#[tokio::test]
async fn select_tool_frame_rejects_invalid_frame() {
    use thalos_engine::core::robot::tool_frame::ToolFrame;
    use thalos_engine::core::spatial::frame::FrameId;

    let (svc, _mgr) = make_service(RobotModel::Scara).await;

    // Try to set a TCP with a non-existent frame ID
    let invalid_tcp = ToolFrame::identity(FrameId::Id(99999));
    let result = svc
        .execute(Command::SelectToolFrame(Some(invalid_tcp)))
        .await;

    assert!(
        result.is_err(),
        "SelectToolFrame should reject invalid frame ID"
    );

    // Verify the error type
    match result {
        Err(crate::RuntimeError::ToolFrameNotFound { frame_id }) => {
            assert_eq!(frame_id, 99999, "error should contain the invalid frame ID");
        }
        Err(other) => panic!("expected ToolFrameNotFound error, got {:?}", other),
        Ok(_) => panic!("expected error, got Ok"),
    }

    // Verify TCP was not set
    let snap = svc.snapshot().await.unwrap();
    assert!(
        snap.active_tcp.is_none(),
        "active_tcp should remain None after invalid selection"
    );
}

// ═════════════════════════════════════════════════════════════════════
// PR 3 — Runtime event dispatch through the scene tick loop
// ═════════════════════════════════════════════════════════════════════

use std::time::Duration;
use thalos_engine::core::{
    execution::runtime::{RuntimeAction, RuntimeEvent, RuntimeProgram},
    ids::OperationId,
    motion::target::{OutputChannel, OutputValue},
    trajectory::TrajectoryPoint,
};
use thalos_engine::planning::motion::program::CompiledPlan;

/// Schedule a program with a SetOutput at t=1.0s, start execution, and
/// verify the tick loop dispatches it at exactly clock 1.0s (rt).
#[tokio::test]
async fn scheduled_runtime_events_dispatch_via_tick() {
    // Concrete controller so the test can observe dispatched events.
    let concrete = Arc::new(RwLock::new(SimulationController::new(
        RobotModel::Scara.metadata().dof,
    )));
    let controller = concrete.clone() as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(
        manager.clone(),
        RobotModel::Scara,
    );

    // A trivial 2-waypoint 4-DOF plan over 2.0s.
    let plan = CompiledPlan::new(
        thalos_engine::core::trajectory::Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, -0.3, 0.1, 0.0], 2.0),
        ]),
        vec![],
    );
    let runtime = RuntimeProgram::new(vec![RuntimeEvent {
        at_time: Duration::from_secs_f64(1.0),
        operation_id: OperationId("op-out".to_string()),
        action: RuntimeAction::SetOutput {
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        },
    }]);

    svc.schedule_program(plan, runtime).await.unwrap();
    svc.start_execution().await.unwrap();

    // clock 0.5s — nothing dispatched yet.
    svc.tick_execution_delta(0.5).await.unwrap();
    assert!(concrete.read().await.dispatched_events().await.is_empty());

    // clock 1.0s — the SetOutput fires at its absolute at_time.
    svc.tick_execution_delta(0.5).await.unwrap();
    let dispatched = concrete.read().await.dispatched_events().await;
    assert_eq!(dispatched.len(), 1, "SetOutput dispatched via tick");
    assert_eq!(
        dispatched[0].operation_id,
        OperationId("op-out".to_string())
    );
    assert_eq!(
        concrete.read().await.clock_time().await,
        Duration::from_secs_f64(1.0)
    );
}

/// A Delay scheduled into the scene freezes the robot while the clock
/// advances, then the trajectory resumes (rt — spec Delay semantics).
#[tokio::test]
async fn scheduled_delay_freezes_execution_through_tick() {
    let concrete = Arc::new(RwLock::new(SimulationController::new(
        RobotModel::Scara.metadata().dof,
    )));
    let controller = concrete.clone() as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(
        manager.clone(),
        RobotModel::Scara,
    );

    let plan = CompiledPlan::new(
        thalos_engine::core::trajectory::Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 0.0, 0.0, 0.0], 2.0),
        ]),
        vec![],
    );
    // Delay at 1.0s for 500ms: robot holds from 1.0s to 1.5s.
    let runtime = RuntimeProgram::new(vec![RuntimeEvent {
        at_time: Duration::from_secs_f64(1.0),
        operation_id: OperationId("op-wait".to_string()),
        action: RuntimeAction::Delay(Duration::from_millis(500)),
    }]);

    svc.schedule_program(plan, runtime).await.unwrap();
    svc.start_execution().await.unwrap();

    // Reach clock 1.0s (4 × 0.25s): joint[0] = 0.5 (linear over 2s).
    for _ in 0..4 {
        svc.tick_execution_delta(0.25).await.unwrap();
    }
    let held = svc.snapshot().await.unwrap().joints[0];
    assert!(
        (held - 0.5).abs() < 1e-9,
        "joint[0] at delay start = {held}"
    );

    // clock 1.25s — inside the delay: clock advances, robot holds.
    svc.tick_execution_delta(0.25).await.unwrap();
    assert_eq!(
        concrete.read().await.clock_time().await,
        Duration::from_secs_f64(1.25)
    );
    let still = svc.snapshot().await.unwrap().joints[0];
    assert!(
        (still - held).abs() < 1e-9,
        "robot must hold during delay (joint[0] = {still})"
    );
    assert_eq!(
        concrete.read().await.traj_time().await,
        Duration::from_secs_f64(1.0),
        "traj time frozen during delay"
    );

    // clock 1.5s — delay elapsed: trajectory resumes.
    svc.tick_execution_delta(0.25).await.unwrap();
    let resumed = svc.snapshot().await.unwrap().joints[0];
    assert!(
        (resumed - 0.625).abs() < 1e-9,
        "trajectory resumes from held state (joint[0] = {resumed})"
    );
}

// ═════════════════════════════════════════════════════════════════════
// esp32-execute-real-timestamps — build_execution_plan (2.1/2.2)
// ═════════════════════════════════════════════════════════════════════
//
// The scene MUST hand the controller an `ExecutionPlan` carrying the REAL
// trajectory timestamps (via ExecutionPlanBuilder for scheduled_plan, inline
// for active_plan) — never the even-spacing reconstruction the legacy
// `trajectory_to_waypoints` shim produced. And `start_execution_with_mode`
// MUST skip `execute` for empty/zero-duration plans while still registering
// the session (Once-without-plan preserved).

use std::sync::atomic::Ordering;

/// 2.1 RED: `build_execution_plan` keeps the scheduled_plan's real
/// timestamps — a non-uniform trajectory (0.0, 0.5, 2.0) must reach the
/// controller as-is, NOT re-spaced even (0.0, 1.0, 2.0) by the legacy shim.
#[tokio::test]
async fn start_execution_preserves_scheduled_plan_timestamps() {
    let mut mock = MockController::new();
    let concrete = Arc::new(RwLock::new(mock));
    let controller = concrete.clone() as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(manager.clone(), RobotModel::Scara);

    // NON-UNIFORM timestamps: 0.0 → 0.5 → 2.0. Even-spacing would yield
    // 0.0 → 1.0 → 2.0 (2.0s / 2 gaps) — the false-positive bug source.
    let plan = CompiledPlan::new(
        thalos_engine::core::trajectory::Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, -0.3, 0.1, 0.0], 0.5),
            TrajectoryPoint::new(vec![1.0, -0.6, 0.2, 0.0], 2.0),
        ]),
        vec![],
    );
    svc.schedule_program(plan, Default::default()).await.unwrap();
    svc.start_execution().await.unwrap();

    let received = concrete
        .read()
        .await
        .last_plan
        .lock()
        .unwrap()
        .clone()
        .expect("execute must have received an ExecutionPlan");
    let ts: Vec<f64> = received.waypoints.iter().map(|w| w.timestamp).collect();
    assert_eq!(
        ts,
        vec![0.0, 0.5, 2.0],
        "the controller must receive the REAL (non-uniform) timestamps"
    );
    assert_eq!(received.duration, 2.0);
    // The waypoints themselves flow through untouched.
    assert_eq!(received.waypoints[1].joints, vec![0.5, -0.3, 0.1, 0.0]);
}

/// 2.1 RED: `build_execution_plan` maps the active_plan INLINE — single-shot
/// PlanAndMoveJ sets only `active_plan` (no scheduled_plan), so the scene
/// must build the plan from the active trajectory, preserving timestamps and
/// emitting a single MoveJ segment covering every waypoint.
#[tokio::test]
async fn start_execution_maps_active_plan_inline_with_segments() {
    let mut mock = MockController::new();
    let concrete = Arc::new(RwLock::new(mock));
    let controller = concrete.clone() as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(manager.clone(), RobotModel::Planar2R);

    // Single-shot move: only `active_plan` is set (segments = None).
    let snap = svc
        .execute(Command::Motion(MotionCommands::PlanAndMoveJ {
            target: vec![1.0, 0.5],
            max_velocity: None,
            max_acceleration: None,
            time_step: None,
        }))
        .await
        .unwrap();
    let active = snap
        .active_plan
        .as_ref()
        .expect("PlanAndMoveJ must set the active plan");
    assert!(
        active.segments.is_none(),
        "single-shot move has no segment metadata"
    );
    let traj = &active.trajectory;

    svc.start_execution().await.unwrap();

    let received = concrete
        .read()
        .await
        .last_plan
        .lock()
        .unwrap()
        .clone()
        .expect("execute must have received an ExecutionPlan");
    // Inline mapping preserves the active trajectory's real timestamps.
    let n = traj.len();
    assert_eq!(received.waypoints.len(), n);
    for (wp, tp) in received.waypoints.iter().zip(traj.waypoints()) {
        assert_eq!(wp.joints, tp.joints().to_vec());
        assert_eq!(wp.timestamp, tp.timestamp());
    }
    // No segment metadata → a single MoveJ segment over all waypoints.
    assert_eq!(received.segments.len(), 1, "fallback single segment");
    assert_eq!(
        received.segments[0].instruction,
        thalos_engine::core::execution::plan::PlanInstruction::MoveJ
    );
    assert_eq!(received.segments[0].waypoint_range, 0..n);
    assert_eq!(received.duration, traj.duration());
}

/// 2.2 RED: an EMPTY plan (zero waypoints, zero duration) MUST NOT reach
/// the controller's `execute` — no wire traffic — but the session is still
/// registered (Once-without-plan behavior preserved).
#[tokio::test]
async fn start_execution_skips_execute_for_empty_plan_but_registers_session() {
    let mut mock = MockController::new();
    let concrete = Arc::new(RwLock::new(mock));
    let controller = concrete.clone() as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(manager.clone(), RobotModel::Scara);

    // Zero waypoints, zero duration — the has_wps guard must skip execute.
    let plan = CompiledPlan::new(thalos_engine::core::trajectory::Trajectory::new(vec![]), vec![]);
    svc.schedule_program(plan, Default::default()).await.unwrap();

    let snap = svc.start_execution().await.unwrap();
    assert_eq!(
        concrete.read().await.execute_count.load(Ordering::SeqCst),
        0,
        "empty plan must NOT call controller execute"
    );
    assert!(
        snap.execution.is_some(),
        "session must still be registered for an empty plan"
    );
}

/// 2.2 RED: `Once` without ANY plan still succeeds (legacy behavior) and
/// never calls `execute` — the session registers with zero motion.
#[tokio::test]
async fn start_execution_once_without_plan_still_succeeds() {
    let mut mock = MockController::new();
    let concrete = Arc::new(RwLock::new(mock));
    let controller = concrete.clone() as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(manager.clone(), RobotModel::Scara);

    // No plan at all.
    let snap = svc.start_execution().await.unwrap();
    assert!(
        snap.execution.is_some(),
        "Once-without-plan must still register a session"
    );
    assert_eq!(
        concrete.read().await.execute_count.load(Ordering::SeqCst),
        0,
        "no plan → no execute call"
    );
}

/// R4-001: the execution source must reflect the ACTIVE controller — a
/// non-simulation controller (Hardware) reports Hardware on the snapshot's
/// execution session, not the hardcoded Simulation.
#[tokio::test]
async fn start_execution_reports_active_controller_source() {
    let mut mock = MockController::new();
    mock.source = ExecutionSource::Hardware;
    let controller = Arc::new(RwLock::new(mock)) as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(
        manager.clone(),
        RobotModel::Scara,
    );

    let snap = svc.start_execution().await.unwrap();
    let exe = snap
        .execution
        .expect("start_execution must report an execution session");
    assert_eq!(exe.source, ExecutionSource::Hardware);
}

/// R4-001: `start_execution` MUST propagate a `ConnectionLost` failure from
/// the controller's `execute` instead of swallowing it — the code has to
/// reach the API so the frontend can offer the Reconectar CTA.
#[tokio::test]
async fn start_execution_propagates_connection_lost_from_controller() {
    let mut mock = MockController::new();
    mock.execute_error = Some(crate::error::ControllerError::ConnectionLost);
    let controller = Arc::new(RwLock::new(mock)) as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(
        manager.clone(),
        RobotModel::Scara,
    );

    // A real scheduled plan so `execute` is actually invoked.
    svc.schedule_program(compiled_plan(1.0), Default::default())
        .await
        .unwrap();

    let err = match svc.start_execution().await {
        Err(e) => e,
        Ok(_) => panic!("start_execution must fail when the controller reports ConnectionLost"),
    };
    match err {
        crate::RuntimeError::ControllerFailed { source } => {
            assert_eq!(source, crate::error::ControllerError::ConnectionLost);
        }
        other => panic!("expected ControllerFailed(ConnectionLost), got {other:?}"),
    }
}

/// R4-001: `tick_execution_delta` must NOT swallow a `ConnectionLost` from
/// the controller's `advance` — it propagates as an execution failure so the
/// frontend tick loop marks the session failed with `connection_lost`.
#[tokio::test]
async fn tick_propagates_connection_lost_from_advance() {
    let mut mock = MockController::new();
    mock.advance_error = Some(crate::error::ControllerError::ConnectionLost);
    let controller = Arc::new(RwLock::new(mock)) as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(
        manager.clone(),
        RobotModel::Scara,
    );

    let err = match svc.tick_execution_delta(0.01).await {
        Err(e) => e,
        Ok(_) => panic!("tick must fail when the controller reports ConnectionLost"),
    };
    match err {
        crate::RuntimeError::ControllerFailed { source } => {
            assert_eq!(source, crate::error::ControllerError::ConnectionLost);
        }
        other => panic!("expected ControllerFailed(ConnectionLost), got {other:?}"),
    }
}

/// R4-001: a hardware backend's default `advance` (`UnsupportedCapability` —
/// time is real) must NOT fail the tick; it is the normal, ignorable case.
#[tokio::test]
async fn tick_ignores_unsupported_capability_from_advance() {
    let mut mock = MockController::new();
    mock.advance_error = Some(crate::error::ControllerError::UnsupportedCapability);
    let controller = Arc::new(RwLock::new(mock)) as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    let svc = SceneService::new(
        manager.clone(),
        RobotModel::Scara,
    );

    let delta = svc
        .tick_execution_delta(0.01)
        .await
        .expect("UnsupportedCapability from advance must not fail the tick");
    assert!(
        delta.execution.is_none() || delta.execution.as_ref().is_some_and(|e| !e.status.is_terminal()),
        "tick must return a normal delta for hardware backends"
    );
}

/// R3-001: `start_execution` with NO active controller (hardware backend
/// activated but never connected) must fail EXPLICITLY with `not_connected` —
/// never return a silent 200 snapshot.
#[tokio::test]
async fn start_execution_without_controller_returns_not_connected() {
    let manager = Arc::new(BackendManager::new());
    manager
        .register(crate::backends::manager::BackendEntry {
            id: "esp32".into(),
            name: "Hardware (ESP32)".into(),
            controller: None,
            port: Some("/dev/ttyUSB0".into()),
        })
        .await;

    manager.activate("esp32").await.unwrap();
    assert!(
        manager.get_controller().await.is_none(),
        "esp32 active-but-not-connected has no controller"
    );

    let svc = SceneService::new(
        manager.clone(),
        RobotModel::Scara,
    );

    let err = match svc.start_execution().await {
        Err(e) => e,
        Ok(_) => panic!("start_execution must NOT return a silent 200 without a controller"),
    };
    match err {
        crate::RuntimeError::ControllerFailed { source } => {
            assert_eq!(source, crate::error::ControllerError::NotConnected);
            assert_eq!(source.error_code(), "not_connected");
        }
        other => panic!("expected ControllerFailed(NotConnected), got {other:?}"),
    }
}

/// S3.6: on a completed tick, the scene drains the controller's hardware
/// execution trace (SAMPLES) and persists it as an `ExecutionTrace` JSON with
/// µs-derived timestamps — even for a hardware source that reports progress
/// in SECONDS (recording timestamp must use seconds directly, not
/// fraction × plan_duration).
#[tokio::test]
async fn hardware_execution_trace_is_persisted_on_completion() {
    use crate::execution_boundary::ExecutionSample;
    use crate::session::SessionManager;

    let mut mock = MockController::new();
    mock.source = ExecutionSource::Hardware;
    // RISK-1 gate: hardware completion fires on SECONDS progress reaching
    // plan_duration (floor 1.0) — report a completing state, not the default
    // Idle (which would never finalize a hardware session mid-run).
    let mut done = RobotState::default();
    done.motion.mode = MotionMode::Moving;
    done.execution.progress = 2.0;
    mock.state = Some(done);
    mock.execution_trace = Some(vec![
        ExecutionSample {
            timestamp_us: 0,
            joints: vec![0.1, 0.2],
        },
        ExecutionSample {
            timestamp_us: 1_000_000,
            joints: vec![0.5, 0.3],
        },
    ]);
    let controller = Arc::new(RwLock::new(mock)) as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();

    // Dedicated session store in a temp dir — do not pollute ~/.thls.
    let dir = std::env::temp_dir().join(format!("thalos-scene-hw-trace-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let sessions = Arc::new(SessionManager::with_path(dir.clone()));
    let svc = SceneService::with_session_manager(
        manager.clone(),
        RobotModel::Scara,
        sessions.clone(),
    );

    svc.start_execution().await.unwrap();
    // The mock reports a completing hardware state → completion detected on
    // the first tick, which must drain + persist the hardware trace.
    svc.tick_execution_delta(0.1).await.unwrap();

    let trace = sessions
        .get_execution_trace(1)
        .await
        .expect("hardware execution trace must be persisted");
    assert_eq!(trace.samples.len(), 2);
    assert_eq!(trace.samples[0].timestamp, Duration::from_micros(0));
    assert_eq!(trace.samples[1].timestamp, Duration::from_micros(1_000_000));
    assert_eq!(trace.samples[0].joints, vec![0.1, 0.2]);
    assert!(trace.samples[0].velocities.is_empty());
    assert_eq!(trace.metadata.source, ExecutionSource::Hardware);

    let _ = std::fs::remove_dir_all(&dir);
}

// ═════════════════════════════════════════════════════════════════════
// Review correction — RISK-1 / REL-01 + REL-03 / RES-06 (completion gate)
// ═════════════════════════════════════════════════════════════════════

/// RISK-1 / REL-01 (RED): for a HARDWARE source, `execution.progress` is in
/// SECONDS. A Moving state with seconds-progress >= 1.0 but < plan_duration
/// must NOT finalize the session mid-run — the old fraction gate
/// (`progress >= 1.0`) did exactly that on any plan > 1s, and the trace was
/// then drained-and-dropped at true completion. The hardware trace must be
/// persisted on the TRUE-completion tick (progress == plan_duration).
#[tokio::test]
async fn hardware_running_seconds_progress_below_plan_duration_does_not_finalize() {
    use crate::execution_boundary::ExecutionSample;
    use crate::session::SessionManager;

    let mut mock = MockController::new();
    mock.source = ExecutionSource::Hardware;
    let mut running = RobotState::default();
    running.motion.mode = MotionMode::Moving;
    running.execution.progress = 1.2; // seconds — >= 1.0 but < 2.0s plan
    mock.state = Some(running);
    mock.execution_trace = Some(vec![ExecutionSample {
        timestamp_us: 0,
        joints: vec![0.1, 0.2],
    }]);

    let concrete = Arc::new(RwLock::new(mock));
    let controller = concrete.clone() as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();

    let dir = std::env::temp_dir().join(format!("thalos-scene-hw-gate-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let sessions = Arc::new(SessionManager::with_path(dir.clone()));
    let svc = SceneService::with_session_manager(
        manager.clone(),
        RobotModel::Scara,
        sessions.clone(),
    );

    // 2.0s plan → plan_duration = 2.0; hardware progress is seconds.
    let plan = CompiledPlan::new(
        thalos_engine::core::trajectory::Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, -0.3, 0.1, 0.0], 2.0),
        ]),
        vec![],
    );
    svc.schedule_program(plan, Default::default()).await.unwrap();
    svc.start_execution().await.unwrap();

    // Tick with 1.2s progress (>= 1.0, < 2.0s plan): MUST NOT finalize.
    svc.tick_execution_delta(0.1).await.unwrap();
    let session = sessions.get(1).await.expect("session registered");
    assert_eq!(
        session.status,
        crate::plan::SessionStatus::Running,
        "mid-run seconds progress must NOT finalize the session"
    );
    assert!(
        sessions.get_execution_trace(1).await.is_none(),
        "hardware trace must not be drained before true completion"
    );

    // True completion: progress reaches plan_duration → finalize + persist.
    let mut done = RobotState::default();
    done.motion.mode = MotionMode::Moving;
    done.execution.progress = 2.0;
    concrete.write().await.state = Some(done);
    svc.tick_execution_delta(0.1).await.unwrap();
    let session = sessions.get(1).await.expect("session registered");
    assert_eq!(
        session.status,
        crate::plan::SessionStatus::Completed,
        "completion at plan_duration must finalize"
    );
    let trace = sessions
        .get_execution_trace(1)
        .await
        .expect("hardware trace persisted on the true-completion tick");
    assert_eq!(trace.samples.len(), 1);
    assert_eq!(trace.samples[0].joints, vec![0.1, 0.2]);
    assert_eq!(trace.metadata.source, ExecutionSource::Hardware);

    let _ = std::fs::remove_dir_all(&dir);
}

/// REL-03 / RES-06 (RED): an EStop state must be a TERMINAL Failed
/// finalization in the tick — not leave the session Running forever.
#[tokio::test]
async fn estop_state_finalizes_session_as_failed() {
    use crate::session::SessionManager;

    let mut mock = MockController::new();
    mock.source = ExecutionSource::Hardware;
    let mut stopped = RobotState::default();
    stopped.motion.mode = MotionMode::EStop;
    mock.state = Some(stopped);

    let concrete = Arc::new(RwLock::new(mock));
    let controller = concrete.clone() as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();

    let dir = std::env::temp_dir().join(format!("thalos-scene-estop-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let sessions = Arc::new(SessionManager::with_path(dir.clone()));
    let svc = SceneService::with_session_manager(
        manager.clone(),
        RobotModel::Scara,
        sessions.clone(),
    );

    let plan = CompiledPlan::new(
        thalos_engine::core::trajectory::Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, -0.3, 0.1, 0.0], 2.0),
        ]),
        vec![],
    );
    svc.schedule_program(plan, Default::default()).await.unwrap();
    svc.start_execution().await.unwrap();

    svc.tick_execution_delta(0.1).await.unwrap();
    let session = sessions.get(1).await.expect("session registered");
    assert_eq!(
        session.status,
        crate::plan::SessionStatus::Failed,
        "EStop must finalize the session as Failed"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

// ═════════════════════════════════════════════════════════════════════
// Execution Mode Repeat — orchestration gate (S1-S3, S10 / R3-R6, R11-R12)
// ═════════════════════════════════════════════════════════════════════

/// A completing hardware state: seconds-progress >= the 2.0s plan duration.
fn repeat_done_state() -> RobotState {
    let mut s = RobotState::default();
    s.motion.mode = MotionMode::Moving;
    s.execution.progress = 2.0;
    s
}

/// A mid-run hardware state: seconds-progress below the plan duration.
fn repeat_running_state() -> RobotState {
    let mut s = RobotState::default();
    s.motion.mode = MotionMode::Moving;
    s.execution.progress = 0.5;
    s
}

/// A 2.0s Scara plan — the standard fixture for the repeat tests.
fn repeat_plan() -> thalos_engine::planning::motion::program::CompiledPlan {
    CompiledPlan::new(
        thalos_engine::core::trajectory::Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, -0.3, 0.1, 0.0], 2.0),
        ]),
        vec![],
    )
}

/// Build a SceneService with a Hardware-source MockController whose state the
/// test drives tick-by-tick, plus a temp-dir SessionManager.
async fn repeat_service(
    mock: MockController,
) -> (
    SceneService,
    Arc<RwLock<MockController>>,
    Arc<SessionManager>,
    std::path::PathBuf,
) {
    let concrete = Arc::new(RwLock::new(mock));
    let controller = concrete.clone() as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();

    let dir = std::env::temp_dir().join(format!(
        "thalos-scene-repeat-{}-{}",
        std::process::id(),
        rand_suffix()
    ));
    let _ = std::fs::remove_dir_all(&dir);
    let sessions = Arc::new(SessionManager::with_path(dir.clone()));
    let svc = SceneService::with_session_manager(
        manager.clone(),
        RobotModel::Scara,
        sessions.clone(),
    );
    (svc, concrete, sessions, dir)
}

fn rand_suffix() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

/// Wait until the async repeat re-execute (B) has landed — i.e. the recording
/// phase returned to `Idle`. Without this, tests that drive the controller
/// state tick-by-tick would race the background upload task.
async fn wait_for_phase(svc: &SceneService, phase: RepeatPhase) {
    for _ in 0..200 {
        if svc.recording_repeat_phase().await == Some(phase) {
            return;
        }
        tokio::time::sleep(std::time::Duration::from_millis(2)).await;
    }
    panic!("repeat phase did not reach {phase:?} within timeout");
}

/// RobotController wrapper that parks the 2nd+ `execute` until released —
/// makes the repeat upload window (B) deterministically observable.
struct BlockingExecuteController {
    inner: MockController,
    /// Test-driven state, like `MockController.state`.
    state: Option<RobotState>,
    /// While true, `execute` parks until `release` fires.
    blocking: AtomicBool,
    /// Notified while a parking execute is in flight (upload window active).
    blocked: std::sync::Arc<tokio::sync::Notify>,
    /// `notify_one` releases a parking execute.
    release: std::sync::Arc<tokio::sync::Notify>,
}

/// RobotController wrapper that fails the Nth `execute` call deterministically.
struct FailOnExecuteN {
    inner: MockController,
    /// Test-driven state, like `MockController.state`.
    state: Option<RobotState>,
    fail_on: u32,
    calls: AtomicU32,
}

macro_rules! delegate_controller {
    ($t:ty) => {
        #[async_trait]
        impl RobotController for $t {
            async fn connect(&mut self) -> Result<(), ControllerError> {
                self.inner.connect().await
            }
            async fn disconnect(&mut self) -> Result<(), ControllerError> {
                self.inner.disconnect().await
            }
            fn is_connected(&self) -> bool {
                self.inner.is_connected()
            }
            async fn execute(&mut self, plan: ExecutionPlan) -> Result<(), ControllerError> {
                self.custom_execute(plan).await
            }
            async fn stop(&mut self) -> Result<(), ControllerError> {
                self.inner.stop().await
            }
            async fn pause(&mut self) -> Result<(), ControllerError> {
                self.inner.pause().await
            }
            async fn resume(&mut self) -> Result<(), ControllerError> {
                self.inner.resume().await
            }
            async fn advance(&self, dt: f64) -> Result<(), ControllerError> {
                self.inner.advance(dt).await
            }
            async fn robot_state(&self) -> Arc<RobotState> {
                Arc::new(self.state.clone().unwrap_or_default())
            }
            async fn take_execution_trace(&self) -> Option<Vec<ExecutionSample>> {
                self.inner.take_execution_trace().await
            }
            fn capabilities(&self) -> BackendCapabilities {
                self.inner.capabilities()
            }
            fn execution_source(&self) -> ExecutionSource {
                self.inner.execution_source()
            }
        }
    };
}

delegate_controller!(BlockingExecuteController);

impl BlockingExecuteController {
    /// Inherent hook the trait impl delegates to (the macro cannot split a
    /// trait impl across blocks).
    async fn custom_execute(&mut self, plan: ExecutionPlan) -> Result<(), ControllerError> {
        if self.blocking.load(std::sync::atomic::Ordering::SeqCst) {
            self.blocked.notify_one();
            self.release.notified().await;
        }
        self.inner.execute(plan).await
    }
}

delegate_controller!(FailOnExecuteN);

impl FailOnExecuteN {
    async fn custom_execute(&mut self, plan: ExecutionPlan) -> Result<(), ControllerError> {
        let n = self.calls.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1;
        if n == self.fail_on {
            return Err(ControllerError::Protocol(
                "simulated re-upload failure".into(),
            ));
        }
        self.inner.execute(plan).await
    }
}

/// Build a SceneService + temp-dir SessionManager around a custom controller
/// (used by the B race/failure tests — `repeat_service` stays for the plain
/// MockController suites).
async fn repeat_service_custom<T: RobotController + Send + Sync + 'static>(
    controller: Arc<RwLock<T>>,
) -> (SceneService, Arc<SessionManager>, std::path::PathBuf) {
    let manager = Arc::new(BackendManager::new());
    let trait_obj = controller as Arc<RwLock<dyn RobotController + Send + Sync>>;
    manager.set_active(trait_obj).await.unwrap();

    let dir = std::env::temp_dir().join(format!(
        "thalos-scene-custom-{}-{}",
        std::process::id(),
        rand_suffix()
    ));
    let _ = std::fs::remove_dir_all(&dir);
    let sessions = Arc::new(SessionManager::with_path(dir.clone()));
    let svc = SceneService::with_session_manager(manager, RobotModel::Scara, sessions.clone());
    (svc, sessions, dir)
}

/// S1 (R4, R6, NF3): `Repeat { count: 5 }` completes 5 iterations — the gate
/// re-executes the plan after each intermediate completion and finalizes ONLY
/// at iteration == total, writing exactly one MotionTrace and draining the
/// hardware execution trace exactly once (final iteration).
#[tokio::test]
async fn repeat_five_completes_five_iterations_with_single_trace() {
    use crate::execution_boundary::ExecutionSample;
    use std::sync::atomic::Ordering;

    let mut mock = MockController::new();
    mock.source = ExecutionSource::Hardware;
    mock.execution_trace = Some(vec![ExecutionSample {
        timestamp_us: 1_000_000,
        joints: vec![0.1, 0.2],
    }]);

    let (svc, concrete, sessions, dir) = repeat_service(mock).await;
    svc.schedule_program(repeat_plan(), Default::default())
        .await
        .unwrap();
    svc.start_execution_with_mode(crate::plan::ExecutionMode::Repeat { count: 5 })
        .await
        .unwrap();

    for iteration in 1..=5 {
        // Completion tick: iteration `iteration` finishes → the gate runs.
        concrete.write().await.state = Some(repeat_done_state());
        svc.tick_execution_delta(0.1).await.unwrap();
        if iteration < 5 {
            // B: the re-execute is now a BACKGROUND task — wait for its upload
            // to land (phase back to Idle) before driving the next iteration.
            wait_for_phase(&svc, RepeatPhase::Idle).await;
            // The gate re-executes the plan for the next iteration; the
            // controller reports a fresh run until the next completion.
            concrete.write().await.state = Some(repeat_running_state());
            svc.tick_execution_delta(0.1).await.unwrap();
        }
    }

    let session = sessions
        .get(1)
        .await
        .expect("session registered at start");
    assert_eq!(
        session.status,
        crate::plan::SessionStatus::Completed,
        "S1: session must be Completed after 5 iterations"
    );
    assert_eq!(session.iteration, 5, "R4: iteration == total_iterations");
    assert_eq!(session.total_iterations, Some(5));
    assert_eq!(
        concrete.read().await.execute_count.load(Ordering::SeqCst),
        5,
        "S1: 1 initial execute + 4 re-executes = 5"
    );

    // R6/NF3: exactly ONE motion trace for the session…
    let trace = sessions
        .get_trace(1)
        .await
        .expect("single MotionTrace must be stored");
    assert!(!trace.samples().is_empty(), "recorder accumulated samples");

    // …and the hardware execution trace was drained exactly once (final tick).
    assert_eq!(
        concrete.read().await.take_trace_calls.load(Ordering::SeqCst),
        1,
        "NF3: hardware execution trace drained exactly once (final iteration)"
    );
    let et = sessions
        .get_execution_trace(1)
        .await
        .expect("execution trace persisted at the final iteration");
    assert_eq!(et.samples.len(), 1);

    let _ = std::fs::remove_dir_all(&dir);
}

/// S2 (R5, R12): `Repeat { count: 3 }` with EStop during iteration 3 →
/// `Failed(iteration=3)`, iterations 4+ never start, and NO execution trace
/// is written (failure emits no trace).
#[tokio::test]
async fn repeat_three_estop_at_third_iteration_fails_with_iteration_and_no_trace() {
    use crate::execution_boundary::ExecutionSample;
    use std::sync::atomic::Ordering;

    let mut mock = MockController::new();
    mock.source = ExecutionSource::Hardware;
    // Samples exist on the device — they must NEVER be drained on a failure.
    mock.execution_trace = Some(vec![ExecutionSample {
        timestamp_us: 0,
        joints: vec![0.1, 0.2],
    }]);

    let (svc, concrete, sessions, dir) = repeat_service(mock).await;
    svc.schedule_program(repeat_plan(), Default::default())
        .await
        .unwrap();
    svc.start_execution_with_mode(crate::plan::ExecutionMode::Repeat { count: 3 })
        .await
        .unwrap();

    // Iterations 1-2 complete normally (re-executing in between).
    for _ in 0..2 {
        concrete.write().await.state = Some(repeat_done_state());
        svc.tick_execution_delta(0.1).await.unwrap();
        // B: wait for the background re-execute to land before the next tick.
        wait_for_phase(&svc, RepeatPhase::Idle).await;
        concrete.write().await.state = Some(repeat_running_state());
        svc.tick_execution_delta(0.1).await.unwrap();
    }
    // Iteration 3: the backend reports EStop → terminal failure.
    let mut estop = RobotState::default();
    estop.motion.mode = MotionMode::EStop;
    concrete.write().await.state = Some(estop);
    svc.tick_execution_delta(0.1).await.unwrap();

    let session = sessions.get(1).await.expect("session registered");
    assert_eq!(
        session.status,
        crate::plan::SessionStatus::Failed,
        "S2: EStop at iteration 3 must finalize as Failed"
    );
    assert_eq!(session.iteration, 3, "R5: iteration must be the FAILING one");
    assert_eq!(session.total_iterations, Some(3));
    assert_eq!(
        concrete.read().await.execute_count.load(Ordering::SeqCst),
        3,
        "S2: iterations 4+ must never start (no re-execute after EStop)"
    );
    // S2/R6: the hardware execution trace is NEVER drained on a failure —
    // `take_execution_trace` (clear-on-take) must not have been called.
    assert_eq!(
        concrete.read().await.take_trace_calls.load(Ordering::SeqCst),
        0,
        "S2: no execution trace drain on failure"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

/// S10 (R11): after a completed Repeat session, `reset_execution()` clears the
/// iteration/mode transient state — the next start registers a NEW session
/// beginning at iteration 1.
#[tokio::test]
async fn reset_execution_clears_repeat_state_and_next_start_begins_at_iteration_one() {
    use std::sync::atomic::Ordering;

    let (svc, concrete, sessions, dir) = repeat_service(MockController::new()).await;
    svc.schedule_program(repeat_plan(), Default::default())
        .await
        .unwrap();
    svc.start_execution_with_mode(crate::plan::ExecutionMode::Repeat { count: 2 })
        .await
        .unwrap();

    // Run two iterations to a Completed session.
    concrete.write().await.state = Some(repeat_done_state());
    svc.tick_execution_delta(0.1).await.unwrap();
    // B: wait for the background re-execute to land before the next tick.
    wait_for_phase(&svc, RepeatPhase::Idle).await;
    concrete.write().await.state = Some(repeat_running_state());
    svc.tick_execution_delta(0.1).await.unwrap();
    concrete.write().await.state = Some(repeat_done_state());
    svc.tick_execution_delta(0.1).await.unwrap();
    let first = sessions.get(1).await.expect("first session");
    assert_eq!(first.status, crate::plan::SessionStatus::Completed);
    assert_eq!(first.iteration, 2, "setup: repeat-2 completed at iteration 2");
    assert_eq!(
        concrete.read().await.execute_count.load(Ordering::SeqCst),
        2,
        "setup: 2 execute calls for 2 iterations"
    );

    // R11: reset clears the repeat transient state.
    svc.reset_execution().await.unwrap();

    // The next start is a FRESH session beginning at iteration 1.
    svc.start_execution_with_mode(crate::plan::ExecutionMode::Repeat { count: 2 })
        .await
        .unwrap();
    let second = sessions.get(2).await.expect("second session");
    assert_eq!(second.iteration, 1, "R11: next start begins at iteration 1");
    assert_eq!(second.total_iterations, Some(2));
    assert_eq!(
        concrete.read().await.execute_count.load(Ordering::SeqCst),
        3,
        "R11: iteration 1 of the new session executes once"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

/// R8 regression: the tick that finishes iteration k must report Running
/// (iteration k+1) — a stale Completed status on that tick would make the
/// frontend treat the session as finished and stop its tick loop after the
/// first iteration.
#[tokio::test]
async fn repeat_boundary_tick_reports_running_for_next_iteration() {
    let mut mock = MockController::new();
    mock.source = ExecutionSource::Hardware;

    let (svc, concrete, _sessions, dir) = repeat_service(mock).await;
    svc.schedule_program(repeat_plan(), Default::default())
        .await
        .unwrap();
    svc.start_execution_with_mode(crate::plan::ExecutionMode::Repeat { count: 3 })
        .await
        .unwrap();

    // Iteration 1 mid-run → Running.
    concrete.write().await.state = Some(repeat_running_state());
    let delta = svc.tick_execution_delta(0.1).await.unwrap();
    let exe = delta.execution.expect("delta carries an execution session");
    assert_eq!(exe.status, crate::plan::SessionStatus::Running);
    assert_eq!(exe.iteration, 1);
    assert_eq!(exe.total_iterations, Some(3));

    // Iteration 1 completes → the BOUNDARY tick must report Running(2/3),
    // never Completed — the frontend keeps polling (R8).
    concrete.write().await.state = Some(repeat_done_state());
    let delta = svc.tick_execution_delta(0.1).await.unwrap();
    let exe = delta.execution.expect("delta carries an execution session");
    assert_eq!(
        exe.status,
        crate::plan::SessionStatus::Running,
        "boundary tick must NOT report Completed — the frontend would stop"
    );
    assert_eq!(exe.iteration, 2, "boundary tick reports the NEXT iteration");
    assert_eq!(exe.total_iterations, Some(3));
    assert_eq!(exe.progress(2.0), 0.0, "fresh iteration starts at progress 0");

    let _ = std::fs::remove_dir_all(&dir);
}

/// B (race guard): while a background re-execute is in flight (the repeat
/// upload window), ticks return a synthetic Running(k+1) delta and MUST NOT
/// trigger another re-execute. The stale Completed state of the previous pass
/// would otherwise re-fire the completion gate and queue a second upload — the
/// original synchronous path blocked the tick request for the whole upload and
/// the frontend timed out at 10s, killing its loop (observed: Repeat 5 did 2).
#[tokio::test]
async fn repeat_ticks_during_upload_window_are_synthetic_and_never_reexecute() {
    use std::sync::atomic::Ordering;

    let mut mock = MockController::new();
    mock.source = ExecutionSource::Hardware;
    let blocked = std::sync::Arc::new(tokio::sync::Notify::new());
    let release = std::sync::Arc::new(tokio::sync::Notify::new());
    let wrapper = BlockingExecuteController {
        inner: mock,
        state: None,
        blocking: AtomicBool::new(false),
        blocked: blocked.clone(),
        release: release.clone(),
    };

    let concrete = Arc::new(RwLock::new(wrapper));
    let (svc, sessions, dir) = repeat_service_custom(concrete.clone()).await;
    svc.schedule_program(repeat_plan(), Default::default())
        .await
        .unwrap();
    svc.start_execution_with_mode(crate::plan::ExecutionMode::Repeat { count: 3 })
        .await
        .unwrap();
    assert_eq!(
        concrete.read().await.inner.execute_count.load(Ordering::SeqCst),
        1,
        "setup: initial execute"
    );

    // Park the next re-execute: iteration 1 completes → the boundary tick
    // spawns the upload for iteration 2, which blocks inside the controller.
    concrete.write().await.blocking.store(true, Ordering::SeqCst);
    concrete.write().await.state = Some(repeat_done_state());
    let boundary = svc.tick_execution_delta(0.1).await.unwrap();
    let exe = boundary.execution.expect("boundary delta carries a session");
    assert_eq!(exe.status, crate::plan::SessionStatus::Running);
    assert_eq!(exe.iteration, 2, "boundary tick reports the NEXT iteration");

    // Wait until the re-execute is parked (upload window active).
    tokio::time::timeout(
        std::time::Duration::from_secs(2),
        blocked.notified(),
    )
    .await
    .expect("re-execute must park on the blocking controller");

    // Ticks during the upload window: synthetic Running(2) — and the phase
    // stays Uploading (the gate must NOT re-fire on the stale Completed state).
    let synth = svc.tick_execution_delta(0.1).await.unwrap();
    let exe = synth.execution.expect("upload-window delta carries a session");
    assert_eq!(
        exe.status,
        crate::plan::SessionStatus::Running,
        "upload-window ticks must never report Completed (R8)"
    );
    assert_eq!(exe.iteration, 2);
    assert_eq!(exe.progress(2.0), 0.0, "synthetic delta starts the iteration at 0");
    assert_eq!(
        svc.recording_repeat_phase().await,
        Some(RepeatPhase::Uploading),
        "the upload window persists — no second re-execute was queued"
    );
    // NOTE: we must NOT read the controller here — the parked re-execute holds
    // its write lock, so any `concrete.read()` would deadlock the test. The
    // gate-re-fire proof is the final `execute_count == 3`: a spurious second
    // re-execute would push it to 4+.

    // Release the upload → the task completes → the next tick drives normally.
    release.notify_one();
    wait_for_phase(&svc, RepeatPhase::Idle).await;
    assert_eq!(
        concrete.read().await.inner.execute_count.load(Ordering::SeqCst),
        2,
        "the parked re-execute landed after release"
    );

    concrete.write().await.state = Some(repeat_running_state());
    let mid = svc.tick_execution_delta(0.1).await.unwrap();
    assert_eq!(
        mid.execution.expect("mid-run delta").iteration,
        2,
        "iteration 2 runs normally after the upload landed"
    );

    // Iteration 2 completes → spawn for iteration 3 (unparked).
    concrete.write().await.blocking.store(false, Ordering::SeqCst);
    concrete.write().await.state = Some(repeat_done_state());
    svc.tick_execution_delta(0.1).await.unwrap();
    wait_for_phase(&svc, RepeatPhase::Idle).await;
    concrete.write().await.state = Some(repeat_running_state());
    svc.tick_execution_delta(0.1).await.unwrap();

    // Iteration 3 completes → final Completed.
    concrete.write().await.state = Some(repeat_done_state());
    svc.tick_execution_delta(0.1).await.unwrap();
    let session = sessions.get(1).await.expect("session registered");
    assert_eq!(
        session.status,
        crate::plan::SessionStatus::Completed,
        "B: Repeat 3 completes all 3 iterations even with a parked upload window"
    );
    assert_eq!(session.iteration, 3);
    assert_eq!(
        concrete.read().await.inner.execute_count.load(Ordering::SeqCst),
        3,
        "exactly 3 executions: 1 initial + 2 re-executes — the stale Completed state never re-fired the gate"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

/// B/R5: an async re-execute failure (upload fails mid-repeat) does NOT
/// propagate through the boundary tick — it lands in the pending-error slot and
/// the NEXT tick drains it, failing the session with the real controller code.
#[tokio::test]
async fn repeat_async_reexecute_failure_fails_session_on_next_tick() {
    use std::sync::atomic::Ordering;

    let mut mock = MockController::new();
    mock.source = ExecutionSource::Hardware;
    let wrapper = FailOnExecuteN {
        inner: mock,
        state: None,
        fail_on: 2, // the FIRST re-execute fails (iteration 2's upload)
        calls: AtomicU32::new(0),
    };

    let concrete = Arc::new(RwLock::new(wrapper));
    let (svc, sessions, dir) = repeat_service_custom(concrete.clone()).await;
    svc.schedule_program(repeat_plan(), Default::default())
        .await
        .unwrap();
    svc.start_execution_with_mode(crate::plan::ExecutionMode::Repeat { count: 3 })
        .await
        .unwrap();
    assert_eq!(
        concrete.read().await.inner.execute_count.load(Ordering::SeqCst),
        1,
        "setup: initial execute"
    );

    // Iteration 1 completes → the boundary tick spawns the failing re-execute
    // and responds immediately with Running(2) (no synchronous error).
    concrete.write().await.state = Some(repeat_done_state());
    let boundary = svc.tick_execution_delta(0.1).await.unwrap();
    let exe = boundary.execution.expect("boundary delta carries a session");
    assert_eq!(exe.status, crate::plan::SessionStatus::Running);
    assert_eq!(exe.iteration, 2, "boundary tick reports the NEXT iteration");

    // The async failure surfaces on the NEXT tick (poll until the task lands).
    let mut surfaced: Option<RuntimeError> = None;
    for _ in 0..200 {
        match svc.tick_execution_delta(0.1).await {
            Ok(_) => tokio::time::sleep(std::time::Duration::from_millis(2)).await,
            Err(e) => {
                surfaced = Some(e);
                break;
            }
        }
    }
    let err = surfaced.expect("the async re-execute failure must surface on a tick");
    match err {
        RuntimeError::ControllerFailed { source } => {
            assert!(
                matches!(source, ControllerError::Protocol(_)),
                "the real controller code must reach the frontend: {source:?}"
            );
        }
        other => panic!("expected ControllerFailed, got {other:?}"),
    }

    let session = sessions.get(1).await.expect("session registered");
    assert_eq!(session.status, crate::plan::SessionStatus::Failed);
    assert_eq!(
        session.iteration, 1,
        "the failure is attributed to the COMPLETED iteration whose follow-up failed to start (parity with the old synchronous path)"
    );
    assert_eq!(session.total_iterations, Some(3));
    assert_eq!(
        concrete.read().await.inner.execute_count.load(Ordering::SeqCst),
        1,
        "the failed re-execute never started a real execution"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

/// Progress-unit regression: Simulation robot_state reports a 0..1 FRACTION,
/// but the tick delta's execution session must carry SECONDS on the wire
/// (the DTO mapper divides by plan_duration for the progress bar). Without
/// the normalization the bar caps at 1/plan_duration (~10% for a 10s plan).
#[tokio::test]
async fn simulation_tick_delta_current_time_is_seconds() {
    let mut mock = MockController::new();
    mock.source = ExecutionSource::Simulation;

    let (svc, concrete, _sessions, dir) = repeat_service(mock).await;
    svc.schedule_program(repeat_plan(), Default::default())
        .await
        .unwrap();
    svc.start_execution_with_mode(crate::plan::ExecutionMode::Once)
        .await
        .unwrap();

    // Half-way through a 2.0s plan: Simulation reports fraction 0.5.
    let mut s = RobotState::default();
    s.motion.mode = MotionMode::Moving;
    s.execution.progress = 0.5;
    concrete.write().await.state = Some(s);

    let delta = svc.tick_execution_delta(0.1).await.unwrap();
    let exe = delta.execution.expect("delta carries an execution session");
    assert_eq!(
        exe.current_time, 1.0,
        "Simulation current_time must be SECONDS (0.5 × 2.0s plan), not a fraction"
    );
    assert_eq!(
        exe.progress(2.0),
        0.5,
        "the progress fraction is preserved on the wire"
    );

    let _ = std::fs::remove_dir_all(&dir);
}
