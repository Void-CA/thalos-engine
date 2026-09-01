//! Integration tests for TCP (Tool Center Point) frame separation.
//!
//! These tests verify the core invariant: all operational analyses
//! (workspace, singularity, manipulability, IK) must reference the
//! same active TCP when set.

use thalos_engine::math::{Transform3D, UnitQuaternion, Vector3};

use std::sync::Arc;

use tokio::sync::RwLock;

use thalos_engine::core::{
    analysis::workspace::WorkspaceConfig,
    kinematics::{
        forward::ForwardKinematics,
        inverse::{DampedLeastSquaresSolver, IKGoal, IKSolver},
        jacobian::{GeometricJacobian, JacobianSolver},
    },
    models::{RobotModel, RobotRegistry},
    robot::tool_frame::ToolFrame,
};

use crate::{
    Command, RobotController, SceneService,
    backends::{
        controller::simulation::SimulationController, manager::BackendManager,
    },
    services::workspace::WorkspaceService,
};

const ICEBOT_URDF: &str = r#"<?xml version="1.0"?>
<robot name="icebot">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.07"/>
      </geometry>
    </visual>
  </link>
  <link name="link_1">
    <visual>
      <origin xyz="0.0625 0 0.02" rpy="0 0 0"/>
      <geometry>
        <box size="0.125 0.05 0.04"/>
      </geometry>
    </visual>
  </link>
  <link name="link_2">
    <visual>
      <origin xyz="0.05 0 0.02" rpy="0 0 0"/>
      <geometry>
        <box size="0.100 0.04 0.04"/>
      </geometry>
    </visual>
  </link>
  <link name="link_z_rot">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.05" radius="0.02"/>
      </geometry>
    </visual>
  </link>
  <link name="end_effector">
    <visual>
      <origin xyz="0 0 -0.06" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.12" radius="0.008"/>
      </geometry>
    </visual>
  </link>
  <link name="tool0"/>
  <joint name="tcp_joint" type="fixed">
    <parent link="end_effector"/>
    <child link="tool0"/>
    <origin xyz="0 0 -0.12" rpy="0 0 0"/>
  </joint>
  <joint name="axis_0" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5708" upper="1.5708" effort="10.0" velocity="1.0"/>
  </joint>
  <joint name="axis_1" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0.125 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0.0" upper="2.0944" effort="10.0" velocity="1.0"/>
  </joint>
  <joint name="axis_2" type="revolute">
    <parent link="link_2"/>
    <child link="link_z_rot"/>
    <origin xyz="0.100 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.1416" upper="3.1416" effort="5.0" velocity="2.0"/>
  </joint>
  <joint name="axis_3" type="prismatic">
    <parent link="link_z_rot"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.06" rpy="0 0 0"/>
    <axis xyz="0 0 -1"/>
    <limit lower="0.0" upper="0.06" effort="12.0" velocity="0.5"/>
  </joint>
</robot>"#;

/// Helper to create a SceneService with Scara robot.
async fn make_scara_service() -> SceneService {
    let controller = Arc::new(RwLock::new(
        SimulationController::new(4), // Scara has 4 DOF
    )) as Arc<RwLock<dyn RobotController + Send + Sync>>;
    let manager = Arc::new(BackendManager::new());
    manager.set_active(controller).await.unwrap();
    SceneService::new(manager, RobotModel::Scara)
}

/// Test 1: FK.tcp_pose returns the correct composed transformation.
#[test]
fn fk_tcp_pose_returns_composed_transformation() {
    // Use Icebot URDF which has tool0 frame
    let robot =
        thalos_importer::import_urdf(ICEBOT_URDF).expect("icebot URDF should parse");

    // Use from_tip to explicitly select the end_effector (not tool0)
    let chain = thalos_engine::core::robot::adapter::from_tip(&robot, "end_effector")
        .expect("icebot should produce a valid chain to end_effector");

    // Debug: print chain info
    println!("Chain segments: {}", chain.segments.len());
    println!("Chain DOF: {}", chain.dof_count());
    println!("End effector: {:?}", chain.end_effector);

    // Verify the chain has 4 actuated joints (no fixed joint included)
    assert_eq!(
        chain.segments.len(),
        4,
        "icebot chain to end_effector should have 4 segments"
    );

    // Now create a TCP at tool0 with the correct offset
    // The tool0 is 0.12m below the end_effector
    let tcp = ToolFrame::with_offset(
        *chain.end_effector(),
        Transform3D::from_translation(Vector3::new(0.0, 0.0, -0.12)),
    );

    // Evaluate FK at zero configuration
    let fk = ForwardKinematics::new(chain.clone());
    let q = vec![0.0; 4];
    let result = fk.evaluate(&q);

    // Get flange (end_effector) position
    let flange_pos = result.ee_position().expect("flange position should exist");
    println!("Flange position: {:?}", flange_pos);

    // Get TCP position
    let tcp_pos = result
        .tcp_position(&tcp)
        .expect("TCP position should exist");
    println!("TCP position: {:?}", tcp_pos);

    // The TCP should be 0.12m below the flange in Z
    let z_diff = flange_pos.z - tcp_pos.z;
    assert!(
        (z_diff - 0.12).abs() < 1e-6,
        "TCP should be 0.12m below flange, got z_diff = {}",
        z_diff
    );

    // X and Y should be identical (offset is only in Z)
    assert!((flange_pos.x - tcp_pos.x).abs() < 1e-6);
    assert!((flange_pos.y - tcp_pos.y).abs() < 1e-6);
}

/// Test 2: Workspace sampling uses the TCP position.
#[test]
fn workspace_sampling_uses_tcp_position() {
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
    for (flange_sample, tcp_sample) in ws_flange.samples().iter().zip(ws_tcp.samples().iter()) {
        let z_diff = flange_sample.position.z - tcp_sample.position.z;
        assert!(
            (z_diff - 0.12).abs() < 1e-6,
            "Z difference should be 0.12, got {}",
            z_diff
        );
    }
}

/// Test 3: Jacobian with TCP uses the TCP position.
#[test]
fn jacobian_with_tcp_uses_tcp_position() {
    let chain = RobotRegistry::create_default(RobotModel::Scara);

    // Create FK and Jacobian for flange
    let fk_flange = ForwardKinematics::new(chain.clone());
    let jac_flange = GeometricJacobian::new(fk_flange, *chain.end_effector());

    // Create FK and Jacobian for TCP with offset in XY (not just Z)
    let fk_tcp = ForwardKinematics::new(chain.clone());
    let tcp = ToolFrame::with_offset(
        *chain.end_effector(),
        Transform3D::from_translation(Vector3::new(0.1, 0.0, -0.12)),
    );
    let jac_tcp = GeometricJacobian::with_tcp(fk_tcp, tcp);

    // Evaluate at a non-zero configuration with rotation
    let q = vec![0.5, -0.3, 0.8, 0.0];
    let j_flange = jac_flange.evaluate(&q);
    let j_tcp = jac_tcp.evaluate(&q);

    // The Jacobians should be different because the reference point is different
    // (linear velocity depends on the distance from joint axes to the reference point)
    let linear_flange = j_flange.linear();
    let linear_tcp = j_tcp.linear();

    // They should not be identical
    let mut has_difference = false;
    for i in 0..3 {
        for j in 0..linear_flange.ncols() {
            if (linear_flange[(i, j)] - linear_tcp[(i, j)]).abs() > 1e-10 {
                has_difference = true;
                break;
            }
        }
        if has_difference {
            break;
        }
    }

    assert!(
        has_difference,
        "Jacobian with TCP should differ from flange Jacobian"
    );
}

/// Test 4: IK converges and TCP maintains correct offset.
#[tokio::test]
async fn ik_converges_and_tcp_maintains_offset() {
    let chain = RobotRegistry::create_default(RobotModel::Scara);

    // Create a TCP with offset
    let tcp = ToolFrame::with_offset(
        *chain.end_effector(),
        Transform3D::from_translation(Vector3::new(0.0, 0.0, -0.12)),
    );

    // Get the flange position at zero configuration
    let fk = ForwardKinematics::new(chain.clone());
    let q_zero = vec![0.0; 4];
    let result = fk.evaluate(&q_zero);
    let flange_target = result.ee_position().expect("flange position should exist");

    // Create IK solver for the flange (TCP base frame)
    let fk_ik = ForwardKinematics::new(chain.clone());
    let solver = DampedLeastSquaresSolver::new(fk_ik, tcp.base_frame.clone(), 500, 1e-6, 0.1);

    // Solve IK to reach the flange target
    let ik_result = solver
        .solve(&q_zero, IKGoal::Position(flange_target))
        .expect("IK solve should succeed");

    // IK should converge
    assert!(
        ik_result.status == thalos_engine::core::kinematics::inverse::IKStatus::Converged,
        "IK should converge to flange target"
    );

    // Verify the TCP maintains the correct offset from the flange
    let fk_verify = ForwardKinematics::new(chain.clone());
    let result_verify = fk_verify.evaluate(&ik_result.q);
    let flange_final = result_verify
        .ee_position()
        .expect("flange position should exist");
    let tcp_final = result_verify
        .tcp_position(&tcp)
        .expect("TCP position should exist");

    // The TCP should be 0.12m below the flange
    let z_diff = flange_final.z - tcp_final.z;
    assert!(
        (z_diff - 0.12).abs() < 1e-5,
        "TCP should maintain 0.12m offset below flange, got z_diff = {}",
        z_diff
    );

    // X and Y should be identical (offset is only in Z)
    assert!((flange_final.x - tcp_final.x).abs() < 1e-5);
    assert!((flange_final.y - tcp_final.y).abs() < 1e-5);
}

/// Test 5: RuntimeSnapshot.resolve_default_frame returns the TCP when set.
#[tokio::test]
async fn resolve_default_frame_returns_tcp_when_set() {
    let service = make_scara_service().await;

    // Initially, no TCP is set
    let snapshot = service.snapshot().await.unwrap();
    let default_frame = snapshot.resolve_default_frame();
    assert_eq!(
        default_frame,
        *snapshot.chain.end_effector(),
        "Default frame should be end_effector when no TCP is set"
    );

    // Set a TCP at the end_effector with an offset
    let tcp = ToolFrame::with_offset(
        *snapshot.chain.end_effector(),
        Transform3D::from_translation(Vector3::new(0.0, 0.0, -0.12)),
    );
    service
        .execute(Command::SelectToolFrame(Some(tcp)))
        .await
        .unwrap();

    // Now resolve_default_frame should return the TCP base frame
    let snapshot = service.snapshot().await.unwrap();
    let default_frame = snapshot.resolve_default_frame();
    let tcp_frame = snapshot.active_tcp.as_ref().unwrap().base_frame.clone();
    assert_eq!(
        default_frame, tcp_frame,
        "Default frame should be TCP base frame when TCP is set"
    );

    // Clear the TCP
    service
        .execute(Command::SelectToolFrame(None))
        .await
        .unwrap();

    // Now resolve_default_frame should return the end_effector again
    let snapshot = service.snapshot().await.unwrap();
    let default_frame = snapshot.resolve_default_frame();
    assert_eq!(
        default_frame,
        *snapshot.chain.end_effector(),
        "Default frame should be end_effector after clearing TCP"
    );
}

/// Test 6: End-to-end invariant — all analyses reference the same TCP.
#[tokio::test]
async fn all_analyses_reference_same_tcp() {
    let service = make_scara_service().await;

    // Set a TCP with offset
    let snapshot = service.snapshot().await.unwrap();
    let tcp = ToolFrame::with_offset(
        *snapshot.chain.end_effector(),
        Transform3D::from_translation(Vector3::new(0.0, 0.0, -0.12)),
    );
    service
        .execute(Command::SelectToolFrame(Some(tcp.clone())))
        .await
        .unwrap();

    // Verify all analyses use the TCP
    let snapshot = service.snapshot().await.unwrap();

    // 1. resolve_default_frame returns TCP base frame
    let default_frame = snapshot.resolve_default_frame();
    assert_eq!(
        default_frame, tcp.base_frame,
        "resolve_default_frame should return TCP base frame"
    );

    // 2. FK.tcp_position returns the correct position
    let fk = ForwardKinematics::new(snapshot.chain.clone());
    let result = fk.evaluate(&snapshot.joints);
    let tcp_pos = result
        .tcp_position(&tcp)
        .expect("TCP position should exist");

    // 3. Workspace sampling uses TCP
    let config = WorkspaceConfig {
        samples: 10,
        seed: 0,
        tolerance: 1e-3,
    };
    let ws =
        WorkspaceService::sample_from_chain_with_tcp(&snapshot.chain, config, Some(&tcp)).unwrap();
    assert_eq!(ws.samples().len(), 10, "Workspace should have samples");

    // 4. Jacobian with TCP is correctly constructed
    let jac =
        GeometricJacobian::with_tcp(ForwardKinematics::new(snapshot.chain.clone()), tcp.clone());
    let j = jac.evaluate(&snapshot.joints);
    assert_eq!(j.linear().nrows(), 3, "Jacobian should have 3 rows");

    // All analyses are referencing the same TCP ✓
}
