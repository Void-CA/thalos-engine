//! Integration test: import SCARA via URDF → Robot → from_tip → FK.
//!
//! The SCARA fixture is in `thalos-models/tests/fixtures/scara.urdf`.
//!
//! Canonical spec (from thalos_core::robot::scara::SCARA_CANONICAL):
//!   base_height = 0.5, a1 = 1.0, a2 = 0.8
//!   joint_1: revolute ±140° (±2.443 rad)
//!   joint_2: revolute ±150° (±2.618 rad)
//!   joint_3: prismatic  [-0.5, 0.0]
//!   joint_4: continuous ±360° (±6.283 rad)

use std::fs;
use std::path::PathBuf;

use thalos_core::kinematics::forward::ForwardKinematics;
use thalos_core::robot::adapter;
use thalos_importer::import_urdf;

fn fixture_path() -> PathBuf {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    manifest
        .parent()
        .unwrap() // workspace root (thalos-engine/)
        .join("thalos-models/tests/fixtures/scara.urdf")
}

fn load_scara_urdf() -> String {
    fs::read_to_string(fixture_path()).expect("SCARA fixture file not found")
}

fn load_robot() -> thalos_models::Robot {
    let source = load_scara_urdf();
    import_urdf(&source).expect("SCARA should parse")
}

// ─── from_tip — primary API ─────────────────────────────────────

#[test]
fn from_tip_tool0_produces_5_segments_4_dof() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "tool0").expect("from_tip with tool0 should succeed");

    // world → base_joint → joint_1 → joint_2 → joint_3 → joint_4
    // = 5 joints (base_joint + 4 actuated)
    assert_eq!(
        chain.segment_count(),
        5,
        "path from world to tool0 has 5 joints"
    );
    assert_eq!(
        chain.dof_count(),
        4,
        "4 actuated joints (2 revolute + 1 prismatic + 1 continuous)"
    );
}

#[test]
fn from_tip_link3_produces_4_segments_3_dof() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "link_3").expect("from_tip with link_3 should succeed");

    // world → base_joint → joint_1 → joint_2 → joint_3
    assert_eq!(
        chain.segment_count(),
        4,
        "path from world to link_3 has 4 joints"
    );
    assert_eq!(
        chain.dof_count(),
        3,
        "3 actuated joints before tool0 (joint_4 excluded)"
    );
}

#[test]
fn from_tip_nonexistent_target_errors() {
    let robot = load_robot();
    let err = adapter::from_tip(&robot, "ghost_link").unwrap_err();
    assert!(
        err.to_string().contains("missing link"),
        "expected MissingLink error, got: {err}"
    );
}

// ─── auto — heuristic ─────────────────────────────────────────

#[test]
fn auto_picks_tool0() {
    let robot = load_robot();
    let chain = adapter::auto(&robot).expect("auto should pick a valid chain");

    // tool0 should have the most actuated DOF (4)
    assert_eq!(
        chain.dof_count(),
        4,
        "auto should find 4 DOF chain (tool0 leaf)"
    );
}

// ─── FK from from_tip ──────────────────────────────────────────

#[test]
fn fk_at_zero_pose_succeeds() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "tool0").unwrap();
    let fk = ForwardKinematics::new(chain);

    // Zero config: j1=0, j2=0, j3=0 (top), j4=0
    // → arm fully extended along X at base_height
    // → ee at (1.8, 0.0, 0.5)
    let q = [0.0; 4];
    let result = fk.evaluate(&q);

    let ee_pose = result
        .ee_pose()
        .expect("end-effector pose should be in FK result");

    let t = ee_pose.transform().translation;
    assert!(t.x.is_finite());
    assert!(t.y.is_finite());
    assert!(t.z.is_finite());

    // ADR-0001: URDF import is passthrough Z-up. No conversion applied.
    // At zero config, ee should be near (1.8, 0, 0.5) = a1 + a2 on X, base_height on Z
    assert!(
        (t.x - 1.8).abs() < 1e-3,
        "ee x should be ~1.8 at zero config, got {}",
        t.x
    );
    assert!(
        (t.z - 0.5).abs() < 1e-3,
        "ee z should be ~0.5 at zero config (prismatic at top), got {}",
        t.z
    );
}

#[test]
fn fk_j1_rotation_moves_ee_in_xy() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "tool0").unwrap();
    let fk = ForwardKinematics::new(chain);

    // Rotate only joint_1 by 90°
    let q = [1.5708, 0.0, 0.0, 0.0];
    let result = fk.evaluate(&q);
    let ee_pose = result.ee_pose().unwrap();

    let t = ee_pose.transform().translation;
    let xy_dist = (t.x * t.x + t.y * t.y).sqrt();
    assert!(
        xy_dist > 0.5,
        "after j1 90°, ee should be away from origin in XY, got xy_dist={}",
        xy_dist
    );
}

#[test]
fn fk_j3_prismatic_moves_ee_in_z() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "tool0").unwrap();
    let fk = ForwardKinematics::new(chain);

    // Move prismatic joint_3 to bottom (-0.5)
    let q = [0.0, 0.0, -0.5, 0.0];
    let result = fk.evaluate(&q);
    let ee_pose = result.ee_pose().unwrap();

    let t = ee_pose.transform().translation;
    assert!(
        (t.z - 0.0).abs() < 1e-3,
        "ee z should be ~0.0 when prismatic at -0.5 (base_height=0.5 + j3=-0.5), got {}",
        t.z
    );
}

#[test]
fn fk_frame_count_matches_links() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "tool0").unwrap();
    let fk = ForwardKinematics::new(chain);

    let q = [0.0; 4];
    let result = fk.evaluate(&q);

    // world + 5 segments on the path (world → base_link → link_1 → link_2 → link_3 → tool0)
    assert_eq!(
        result.frames().count(),
        6,
        "FK should produce 6 poses (world + 5 link frames)"
    );
}

#[test]
fn fk_from_tip_and_auto_give_same_dof_count() {
    let robot = load_robot();
    let chain_tip = adapter::from_tip(&robot, "tool0").unwrap();
    let chain_auto = adapter::auto(&robot).unwrap();

    assert_eq!(
        chain_tip.dof_count(),
        chain_auto.dof_count(),
        "from_tip and auto should agree on DOF count"
    );
}

#[test]
fn fk_all_joints_non_zero() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "tool0").unwrap();
    let fk = ForwardKinematics::new(chain);

    // Arbitrary non-zero config
    let q = [0.8, -1.2, -0.3, 1.5];
    let result = fk.evaluate(&q);
    let ee_pose = result.ee_pose().unwrap();

    let t = ee_pose.transform().translation;
    assert!(
        t.norm() > 0.1,
        "ee should be away from origin at non-zero config, got {}",
        t.norm()
    );
}

// ─── Workspace reachability ─────────────────────────────────────

#[test]
fn scara_workspace_point_reachable() {
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use thalos_core::prelude::{Reachability, WorkspaceConfig, WorkspaceSampler};
    use thalos_core::robot::serial_chain::SerialChain;
    use thalos_math::Vector3;

    let robot = load_robot();
    let chain: SerialChain = adapter::auto(&robot).unwrap();

    let mut rng = StdRng::seed_from_u64(0);
    let config = WorkspaceConfig {
        samples: 5000,
        seed: 0,
        tolerance: 1e-3,
    };
    let ws = WorkspaceSampler.sample(&chain, config, &mut rng).unwrap();

    // For the SCARA at full extension (1.8m radius), a point at (0.7, 0, 0.5)
    // should be reachable (well inside the 1.8m reach, at mid-elevation).
    let target = Vector3::new(0.7, 0.0, 0.5);
    let result = ws.is_reachable(&target, 0.1).unwrap();
    assert!(
        matches!(result, Reachability::Reachable),
        "Point ({:.3}, {:.3}, {:.3}) should be reachable within 0.1m tolerance; got: {:?}",
        target.x,
        target.y,
        target.z,
        result,
    );
}

// ─── from_urdf (backward compat) ────────────────────────────────

#[test]
fn from_urdf_still_works() {
    let source = load_scara_urdf();
    let chain = adapter::from_urdf(&source).expect("from_urdf should still work");
    assert_eq!(
        chain.dof_count(),
        4,
        "from_urdf should produce 4 DOF for SCARA"
    );
}
