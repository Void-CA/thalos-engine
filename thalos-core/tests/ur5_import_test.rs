//! Integration test: import UR5 via URDF → RobotGraph → from_tip → FK.
//!
//! The UR5 fixture is in `thalos-models/tests/fixtures/ur5.urdf`.
//! Loaded from a relative path that works when running `cargo test`
//! from the crate root.

use std::fs;
use std::path::PathBuf;

use thalos_core::kinematics::forward::ForwardKinematics;
use thalos_core::robot::adapter;
use thalos_importer::import_urdf;

/// Locate the UR5 fixture relative to the crate manifest directory.
fn fixture_path() -> PathBuf {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    manifest
        .parent()
        .unwrap() // workspace root (thalos-engine/)
        .join("thalos-models/tests/fixtures/ur5.urdf")
}

fn load_ur5_urdf() -> String {
    fs::read_to_string(fixture_path()).expect("UR5 fixture file not found")
}

/// Parse the UR5 URDF into a Robot.
fn load_robot() -> thalos_models::Robot {
    let source = load_ur5_urdf();
    import_urdf(&source).expect("UR5 should parse")
}

// ─── from_tip — primary API ─────────────────────────────────────

#[test]
fn from_tip_tool0_produces_8_segments_6_dof() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "tool0").expect("from_tip with tool0 should succeed");

    // world → base_link → shoulder → upper_arm → forearm → wrist_1 → wrist_2 → wrist_3 → tool0
    // = 8 joints (world_joint + 6 revolute + tool0_fixed)
    assert_eq!(
        chain.segment_count(),
        8,
        "path from world to tool0 has 8 joints"
    );
    assert_eq!(chain.dof_count(), 6, "6 revolute joints are actuated");
}

#[test]
fn from_tip_ee_link_produces_8_segments_6_dof() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "ee_link").expect("from_tip with ee_link should succeed");

    assert_eq!(
        chain.segment_count(),
        8,
        "path from world to ee_link has 8 joints"
    );
    assert_eq!(chain.dof_count(), 6);
}

#[test]
fn from_tip_wrist_3_link_7_segments_6_dof() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "wrist_3_link")
        .expect("from_tip with wrist_3_link should succeed");

    // world → ... → wrist_3_link = 7 joints (world_joint + 6 revolute, no fixed tool frame)
    assert_eq!(chain.segment_count(), 7);
    assert_eq!(chain.dof_count(), 6);
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
fn auto_picks_most_actuated_leaf() {
    let robot = load_robot();
    let chain = adapter::auto(&robot).expect("auto should pick a valid chain");

    // UR5 has 2 leaves with 6 actuated DOF (ee_link and tool0).
    // Tiebreaker picks the one with the lowest LinkId (earlier in BFS).
    assert_eq!(chain.dof_count(), 6, "auto should find 6 DOF chain");
    assert!(
        chain.segment_count() >= 7,
        "auto should include at least 7 joints (world + 6 revolute)"
    );
}

// ─── FK from from_tip ──────────────────────────────────────────

#[test]
fn fk_at_zero_pose_succeeds() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "tool0").unwrap();
    let fk = ForwardKinematics::new(chain);

    let q = [0.0; 6];
    let result = fk.evaluate(&q);

    let ee_pose = result
        .ee_pose()
        .expect("end-effector pose should be in FK result");

    let t = ee_pose.transform().translation;
    assert!(t.x.is_finite());
    assert!(t.y.is_finite());
    assert!(t.z.is_finite());

    // UR5 reach ~0.8m at zero config:
    // z ≈ shoulder_height + upper_arm + forearm = 0.089 + 0.425 + 0.392 = 0.906
    assert!(
        t.norm() > 0.5,
        "ee translation norm should be > 0.5m at zero config, got {}",
        t.norm()
    );
}

#[test]
fn fk_shoulder_pan_moves_ee() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "tool0").unwrap();
    let fk = ForwardKinematics::new(chain);

    let q = [1.57079632679, 0.0, 0.0, 0.0, 0.0, 0.0];
    let result = fk.evaluate(&q);
    let ee_pose = result.ee_pose().unwrap();

    let t = ee_pose.transform().translation;
    let xy_dist = (t.x * t.x + t.y * t.y).sqrt();
    assert!(
        xy_dist > 0.1,
        "after shoulder_pan 90°, ee should have moved in XY plane, got xy_dist={}",
        xy_dist
    );
}

#[test]
fn fk_all_joints_different_values() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "tool0").unwrap();
    let fk = ForwardKinematics::new(chain);

    let q = [0.5, -0.8, 1.2, -0.3, 0.6, 0.0];
    let result = fk.evaluate(&q);
    let ee_pose = result.ee_pose().unwrap();

    let t = ee_pose.transform().translation;
    assert!(
        t.norm() > 0.1,
        "ee should be away from origin at non-zero config, got {}",
        t.norm()
    );
}

#[test]
fn fk_frame_count_matches_links() {
    let robot = load_robot();
    let chain = adapter::from_tip(&robot, "tool0").unwrap();
    let fk = ForwardKinematics::new(chain);

    let q = [0.0; 6];
    let result = fk.evaluate(&q);

    // world + 8 segments on the path (world → base_link → ... → tool0)
    assert_eq!(
        result.frames().count(),
        9,
        "FK should produce 9 poses (world + 8 link frames)"
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

// ─── from_urdf (backward compat, now delegates to auto) ─────────

#[test]
fn from_urdf_still_works() {
    let source = load_ur5_urdf();
    let chain = adapter::from_urdf(&source).expect("from_urdf should still work");
    assert!(
        chain.dof_count() >= 6,
        "from_urdf should produce at least 6 DOF"
    );
}
