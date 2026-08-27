//! Tests for the core workspace data types.
//!
//! These tests verify the public shape, derives (Debug, Clone, Copy, PartialEq,
//! Eq, Hash) and field-level invariants of `WorkspaceSample`, `BoundingBox`,
//! `WorkspaceMetrics` and `WorkspaceKey`.

use crate::analysis::workspace::types::{
    BoundingBox, WorkspaceKey, WorkspaceMetrics, WorkspaceSample,
};
use crate::models::RobotModel;
use thalos_math::Vector3;

// ─── WorkspaceSample ────────────────────────────────────────────────────

#[test]
fn workspace_sample_holds_q_and_position() {
    let sample = WorkspaceSample {
        q: vec![0.1, 0.2, 0.3],
        position: Vector3::new(0.5, 0.5, 0.0),
    };
    assert_eq!(sample.q, vec![0.1, 0.2, 0.3]);
    assert_eq!(sample.position.x, 0.5);
    assert_eq!(sample.position.y, 0.5);
    assert_eq!(sample.position.z, 0.0);
}

#[test]
fn workspace_sample_supports_clone() {
    let s = WorkspaceSample {
        q: vec![1.0],
        position: Vector3::new(0.0, 0.0, 0.0),
    };
    let s2 = s.clone();
    assert_eq!(s.q, s2.q);
    assert_eq!(s.position, s2.position);
}

// ─── BoundingBox ────────────────────────────────────────────────────────

#[test]
fn bounding_box_holds_min_and_max() {
    let bb = BoundingBox {
        min: Vector3::new(-1.0, -2.0, -3.0),
        max: Vector3::new(1.0, 2.0, 3.0),
    };
    assert_eq!(bb.min, Vector3::new(-1.0, -2.0, -3.0));
    assert_eq!(bb.max, Vector3::new(1.0, 2.0, 3.0));
}

#[test]
fn bounding_box_supports_copy() {
    let bb = BoundingBox {
        min: Vector3::new(0.0, 0.0, 0.0),
        max: Vector3::new(1.0, 1.0, 1.0),
    };
    let bb2 = bb; // Copy
    assert_eq!(bb.min, bb2.min);
    assert_eq!(bb.max, bb2.max);
}

// ─── WorkspaceMetrics ───────────────────────────────────────────────────

#[test]
fn workspace_metrics_default_constructor() {
    let m = WorkspaceMetrics {
        bounding_volume: 8.0,
        max_reach: 2.0,
        min_reach: 0.0,
        centroid: Vector3::new(0.0, 0.0, 0.0),
        sample_count: 1_000,
    };
    assert_eq!(m.bounding_volume, 8.0);
    assert_eq!(m.max_reach, 2.0);
    assert_eq!(m.min_reach, 0.0);
    assert_eq!(m.sample_count, 1_000);
}

#[test]
fn workspace_metrics_field_named_bounding_volume_not_volume() {
    // Compile-time guard: the field MUST be named `bounding_volume` to avoid
    // confusion with the workspace shape's true volume (anillo, convex hull, etc.)
    let m = WorkspaceMetrics {
        bounding_volume: 0.0,
        max_reach: 0.0,
        min_reach: 0.0,
        centroid: Vector3::new(0.0, 0.0, 0.0),
        sample_count: 0,
    };
    // If the field were renamed, this line would fail to compile.
    let _ = m.bounding_volume;
}

// ─── WorkspaceKey ───────────────────────────────────────────────────────

#[test]
fn workspace_key_holds_robot_samples_and_seed() {
    let k = WorkspaceKey {
        robot_id: RobotModel::Scara,
        samples: 10_000,
        seed: 42,
    };
    assert_eq!(k.robot_id, RobotModel::Scara);
    assert_eq!(k.samples, 10_000);
    assert_eq!(k.seed, 42);
}

#[test]
fn workspace_key_supports_eq_and_hash() {
    use std::collections::HashSet;

    let k1 = WorkspaceKey {
        robot_id: RobotModel::Planar2R,
        samples: 1_000,
        seed: 0,
    };
    let k2 = WorkspaceKey {
        robot_id: RobotModel::Planar2R,
        samples: 1_000,
        seed: 0,
    };
    let k3 = WorkspaceKey {
        robot_id: RobotModel::Scara,
        samples: 1_000,
        seed: 0,
    };

    assert_eq!(k1, k2, "equal fields must compare equal");
    assert_ne!(k1, k3, "different robot must not compare equal");

    // Hash + Eq together: usable as HashMap key
    let mut set = HashSet::new();
    set.insert(k1);
    set.insert(k2); // duplicate of k1
    set.insert(k3);
    assert_eq!(set.len(), 2, "k1 and k2 collapse to one entry");
}

#[test]
fn workspace_key_supports_copy() {
    let k = WorkspaceKey {
        robot_id: RobotModel::Manipulator3DOF,
        samples: 5_000,
        seed: 7,
    };
    let k2 = k; // Copy, not move
    assert_eq!(k.robot_id, k2.robot_id);
    assert_eq!(k.samples, k2.samples);
}
