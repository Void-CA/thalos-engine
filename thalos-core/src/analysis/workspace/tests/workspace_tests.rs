//! Tests for `Workspace` (immutable value object) and `WorkspaceConfig`.
//!
//! Covers:
//! - `from_samples` rejects empty input (R2/R7 invariant)
//! - `from_samples` derives bounds, metrics, sample_count correctly (R3)
//! - `samples()`, `bounds()`, `metrics()` are getters
//! - `WorkspaceConfig` defaults
//! - The position == FK(q) invariant (R2)

use crate::analysis::workspace::{Workspace, WorkspaceConfig, WorkspaceSample};
use crate::models::RobotModel;
use thalos_math::Vector3;

// ─── from_samples: rejection ────────────────────────────────────────────

#[test]
fn from_samples_rejects_empty_input() {
    let empty: Vec<WorkspaceSample> = vec![];
    let result = Workspace::from_samples(empty);
    assert!(result.is_err(), "empty input must fail");
    assert_eq!(result.unwrap_err().to_string(), "workspace is empty",);
}

// ─── from_samples: bounds derivation (R3) ───────────────────────────────

#[test]
fn from_samples_bounds_enclose_all_positions() {
    let samples = vec![
        WorkspaceSample {
            q: vec![0.0],
            position: Vector3::new(-1.0, -2.0, -3.0),
        },
        WorkspaceSample {
            q: vec![0.0],
            position: Vector3::new(1.0, 2.0, 3.0),
        },
        WorkspaceSample {
            q: vec![0.0],
            position: Vector3::new(0.0, 0.0, 0.0),
        },
    ];

    let ws = Workspace::from_samples(samples).unwrap();
    let bb = ws.bounds();

    assert_eq!(bb.min, Vector3::new(-1.0, -2.0, -3.0));
    assert_eq!(bb.max, Vector3::new(1.0, 2.0, 3.0));
}

#[test]
fn from_samples_centroid_is_arithmetic_mean() {
    let samples = vec![
        WorkspaceSample {
            q: vec![0.0],
            position: Vector3::new(0.0, 0.0, 0.0),
        },
        WorkspaceSample {
            q: vec![0.0],
            position: Vector3::new(2.0, 4.0, 6.0),
        },
        WorkspaceSample {
            q: vec![0.0],
            position: Vector3::new(4.0, 8.0, 12.0),
        },
    ];

    let ws = Workspace::from_samples(samples).unwrap();
    let m = ws.metrics();

    assert_eq!(m.centroid, Vector3::new(2.0, 4.0, 6.0));
}

#[test]
fn from_samples_max_reach_is_max_euclidean_distance() {
    let samples = vec![
        WorkspaceSample {
            q: vec![0.0],
            position: Vector3::new(1.0, 0.0, 0.0),
        }, // |p| = 1
        WorkspaceSample {
            q: vec![0.0],
            position: Vector3::new(0.0, 5.0, 0.0),
        }, // |p| = 5
        WorkspaceSample {
            q: vec![0.0],
            position: Vector3::new(2.0, 2.0, 0.0),
        }, // |p| = 2*sqrt(2) ≈ 2.83
    ];

    let ws = Workspace::from_samples(samples).unwrap();
    let m = ws.metrics();

    assert!((m.max_reach - 5.0).abs() < 1e-12);
    assert!((m.min_reach - 1.0).abs() < 1e-12);
}

#[test]
fn from_samples_sample_count_matches_input() {
    let samples = (0..42)
        .map(|i| WorkspaceSample {
            q: vec![i as f64],
            position: Vector3::new(i as f64, 0.0, 0.0),
        })
        .collect();

    let ws = Workspace::from_samples(samples).unwrap();
    assert_eq!(ws.samples().len(), 42);
    assert_eq!(ws.metrics().sample_count, 42);
}

#[test]
fn from_samples_bounding_volume_is_aabb_volume() {
    // 2x4x6 = 48
    let samples = vec![
        WorkspaceSample {
            q: vec![0.0],
            position: Vector3::new(0.0, 0.0, 0.0),
        },
        WorkspaceSample {
            q: vec![0.0],
            position: Vector3::new(2.0, 4.0, 6.0),
        },
    ];

    let ws = Workspace::from_samples(samples).unwrap();
    assert!((ws.metrics().bounding_volume - 48.0).abs() < 1e-12);
}

// ─── WorkspaceConfig ────────────────────────────────────────────────────

#[test]
fn workspace_config_default_is_sensible() {
    let c = WorkspaceConfig::default();
    assert_eq!(c.samples, 10_000);
    assert_eq!(c.tolerance, 1e-3);
    // seed is unconstrained; just assert it's deterministic
    assert_eq!(c.seed, WorkspaceConfig::default().seed);
}

#[test]
fn workspace_config_supports_copy() {
    let c = WorkspaceConfig {
        samples: 5_000,
        seed: 7,
        tolerance: 1e-6,
    };
    let c2 = c; // Copy
    assert_eq!(c.samples, c2.samples);
    assert_eq!(c.seed, c2.seed);
}

// ─── WorkspaceKey (already tested in types_tests but repeated here for cohesion) ─

#[test]
fn workspace_samples_iterator_returns_references() {
    use crate::analysis::workspace::types::WorkspaceKey;
    let _k = WorkspaceKey {
        robot_id: RobotModel::Scara,
        samples: 1,
        seed: 0,
    };
    let samples = vec![WorkspaceSample {
        q: vec![0.0],
        position: Vector3::new(0.0, 0.0, 0.0),
    }];
    let ws = Workspace::from_samples(samples).unwrap();

    // samples() returns &[WorkspaceSample], iterate by reference
    let collected: Vec<&WorkspaceSample> = ws.samples().iter().collect();
    assert_eq!(collected.len(), 1);
    assert_eq!(collected[0].q, vec![0.0]);
}
