use thalos_engine::core::analysis::singularity::SingularityConfig;
use thalos_engine::core::analysis::workspace::WorkspaceConfig;
use thalos_engine::core::models::RobotModel;

use crate::services::singularity::SingularityService;

#[test]
fn analyze_returns_valid_metrics() {
    let config = WorkspaceConfig {
        samples: 200,
        seed: 42,
        tolerance: 1e-3,
    };
    let singularity_config = SingularityConfig::default();

    let analysis = SingularityService::analyze(RobotModel::Planar2R, config, singularity_config)
        .expect("analysis must succeed");

    assert_eq!(analysis.metrics.total_samples, 200);
    assert_eq!(analysis.samples.len(), 200);

    // With random sampling of planar 2R, some normal samples should exist
    assert!(
        analysis.metrics.normal_count > 0,
        "expected at least some normal samples, got 0"
    );
}

#[test]
fn analyze_rejects_zero_samples() {
    let config = WorkspaceConfig {
        samples: 0,
        seed: 0,
        tolerance: 1e-3,
    };
    let result =
        SingularityService::analyze(RobotModel::Planar2R, config, SingularityConfig::default());
    assert!(result.is_err());
}

#[test]
fn analyze_determinism_same_seed() {
    let config = WorkspaceConfig {
        samples: 100,
        seed: 123,
        tolerance: 1e-3,
    };
    let sc = SingularityConfig::default();

    let a = SingularityService::analyze(RobotModel::Scara, config, sc).unwrap();
    let b = SingularityService::analyze(RobotModel::Scara, config, sc).unwrap();

    assert_eq!(a.metrics.total_samples, b.metrics.total_samples);
    assert_eq!(a.metrics.singular_count, b.metrics.singular_count);
    assert_eq!(a.metrics.near_singular_count, b.metrics.near_singular_count);
    assert_eq!(a.metrics.normal_count, b.metrics.normal_count);
}
