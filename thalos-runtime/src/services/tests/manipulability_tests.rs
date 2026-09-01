use thalos_engine::core::analysis::workspace::WorkspaceConfig;
use thalos_engine::core::models::RobotModel;

use crate::services::manipulability::ManipulabilityService;

#[test]
fn analyze_returns_valid_metrics() {
    let config = WorkspaceConfig {
        samples: 200,
        seed: 42,
        tolerance: 1e-3,
    };

    let analysis = ManipulabilityService::analyze(RobotModel::Planar2R, config)
        .expect("analysis must succeed");

    assert_eq!(analysis.metrics.total_samples, 200);
    assert_eq!(analysis.samples.len(), 200);

    // Yoshikawa should be > 0 for most samples (planar 2R has some isotropic configs)
    assert!(analysis.metrics.avg_yoshikawa > 0.0);

    // Isotropy is always in [0, 1]
    assert!(analysis.metrics.min_isotropy >= 0.0);
    assert!(analysis.metrics.max_isotropy <= 1.0);
}

#[test]
fn analyze_rejects_zero_samples() {
    let config = WorkspaceConfig {
        samples: 0,
        seed: 0,
        tolerance: 1e-3,
    };
    let result = ManipulabilityService::analyze(RobotModel::Planar2R, config);
    assert!(result.is_err());
}

#[test]
fn analyze_determinism() {
    let config = WorkspaceConfig {
        samples: 100,
        seed: 123,
        tolerance: 1e-3,
    };

    let a = ManipulabilityService::analyze(RobotModel::Scara, config).unwrap();
    let b = ManipulabilityService::analyze(RobotModel::Scara, config).unwrap();

    assert!((a.metrics.avg_yoshikawa - b.metrics.avg_yoshikawa).abs() < 1e-12);
}
