//! Criterion benchmarks for reachability queries.
//!
//! All benchmarks use a pre-sampled `Workspace` so setup time is NOT
//! included in the measurement (only `is_reachable()` execution).

use criterion::{Criterion, black_box, criterion_group, criterion_main};
use rand::SeedableRng;
use rand::rngs::StdRng;

use thalos_core::analysis::workspace::{WorkspaceConfig, WorkspaceSampler};
use thalos_core::models::factory::{RobotModel, RobotRegistry};
use thalos_math::Vector3;

/// Build a sampled workspace for reachability bench setup.
fn scara_workspace_10k() -> thalos_core::analysis::workspace::Workspace {
    let chain = RobotRegistry::create(RobotModel::Scara, RobotModel::Scara.default_spec())
        .expect("Scara chain");
    let sampler = WorkspaceSampler;
    let config = WorkspaceConfig {
        samples: 10_000,
        seed: 42,
        tolerance: 1e-3,
    };
    let mut rng = StdRng::seed_from_u64(config.seed);
    sampler.sample(&chain, config, &mut rng).expect("Workspace")
}

fn reachability_reachable(c: &mut Criterion) {
    let ws = scara_workspace_10k();
    // Use the centroid — the mean of all samples, highly likely reachable.
    let point = ws.metrics().centroid;

    c.bench_function("reachability/reachable", |b| {
        b.iter(|| ws.is_reachable(black_box(&point), black_box(0.01)))
    });
}

fn reachability_out_of_workspace(c: &mut Criterion) {
    let ws = scara_workspace_10k();
    // Far point: well beyond any SCARA reach (planar max ~2m, z ~[-1.0, 1.0]).
    let far_point = Vector3::new(100.0, 100.0, 100.0);

    c.bench_function("reachability/out_of_workspace", |b| {
        b.iter(|| ws.is_reachable(black_box(&far_point), black_box(0.01)))
    });
}

fn reachability_bulk_100(c: &mut Criterion) {
    let ws = scara_workspace_10k();
    let centroid = ws.metrics().centroid;

    // Generate 100 query points: mix of near-centroid and far points.
    let points: Vec<Vector3> = (0..100)
        .map(|i| {
            let offset = if i < 80 {
                // 80 points near centroid (likely reachable)
                Vector3::new(
                    (i as f64 - 40.0) * 0.01,
                    (i as f64 - 40.0) * 0.01,
                    (i as f64 - 40.0) * 0.01,
                )
            } else {
                // 20 points far away
                Vector3::new(100.0 + i as f64, 100.0, 100.0)
            };
            centroid + offset
        })
        .collect();

    c.bench_function("reachability/bulk_100", |b| {
        b.iter(|| {
            for point in &points {
                let _ = ws.is_reachable(black_box(point), black_box(0.01));
            }
        })
    });
}

criterion_group!(
    reachability,
    reachability_reachable,
    reachability_out_of_workspace,
    reachability_bulk_100,
);
criterion_main!(reachability);
