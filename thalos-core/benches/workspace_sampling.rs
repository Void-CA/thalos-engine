//! Criterion benchmarks for workspace sampling.
//!
//! Measures the throughput of `WorkspaceSampler::sample()` across different
//! robot models and sample sizes. All benchmarks use `StdRng::seed_from_u64`
//! for deterministic RNG (D4).

use criterion::{Criterion, black_box, criterion_group, criterion_main};
use rand::SeedableRng;
use rand::rngs::StdRng;

use thalos_core::analysis::workspace::{WorkspaceConfig, WorkspaceSampler};
use thalos_core::models::factory::{RobotModel, RobotRegistry};

fn sample_scara_10k(c: &mut Criterion) {
    let chain = RobotRegistry::create(RobotModel::Scara, RobotModel::Scara.default_spec())
        .expect("Scara chain");
    let sampler = WorkspaceSampler;
    let config = WorkspaceConfig {
        samples: 10_000,
        seed: 42,
        tolerance: 1e-3,
    };

    c.bench_function("sample/scara_10k", |b| {
        b.iter(|| {
            let mut rng = StdRng::seed_from_u64(config.seed);
            sampler.sample(black_box(&chain), black_box(config), &mut rng)
        })
    });
}

fn sample_scara_100k(c: &mut Criterion) {
    let chain = RobotRegistry::create(RobotModel::Scara, RobotModel::Scara.default_spec())
        .expect("Scara chain");
    let sampler = WorkspaceSampler;
    let config = WorkspaceConfig {
        samples: 100_000,
        seed: 42,
        tolerance: 1e-3,
    };

    c.bench_function("sample/scara_100k", |b| {
        b.iter(|| {
            let mut rng = StdRng::seed_from_u64(config.seed);
            sampler.sample(black_box(&chain), black_box(config), &mut rng)
        })
    });
}

fn sample_planar2r_10k(c: &mut Criterion) {
    let chain = RobotRegistry::create(RobotModel::Planar2R, RobotModel::Planar2R.default_spec())
        .expect("Planar2R chain");
    let sampler = WorkspaceSampler;
    let config = WorkspaceConfig {
        samples: 10_000,
        seed: 42,
        tolerance: 1e-3,
    };

    c.bench_function("sample/planar2r_10k", |b| {
        b.iter(|| {
            let mut rng = StdRng::seed_from_u64(config.seed);
            sampler.sample(black_box(&chain), black_box(config), &mut rng)
        })
    });
}

criterion_group!(
    sampling,
    sample_scara_10k,
    sample_scara_100k,
    sample_planar2r_10k
);
criterion_main!(sampling);
