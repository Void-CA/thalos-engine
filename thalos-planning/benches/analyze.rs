use criterion::{Criterion, black_box, criterion_group, criterion_main};

use thalos_core::{
    analysis::observation::ArtifactRef,
    ids::MotionPlanId,
    models::{RobotModel, RobotRegistry},
    trajectory::{Trajectory, TrajectoryPoint},
};
use thalos_planning::analysis::TrajectoryAnalyzer;

fn make_trajectory(n_waypoints: usize) -> Trajectory {
    let waypoints: Vec<_> = (0..n_waypoints)
        .map(|i| {
            let t = i as f64 * 0.01;
            // Planar2R: avoid singularities for normal operation
            let q1 = (t * 0.5).sin();
            let q2 = 1.0 + (t * 0.3).sin() * 0.5; // q2 near 1.0 (good manipulability)
            TrajectoryPoint::new(vec![q1, q2], t)
        })
        .collect();
    Trajectory::new(waypoints)
}

/// I3 anchor for the benchmarked observations.
fn artifact() -> ArtifactRef {
    ArtifactRef::MotionPlan(MotionPlanId("bench".to_string()))
}

fn bench_analyze_10(c: &mut Criterion) {
    let chain = RobotRegistry::create_default(RobotModel::Planar2R);
    let traj = make_trajectory(10);
    let analyzer = TrajectoryAnalyzer::new(&chain, None);

    c.bench_function("analyze_10", |b| {
        b.iter(|| analyzer.analyze(artifact(), black_box(&traj)))
    });
}

fn bench_analyze_100(c: &mut Criterion) {
    let chain = RobotRegistry::create_default(RobotModel::Planar2R);
    let traj = make_trajectory(100);
    let analyzer = TrajectoryAnalyzer::new(&chain, None);

    c.bench_function("analyze_100", |b| {
        b.iter(|| analyzer.analyze(artifact(), black_box(&traj)))
    });
}

fn bench_analyze_1000(c: &mut Criterion) {
    let chain = RobotRegistry::create_default(RobotModel::Planar2R);
    let traj = make_trajectory(1000);
    let analyzer = TrajectoryAnalyzer::new(&chain, None);

    c.bench_function("analyze_1000", |b| {
        b.iter(|| analyzer.analyze(artifact(), black_box(&traj)))
    });
}

criterion_group!(
    benches,
    bench_analyze_10,
    bench_analyze_100,
    bench_analyze_1000
);
criterion_main!(benches);
