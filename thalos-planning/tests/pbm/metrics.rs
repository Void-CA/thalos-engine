//! Pipeline benchmark metrics — comparators, assertions, and helpers.
//!
//! Defines benchmark-specific metric kinds (distinct from production `MetricKind`)
//! and functions to compare before/after metrics and assert expected improvements.

use thalos_core::{evaluation::PlanMetrics, trajectory::Trajectory};

/// Benchmark-specific metric identifiers.
///
/// These correspond to the metrics tracked in `PlanMetrics` plus
/// trajectory-derived values and pipeline-level counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MetricKind {
    JointMargin,
    MaxSegmentError,
    OrientationError,
    MaxVelocity,
    Manipulability,
    OperatorCount,
}

impl MetricKind {
    /// Human-readable name for display in assertion messages.
    pub fn name(&self) -> &'static str {
        match self {
            MetricKind::JointMargin => "joint_margin",
            MetricKind::MaxSegmentError => "max_segment_error",
            MetricKind::OrientationError => "orientation_error",
            MetricKind::MaxVelocity => "max_velocity",
            MetricKind::Manipulability => "manipulability",
            MetricKind::OperatorCount => "operator_count",
        }
    }
}

/// Direction of expected improvement for a metric.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImprovementDirection {
    /// Higher values are better (e.g. manipulability, joint margin).
    Increase,
    /// Lower values are better (e.g. velocity, segment error).
    Decrease,
}

/// Declares an expected improvement for a specific metric and operator.
#[derive(Debug, Clone)]
pub struct ExpectedImprovement {
    /// Operator identifier (e.g. "joint_centering", "retime").
    pub operator_id: &'static str,
    /// Metric that should improve.
    pub metric: MetricKind,
    /// Whether the metric should increase or decrease.
    pub direction: ImprovementDirection,
}

/// Result of comparing a single metric before and after an optimization run.
#[derive(Debug, Clone)]
pub struct MetricDelta {
    /// Which metric this delta refers to.
    pub metric: MetricKind,
    /// Value before optimization.
    pub before: f64,
    /// Value after optimization.
    pub after: f64,
    /// Whether the metric changed in the expected direction.
    pub improved: bool,
}

/// Compare before/after `PlanMetrics` and produce per-metric deltas.
///
/// `trajectory_before` and `trajectory_after` are used for trajectory-level
/// metrics such as `MaxSegmentError` and `MaxVelocity` that are not directly
/// stored in `PlanMetrics`.
pub fn compare_metrics(
    before: &PlanMetrics,
    after: &PlanMetrics,
    trajectory_before: &Trajectory,
    trajectory_after: &Trajectory,
) -> Vec<MetricDelta> {
    let mut deltas = Vec::new();

    // ── JointMargin (from JointSafetyMetrics) ─────────────────
    // Higher min_margin is better (more distance from joint limits).
    let margin_before = before.joint_safety.min_margin;
    let margin_after = after.joint_safety.min_margin;
    deltas.push(MetricDelta {
        metric: MetricKind::JointMargin,
        before: margin_before,
        after: margin_after,
        improved: margin_after > margin_before,
    });

    // ── Manipulability (average) ──────────────────────────────
    // Higher average manipulability is better.
    let manip_before = before.manipulability.average;
    let manip_after = after.manipulability.average;
    deltas.push(MetricDelta {
        metric: MetricKind::Manipulability,
        before: manip_before,
        after: manip_after,
        improved: manip_after > manip_before,
    });

    // ── MaxSegmentError (trajectory-level) ────────────────────
    // Maximum L2 distance between consecutive waypoints.
    let max_seg_before = compute_max_segment_error(trajectory_before);
    let max_seg_after = compute_max_segment_error(trajectory_after);
    deltas.push(MetricDelta {
        metric: MetricKind::MaxSegmentError,
        before: max_seg_before,
        after: max_seg_after,
        improved: max_seg_after < max_seg_before,
    });

    // ── MaxVelocity (trajectory-level) ────────────────────────
    // Maximum joint velocity (dq/dt) across all segments.
    let vel_before = compute_max_velocity(trajectory_before);
    let vel_after = compute_max_velocity(trajectory_after);
    deltas.push(MetricDelta {
        metric: MetricKind::MaxVelocity,
        before: vel_before,
        after: vel_after,
        improved: vel_after < vel_before,
    });

    // ── OrientationError (trajectory-level) ───────────────────
    // Total orientation change from PlanMetrics (proxy).
    let orient_before = before.orientation_change;
    let orient_after = after.orientation_change;
    deltas.push(MetricDelta {
        metric: MetricKind::OrientationError,
        before: orient_before,
        after: orient_after,
        improved: orient_after < orient_before,
    });

    // ── OperatorCount (not from PlanMetrics — added later) ────
    // Placeholder: will be filled by PipelineReport when available.
    deltas.push(MetricDelta {
        metric: MetricKind::OperatorCount,
        before: 0.0,
        after: 0.0,
        improved: false,
    });

    deltas
}

/// Compute the maximum L2 joint-space distance between consecutive waypoints.
fn compute_max_segment_error(trajectory: &Trajectory) -> f64 {
    let wps = trajectory.waypoints();
    if wps.len() < 2 {
        return 0.0;
    }
    wps.windows(2)
        .map(|w| {
            w[0].joints()
                .iter()
                .zip(w[1].joints().iter())
                .map(|(a, b)| (b - a).powi(2))
                .sum::<f64>()
                .sqrt()
        })
        .fold(0.0_f64, f64::max)
}

/// Compute the maximum joint velocity (dq/dt) across all segments.
fn compute_max_velocity(trajectory: &Trajectory) -> f64 {
    let wps = trajectory.waypoints();
    if wps.len() < 2 {
        return 0.0;
    }
    wps.windows(2)
        .map(|w| {
            let dt = (w[1].timestamp() - w[0].timestamp()).max(1e-12);
            let dq_max: f64 = w[0]
                .joints()
                .iter()
                .zip(w[1].joints().iter())
                .map(|(a, b)| (b - a).abs() / dt)
                .fold(0.0_f64, f64::max);
            dq_max
        })
        .fold(0.0_f64, f64::max)
}

/// Assert that expected improvements materialized in the metric deltas.
///
/// # Panics
///
/// Panics if any expected metric is not found in `deltas`, or if the
/// actual change direction does not match the expected direction.
pub fn assert_improvements(expected: &[ExpectedImprovement], deltas: &[MetricDelta]) {
    for exp in expected {
        let delta = deltas
            .iter()
            .find(|d| d.metric == exp.metric)
            .unwrap_or_else(|| panic!("Expected metric {:?} not found in deltas", exp.metric));

        assert!(
            delta.improved,
            "Expected {} to {:?} for operator {}, but got {:.6} → {:.6} (direction: {})",
            exp.metric.name(),
            exp.direction,
            exp.operator_id,
            delta.before,
            delta.after,
            if delta.after > delta.before {
                "increase"
            } else {
                "decrease"
            },
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::{
        evaluation::{CollisionMetrics, JointSafetyMetrics, ManipulabilityMetrics},
        trajectory::TrajectoryPoint,
    };

    fn make_metrics(
        joint_margin: f64,
        manipulability_avg: f64,
        orientation_change: f64,
    ) -> PlanMetrics {
        PlanMetrics::new(
            0.0, // length
            0,   // waypoint_count
            ManipulabilityMetrics::new(0.0, manipulability_avg, 0, 0),
            JointSafetyMetrics::new(joint_margin, 0.0, 0),
            CollisionMetrics::new(1.0, 0, 0),
            0.0, // smoothness
            orientation_change,
        )
    }

    fn simple_traj(joints: Vec<Vec<f64>>, dt: f64) -> Trajectory {
        let pts: Vec<TrajectoryPoint> = joints
            .into_iter()
            .enumerate()
            .map(|(i, j)| TrajectoryPoint::new(j, i as f64 * dt))
            .collect();
        Trajectory::new(pts)
    }

    // ── compare_metrics tests ──────────────────────────────

    #[test]
    fn compare_identical_metrics_produces_zero_deltas() {
        let metrics = make_metrics(0.5, 0.4, 0.3);
        let traj = simple_traj(vec![vec![0.0], vec![0.5], vec![1.0]], 1.0);
        let deltas = compare_metrics(&metrics, &metrics, &traj, &traj);

        // JointMargin, Manipulability unchanged
        let margin = deltas
            .iter()
            .find(|d| d.metric == MetricKind::JointMargin)
            .unwrap();
        assert!((margin.before - margin.after).abs() < 1e-12);
        assert!(!margin.improved);

        let manip = deltas
            .iter()
            .find(|d| d.metric == MetricKind::Manipulability)
            .unwrap();
        assert!((manip.before - manip.after).abs() < 1e-12);
        assert!(!manip.improved);
    }

    #[test]
    fn compare_joint_margin_improvement() {
        let before = make_metrics(0.15, 0.4, 0.3);
        let after = make_metrics(0.45, 0.4, 0.3);
        let traj = simple_traj(vec![vec![0.0], vec![0.5]], 1.0);
        let deltas = compare_metrics(&before, &after, &traj, &traj);

        let margin = deltas
            .iter()
            .find(|d| d.metric == MetricKind::JointMargin)
            .unwrap();
        assert!((margin.before - 0.15).abs() < 1e-12);
        assert!((margin.after - 0.45).abs() < 1e-12);
        assert!(margin.improved, "joint margin should show improvement");
    }

    #[test]
    fn compare_manipulability_degradation() {
        let before = make_metrics(0.5, 0.6, 0.3);
        let after = make_metrics(0.5, 0.4, 0.3);
        let traj = simple_traj(vec![vec![0.0], vec![0.5]], 1.0);
        let deltas = compare_metrics(&before, &after, &traj, &traj);

        let manip = deltas
            .iter()
            .find(|d| d.metric == MetricKind::Manipulability)
            .unwrap();
        assert!(
            !manip.improved,
            "manipulability degraded, should NOT be marked improved"
        );
    }

    #[test]
    fn compare_max_segment_error_detects_improvement() {
        let traj_rough = simple_traj(vec![vec![0.0], vec![2.0], vec![0.0], vec![2.0]], 0.5);
        let traj_smooth = simple_traj(vec![vec![0.0], vec![1.0], vec![0.5], vec![1.0]], 0.5);
        let metrics = make_metrics(0.5, 0.4, 0.3);
        let deltas = compare_metrics(&metrics, &metrics, &traj_rough, &traj_smooth);

        let seg = deltas
            .iter()
            .find(|d| d.metric == MetricKind::MaxSegmentError)
            .unwrap();
        assert!(
            seg.before > seg.after,
            "rough trajectory should have higher segment error"
        );
        assert!(seg.improved);
    }

    #[test]
    fn compare_max_velocity_detects_reduction() {
        let traj_fast = simple_traj(vec![vec![0.0], vec![5.0]], 0.1); // dq/dt = 50
        let traj_slow = simple_traj(vec![vec![0.0], vec![1.0]], 1.0); // dq/dt = 1
        let metrics = make_metrics(0.5, 0.4, 0.3);
        let deltas = compare_metrics(&metrics, &metrics, &traj_fast, &traj_slow);

        let vel = deltas
            .iter()
            .find(|d| d.metric == MetricKind::MaxVelocity)
            .unwrap();
        assert!(
            vel.before > vel.after,
            "fast trajectory should have higher velocity"
        );
        assert!(vel.improved);
    }

    // ── assert_improvements tests ──────────────────────────

    #[test]
    fn assert_expected_improvement_passes() {
        let before = make_metrics(0.15, 0.4, 0.3);
        let after = make_metrics(0.45, 0.4, 0.3);
        let traj = simple_traj(vec![vec![0.0], vec![0.5]], 1.0);
        let deltas = compare_metrics(&before, &after, &traj, &traj);

        let expected = vec![ExpectedImprovement {
            operator_id: "joint_centering",
            metric: MetricKind::JointMargin,
            direction: ImprovementDirection::Increase,
        }];

        // Should not panic
        assert_improvements(&expected, &deltas);
    }

    #[test]
    #[should_panic(expected = "Expected metric")]
    fn assert_missing_metric_panics() {
        let deltas = vec![];
        let expected = vec![ExpectedImprovement {
            operator_id: "test",
            metric: MetricKind::OrientationError,
            direction: ImprovementDirection::Decrease,
        }];
        assert_improvements(&expected, &deltas);
    }

    #[test]
    #[should_panic(expected = "but got")]
    fn assert_unmet_improvement_panics() {
        let before = make_metrics(0.5, 0.6, 0.3);
        let after = make_metrics(0.5, 0.4, 0.3);
        let traj = simple_traj(vec![vec![0.0], vec![0.5]], 1.0);
        let deltas = compare_metrics(&before, &after, &traj, &traj);

        let expected = vec![ExpectedImprovement {
            operator_id: "test",
            metric: MetricKind::Manipulability,
            direction: ImprovementDirection::Increase,
        }];
        assert_improvements(&expected, &deltas);
    }

    // ── compute_max_segment_error tests ───────────────────

    #[test]
    fn max_segment_error_single_waypoint_is_zero() {
        let traj = simple_traj(vec![vec![1.0, 2.0]], 0.0);
        assert!((compute_max_segment_error(&traj) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn max_segment_error_empty_is_zero() {
        let traj = Trajectory::new(vec![]);
        assert!((compute_max_segment_error(&traj) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn max_segment_error_computes_largest_gap() {
        let traj = simple_traj(vec![vec![0.0, 0.0], vec![3.0, 4.0], vec![0.0, 0.0]], 1.0);
        // Segment 0→1: sqrt(3²+4²) = 5.0
        // Segment 1→2: sqrt(3²+4²) = 5.0
        let err = compute_max_segment_error(&traj);
        assert!((err - 5.0).abs() < 1e-10, "expected 5.0, got {}", err);
    }

    // ── compute_max_velocity tests ─────────────────────────

    #[test]
    fn max_velocity_single_waypoint_is_zero() {
        let traj = simple_traj(vec![vec![1.0]], 0.0);
        assert!((compute_max_velocity(&traj) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn max_velocity_computes_peak_joint_speed() {
        // dq = 2.0, dt = 0.5 → velocity = 4.0
        let traj = simple_traj(vec![vec![0.0, 0.0], vec![2.0, 0.5]], 0.5);
        let vel = compute_max_velocity(&traj);
        // Joint 0: |2.0-0.0| / 0.5 = 4.0
        // Joint 1: |0.5-0.0| / 0.5 = 1.0
        assert!((vel - 4.0).abs() < 1e-10, "expected 4.0, got {}", vel);
    }
}
