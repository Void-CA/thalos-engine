use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Quantifiable metrics of a plan — independent of how they were obtained.
///
/// They can come from `WaypointAnalysis` (analyzed plan), a `MotionTrace`
/// (actual execution), or a future physics simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanMetrics {
    /// Total path length in joint space (sum of Euclidean distances).
    pub length: f64,
    /// Number of waypoints.
    pub waypoint_count: usize,
    /// Manipulability metrics.
    pub manipulability: ManipulabilityMetrics,
    /// Joint safety metrics.
    pub joint_safety: JointSafetyMetrics,
    /// Collision metrics.
    pub collision: CollisionMetrics,
    /// Trajectory smoothness (average jerk between consecutive waypoints).
    /// Lower = smoother.
    pub smoothness: f64,
    /// Total TCP orientation change (radians).
    pub orientation_change: f64,
}

impl PlanMetrics {
    /// Create metrics from pre-computed values.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        length: f64,
        waypoint_count: usize,
        manipulability: ManipulabilityMetrics,
        joint_safety: JointSafetyMetrics,
        collision: CollisionMetrics,
        smoothness: f64,
        orientation_change: f64,
    ) -> Self {
        Self {
            length,
            waypoint_count,
            manipulability,
            joint_safety,
            collision,
            smoothness,
            orientation_change,
        }
    }

    /// Continuous-quality component of the dual-component score (design
    /// ADR-1), computed from the typed metrics: projects this struct into the
    /// `report.metrics` key space and applies the weighted norm. One formula
    /// with the map path used by [`DefaultScoringPolicy`](crate::analysis::scoring::DefaultScoringPolicy).
    pub fn continuous_quality_score(&self) -> f64 {
        continuous_quality_score(&self.to_metric_map())
    }

    /// Projection into the stable `report.metrics` key space (design ADR-1).
    fn to_metric_map(&self) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        map.insert(
            "avg_manipulability".to_string(),
            self.manipulability.average,
        );
        map.insert("smoothness".to_string(), self.smoothness);
        map.insert(
            "min_collision_distance".to_string(),
            self.collision.min_distance,
        );
        map.insert(
            "joint_safety.min_margin".to_string(),
            self.joint_safety.min_margin,
        );
        map.insert("orientation_change".to_string(), self.orientation_change);
        map
    }
}

/// A single slot of the continuous-quality component (design ADR-1): the
/// stable `report.metrics` key, the [`MetricKind`] whose default weight feeds
/// the weighted norm, and the value normalization (always in [0, 1]).
struct ContinuousMetric {
    /// Stable key in `report.metrics` (`BTreeMap<String, f64>`).
    key: &'static str,
    /// The [`MetricKind`] whose `default_weight` is the weight source — the
    /// weights are wired from here, never duplicated as magic numbers.
    kind: MetricKind,
    /// Raw normalization for a present value; the caller clamps to [0, 1].
    normalize: fn(f64) -> f64,
}

impl ContinuousMetric {
    fn weight(&self) -> f64 {
        self.kind.default_weight()
    }
}

/// The five continuous metrics of the quality score (design ADR-1 table):
/// manipulability, smoothness, collision clearance, joint margin, orientation
/// change. `MetricKind::default_weight()` is the single source of weight
/// truth.
const CONTINUOUS_METRICS: [ContinuousMetric; 5] = [
    ContinuousMetric {
        key: "avg_manipulability",
        kind: MetricKind::Manipulability,
        normalize: |v| (v / 0.5).min(1.0),
    },
    ContinuousMetric {
        key: "smoothness",
        kind: MetricKind::Smoothness,
        normalize: |v| 1.0 / (1.0 + v),
    },
    ContinuousMetric {
        key: "min_collision_distance",
        kind: MetricKind::CollisionRisk,
        normalize: |v| (v / 0.1).min(1.0),
    },
    ContinuousMetric {
        key: "joint_safety.min_margin",
        kind: MetricKind::JointMargin,
        normalize: |v| v,
    },
    ContinuousMetric {
        key: "orientation_change",
        kind: MetricKind::OrientationChange,
        normalize: |v| 1.0 / (1.0 + v / std::f64::consts::PI),
    },
];

/// Continuous-quality component of the dual-component score (design ADR-1):
/// `Σ(w_i × norm_i(metric_i)) / Σ(w_i)` over the five continuous metrics.
///
/// Absent keys (and NaN values — never produced by the analyzers, guarded for
/// robustness) map to NEUTRAL (1.0), so a sparse `report.metrics` map
/// preserves the hard-safety pins. Every norm is clamped to [0, 1], so the
/// result is a valid fraction in [0, 1], deterministic and NaN-free.
pub fn continuous_quality_score(metrics: &BTreeMap<String, f64>) -> f64 {
    let mut numerator = 0.0;
    let mut denominator = 0.0;
    for slot in CONTINUOUS_METRICS {
        let weight = slot.weight();
        denominator += weight;
        let normalized = match metrics.get(slot.key) {
            Some(&value) if !value.is_nan() => (slot.normalize)(value).clamp(0.0, 1.0),
            _ => 1.0, // absent or NaN → NEUTRAL
        };
        numerator += weight * normalized;
    }
    numerator / denominator
}

/// Manipulability metrics along the trajectory.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManipulabilityMetrics {
    /// Minimum Yoshikawa value across any waypoint.
    pub min: f64,
    /// Average Yoshikawa value.
    pub average: f64,
    /// Number of waypoints at or near singularity.
    pub near_singular_count: usize,
    /// Number of waypoints in singularity.
    pub singular_count: usize,
}

impl ManipulabilityMetrics {
    pub fn new(min: f64, average: f64, near_singular_count: usize, singular_count: usize) -> Self {
        Self {
            min,
            average,
            near_singular_count,
            singular_count,
        }
    }
}

/// Safety metrics regarding joint limits.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointSafetyMetrics {
    /// Minimum margin to any joint limit (fraction 0.0–1.0).
    /// 1.0 = centered in range, 0.0 = at limit.
    pub min_margin: f64,
    /// Average of the worst utilization per waypoint.
    pub avg_max_utilization: f64,
    /// Number of limit violations.
    pub violation_count: usize,
}

impl JointSafetyMetrics {
    pub fn new(min_margin: f64, avg_max_utilization: f64, violation_count: usize) -> Self {
        Self {
            min_margin,
            avg_max_utilization,
            violation_count,
        }
    }
}

/// Collision metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CollisionMetrics {
    /// Minimum distance to obstacles (negative = collision).
    pub min_distance: f64,
    /// Number of waypoints in collision.
    pub collision_count: usize,
    /// Number of waypoints near collision.
    pub near_miss_count: usize,
}

impl CollisionMetrics {
    pub fn new(min_distance: f64, collision_count: usize, near_miss_count: usize) -> Self {
        Self {
            min_distance,
            collision_count,
            near_miss_count,
        }
    }
}

/// Identifier for a metric — used as key in `CostFunction.weights`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MetricKind {
    PathLength,
    Manipulability,
    JointMargin,
    CollisionRisk,
    Smoothness,
    OrientationChange,
}

impl MetricKind {
    /// Default weight for each metric.
    pub fn default_weight(&self) -> f64 {
        match self {
            MetricKind::PathLength => 0.3,
            MetricKind::Manipulability => 1.0,
            MetricKind::JointMargin => 0.5,
            MetricKind::CollisionRisk => 2.0,
            MetricKind::Smoothness => 0.4,
            MetricKind::OrientationChange => 0.2,
        }
    }

    /// Return all kinds with their default weights.
    pub fn all_with_defaults() -> Vec<(Self, f64)> {
        vec![
            (Self::PathLength, 0.3),
            (Self::Manipulability, 1.0),
            (Self::JointMargin, 0.5),
            (Self::CollisionRisk, 2.0),
            (Self::Smoothness, 0.4),
            (Self::OrientationChange, 0.2),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plan_metrics_new() {
        let m = PlanMetrics::new(
            1.5,
            100,
            ManipulabilityMetrics::new(0.1, 0.5, 2, 0),
            JointSafetyMetrics::new(0.3, 0.7, 0),
            CollisionMetrics::new(0.05, 0, 1),
            0.8,
            1.2,
        );
        assert!((m.length - 1.5).abs() < 1e-10);
        assert_eq!(m.waypoint_count, 100);
    }

    #[test]
    fn metric_kind_default_weights() {
        let all = MetricKind::all_with_defaults();
        assert_eq!(all.len(), 6);
        let map: std::collections::HashMap<_, _> = all.into_iter().collect();
        assert!((map[&MetricKind::CollisionRisk] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn metric_kind_equality() {
        assert_eq!(MetricKind::PathLength, MetricKind::PathLength);
        assert_ne!(MetricKind::PathLength, MetricKind::Manipulability);
    }
}

/// Tests for the continuous-quality component (design ADR-1: the weighted,
/// sum-normalized norm over the five continuous metrics, absent key → NEUTRAL).
#[cfg(test)]
mod continuous_quality_tests {
    use super::*;
    use std::collections::BTreeMap;

    fn map(entries: &[(&str, f64)]) -> BTreeMap<String, f64> {
        entries.iter().map(|(k, v)| (k.to_string(), *v)).collect()
    }

    #[test]
    fn empty_metrics_are_neutral_one() {
        // ADR-1: with NO metric keys present, every slot contributes its
        // NEUTRAL 1.0 → the weighted norm is exactly 1.0. This is what keeps
        // the 0E→1.0 pin when the test harness does not populate the map.
        assert_eq!(continuous_quality_score(&BTreeMap::new()), 1.0);
    }

    #[test]
    fn perfect_values_score_exactly_one() {
        // All five metrics at their ideal values → each norm is 1.0 → the
        // weighted mean is 1.0 (x / x in f64, same summation order both sides).
        let m = map(&[
            ("avg_manipulability", 0.5),
            ("smoothness", 0.0),
            ("min_collision_distance", 0.1),
            ("joint_safety.min_margin", 1.0),
            ("orientation_change", 0.0),
        ]);
        assert_eq!(continuous_quality_score(&m), 1.0);
    }

    #[test]
    fn single_key_norms_follow_design_formulas() {
        // Each norm formula from the ADR-1 table, isolated via single-key maps
        // (all other slots absent → NEUTRAL 1.0). Expected values are the
        // weighted mean derived from the design formula:
        //   score = (w·norm + Σ(other w·1.0)) / Σ(w)
        let avg_manip = continuous_quality_score(&map(&[("avg_manipulability", 0.25)]));
        assert!(
            (avg_manip - 3.6 / 4.1).abs() < 1e-12,
            "avg_manipulability 0.25 → norm 0.5 → {avg_manip}, want 3.6/4.1"
        );

        let collision = continuous_quality_score(&map(&[("min_collision_distance", 0.05)]));
        assert!(
            (collision - 3.1 / 4.1).abs() < 1e-12,
            "min_collision_distance 0.05 → norm 0.5 → {collision}, want 3.1/4.1"
        );

        let smoothness = continuous_quality_score(&map(&[("smoothness", 1.0)]));
        assert!(
            (smoothness - 3.9 / 4.1).abs() < 1e-12,
            "smoothness 1.0 → 1/(1+1) = 0.5 → {smoothness}, want 3.9/4.1"
        );

        let margin = continuous_quality_score(&map(&[("joint_safety.min_margin", 0.3)]));
        assert!(
            (margin - 3.75 / 4.1).abs() < 1e-12,
            "joint_safety.min_margin 0.3 → direct 0.3 → {margin}, want 3.75/4.1"
        );

        let orientation =
            continuous_quality_score(&map(&[("orientation_change", std::f64::consts::PI)]));
        assert!(
            (orientation - 4.0 / 4.1).abs() < 1e-12,
            "orientation_change π → 1/(1+1) = 0.5 → {orientation}, want 4.0/4.1"
        );
    }

    #[test]
    fn norms_clamp_to_unit_interval_for_extreme_values() {
        // Domain safety: whatever the map carries, every norm must stay in
        // [0, 1] so the weighted mean is a valid fraction.
        let huge_manip = continuous_quality_score(&map(&[("avg_manipulability", 10.0)]));
        assert_eq!(
            huge_manip, 1.0,
            "avg_manipulability ≥ 0.5 saturates to norm 1.0"
        );

        let negative_manip = continuous_quality_score(&map(&[("avg_manipulability", -1.0)]));
        assert!(
            (negative_manip - 3.1 / 4.1).abs() < 1e-12,
            "negative manipulability clamps to norm 0.0 → {negative_manip}"
        );

        let penetrating = continuous_quality_score(&map(&[("min_collision_distance", -0.5)]));
        assert!(
            (penetrating - 2.1 / 4.1).abs() < 1e-12,
            "negative clearance clamps to norm 0.0 → {penetrating}"
        );

        let high_margin = continuous_quality_score(&map(&[("joint_safety.min_margin", 5.0)]));
        assert_eq!(high_margin, 1.0, "margin > 1.0 clamps to norm 1.0");
    }

    #[test]
    fn absent_keys_are_exactly_neutral_not_zero() {
        // The neutral semantics: a MISSING key contributes 1.0 (no penalty),
        // which is what preserves the harness pins. If absent meant 0.0, this
        // single-key map would score (0.0 + 3.1)/4.1 ≈ 0.756 instead of 1.0.
        let m = map(&[("smoothness", 0.0)]);
        let expected_if_neutral = (0.4 * 1.0 + 3.7) / 4.1; // smooth neutral 1.0
        assert!((continuous_quality_score(&m) - expected_if_neutral).abs() < 1e-12);
        assert!(
            continuous_quality_score(&m) > 0.9,
            "absent keys must be neutral (1.0), not penalizing"
        );
    }

    #[test]
    fn weights_match_metric_kind_defaults() {
        // The continuous component consumes the MetricKind weights — the
        // "dead code" must be wired, not duplicated as magic numbers.
        let kinds: Vec<MetricKind> = CONTINUOUS_METRICS.iter().map(|m| m.kind).collect();
        assert_eq!(
            kinds,
            vec![
                MetricKind::Manipulability,
                MetricKind::Smoothness,
                MetricKind::CollisionRisk,
                MetricKind::JointMargin,
                MetricKind::OrientationChange,
            ]
        );
        let total: f64 = CONTINUOUS_METRICS.iter().map(|m| m.weight()).sum();
        assert!(
            (total - 4.1).abs() < 1e-12,
            "denominator is the sum of the five weights: 1.0+0.4+2.0+0.5+0.2 = 4.1, got {total}"
        );
    }

    #[test]
    fn plan_metrics_typed_helper_matches_map_path() {
        // The typed PlanMetrics projection must agree with the raw map
        // function used by scoring — one formula, two entry points.
        let metrics = PlanMetrics::new(
            1.5,
            100,
            ManipulabilityMetrics::new(0.1, 0.4, 2, 0),
            JointSafetyMetrics::new(0.6, 0.7, 0),
            CollisionMetrics::new(0.05, 0, 1),
            2.0,
            3.0,
        );
        let projected = map(&[
            ("avg_manipulability", metrics.manipulability.average),
            ("smoothness", metrics.smoothness),
            ("min_collision_distance", metrics.collision.min_distance),
            ("joint_safety.min_margin", metrics.joint_safety.min_margin),
            ("orientation_change", metrics.orientation_change),
        ]);
        assert!(
            (metrics.continuous_quality_score() - continuous_quality_score(&projected)).abs()
                < 1e-12
        );
    }
}
