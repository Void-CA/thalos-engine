//! Scoring policy — deterministic penalty configuration and the derived
//! `quality_index` / `grade` mapping (spec `analysis-score-semantics`).
//!
//! The policy is intentionally SEPARATE from the aggregator (design C2):
//! [`DefaultAggregator`](crate::analysis::aggregator::DefaultAggregator) only
//! knows the [`ScoringPolicy`] trait, never concrete weights. Concrete weight
//! values evolve with operational experience without touching the spec or the
//! observation model.
//!
//! # Score semantics (spec `analysis-score-semantics` + `quality-scoring-contract`)
//!
//! - `quality_index` is a dual-component score (design ADR-1):
//!   `hard_safety × continuous_quality`, both in `[0, 1]`.
//! - The **hard-safety component** is derived from the report's observations:
//!   `max(0, 1 − Σ penalty_i)`, the discrete band 0E→1.0, 1E→0.70, 2E→0.40,
//!   3E→0.10, ≥4E→0.0. The unclamped raw value is retained internally
//!   ([`ScoringPolicy::raw_hard_safety`]) as the monotonic signal beyond
//!   saturation.
//! - The **continuous component** is a weighted, sum-normalized norm over the
//!   five continuous metrics (manipulability, smoothness, collision clearance,
//!   joint margin, orientation change) — absent keys map to NEUTRAL (1.0), so
//!   a sparse `report.metrics` map preserves the discrete pins.
//! - `quality_index` lives in `[0, 1]`, is deterministic, finite and NaN-free;
//!   the wire DTO projects `× 100` in a later phase (PR 7a), never here.
//! - Grade mapping: Excellent ≥ 0.9, Good ≥ 0.7, Fair ≥ 0.5, Poor < 0.5
//!   (deterministic, thresholds inclusive).
//!
//! # Monotonicity (D6 + quality-scoring-contract)
//!
//! Every `penalty_i ≥ 0`, so the hard-safety component is non-increasing when
//! observations are added and the continuous component is observation-free —
//! `Obs ⊆ Obs' ⇒ quality(Obs') ≤ quality(Obs)` for a fixed metric map, proven
//! by property tests in CI, never by runtime asserts (closed decision).

use crate::analysis::observation::{Observation, Severity};
use crate::analysis::summary::Grade;
use crate::evaluation::metrics::continuous_quality_score;
use std::collections::BTreeMap;

/// Deterministic penalty and quality configuration (design C2).
///
/// Implementations define `penalty(severity)`; `quality_index`, its components
/// and `grade_for` are derived from it and inherited unchanged.
pub trait ScoringPolicy {
    /// Penalty weight for a severity. MUST be `>= 0` (D6 monotonicity proof).
    fn penalty(&self, severity: Severity) -> f64;

    /// Unclamped hard-safety component: `1 − Σ penalty_i`, summed **per
    /// severity in canonical order** (Info, Warning, Error), never per
    /// observation in input order. Floating-point addition is not associative,
    /// so an order-sensitive sum could vary by ulps between runs; the
    /// canonical-order sum is exactly independent of the input order — this is
    /// what makes the component deterministic AND commutative at the `f64`
    /// level.
    ///
    /// This is the INTERNAL monotonic signal: while the observable score is
    /// clamped at 0.0 (≥ 4 Errors), the raw value keeps decreasing as Errors
    /// are added (17→16 Errors: −4.1→−3.8), preserving the spec's
    /// "removing one of many Errors still shows improvement" requirement.
    fn raw_hard_safety(&self, observations: &[Observation]) -> f64 {
        let mut counts = [0usize; 3]; // [Info, Warning, Error]
        for observation in observations {
            match observation.severity {
                Severity::Info => counts[0] += 1,
                Severity::Warning => counts[1] += 1,
                Severity::Error => counts[2] += 1,
            }
        }
        let total = counts[0] as f64 * self.penalty(Severity::Info)
            + counts[1] as f64 * self.penalty(Severity::Warning)
            + counts[2] as f64 * self.penalty(Severity::Error);
        1.0 - total
    }

    /// Clamped hard-safety component in `[0, 1]` — `max(0, raw_hard_safety)`.
    /// The discrete band: 0E→1.0, 1E→0.70, 2E→0.40, 3E→0.10, ≥4E→0.0.
    fn hard_safety(&self, observations: &[Observation]) -> f64 {
        self.raw_hard_safety(observations).max(0.0)
    }

    /// The aggregate quality index of a set of observations (design ADR-1):
    /// `hard_safety × continuous_quality`, both components in `[0, 1]`.
    ///
    /// `metrics` is the report's `BTreeMap<String, f64>`; absent keys map to
    /// NEUTRAL (1.0) in the continuous component, so with an empty map the
    /// score reduces to the discrete hard-safety pins exactly.
    ///
    /// Determinism: the hard component sums in canonical severity order
    /// (order-independent), and the continuous component iterates a fixed
    /// table of metric keys — both exact at the `f64` level.
    fn quality_index(&self, observations: &[Observation], metrics: &BTreeMap<String, f64>) -> f64 {
        self.hard_safety(observations) * continuous_quality_score(metrics)
    }

    /// Deterministic grade mapping (spec `analysis-score-semantics`).
    ///
    /// Thresholds are inclusive: `0.9 → Excellent`, `0.7 → Good`, `0.5 → Fair`,
    /// strictly below `0.5 → Poor`.
    fn grade_for(&self, quality_index: f64) -> Grade {
        match quality_index {
            q if q >= 0.9 => Grade::Excellent,
            q if q >= 0.7 => Grade::Good,
            q if q >= 0.5 => Grade::Fair,
            _ => Grade::Poor,
        }
    }
}

/// Default scoring policy — a reasonable starting point, documented as
/// CONFIGURABLE policy, not a sacred constant (spec: concrete weights MAY
/// evolve). Penalization ordering: Info < Warning < Error.
#[derive(Debug, Clone, Copy, Default)]
pub struct DefaultScoringPolicy;

impl DefaultScoringPolicy {
    /// Penalty weight for [`Severity::Info`].
    pub const INFO_PENALTY: f64 = 0.05;
    /// Penalty weight for [`Severity::Warning`].
    pub const WARNING_PENALTY: f64 = 0.15;
    /// Penalty weight for [`Severity::Error`].
    pub const ERROR_PENALTY: f64 = 0.30;
}

impl ScoringPolicy for DefaultScoringPolicy {
    fn penalty(&self, severity: Severity) -> f64 {
        // Exhaustive match (no wildcard): `Severity` is defined in this crate,
        // so adding a severity breaks compilation here until the policy assigns
        // it a weight — no silent zero-penalty default.
        match severity {
            Severity::Info => Self::INFO_PENALTY,
            Severity::Warning => Self::WARNING_PENALTY,
            Severity::Error => Self::ERROR_PENALTY,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{DefaultScoringPolicy, ScoringPolicy};
    use crate::analysis::location::Location;
    use crate::analysis::observation::{
        ArtifactRef, Observation, ObservationId, ObservationKind, Severity,
    };
    use crate::analysis::summary::Grade;
    use crate::ids::MotionPlanId;
    use std::collections::BTreeMap;

    fn observation(severity: Severity) -> Observation {
        Observation {
            id: ObservationId(0), // incoming ids are ignored by the aggregator
            kind: ObservationKind::ResidualError,
            severity,
            artifact: ArtifactRef::MotionPlan(MotionPlanId("mp-1".to_string())),
            location: Location::Waypoint(0),
            attributes: BTreeMap::new(),
            causes: Vec::new(),
            related: Vec::new(),
        }
    }

    /// A metrics map with every continuous slot at its ideal value → the
    /// continuous component is exactly 1.0 (design ADR-1 harness preservation).
    fn good_metrics() -> BTreeMap<String, f64> {
        let mut metrics = BTreeMap::new();
        metrics.insert("avg_manipulability".to_string(), 0.5);
        metrics.insert("smoothness".to_string(), 0.0);
        metrics.insert("min_collision_distance".to_string(), 0.1);
        metrics.insert("joint_safety.min_margin".to_string(), 1.0);
        metrics.insert("orientation_change".to_string(), 0.0);
        metrics
    }

    fn errors(n: usize) -> Vec<Observation> {
        (0..n).map(|_| observation(Severity::Error)).collect()
    }

    #[test]
    fn grade_boundaries_map_to_grades() {
        // Spec analysis-score-semantics "Grade boundaries": [0.95, 0.75, 0.55,
        // 0.30] → [Excellent, Good, Fair, Poor]. Weight-independent: the mapping
        // is a pure function of quality_index.
        let policy = DefaultScoringPolicy;
        let cases = [
            (0.95, Grade::Excellent),
            (0.75, Grade::Good),
            (0.55, Grade::Fair),
            (0.30, Grade::Poor),
        ];
        for (index, expected) in cases {
            assert_eq!(policy.grade_for(index), expected, "grade_for({index})");
        }
    }

    #[test]
    fn grade_boundary_value_09_is_excellent() {
        // Spec "Boundary value": 0.9 → Excellent (>= 0.9, inclusive).
        let policy = DefaultScoringPolicy;
        assert_eq!(policy.grade_for(0.9), Grade::Excellent);
    }

    #[test]
    fn grade_inclusive_lower_boundaries() {
        // Triangulation: every threshold is inclusive (>=), so 0.7 → Good and
        // 0.5 → Fair; only values strictly below 0.5 are Poor.
        let policy = DefaultScoringPolicy;
        assert_eq!(policy.grade_for(0.7), Grade::Good);
        assert_eq!(policy.grade_for(0.5), Grade::Fair);
        assert_eq!(policy.grade_for(0.49), Grade::Poor);
        assert_eq!(policy.grade_for(0.0), Grade::Poor);
    }

    #[test]
    fn zero_observations_produce_perfect_quality() {
        // Spec "Observations drive quality": zero observations → 1.0 (with
        // neutral metrics, the continuous component cannot lower the score).
        let policy = DefaultScoringPolicy;
        assert_eq!(policy.quality_index(&[], &BTreeMap::new()), 1.0);
    }

    #[test]
    fn quality_is_one_minus_total_penalty() {
        // D6: quality = max(0, 1 - Σ penalty_i) with neutral (absent) metrics.
        // Expected values are derived from policy.penalty(...) so the test
        // survives weight evolution. The two expressions associate the float
        // additions differently (impl: 1 - (pE + pW + pI); test:
        // (1 - pE) - pW - pI), which can differ by one ulp — compare with
        // tolerance, asserting semantics.
        let policy = DefaultScoringPolicy;
        let error = observation(Severity::Error);
        let warning = observation(Severity::Warning);
        let info = observation(Severity::Info);
        let expected = 1.0
            - policy.penalty(Severity::Error)
            - policy.penalty(Severity::Warning)
            - policy.penalty(Severity::Info);
        let actual = policy.quality_index(&[error, warning, info], &BTreeMap::new());
        assert!(
            (actual - expected).abs() < 1e-12,
            "quality_index {actual} must equal 1 - Σ penalties ≈ {expected}"
        );
    }

    #[test]
    fn quality_clamps_at_zero_when_penalties_exceed_one() {
        // D6: the max(0, ·) floor — a saturated artifact cannot go negative.
        let policy = DefaultScoringPolicy;
        let errors_per_penalty = policy.penalty(Severity::Error);
        let n = (1.0 / errors_per_penalty) as usize + 2;
        let saturated: Vec<_> = (0..n).map(|_| observation(Severity::Error)).collect();
        assert_eq!(policy.quality_index(&saturated, &BTreeMap::new()), 0.0);
    }

    #[test]
    fn penalty_increases_with_severity_and_is_non_negative() {
        // Penalization ordering (PR 2a): Info < Warning < Error, each >= 0 so
        // the D6 additive-penalty monotonicity proof holds.
        let policy = DefaultScoringPolicy;
        let info = policy.penalty(Severity::Info);
        let warning = policy.penalty(Severity::Warning);
        let error = policy.penalty(Severity::Error);
        assert!(info >= 0.0 && warning >= 0.0 && error >= 0.0);
        assert!(info < warning && warning < error);
    }

    // ─── Dual-component score (design ADR-1, spec quality-scoring-contract) ───

    #[test]
    fn zero_errors_with_good_metrics_score_exactly_one() {
        // "Perfect plan with good metrics": 0E × continuous 1.0 → exactly 1.0,
        // Excellent grade.
        let policy = DefaultScoringPolicy;
        let quality = policy.quality_index(&[], &good_metrics());
        assert_eq!(quality, 1.0);
        assert_eq!(policy.grade_for(quality), Grade::Excellent);
    }

    #[test]
    fn error_pins_with_good_metrics() {
        // ADR-1 discrete pins × continuous(1.0): 0E→1.0, 1E→0.70, 2E→0.40,
        // 3E→0.10, 4E→0.0, 5E→0.0 (threshold-crossing contract).
        let policy = DefaultScoringPolicy;
        let cases = [
            (0, 1.0),
            (1, 0.70),
            (2, 0.40),
            (3, 0.10),
            (4, 0.0),
            (5, 0.0),
        ];
        for (n_errors, expected) in cases {
            let quality = policy.quality_index(&errors(n_errors), &good_metrics());
            assert!(
                (quality - expected).abs() < 1e-9,
                "{n_errors} Errors with good metrics must score ≈ {expected}, got {quality}"
            );
        }
    }

    #[test]
    fn absent_metric_keys_preserve_zero_error_pin() {
        // "Absent metric keys default to neutral": with an EMPTY metrics map
        // the continuous component is exactly 1.0, so the 0E→1.0 pin holds —
        // the test harness populates only a subset of the keys.
        let policy = DefaultScoringPolicy;
        assert_eq!(policy.quality_index(&[], &BTreeMap::new()), 1.0);
    }

    #[test]
    fn same_severity_different_metrics_score_differently() {
        // "Same severity different metrics yield different scores": identical
        // observation sets must score differently when the continuous quality
        // differs (the acceptance gate identical_severity... depends on this).
        let policy = DefaultScoringPolicy;
        let observations = errors(2); // hard_safety pin 0.40 for both

        let mut dexterous = BTreeMap::new();
        dexterous.insert("avg_manipulability".to_string(), 0.75); // norm 1.0
        let mut stiff = BTreeMap::new();
        stiff.insert("avg_manipulability".to_string(), 0.36); // norm 0.72

        let score_dexterous = policy.quality_index(&observations, &dexterous);
        let score_stiff = policy.quality_index(&observations, &stiff);
        assert!(
            (score_dexterous - score_stiff).abs() > 1e-9,
            "identical severity counts must not produce identical scores: \
             {score_dexterous} vs {score_stiff}"
        );
        assert!(
            score_dexterous > score_stiff,
            "better manipulability must score higher: {score_dexterous} vs {score_stiff}"
        );
        // 0.40 × continuous: dexterous continuous = 1.0 → 0.40 exactly.
        assert!((score_dexterous - 0.40).abs() < 1e-9);
    }

    #[test]
    fn perfect_plan_with_bad_metrics_scores_below_one() {
        // "Perfect plan with bad metrics": zero observations but avg
        // manipulability < 0.3 → the continuous component penalizes, so the
        // score MUST NOT automatically be 1.0.
        let policy = DefaultScoringPolicy;
        let mut bad = BTreeMap::new();
        bad.insert("avg_manipulability".to_string(), 0.2); // < 0.3 per spec
        let quality = policy.quality_index(&[], &bad);
        assert!(
            (0.0..=1.0).contains(&quality) && quality < 1.0,
            "bad metrics must drag the score below 1.0, got {quality}"
        );
    }

    #[test]
    fn removing_only_error_restores_score() {
        // "Removing the only Error restores score": 1E with good metrics ≈
        // 0.70; removing it recovers to ≥ 0.99.
        let policy = DefaultScoringPolicy;
        let before = policy.quality_index(&errors(1), &good_metrics());
        assert!((before - 0.70).abs() < 1e-9, "1E → 0.70, got {before}");
        let after = policy.quality_index(&[], &good_metrics());
        assert!(after >= 0.99, "0E must restore score, got {after}");
    }

    #[test]
    fn score_recovery_requires_crossing_threshold() {
        // "Score recovery requires crossing threshold": 5E → 0.0, removing one
        // (4E) stays 0.0, removing another (3E) recovers to ≈ 0.10 × continuous.
        let policy = DefaultScoringPolicy;
        assert_eq!(policy.quality_index(&errors(5), &good_metrics()), 0.0);
        assert_eq!(policy.quality_index(&errors(4), &good_metrics()), 0.0);
        let recovered = policy.quality_index(&errors(3), &good_metrics());
        assert!(
            (recovered - 0.10).abs() < 1e-9,
            "3E must recover to ≈ 0.10, got {recovered}"
        );
    }

    #[test]
    fn hard_safety_matches_documented_bands() {
        // The hard-safety component is the discrete band × continuous: the
        // observable pins without continuous influence.
        let policy = DefaultScoringPolicy;
        let expected = [1.0, 0.70, 0.40, 0.10, 0.0, 0.0];
        for (n, want) in expected.iter().enumerate() {
            let hard = policy.hard_safety(&errors(n));
            assert!(
                (hard - want).abs() < 1e-12,
                "hard_safety({n}E) must be {want}, got {hard}"
            );
        }
    }

    #[test]
    fn raw_hard_safety_tracks_monotonically_beyond_saturation() {
        // "Removing one of many Errors still shows improvement": while the
        // observable score is clamped at 0.0, the internal raw component must
        // keep reflecting the reduction — 17E raw < 16E raw < 0.
        let policy = DefaultScoringPolicy;
        let raw_17 = policy.raw_hard_safety(&errors(17));
        let raw_16 = policy.raw_hard_safety(&errors(16));
        let raw_4 = policy.raw_hard_safety(&errors(4));
        assert!(
            raw_17 < raw_16,
            "17E raw {raw_17} must be below 16E raw {raw_16}"
        );
        assert!(
            raw_16 < raw_4,
            "16E raw {raw_16} must be below 4E raw {raw_4}"
        );
        // Both observable scores are pinned at 0.0 while saturated.
        assert_eq!(policy.quality_index(&errors(17), &good_metrics()), 0.0);
        assert_eq!(policy.quality_index(&errors(16), &good_metrics()), 0.0);
        // The raw component is exactly the unclamped 1 − Σ penalties.
        assert!((raw_17 - (1.0 - 17.0 * policy.penalty(Severity::Error))).abs() < 1e-12);
    }

    #[test]
    fn same_inputs_produce_exact_same_score() {
        // Determinism (C3): identical observations + identical metrics →
        // EXACT float equality (canonical severity-order sum, fixed map).
        let policy = DefaultScoringPolicy;
        let observations = vec![
            observation(Severity::Error),
            observation(Severity::Warning),
            observation(Severity::Info),
        ];
        assert_eq!(
            policy.quality_index(&observations, &good_metrics()),
            policy.quality_index(&observations, &good_metrics())
        );
    }
}

/// Property tests for the dual-component scoring semantics (design ADR-6,
/// spec quality-scoring-contract "Score Domain" + "Monotonic Improvement").
///
/// Properties:
/// 1. adding observations never raises the score (metrics fixed);
/// 2. removing observations never lowers the score (metrics fixed);
/// 3. the score stays in [0, 1] and is finite — even for arbitrary metric
///    values including NaN/±∞ (NaN → NEUTRAL, extremes clamp).
///
/// Metrics are randomized per case so the properties hold for ANY metric map,
/// not just the neutral one.
#[cfg(test)]
mod property_tests {
    use proptest::prelude::*;

    use super::{DefaultScoringPolicy, ScoringPolicy};
    use crate::analysis::location::Location;
    use crate::analysis::observation::{
        ArtifactRef, Observation, ObservationId, ObservationKind, Severity,
    };
    use crate::ids::MotionPlanId;
    use std::collections::BTreeMap;

    fn artifact() -> ArtifactRef {
        ArtifactRef::MotionPlan(MotionPlanId("mp-1".to_string()))
    }

    fn observation_strategy() -> impl Strategy<Value = Observation> {
        prop_oneof![
            Just(Severity::Info),
            Just(Severity::Warning),
            Just(Severity::Error),
        ]
        .prop_map(|severity| Observation {
            id: ObservationId(0),
            kind: ObservationKind::ResidualError,
            severity,
            artifact: artifact(),
            location: Location::Waypoint(0),
            attributes: BTreeMap::new(),
            causes: Vec::new(),
            related: Vec::new(),
        })
    }

    /// Arbitrary metric values: finite ranges (including negative and extreme)
    /// plus the specials — NaN MUST be treated as neutral, ±∞ MUST clamp.
    fn metric_value_strategy() -> impl Strategy<Value = f64> {
        prop_oneof![
            -1000.0..1000.0,
            -1e9f64..1e9,
            Just(f64::NAN),
            Just(f64::INFINITY),
            Just(f64::NEG_INFINITY),
        ]
    }

    fn metrics_strategy() -> impl Strategy<Value = BTreeMap<String, f64>> {
        let keys = prop::sample::select(vec![
            "avg_manipulability".to_string(),
            "smoothness".to_string(),
            "min_collision_distance".to_string(),
            "joint_safety.min_margin".to_string(),
            "orientation_change".to_string(),
            "unknown-key".to_string(),
            "another-unknown-key".to_string(),
        ]);
        prop::collection::btree_map(keys, metric_value_strategy(), 0..8)
    }

    proptest! {
        #![proptest_config(proptest::test_runner::Config::with_cases(128))]

        #[test]
        fn adding_observations_never_increases_quality(
            base in prop::collection::vec(observation_strategy(), 0..20),
            extra in prop::collection::vec(observation_strategy(), 0..20),
            metrics in metrics_strategy(),
        ) {
            // D6 + ADR-1: quality(S ∪ Δ, M) ≤ quality(S, M) for any fixed M —
            // penalties are additive non-negative, continuous is fixed.
            let policy = DefaultScoringPolicy;
            let base_quality = policy.quality_index(&base, &metrics);
            let mut union = base;
            union.extend(extra);
            let union_quality = policy.quality_index(&union, &metrics);
            prop_assert!(
                union_quality <= base_quality,
                "adding observations must not raise quality: \
                 {union_quality} > {base_quality}"
            );
        }

        #[test]
        fn removing_observations_never_decreases_quality(
            base in prop::collection::vec(observation_strategy(), 0..20),
            keep in prop::collection::vec(proptest::bool::ANY, 0..20),
            metrics in metrics_strategy(),
        ) {
            // Spec: if S' ⊂ S then quality(S') ≥ quality(S).
            let policy = DefaultScoringPolicy;
            let subset: Vec<Observation> = base
                .iter()
                .zip(keep.iter())
                .filter(|(_, keep)| **keep)
                .map(|(obs, _)| obs.clone())
                .collect();
            let base_quality = policy.quality_index(&base, &metrics);
            let subset_quality = policy.quality_index(&subset, &metrics);
            prop_assert!(
                subset_quality >= base_quality,
                "removing observations must not lower quality: \
                 {subset_quality} < {base_quality}"
            );
        }

        #[test]
        fn quality_stays_in_unit_interval_and_is_finite(
            observations in prop::collection::vec(observation_strategy(), 0..30),
            metrics in metrics_strategy(),
        ) {
            // Spec "Range invariant" + "NaN-free": [0, 1], finite, deterministic.
            let quality = DefaultScoringPolicy.quality_index(&observations, &metrics);
            prop_assert!(
                (0.0..=1.0).contains(&quality),
                "quality {quality} must be in [0, 1]"
            );
            prop_assert!(
                quality.is_finite(),
                "quality {quality} must be a finite number"
            );
        }
    }
}
