//! Aggregator — the source-agnostic `Vec<Observation> → AnalysisReport` step
//! (design D2/D3, spec I7/I8).
//!
//! # Source agnosticism (user contract C1)
//!
//! The aggregator knows ONLY the observation model and the scoring policy. It
//! has zero knowledge of `TrajectoryAnalyzer`, `ExecutionAnalyzer`, `Planner`
//! or the runtime — any producer of `Vec<Observation>` can be aggregated.
//!
//! # Scoring separation (design C2)
//!
//! [`DefaultAggregator`] is generic over [`ScoringPolicy`]: it composes a
//! policy (penalties + continuous metrics → `quality_index` → `grade`, design
//! ADR-1) but never owns weight values. Only the default policy exists today;
//! the trait is the seam for future policies.
//!
//! # Metrics threading (design ADR-1)
//!
//! [`Aggregator::aggregate_with_metrics`] is the production aggregation path:
//! the metrics map populates `report.metrics` and feeds the summary's
//! continuous-quality component in one call. The observation-only
//! [`Aggregator::aggregate`] treats every metric key as ABSENT (NEUTRAL 1.0),
//! reducing the score to the hard-safety pins — the historical behavior.
//!
//! # Sole report constructor (user contract C4)
//!
//! [`DefaultAggregator`] is the ONLY production path that builds an
//! [`AnalysisReport`] and its [`AnalysisSummary`]. No analyzer, service or DTO
//! constructs a summary by hand, so `quality_index` keeps a single,
//! aggregator-owned semantics.
//!
//! # Observation identity policy (closed decision)
//!
//! Incoming `ObservationId`s are IGNORED. The aggregator assigns a fresh
//! counter `1..=n` in input order and rewrites every `causes`/`related`
//! reference through the old-id → new-id map. This makes merging outputs of
//! independent analyzers collision-free (I8) and keeps the report fully
//! deterministic. A reference to an id that is not among the aggregated
//! observations cannot be remapped — it is an analyzer contract violation
//! (I4), surfaced loudly as a panic.
//!
//! # Structural safety net (design C1)
//!
//! After construction, [`AnalysisReport::validate`] runs as a safety net: the
//! aggregator guarantees structural validity by construction, so a violation
//! here means a programming error, not a user error. The infallible trait
//! signature has no error channel to carry an invalid report in — panicking
//! beats silently shipping a structurally invalid report to every consumer.

use std::collections::HashMap;

use crate::analysis::observation::{ArtifactRef, Observation, ObservationId};
use crate::analysis::report::AnalysisReport;
use crate::analysis::scoring::ScoringPolicy;
use crate::analysis::summary::AnalysisSummary;
use std::collections::BTreeMap;

/// Source-agnostic aggregator contract (design D2/D3): observations in,
/// a canonical [`AnalysisReport`] out.
pub trait Aggregator {
    /// Aggregates observations produced by any analyzer(s) into a canonical
    /// report with a derived summary (quality_index, counts, grade).
    ///
    /// Observation-only path (design ADR-1): with no continuous-metric
    /// information available, every metric key is treated as ABSENT — the
    /// continuous component is NEUTRAL (1.0) and the score reduces to the
    /// hard-safety pins. Backward-compatible; `report.metrics` is empty.
    fn aggregate(&self, artifact: ArtifactRef, observations: Vec<Observation>) -> AnalysisReport {
        self.aggregate_with_metrics(artifact, observations, BTreeMap::new())
    }

    /// Full aggregation (design ADR-1): the metrics map populates
    /// `report.metrics` AND feeds the summary's continuous-quality component
    /// in the same call. THE production path — `PlanAnalysisService` (and the
    /// usability harness that mirrors it) calls this with the technical
    /// analysis metrics, so the score reflects trajectory reality.
    fn aggregate_with_metrics(
        &self,
        artifact: ArtifactRef,
        observations: Vec<Observation>,
        metrics: BTreeMap<String, f64>,
    ) -> AnalysisReport;
}

/// Default aggregator implementation, generic over the [`ScoringPolicy`]
/// (design C2: aggregator composes policy, never owns weights).
#[derive(Debug, Clone, Copy)]
pub struct DefaultAggregator<P: ScoringPolicy> {
    policy: P,
}

impl<P: ScoringPolicy> DefaultAggregator<P> {
    /// Creates an aggregator driven by the given scoring policy.
    pub fn new(policy: P) -> Self {
        Self { policy }
    }

    /// Builds the derived summary over the report's observations (I7): a small
    /// projection computed here, never hand-written by analyzers. The
    /// continuous metrics feed the score (design ADR-1).
    fn build_summary(
        &self,
        observations: &[Observation],
        metrics: &BTreeMap<String, f64>,
    ) -> AnalysisSummary {
        let quality_index = self.policy.quality_index(observations, metrics);
        let mut severity_distribution = BTreeMap::new();
        for observation in observations {
            *severity_distribution
                .entry(observation.severity)
                .or_insert(0) += 1;
        }
        AnalysisSummary {
            quality_index,
            observation_count: observations.len(),
            severity_distribution,
            grade: self.policy.grade_for(quality_index),
        }
    }
}

impl<P: ScoringPolicy> Aggregator for DefaultAggregator<P> {
    fn aggregate_with_metrics(
        &self,
        artifact: ArtifactRef,
        mut observations: Vec<Observation>,
        metrics: BTreeMap<String, f64>,
    ) -> AnalysisReport {
        // 1. Identity: ignore incoming ids, assign a fresh counter 1..=n in
        //    input order (closed decision; I8 collision-free merging).
        let mut id_map: HashMap<ObservationId, ObservationId> = HashMap::new();
        for (index, observation) in observations.iter_mut().enumerate() {
            let fresh = ObservationId((index + 1) as u32);
            id_map.insert(observation.id, fresh);
            observation.id = fresh;
        }

        // 2. Rewrite causal/related references to the fresh ids (I4). A
        //    reference to an id outside the aggregated set is an analyzer
        //    contract violation — cannot be remapped, surfaced loudly.
        for observation in &mut observations {
            for cause in &mut observation.causes {
                *cause = *id_map.get(cause).unwrap_or_else(|| {
                    panic!(
                        "aggregator: observation {:?} references observation id {cause:?} \
                         which is not among the aggregated observations (I4 contract violation)",
                        observation.id
                    )
                });
            }
            for related in &mut observation.related {
                *related = *id_map.get(related).unwrap_or_else(|| {
                    panic!(
                        "aggregator: observation {:?} references observation id {related:?} \
                         which is not among the aggregated observations (I4 contract violation)",
                        observation.id
                    )
                });
            }
        }

        let summary = self.build_summary(&observations, &metrics);
        let report = AnalysisReport {
            artifact,
            observations,
            actions: Vec::new(),
            metrics,
            summary,
            // The aggregator does not know the robot identity (spec
            // robot-identity): the scene-owned `robot_id` is stamped by the
            // caller (plan_analysis handler) from the runtime snapshot.
            robot_id: None,
        };

        // 3. Structural safety net (design C1): the aggregator guarantees
        //    validity by construction; a violation is a programming error.
        report
            .validate()
            .expect("aggregator produced a structurally invalid report");

        report
    }
}

#[cfg(test)]
mod tests {
    use super::{Aggregator, DefaultAggregator};
    use crate::analysis::location::Location;
    use crate::analysis::observation::{
        ArtifactRef, Observation, ObservationId, ObservationKind, Severity,
    };
    use crate::analysis::scoring::{DefaultScoringPolicy, ScoringPolicy};
    use crate::analysis::summary::Grade;
    use crate::ids::MotionPlanId;
    use serde_json::json;
    use std::collections::BTreeMap;

    fn artifact() -> ArtifactRef {
        ArtifactRef::MotionPlan(MotionPlanId("mp-1".to_string()))
    }

    fn observation(id: u32, severity: Severity) -> Observation {
        Observation {
            id: ObservationId(id),
            kind: ObservationKind::ResidualError,
            severity,
            artifact: artifact(),
            location: Location::Waypoint(0),
            attributes: BTreeMap::new(),
            causes: Vec::new(),
            related: Vec::new(),
        }
    }

    fn aggregator() -> DefaultAggregator<DefaultScoringPolicy> {
        DefaultAggregator::new(DefaultScoringPolicy)
    }

    #[test]
    fn zero_observations_yield_perfect_quality() {
        // Spec "Observations drive quality": an artifact with nothing wrong
        // observed gets quality_index 1.0, Excellent, and an empty distribution.
        let report = aggregator().aggregate(artifact(), Vec::new());
        assert_eq!(report.summary.quality_index, 1.0);
        assert_eq!(report.summary.grade, Grade::Excellent);
        assert_eq!(report.summary.observation_count, 0);
        assert!(report.summary.severity_distribution.is_empty());
    }

    #[test]
    fn aggregation_is_deterministic() {
        // Spec "Deterministic computation": same input → same report, exactly.
        let observations = vec![
            observation(1, Severity::Error),
            observation(2, Severity::Warning),
            observation(3, Severity::Info),
        ];
        let first = aggregator().aggregate(artifact(), observations.clone());
        let second = aggregator().aggregate(artifact(), observations);
        assert_eq!(first, second);
        assert_eq!(first.summary.quality_index, second.summary.quality_index);
    }

    #[test]
    fn aggregator_assigns_unique_sequential_ids() {
        // I8 + closed decision "ObservationId": incoming ids are ignored and
        // replaced by a fresh counter 1..=n in input order, so merging outputs
        // of independent analyzers can never collide — even when analyzers pass
        // duplicate or arbitrary ids.
        let observations = vec![
            observation(7, Severity::Error),
            observation(7, Severity::Warning),
            observation(99, Severity::Info),
        ];
        let report = aggregator().aggregate(artifact(), observations);
        let ids: Vec<u32> = report.observations.iter().map(|o| o.id.0).collect();
        assert_eq!(ids, vec![1, 2, 3]);
    }

    #[test]
    fn causes_are_remapped_to_aggregator_assigned_ids() {
        // I4: causal references written by analyzers against THEIR ids must be
        // rewritten to the aggregator ids, keeping the report structurally valid.
        let mut child = observation(8, Severity::Warning);
        let mut parent = observation(9, Severity::Error);
        child.causes = vec![ObservationId(9)];
        parent.causes = Vec::new();
        let report = aggregator().aggregate(artifact(), vec![child, parent]);
        assert_eq!(report.observations[0].id, ObservationId(1));
        assert_eq!(report.observations[1].id, ObservationId(2));
        assert_eq!(report.observations[0].causes, vec![ObservationId(2)]);
    }

    #[test]
    #[should_panic(expected = "I4 contract violation")]
    fn dangling_cause_reference_is_a_contract_violation() {
        // A cause referencing an id that is not among the aggregated
        // observations cannot be remapped — it is an analyzer contract
        // violation, surfaced loudly by the aggregator (safety net, design C1).
        let mut dangling = observation(1, Severity::Error);
        dangling.causes = vec![ObservationId(42)];
        aggregator().aggregate(artifact(), vec![dangling]);
    }

    #[test]
    fn summary_reflects_observation_counts() {
        // The summary is derived from the report's observations: counts and
        // severity distribution match exactly.
        let observations = vec![
            observation(1, Severity::Error),
            observation(2, Severity::Error),
            observation(3, Severity::Warning),
        ];
        let report = aggregator().aggregate(artifact(), observations);
        assert_eq!(report.summary.observation_count, 3);
        assert_eq!(report.summary.severity_distribution[&Severity::Error], 2);
        assert_eq!(report.summary.severity_distribution[&Severity::Warning], 1);
        assert!(
            !report
                .summary
                .severity_distribution
                .contains_key(&Severity::Info)
        );
    }

    #[test]
    fn summary_grade_is_consistent_with_the_policy() {
        // grade is a deterministic projection of quality_index per the policy.
        let observations = vec![
            observation(1, Severity::Error),
            observation(2, Severity::Warning),
        ];
        let report = aggregator().aggregate(artifact(), observations);
        let policy = DefaultScoringPolicy;
        assert_eq!(
            report.summary.grade,
            policy.grade_for(report.summary.quality_index)
        );
    }

    #[test]
    fn report_has_single_quality_measure() {
        // I7 negative: after aggregation the summary carries quality_index as
        // the ONLY aggregate quality field — no health_score, no summary.score.
        let observations = vec![observation(1, Severity::Error)];
        let report = aggregator().aggregate(artifact(), observations);
        let value = serde_json::to_value(&report.summary).expect("serialize");
        let obj = value.as_object().expect("object");
        for banned in ["health_score", "score"] {
            assert!(
                !obj.contains_key(banned),
                "aggregated summary must not carry `{banned}`"
            );
        }
        assert_eq!(obj["quality_index"], json!(report.summary.quality_index));
    }

    #[test]
    fn aggregated_report_passes_structural_validation() {
        // Safety net (design C1): whatever the aggregator builds must satisfy
        // the report's structural invariants (unique ids, acyclic causes, …).
        let observations = vec![
            observation(1, Severity::Error),
            observation(2, Severity::Warning),
            observation(3, Severity::Info),
        ];
        let report = aggregator().aggregate(artifact(), observations);
        assert_eq!(report.validate(), Ok(()));
    }

    // ─── Metrics threading (design ADR-1, T2) ───────────────────────────

    #[test]
    fn aggregate_with_metrics_populates_report_metrics() {
        // The metrics map passed to `aggregate_with_metrics` must ride into
        // `report.metrics` (the production path: the runtime service populates
        // the report's metrics via the aggregator, not after it).
        let mut metrics = BTreeMap::new();
        metrics.insert("avg_manipulability".to_string(), 0.5);
        let report = aggregator().aggregate_with_metrics(
            artifact(),
            vec![observation(1, Severity::Error)],
            metrics.clone(),
        );
        assert_eq!(report.metrics, metrics);
    }

    #[test]
    fn aggregate_with_metrics_feeds_the_summary_quality() {
        // The summary's quality_index must reflect the metrics: 1 Error scores
        // 0.70 with neutral/perfect metrics, but LESS when the continuous
        // component penalizes (avg_manipulability 0.1 → norm 0.2).
        let good = {
            let mut m = BTreeMap::new();
            m.insert("avg_manipulability".to_string(), 0.5);
            m
        };
        let bad = {
            let mut m = BTreeMap::new();
            m.insert("avg_manipulability".to_string(), 0.1);
            m
        };
        let observations = vec![observation(1, Severity::Error)];
        let good_report =
            aggregator().aggregate_with_metrics(artifact(), observations.clone(), good);
        let bad_report = aggregator().aggregate_with_metrics(artifact(), observations, bad);

        assert!(
            (good_report.summary.quality_index - 0.70).abs() < 1e-9,
            "1 Error + good metrics must score ≈ 0.70, got {}",
            good_report.summary.quality_index
        );
        assert!(
            bad_report.summary.quality_index < good_report.summary.quality_index,
            "penalized metrics must lower the score: {} vs {}",
            bad_report.summary.quality_index,
            good_report.summary.quality_index
        );
        assert!(
            bad_report.summary.quality_index > 0.0,
            "1 Error with bad metrics must stay above 0.0 (not saturated)"
        );
    }

    #[test]
    fn observation_only_aggregate_keeps_neutral_continuous() {
        // The 2-arg `aggregate` (no metrics available) must keep the historical
        // behavior: absent keys → NEUTRAL 1.0 → score = hard-safety pins.
        let observations = vec![
            observation(1, Severity::Error),
            observation(2, Severity::Error),
        ];
        let report = aggregator().aggregate(artifact(), observations);
        assert!((report.summary.quality_index - 0.40).abs() < 1e-9);
        assert!(report.metrics.is_empty());
    }
}

/// Property tests for the scoring semantics (spec `analysis-score-semantics`
/// and the closed monotonicity decision). All properties are INDEPENDENT of
/// the concrete default weights — they hold for any non-negative penalties,
/// and the zero-penalty property is exercised with a dedicated policy whose
/// `Info` weight is exactly zero.
///
/// Properties (user contract C3):
/// 1. determinism — same input → same output;
/// 2. range — `quality_index` stays in `[0, 1]`;
/// 3. absence of NaN — the index is always finite;
/// 4. zero-penalty observations do not change the index;
/// 5. commutativity — observation order does not affect the index;
/// 6. monotonicity (D6) — `quality(S ∪ Δ) ≤ quality(S)`.
#[cfg(test)]
mod property_tests {
    use proptest::prelude::*;

    use super::{Aggregator, DefaultAggregator};
    use crate::analysis::location::Location;
    use crate::analysis::observation::{
        ArtifactRef, Observation, ObservationId, ObservationKind, Severity,
    };
    use crate::analysis::scoring::{DefaultScoringPolicy, ScoringPolicy};
    use crate::ids::MotionPlanId;
    use std::collections::BTreeMap;

    /// A policy with a genuinely zero `Info` penalty — the setup under which
    /// "adding a zero-penalty observation does not change the index" holds.
    #[derive(Debug)]
    struct ZeroInfoPenaltyPolicy;

    impl ScoringPolicy for ZeroInfoPenaltyPolicy {
        fn penalty(&self, severity: Severity) -> f64 {
            match severity {
                Severity::Info => 0.0,
                Severity::Warning => 0.15,
                Severity::Error => 0.30,
            }
        }
    }

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
            id: ObservationId(0), // incoming ids are ignored by the aggregator
            kind: ObservationKind::ResidualError,
            severity,
            artifact: artifact(),
            location: Location::Waypoint(0),
            attributes: BTreeMap::new(),
            causes: Vec::new(),
            related: Vec::new(),
        })
    }

    /// An `Info` observation — the zero-penalty case under
    /// [`ZeroInfoPenaltyPolicy`].
    fn info_observation() -> Observation {
        Observation {
            id: ObservationId(0),
            kind: ObservationKind::ResidualError,
            severity: Severity::Info,
            artifact: artifact(),
            location: Location::Waypoint(0),
            attributes: BTreeMap::new(),
            causes: Vec::new(),
            related: Vec::new(),
        }
    }

    proptest! {
        // 128 cases per property (reasonable CI load; each case is a few
        // pure float ops). Proptest 1.11 configures the block via inner
        // attribute — module-level `proptest_config` was removed.
        #![proptest_config(proptest::test_runner::Config::with_cases(128))]

        #[test]
        fn monotonicity_adding_observations_never_raises_quality(
            base in prop::collection::vec(observation_strategy(), 0..20),
            extra in prop::collection::vec(observation_strategy(), 0..20),
        ) {
            // D6 (closed decision): quality(S ∪ Δ) ≤ quality(S). Proven by
            // property test in CI, never by runtime asserts.
            let policy = DefaultAggregator::new(DefaultScoringPolicy);
            let base_quality = policy
                .aggregate(artifact(), base.clone())
                .summary
                .quality_index;
            let mut union = base;
            union.extend(extra);
            let union_quality = policy.aggregate(artifact(), union).summary.quality_index;
            prop_assert!(
                union_quality <= base_quality,
                "quality({union_quality}) must not exceed base({base_quality})"
            );
        }

        #[test]
        fn quality_index_stays_in_unit_interval(
            observations in prop::collection::vec(observation_strategy(), 0..50),
        ) {
            let quality = DefaultAggregator::new(DefaultScoringPolicy)
                .aggregate(artifact(), observations)
                .summary
                .quality_index;
            prop_assert!(
                (0.0..=1.0).contains(&quality),
                "quality {quality} must be in [0, 1]"
            );
        }

        #[test]
        fn quality_index_is_never_nan(
            observations in prop::collection::vec(observation_strategy(), 0..50),
        ) {
            let quality = DefaultAggregator::new(DefaultScoringPolicy)
                .aggregate(artifact(), observations)
                .summary
                .quality_index;
            prop_assert!(
                quality.is_finite(),
                "quality {quality} must be a finite number"
            );
        }

        #[test]
        fn zero_penalty_observations_do_not_change_quality(
            base in prop::collection::vec(observation_strategy(), 0..20),
            infos in prop::collection::vec(Just(()), 0..10),
        ) {
            // C3 property 4: under a policy where Info carries zero penalty,
            // appending Info observations leaves the index untouched. The
            // default weights are irrelevant here — only the model matters.
            let policy = ZeroInfoPenaltyPolicy;
            let base_quality = policy.quality_index(&base, &BTreeMap::new());
            let mut union = base;
            union.extend(infos.into_iter().map(|_| info_observation()));
            let union_quality = policy.quality_index(&union, &BTreeMap::new());
            prop_assert_eq!(
                base_quality,
                union_quality,
                "zero-penalty observations must not change the index"
            );
        }

        #[test]
        fn quality_is_independent_of_observation_order(
            observations in prop::collection::vec(observation_strategy(), 0..30),
            rotation in 0usize..30,
        ) {
            // C3 property 5: permutation invariance. Rotation + reversal are
            // non-trivial permutations; the canonical per-severity summation
            // guarantees EXACT float equality, not just tolerance.
            let policy = DefaultScoringPolicy;
            let quality = policy.quality_index(&observations, &BTreeMap::new());
            let mut rotated = observations.clone();
            let amount = rotation % rotated.len().max(1);
            rotated.rotate_left(amount);
            let mut reversed = observations.clone();
            reversed.reverse();
            prop_assert_eq!(quality, policy.quality_index(&rotated, &BTreeMap::new()));
            prop_assert_eq!(quality, policy.quality_index(&reversed, &BTreeMap::new()));
        }

        #[test]
        fn aggregation_is_deterministic(
            observations in prop::collection::vec(observation_strategy(), 0..30),
        ) {
            // C3 property 1: the same input always yields the exact same
            // report — id assignment, summary and all.
            let aggregator = DefaultAggregator::new(DefaultScoringPolicy);
            let first = aggregator.aggregate(artifact(), observations.clone());
            let second = aggregator.aggregate(artifact(), observations);
            prop_assert_eq!(first, second);
        }
    }
}
