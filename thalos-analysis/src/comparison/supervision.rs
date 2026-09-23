//! Expected-vs-observed comparison of a supervision tick.
//!
//! This is A4: it answers **"what was expected and what occurred?"** It is NOT a
//! decision and NOT a deviation. The deviation is the outcome A5 derives from a
//! `ConditionOutcome`/kinematic error; the comparison itself only pairs the
//! two sides and preserves the correspondence `condition → observation → result`
//! so the pairing can be reconstructed without consulting the device again.
//!
//! Two branches, two expectation sources, neither invented here:
//!
//! - **kinematic** — the plan-derived `ExpectedState` vs the observed
//!   `RobotSample`, through the SAME neutral metric core used for
//!   plan-vs-execution analysis (`compute_metrics`);
//! - **signal** — each declared [`SignalCondition`] vs the tick's observation.

use std::time::Duration;

use serde::Serialize;

use thalos_core::device::ChannelObservation;
use thalos_core::execution::{SignalCondition, TickEvaluation};

use super::alignment::AlignedPair;
use super::metrics::{ComparisonMetrics, compute_metrics};

/// Result of evaluating one declared signal condition against an observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ConditionOutcome {
    /// The condition was evaluated and holds.
    Satisfied,
    /// The condition was evaluated and does not hold.
    Violated,
    /// The condition could NOT be evaluated: the observation is absent, or its
    /// value cannot be compared. This is NEVER collapsed into `Violated`.
    Unknown,
}

/// Correspondence `condition → observation → result` for one signal.
///
/// The observation is retained so the pairing is reconstructible offline.
#[derive(Debug, Clone, Serialize)]
pub struct SignalComparison {
    pub condition: SignalCondition,
    /// The observation the condition was checked against; `None` ⇒ `Unknown`.
    pub observation: Option<ChannelObservation>,
    pub outcome: ConditionOutcome,
}

/// Expected-vs-observed comparison of one tick.
///
/// A tick ALWAYS produces a comparison; whether the two sides agree is the
/// outcome, not a precondition. This is data, not a decision.
#[derive(Debug, Clone, Serialize)]
pub struct TickComparison {
    pub index: u64,
    pub timestamp_ns: u64,
    /// Kinematic comparison of the tick (empty when no joint reference exists).
    pub kinematic: ComparisonMetrics,
    /// One entry per declared condition, in declaration order.
    pub conditions: Vec<SignalComparison>,
}

/// Compare a tick evaluation against the declared signal conditions.
pub fn compare_tick(evaluation: &TickEvaluation, conditions: &[SignalCondition]) -> TickComparison {
    TickComparison {
        index: evaluation.index,
        timestamp_ns: evaluation.timestamp_ns,
        kinematic: kinematic_comparison(evaluation),
        conditions: signal_comparisons(evaluation, conditions),
    }
}

/// Kinematic branch: plan-derived expected joints vs observed joints.
///
/// When no comparable joint reference exists (either side empty, or differing
/// dimensionality), no kinematic reference is produced — reported as
/// `aligned_count == 0` rather than a fabricated error.
fn kinematic_comparison(evaluation: &TickEvaluation) -> ComparisonMetrics {
    let expected = &evaluation.expected.simulated_joints;
    let observed = &evaluation.observed.joints;

    if expected.is_empty() || observed.len() != expected.len() {
        return compute_metrics(&[]);
    }

    let pair = AlignedPair {
        timestamp: Duration::from_nanos(evaluation.timestamp_ns),
        planned_joints: expected.clone(),
        actual_joints: observed.clone(),
        // The plan-derived expected state carries positions only; no expected
        // velocity is invented.
        planned_velocities: Vec::new(),
        actual_velocities: evaluation.observed.velocities.clone(),
        tracking_error: None,
    };
    compute_metrics(&[pair])
}

/// Signal branch: each declared condition against its observation.
fn signal_comparisons(
    evaluation: &TickEvaluation,
    conditions: &[SignalCondition],
) -> Vec<SignalComparison> {
    conditions
        .iter()
        .map(|condition| {
            let observation = evaluation
                .observations
                .observations
                .get(&condition.channel_id)
                .cloned();

            let outcome = match &observation {
                Some(obs) => match condition.evaluate(obs) {
                    Some(true) => ConditionOutcome::Satisfied,
                    Some(false) => ConditionOutcome::Violated,
                    // Present but not comparable → Unknown, not Violated.
                    None => ConditionOutcome::Unknown,
                },
                None => ConditionOutcome::Unknown,
            };

            SignalComparison {
                condition: condition.clone(),
                observation,
                outcome,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::device::{ChannelValue, SignalQuality};
    use thalos_core::execution::{
        ComparisonOp, ExpectedState, ObservationBundle, RobotSample,
    };

    fn evaluation_with(
        expected_joints: Vec<f64>,
        observed_joints: Vec<f64>,
        observations: ObservationBundle,
    ) -> TickEvaluation {
        TickEvaluation {
            index: 1,
            timestamp_ns: 16_000_000,
            expected: ExpectedState {
                simulated_joints: expected_joints,
            },
            observed: RobotSample {
                joints: observed_joints,
                velocities: Vec::new(),
            },
            observations,
        }
    }

    fn observation(channel: &str, value: ChannelValue) -> ChannelObservation {
        ChannelObservation {
            channel_id: channel.to_string(),
            sequence: 1,
            sampled_at_ns: 1_000_000,
            received_at_ns: 1_005_000,
            value,
            unit: None,
            quality: SignalQuality::Nominal,
        }
    }

    fn bundle(channel: &str, value: ChannelValue) -> ObservationBundle {
        let mut bundle = ObservationBundle::default();
        bundle
            .observations
            .insert(channel.to_string(), observation(channel, value));
        bundle
    }

    #[test]
    fn matching_expected_and_observed_produce_zero_error() {
        let evaluation = evaluation_with(vec![0.0, 0.0], vec![0.0, 0.0], ObservationBundle::default());
        let comparison = compare_tick(&evaluation, &[]);
        assert_eq!(comparison.kinematic.aligned_count, 1);
        assert!(comparison.kinematic.global_max_error.abs() < 1e-12);
        assert!(comparison.kinematic.global_rmse.abs() < 1e-12);
    }

    #[test]
    fn diverging_expected_and_observed_are_reported_without_deciding() {
        let evaluation = evaluation_with(vec![0.0, 0.0], vec![0.1, 0.0], ObservationBundle::default());
        let comparison = compare_tick(&evaluation, &[]);
        assert!((comparison.kinematic.global_max_error - 0.1).abs() < 1e-12);
        assert!(comparison.kinematic.global_rmse > 0.0);
    }

    #[test]
    fn no_kinematic_reference_is_reported_instead_of_a_fabricated_error() {
        let evaluation = evaluation_with(vec![], vec![0.1], ObservationBundle::default());
        let comparison = compare_tick(&evaluation, &[]);
        assert_eq!(comparison.kinematic.aligned_count, 0);
    }

    #[test]
    fn condition_is_satisfied() {
        let conditions = vec![SignalCondition::new("temperature", ComparisonOp::Le, 30.0)];
        let evaluation = evaluation_with(
            vec![0.0],
            vec![0.0],
            bundle("temperature", ChannelValue::Scalar(25.0)),
        );
        let comparison = compare_tick(&evaluation, &conditions);
        assert_eq!(comparison.conditions.len(), 1);
        assert_eq!(comparison.conditions[0].outcome, ConditionOutcome::Satisfied);
    }

    #[test]
    fn condition_is_violated() {
        let conditions = vec![SignalCondition::new("temperature", ComparisonOp::Le, 30.0)];
        let evaluation = evaluation_with(
            vec![0.0],
            vec![0.0],
            bundle("temperature", ChannelValue::Scalar(31.5)),
        );
        let comparison = compare_tick(&evaluation, &conditions);
        assert_eq!(comparison.conditions[0].outcome, ConditionOutcome::Violated);
    }

    #[test]
    fn absent_observation_is_unknown_and_never_violated() {
        let conditions = vec![SignalCondition::new("temperature", ComparisonOp::Le, 30.0)];
        let evaluation = evaluation_with(vec![0.0], vec![0.0], ObservationBundle::default());
        let comparison = compare_tick(&evaluation, &conditions);
        assert_eq!(comparison.conditions[0].outcome, ConditionOutcome::Unknown);
        assert_ne!(comparison.conditions[0].outcome, ConditionOutcome::Violated);
        assert!(comparison.conditions[0].observation.is_none());
    }

    #[test]
    fn correspondence_condition_observation_result_is_preserved() {
        let condition = SignalCondition::new("temperature", ComparisonOp::Le, 30.0);
        let observed = observation("temperature", ChannelValue::Scalar(31.5));
        let mut observations = ObservationBundle::default();
        observations
            .observations
            .insert("temperature".to_string(), observed.clone());

        let evaluation = evaluation_with(vec![0.0], vec![0.0], observations);
        let comparison = compare_tick(&evaluation, std::slice::from_ref(&condition));

        let entry = &comparison.conditions[0];
        assert_eq!(entry.condition, condition);
        assert_eq!(entry.observation.as_ref(), Some(&observed));
        assert_eq!(entry.outcome, ConditionOutcome::Violated);
    }

    #[test]
    fn every_declared_condition_is_evaluated() {
        let conditions = vec![
            SignalCondition::new("temperature", ComparisonOp::Le, 30.0),
            SignalCondition::new("ready", ComparisonOp::Eq, 1.0),
        ];
        let evaluation = evaluation_with(
            vec![0.0],
            vec![0.0],
            bundle("temperature", ChannelValue::Scalar(25.0)),
        );
        let comparison = compare_tick(&evaluation, &conditions);
        assert_eq!(comparison.conditions.len(), 2, "declaration order preserved");
        assert_eq!(comparison.conditions[0].outcome, ConditionOutcome::Satisfied);
        // `ready` was not observed → Unknown, not Violated.
        assert_eq!(comparison.conditions[1].outcome, ConditionOutcome::Unknown);
    }
}
