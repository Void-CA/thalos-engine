//! Expected-vs-observed deviations of a supervision tick.
//!
//! This is A5: it answers **"does the observed evidence demonstrate that an
//! expectation was NOT met?"** It is evidence, not a response: there is no
//! decision, priority, severity, policy or reaction here.
//!
//! The three comparison outcomes are preserved:
//!
//! | Comparison outcome | Deviation |
//! |---|---|
//! | `Satisfied` | none |
//! | `Violated`  | deviation / violation |
//! | `Unknown`   | **none** — the evidence does not demonstrate a violation |
//!
//! Collapsing `Unknown` into a deviation would violate NFR-INT-01 through
//! another route, so it is forbidden by construction.
//!
//! The two semantics are kept distinct under a common abstraction — *an
//! expectation that was not met* — while preserving the specific evidence that
//! showed it (kinematic metrics vs the declared condition + observation).

use serde::Serialize;

use thalos_core::device::ChannelObservation;
use thalos_core::deviation::{EnvelopeStatus, TolerancePolicy};
use thalos_core::execution::SignalCondition;

use super::supervision::{ConditionOutcome, TickComparison};

/// A kinematic expectation not met, with the metrics that showed it.
#[derive(Debug, Clone, Serialize)]
pub struct KinematicViolation {
    pub index: u64,
    pub timestamp_ns: u64,
    pub max_abs_error: f64,
    pub rmse: f64,
    /// Per-joint max absolute error (rad).
    pub per_joint_max_error: Vec<f64>,
    /// Per-joint tolerance used to declare the violation.
    pub per_joint_tolerance: Vec<f64>,
    pub envelope: EnvelopeStatus,
}

/// A DECLARED signal condition that was violated, with its observation.
#[derive(Debug, Clone, Serialize)]
pub struct SignalViolation {
    pub index: u64,
    pub timestamp_ns: u64,
    pub condition: SignalCondition,
    pub observation: ChannelObservation,
}

/// An expectation that was not met. The variant preserves the specific
/// evidence; the two semantics are not artificially unified.
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Deviation {
    Kinematic(KinematicViolation),
    Signal(SignalViolation),
}

/// Deviations detected for one tick.
#[derive(Debug, Clone, Serialize)]
pub struct TickDeviation {
    pub index: u64,
    pub timestamp_ns: u64,
    pub deviations: Vec<Deviation>,
}

impl TickDeviation {
    /// Whether the evidence demonstrated any unfulfilled expectation.
    pub fn is_empty(&self) -> bool {
        self.deviations.is_empty()
    }
}

/// Detect deviations from a tick comparison.
///
/// Kinematics are judged against an explicit `TolerancePolicy` (the declared
/// kinematic expectation). Signals are judged by the comparison outcome:
/// only `Violated` yields a deviation — `Satisfied` and `Unknown` do not.
pub fn detect_deviations<P: TolerancePolicy>(
    comparison: &TickComparison,
    kinematic_tolerance: &P,
) -> TickDeviation {
    let mut deviations = Vec::new();

    // ── Kinematic branch: only when a comparable joint reference exists ─────
    if comparison.kinematic.aligned_count > 0 {
        let per_joint_max_error = comparison.kinematic.per_joint.max_error.clone();
        let per_joint_tolerance: Vec<f64> = (0..per_joint_max_error.len())
            .map(|joint| kinematic_tolerance.joint_tolerance(joint).position)
            .collect();

        let exceeded = per_joint_max_error
            .iter()
            .zip(per_joint_tolerance.iter())
            .any(|(error, tolerance)| error > tolerance);

        if exceeded {
            deviations.push(Deviation::Kinematic(KinematicViolation {
                index: comparison.index,
                timestamp_ns: comparison.timestamp_ns,
                max_abs_error: comparison.kinematic.global_max_error,
                rmse: comparison.kinematic.global_rmse,
                per_joint_max_error,
                per_joint_tolerance,
                envelope: EnvelopeStatus::Violated,
            }));
        }
    }

    // ── Signal branch: only `Violated` is a deviation ──────────────────────
    for signal in &comparison.conditions {
        if signal.outcome != ConditionOutcome::Violated {
            // Satisfied → no deviation; Unknown → the evidence does not
            // demonstrate a violation. Neither is a deviation.
            continue;
        }
        if let Some(observation) = &signal.observation {
            deviations.push(Deviation::Signal(SignalViolation {
                index: comparison.index,
                timestamp_ns: comparison.timestamp_ns,
                condition: signal.condition.clone(),
                observation: observation.clone(),
            }));
        }
    }

    TickDeviation {
        index: comparison.index,
        timestamp_ns: comparison.timestamp_ns,
        deviations,
    }
}

#[cfg(test)]
mod tests {
    use super::super::supervision::{TickComparison, compare_tick};
    use super::*;
    use thalos_core::device::{ChannelValue, SignalQuality};
    use thalos_core::deviation::StaticTolerancePolicy;
    use thalos_core::execution::{
        ComparisonOp, ExpectedState, ObservationBundle, RobotSample, SignalCondition,
        TickEvaluation,
    };

    fn tolerance(position: f64) -> StaticTolerancePolicy {
        StaticTolerancePolicy::uniform(2, position, position)
    }

    fn evaluation(
        expected: Vec<f64>,
        observed: Vec<f64>,
        observations: ObservationBundle,
    ) -> TickEvaluation {
        TickEvaluation {
            index: 1,
            timestamp_ns: 16_000_000,
            expected: ExpectedState {
                simulated_joints: expected,
            },
            observed: RobotSample {
                joints: observed,
                velocities: Vec::new(),
            },
            observations,
        }
    }

    fn bundle(channel: &str, value: Option<ChannelValue>) -> ObservationBundle {
        let mut bundle = ObservationBundle::default();
        if let Some(value) = value {
            bundle.observations.insert(
                channel.to_string(),
                ChannelObservation {
                    channel_id: channel.to_string(),
                    sequence: 1,
                    sampled_at_ns: 1_000_000,
                    received_at_ns: 1_005_000,
                    value,
                    unit: None,
                    quality: SignalQuality::Nominal,
                },
            );
        }
        bundle
    }

    fn comparison(condition: &SignalCondition, value: Option<ChannelValue>, divergence: f64) -> TickComparison {
        compare_tick(
            &evaluation(
                vec![0.0, 0.0],
                vec![divergence, 0.0],
                bundle(&condition.channel_id, value),
            ),
            std::slice::from_ref(condition),
        )
    }

    #[test]
    fn satisfied_condition_produces_no_deviation() {
        let condition = SignalCondition::new("temperature", ComparisonOp::Le, 30.0);
        let comparison = comparison(&condition, Some(ChannelValue::Scalar(25.0)), 0.0);
        assert!(detect_deviations(&comparison, &tolerance(1.0)).is_empty());
    }

    #[test]
    fn violated_condition_produces_a_signal_violation_with_its_observation() {
        let condition = SignalCondition::new("temperature", ComparisonOp::Le, 30.0);
        let comparison = comparison(&condition, Some(ChannelValue::Scalar(31.5)), 0.0);
        let deviation = detect_deviations(&comparison, &tolerance(1.0));

        assert_eq!(deviation.deviations.len(), 1);
        match &deviation.deviations[0] {
            Deviation::Signal(violation) => {
                assert_eq!(violation.condition, condition);
                assert_eq!(violation.observation.value, ChannelValue::Scalar(31.5));
                assert_eq!(violation.observation.quality, SignalQuality::Nominal);
            }
            other => panic!("expected a signal violation, got {other:?}"),
        }
    }

    #[test]
    fn unknown_condition_produces_no_deviation() {
        // No observation → comparison outcome Unknown.
        let condition = SignalCondition::new("temperature", ComparisonOp::Le, 30.0);
        let comparison = comparison(&condition, None, 0.0);
        assert!(
            detect_deviations(&comparison, &tolerance(1.0)).is_empty(),
            "Unknown MUST NOT be declared a deviation (NFR-INT-01)"
        );
    }

    #[test]
    fn kinematic_within_tolerance_produces_no_deviation() {
        let condition = SignalCondition::new("unused", ComparisonOp::Le, 0.0);
        let comparison = comparison(&condition, None, 0.0);
        assert!(detect_deviations(&comparison, &tolerance(0.05)).is_empty());
    }

    #[test]
    fn kinematic_beyond_tolerance_produces_a_kinematic_violation() {
        let condition = SignalCondition::new("unused", ComparisonOp::Le, 0.0);
        let comparison = comparison(&condition, None, 0.2);
        let deviation = detect_deviations(&comparison, &tolerance(0.05));

        let kinematic = deviation
            .deviations
            .iter()
            .find_map(|d| match d {
                Deviation::Kinematic(v) => Some(v),
                _ => None,
            })
            .expect("kinematic deviation expected");
        assert!((kinematic.max_abs_error - 0.2).abs() < 1e-12);
        assert_eq!(kinematic.envelope, EnvelopeStatus::Violated);
        assert_eq!(kinematic.per_joint_tolerance, vec![0.05, 0.05]);
    }

    #[test]
    fn without_a_joint_reference_no_kinematic_deviation_is_declared() {
        let comparison = compare_tick(
            &evaluation(vec![], vec![0.1], ObservationBundle::default()),
            &[],
        );
        assert!(detect_deviations(&comparison, &tolerance(0.0)).is_empty());
    }

    #[test]
    fn kinematic_and_signal_violations_are_reported_separately() {
        let condition = SignalCondition::new("temperature", ComparisonOp::Le, 30.0);
        let comparison = comparison(&condition, Some(ChannelValue::Scalar(31.5)), 0.2);
        let deviation = detect_deviations(&comparison, &tolerance(0.05));

        assert_eq!(deviation.deviations.len(), 2);
        assert!(deviation
            .deviations
            .iter()
            .any(|d| matches!(d, Deviation::Kinematic(_))));
        assert!(deviation
            .deviations
            .iter()
            .any(|d| matches!(d, Deviation::Signal(_))));
    }
}
