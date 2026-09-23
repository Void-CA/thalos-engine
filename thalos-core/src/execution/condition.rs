use serde::{Deserialize, Serialize};

use crate::device::{ChannelObservation, ChannelValue};

/// Comparison operator of a declared signal condition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ComparisonOp {
    Eq,
    Ne,
    Gt,
    Ge,
    Lt,
    Le,
}

/// A DECLARED operational expectation over one signal.
///
/// It describes how THIS execution should judge a channel — it is not a
/// universal property of the resource (that belongs to the channel contract).
/// The same channel may therefore carry different conditions in different
/// processes without changing its physical configuration.
///
/// The EXPECTATION must exist before comparison: a comparison never invents it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SignalCondition {
    /// Identifier of the observed channel (same id used by observations).
    pub channel_id: String,
    pub op: ComparisonOp,
    pub value: f64,
}

impl SignalCondition {
    pub fn new(channel_id: impl Into<String>, op: ComparisonOp, value: f64) -> Self {
        Self {
            channel_id: channel_id.into(),
            op,
            value,
        }
    }

    /// The degenerate condition the legacy `TerminationPolicy::Condition`
    /// expresses: `channel > 0`.
    pub fn termination_default(channel_id: impl Into<String>) -> Self {
        Self::new(channel_id, ComparisonOp::Gt, 0.0)
    }

    /// Evaluate the condition against an observation.
    ///
    /// Returns `None` when the value cannot be compared, so a caller never
    /// treats "unknown" as satisfied (NFR-INT-01).
    pub fn evaluate(&self, observation: &ChannelObservation) -> Option<bool> {
        let observed = numeric(&observation.value)?;
        let satisfied = match self.op {
            ComparisonOp::Eq => (observed - self.value).abs() < f64::EPSILON,
            ComparisonOp::Ne => (observed - self.value).abs() >= f64::EPSILON,
            ComparisonOp::Gt => observed > self.value,
            ComparisonOp::Ge => observed >= self.value,
            ComparisonOp::Lt => observed < self.value,
            ComparisonOp::Le => observed <= self.value,
        };
        Some(satisfied)
    }
}

fn numeric(value: &ChannelValue) -> Option<f64> {
    match value {
        ChannelValue::Scalar(v) => Some(*v),
        ChannelValue::Integer(v) => Some(*v as f64),
        ChannelValue::Boolean(v) => Some(if *v { 1.0 } else { 0.0 }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation(value: ChannelValue) -> ChannelObservation {
        ChannelObservation {
            channel_id: "temperature".to_string(),
            sequence: 1,
            sampled_at_ns: 0,
            received_at_ns: 0,
            value,
            unit: None,
            quality: crate::device::SignalQuality::Nominal,
        }
    }

    #[test]
    fn threshold_conditions_are_evaluated_explicitly() {
        let condition = SignalCondition::new("temperature", ComparisonOp::Le, 30.0);
        assert_eq!(condition.evaluate(&observation(ChannelValue::Scalar(25.0))), Some(true));
        assert_eq!(condition.evaluate(&observation(ChannelValue::Scalar(31.5))), Some(false));
        assert_eq!(condition.evaluate(&observation(ChannelValue::Integer(30))), Some(true));
    }

    #[test]
    fn termination_default_matches_the_legacy_channel_gt_zero_semantics() {
        let condition = SignalCondition::termination_default("safety_stop");
        assert_eq!(condition.evaluate(&observation(ChannelValue::Scalar(0.0))), Some(false));
        assert_eq!(condition.evaluate(&observation(ChannelValue::Scalar(1.0))), Some(true));
        assert_eq!(condition.evaluate(&observation(ChannelValue::Boolean(true))), Some(true));
    }
}
