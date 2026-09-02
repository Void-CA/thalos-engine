use serde::{Deserialize, Serialize};
use super::kinematic::KinematicDeviation;

/// Unique causal identity for a deviation event.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct DeviationEventId(pub String);

impl DeviationEventId {
    pub fn new(id: impl Into<String>) -> Self {
        Self(id.into())
    }

    pub fn from_sequence(robot_id: &str, sequence: u64) -> Self {
        Self(format!("dev_{}_{}", robot_id, sequence))
    }
}

impl std::fmt::Display for DeviationEventId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// Specific type and onset metadata for a deviation event.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DeviationEventKind {
    ViolationConfirmed { onset_ns: u64 },
    ViolationRecovered { onset_ns: u64 },
}

/// Domain event capturing the semantic occurrence of a confirmed deviation transition.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DeviationEvent {
    pub event_id: DeviationEventId,
    pub robot_id: String,
    pub observed_at_ns: u64,
    pub observation_sequence: u64,
    pub kind: DeviationEventKind,
    pub deviation: KinematicDeviation,
}

impl DeviationEvent {
    pub fn new(
        event_id: DeviationEventId,
        observation_sequence: u64,
        kind: DeviationEventKind,
        deviation: KinematicDeviation,
    ) -> Self {
        Self {
            event_id,
            robot_id: deviation.robot_id.clone(),
            observed_at_ns: deviation.sampled_at_ns,
            observation_sequence,
            kind,
            deviation,
        }
    }

    pub fn onset_ns(&self) -> u64 {
        match self.kind {
            DeviationEventKind::ViolationConfirmed { onset_ns } => onset_ns,
            DeviationEventKind::ViolationRecovered { onset_ns } => onset_ns,
        }
    }
}
