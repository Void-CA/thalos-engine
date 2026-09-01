use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::time::Duration;

/// Serialize Duration as seconds (f64).
mod duration_secs {
    use super::*;

    pub fn serialize<S>(dur: &Duration, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_f64(dur.as_secs_f64())
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Duration, D::Error>
    where
        D: Deserializer<'de>,
    {
        let secs = f64::deserialize(deserializer)?;
        Ok(Duration::from_secs_f64(secs))
    }
}

/// Evento de ciclo de vida durante la ejecución.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ExecutionEvent {
    Started {
        #[serde(with = "duration_secs")]
        timestamp: Duration,
    },
    Paused {
        #[serde(with = "duration_secs")]
        timestamp: Duration,
    },
    Resumed {
        #[serde(with = "duration_secs")]
        timestamp: Duration,
    },
    WaypointReached {
        #[serde(with = "duration_secs")]
        timestamp: Duration,
        waypoint: usize,
    },
    SegmentCompleted {
        #[serde(with = "duration_secs")]
        timestamp: Duration,
        segment: usize,
    },
    Error {
        #[serde(with = "duration_secs")]
        timestamp: Duration,
        message: String,
    },
    Completed {
        #[serde(with = "duration_secs")]
        timestamp: Duration,
    },
    Cancelled {
        #[serde(with = "duration_secs")]
        timestamp: Duration,
    },
}

impl ExecutionEvent {
    pub fn timestamp(&self) -> Duration {
        match self {
            ExecutionEvent::Started { timestamp }
            | ExecutionEvent::Paused { timestamp }
            | ExecutionEvent::Resumed { timestamp }
            | ExecutionEvent::WaypointReached { timestamp, .. }
            | ExecutionEvent::SegmentCompleted { timestamp, .. }
            | ExecutionEvent::Error { timestamp, .. }
            | ExecutionEvent::Completed { timestamp }
            | ExecutionEvent::Cancelled { timestamp } => *timestamp,
        }
    }
}
