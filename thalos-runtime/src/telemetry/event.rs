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

/// Domain Observation entity capturing physical robot and sensor telemetry.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Observation {
    pub robot_id: String,
    pub sampled_at_ns: u64,
    pub received_at_ns: u64,
    pub observation_sequence: u64,
    pub joint_positions: Vec<f64>,
    pub joint_velocities: Vec<f64>,
    pub cartesian_pose: Option<[f64; 6]>,
    pub signal_quality: f64,
}

/// Telemetry notification event emitted for UI consumption.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "event_type", rename_all = "snake_case")]
pub enum TelemetryEvent {
    ObservationReceived {
        station_id: String,
        module_id: String,
        emitted_at_ns: u64,
        event_sequence: u64,
        observation: Observation,
    },
    ChannelStateChanged {
        station_id: String,
        module_id: String,
        channel_id: String,
        emitted_at_ns: u64,
        event_sequence: u64,
        previous_state: String,
        current_state: String,
    },
    ExecutionStateChanged {
        station_id: String,
        session_id: String,
        program_id: String,
        emitted_at_ns: u64,
        event_sequence: u64,
        previous_state: String,
        current_state: String,
    },
}

impl TelemetryEvent {
    pub fn event_sequence(&self) -> u64 {
        match self {
            TelemetryEvent::ObservationReceived { event_sequence, .. }
            | TelemetryEvent::ChannelStateChanged { event_sequence, .. }
            | TelemetryEvent::ExecutionStateChanged { event_sequence, .. } => *event_sequence,
        }
    }

    pub fn station_id(&self) -> &str {
        match self {
            TelemetryEvent::ObservationReceived { station_id, .. }
            | TelemetryEvent::ChannelStateChanged { station_id, .. }
            | TelemetryEvent::ExecutionStateChanged { station_id, .. } => station_id,
        }
    }

    pub fn observation_sequence(&self) -> Option<u64> {
        match self {
            TelemetryEvent::ObservationReceived { observation, .. } => {
                Some(observation.observation_sequence)
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_telemetry_event_observation_received_roundtrip() {
        let obs = Observation {
            robot_id: "robot_scara_01".to_string(),
            sampled_at_ns: 1_700_000_000_000,
            received_at_ns: 1_700_000_000_200,
            observation_sequence: 1042,
            joint_positions: vec![0.1, 0.2, -0.1],
            joint_velocities: vec![0.01, 0.02, 0.0],
            cartesian_pose: Some([100.0, 200.0, 50.0, 0.0, 0.0, 0.0]),
            signal_quality: 0.98,
        };

        let event = TelemetryEvent::ObservationReceived {
            station_id: "st_cell_01".to_string(),
            module_id: "arm_mod_01".to_string(),
            emitted_at_ns: 1_700_000_000_500,
            event_sequence: 42,
            observation: obs,
        };

        let json_str = serde_json::to_string(&event).expect("Serialization failed");
        assert!(json_str.contains("\"event_type\":\"observation_received\""));
        assert!(json_str.contains("\"observation_sequence\":1042"));
        assert!(json_str.contains("\"event_sequence\":42"));
        assert!(json_str.contains("\"sampled_at_ns\":1700000000000"));
        assert!(json_str.contains("\"received_at_ns\":1700000000200"));
        assert!(json_str.contains("\"emitted_at_ns\":1700000000500"));

        let deserialized: TelemetryEvent =
            serde_json::from_str(&json_str).expect("Deserialization failed");
        assert_eq!(event, deserialized);
    }

    #[test]
    fn test_telemetry_event_channel_state_changed_roundtrip() {
        let event = TelemetryEvent::ChannelStateChanged {
            station_id: "st_01".to_string(),
            module_id: "mod_vision".to_string(),
            channel_id: "target_x".to_string(),
            emitted_at_ns: 1_800_000_000_000,
            event_sequence: 10,
            previous_state: "idle".to_string(),
            current_state: "active".to_string(),
        };

        let json_str = serde_json::to_string(&event).expect("Serialization failed");
        assert!(json_str.contains("\"event_type\":\"channel_state_changed\""));

        let deserialized: TelemetryEvent =
            serde_json::from_str(&json_str).expect("Deserialization failed");
        assert_eq!(event, deserialized);
    }
}
