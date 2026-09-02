use std::sync::{Arc, Mutex};
use super::event::TelemetryEvent;

/// Abstract transport-agnostic interface for publishing telemetry events.
pub trait TelemetryPublisher: Send + Sync {
    fn publish(&self, event: TelemetryEvent);
}

/// In-memory telemetry publisher used for isolated testing of telemetry pipelines.
#[derive(Debug, Clone, Default)]
pub struct InMemoryTelemetryPublisher {
    published_events: Arc<Mutex<Vec<TelemetryEvent>>>,
}

impl InMemoryTelemetryPublisher {
    pub fn new() -> Self {
        Self {
            published_events: Arc::new(Mutex::new(Vec::new())),
        }
    }

    pub fn events(&self) -> Vec<TelemetryEvent> {
        self.published_events.lock().unwrap().clone()
    }

    pub fn clear(&self) {
        self.published_events.lock().unwrap().clear();
    }
}

impl TelemetryPublisher for InMemoryTelemetryPublisher {
    fn publish(&self, event: TelemetryEvent) {
        self.published_events.lock().unwrap().push(event);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::event::Observation;

    #[test]
    fn test_in_memory_publisher_records_events() {
        let publisher = InMemoryTelemetryPublisher::new();
        assert!(publisher.events().is_empty());

        let obs = Observation {
            robot_id: "scara_01".to_string(),
            sampled_at_ns: 1000,
            received_at_ns: 1100,
            observation_sequence: 1,
            joint_positions: vec![0.0, 0.0],
            joint_velocities: vec![0.0, 0.0],
            cartesian_pose: None,
            signal_quality: 1.0,
        };

        let event = TelemetryEvent::ObservationReceived {
            station_id: "st_01".to_string(),
            module_id: "mod_01".to_string(),
            emitted_at_ns: 1200,
            event_sequence: 1,
            observation: obs,
        };

        publisher.publish(event.clone());

        let events = publisher.events();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0], event);
    }
}
