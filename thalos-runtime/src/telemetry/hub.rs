use super::event::TelemetryEvent;
use super::publisher::TelemetryPublisher;
use tokio::sync::broadcast;

/// Multi-consumer telemetry hub using a bounded broadcast channel with non-blocking fan-out.
/// Slow consumers experience dropped presentation frames (`RecvError::Lagged`) without blocking acquisition or runtime.
pub struct TelemetryHub {
    sender: broadcast::Sender<TelemetryEvent>,
}

impl TelemetryHub {
    pub fn new(capacity: usize) -> Self {
        let (sender, _) = broadcast::channel(capacity.max(16));
        Self { sender }
    }

    pub fn subscribe(&self) -> broadcast::Receiver<TelemetryEvent> {
        self.sender.subscribe()
    }

    pub fn receiver_count(&self) -> usize {
        self.sender.receiver_count()
    }
}

impl TelemetryPublisher for TelemetryHub {
    fn publish(&self, event: TelemetryEvent) {
        // Non-blocking broadcast emission. Returns error if no receivers exist, which is safely ignored.
        let _ = self.sender.send(event);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::event::Observation;
    use crate::telemetry::projection::TelemetryProjection;

    fn create_observation(seq: u64, sampled_at_ns: u64) -> Observation {
        Observation {
            robot_id: "robot_01".to_string(),
            sampled_at_ns,
            received_at_ns: sampled_at_ns + 100,
            observation_sequence: seq,
            joint_positions: vec![0.0, 0.0],
            joint_velocities: vec![0.0, 0.0],
            cartesian_pose: None,
            signal_quality: 1.0,
        }
    }

    #[tokio::test]
    async fn test_hub_fanout_and_bounded_lag() {
        let hub = TelemetryHub::new(16);
        let mut rx1 = hub.subscribe();
        let mut rx2 = hub.subscribe();

        let mut projection = TelemetryProjection::new(60);

        // Ingest observations and publish projected events to TelemetryHub
        for i in 1..=100 {
            let obs = create_observation(i, i * 1_000_000); // 1ms intervals
            if let Some(event) = projection.ingest_observation("st_01", "mod_01", obs) {
                hub.publish(event);
            }
        }

        // Both subscribers receive emitted events
        let ev1 = rx1.recv().await.unwrap();
        let ev2 = rx2.recv().await.unwrap();

        assert_eq!(ev1.event_sequence(), 1);
        assert_eq!(ev2.event_sequence(), 1);
    }
}
