use super::event::{Observation, TelemetryEvent};

/// Non-blocking telemetry projection state machine.
/// Downsamples high-frequency physical domain observations (e.g. 1kHz) into
/// UI presentation TelemetryEvents (e.g. 30–60 Hz) based on timestamp intervals.
/// State-changing events bypass sampling and are emitted immediately.
#[derive(Debug, Clone)]
pub struct TelemetryProjection {
    interval_ns: u64,
    last_emitted_at_ns: Option<u64>,
    event_sequence: u64,
}

impl TelemetryProjection {
    /// Create a new projection policy targeting a specific frame rate (e.g., 30 or 60 Hz).
    pub fn new(target_fps: u32) -> Self {
        let fps = target_fps.max(1);
        let interval_ns = 1_000_000_000u64 / (fps as u64);
        Self {
            interval_ns,
            last_emitted_at_ns: None,
            event_sequence: 0,
        }
    }

    /// Ingest a domain observation. Returns Some(TelemetryEvent::ObservationReceived) if
    /// the timestamp sampling interval has elapsed, or None if suppressed.
    pub fn ingest_observation(
        &mut self,
        station_id: &str,
        module_id: &str,
        observation: Observation,
    ) -> Option<TelemetryEvent> {
        let current_ns = observation.sampled_at_ns;

        let should_emit = match self.last_emitted_at_ns {
            None => true,
            Some(last_ns) => current_ns >= last_ns + self.interval_ns,
        };

        if should_emit {
            self.last_emitted_at_ns = Some(current_ns);
            self.event_sequence += 1;
            Some(TelemetryEvent::ObservationReceived {
                station_id: station_id.to_string(),
                module_id: module_id.to_string(),
                emitted_at_ns: current_ns,
                event_sequence: self.event_sequence,
                observation,
            })
        } else {
            None
        }
    }

    /// Immediately projects a channel state change event (bypasses sampling).
    pub fn project_channel_state_change(
        &mut self,
        station_id: &str,
        module_id: &str,
        channel_id: &str,
        emitted_at_ns: u64,
        previous_state: &str,
        current_state: &str,
    ) -> TelemetryEvent {
        self.event_sequence += 1;
        TelemetryEvent::ChannelStateChanged {
            station_id: station_id.to_string(),
            module_id: module_id.to_string(),
            channel_id: channel_id.to_string(),
            emitted_at_ns,
            event_sequence: self.event_sequence,
            previous_state: previous_state.to_string(),
            current_state: current_state.to_string(),
        }
    }

    /// Immediately projects an execution state change event (bypasses sampling).
    pub fn project_execution_state_change(
        &mut self,
        station_id: &str,
        session_id: &str,
        program_id: &str,
        emitted_at_ns: u64,
        previous_state: &str,
        current_state: &str,
    ) -> TelemetryEvent {
        self.event_sequence += 1;
        TelemetryEvent::ExecutionStateChanged {
            station_id: station_id.to_string(),
            session_id: session_id.to_string(),
            program_id: program_id.to_string(),
            emitted_at_ns,
            event_sequence: self.event_sequence,
            previous_state: previous_state.to_string(),
            current_state: current_state.to_string(),
        }
    }

    /// Get current event sequence.
    pub fn event_sequence(&self) -> u64 {
        self.event_sequence
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_mock_observation(seq: u64, sampled_at_ns: u64) -> Observation {
        Observation {
            robot_id: "robot_scara_01".to_string(),
            sampled_at_ns,
            received_at_ns: sampled_at_ns + 100_000,
            observation_sequence: seq,
            joint_positions: vec![0.0, 0.0],
            joint_velocities: vec![0.0, 0.0],
            cartesian_pose: None,
            signal_quality: 1.0,
        }
    }

    #[test]
    fn test_1khz_to_60hz_sampling() {
        let mut proj = TelemetryProjection::new(60);
        let mut emitted_events = Vec::new();

        // 1000 observations at 1kHz (1_000_000 ns = 1 ms interval)
        for i in 1..=1000 {
            let ns = (i as u64) * 1_000_000;
            let obs = create_mock_observation(i, ns);
            if let Some(event) = proj.ingest_observation("st_01", "mod_01", obs) {
                emitted_events.push(event);
            }
        }

        // 1 second (1000 ms) at 60 Hz should yield ~59-60 emitted events
        assert!(emitted_events.len() >= 59 && emitted_events.len() <= 60);

        // Verify event_sequence is strictly monotonic
        for (idx, event) in emitted_events.iter().enumerate() {
            if let TelemetryEvent::ObservationReceived {
                event_sequence,
                observation,
                ..
            } = event
            {
                assert_eq!(*event_sequence, (idx + 1) as u64);
                // Observation sequence corresponds to real raw sample
                assert!(observation.observation_sequence > 0);
            } else {
                panic!("Expected ObservationReceived event");
            }
        }
    }

    #[test]
    fn test_1khz_to_30hz_sampling() {
        let mut proj = TelemetryProjection::new(30);
        let mut emitted_events = Vec::new();

        for i in 1..=1000 {
            let ns = (i as u64) * 1_000_000;
            let obs = create_mock_observation(i, ns);
            if let Some(event) = proj.ingest_observation("st_01", "mod_01", obs) {
                emitted_events.push(event);
            }
        }

        // 1 second (1000 ms) at 30 Hz should yield ~30 emitted events
        assert_eq!(emitted_events.len(), 30);
    }

    #[test]
    fn test_irregular_acquisition_rate() {
        let mut proj = TelemetryProjection::new(60);
        let mut emitted = Vec::new();

        // Variable sampling step between 0.5ms and 2ms
        let steps = vec![500_000u64, 2_000_000, 1_500_000, 800_000, 10_000_000, 20_000_000];
        let mut current_ns = 1_000_000u64;

        for (seq, step) in steps.into_iter().enumerate() {
            current_ns += step;
            let obs = create_mock_observation(seq as u64 + 1, current_ns);
            if let Some(event) = proj.ingest_observation("st_01", "mod_01", obs) {
                emitted.push(event);
            }
        }

        // First observation emits immediately, then large gaps (>16.6ms) emit immediately
        assert!(!emitted.is_empty());
    }

    #[test]
    fn test_state_change_events_immediate() {
        let mut proj = TelemetryProjection::new(60);

        // Ingest observation at 1ms
        let obs1 = create_mock_observation(1, 1_000_000);
        let ev1 = proj.ingest_observation("st_01", "mod_01", obs1).unwrap();
        assert_eq!(ev1.event_sequence(), 1);

        // Immediate state change event at 2ms (would be suppressed if sampled)
        let state_ev = proj.project_execution_state_change(
            "st_01",
            "sess_01",
            "prog_weld",
            2_000_000,
            "created",
            "active",
        );
        assert_eq!(state_ev.event_sequence(), 2);

        // Next observation at 18ms (interval 16.6ms passed relative to 1ms)
        let obs2 = create_mock_observation(2, 18_000_000);
        let ev2 = proj.ingest_observation("st_01", "mod_01", obs2).unwrap();
        assert_eq!(ev2.event_sequence(), 3);
    }
}
