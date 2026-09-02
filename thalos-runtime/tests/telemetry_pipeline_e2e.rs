use thalos_runtime::telemetry::{
    Observation, TelemetryHub, TelemetryProjection, TelemetryPublisher,
};
use tokio::sync::broadcast::error::RecvError;

fn create_observation(seq: u64, sampled_at_ns: u64) -> Observation {
    Observation {
        robot_id: "robot_scara_01".to_string(),
        sampled_at_ns,
        received_at_ns: sampled_at_ns + 100_000, // 0.1ms latency
        observation_sequence: seq,
        joint_positions: vec![0.1 * seq as f64, 0.2 * seq as f64],
        joint_velocities: vec![0.01, 0.02],
        cartesian_pose: None,
        signal_quality: 1.0,
    }
}

/// 1. Happy Path E2E: Fake Acquisition -> Projection -> TelemetryHub -> Subscriber
#[tokio::test]
async fn test_e2e_fake_acquisition_to_telemetry_subscriber() {
    let hub = TelemetryHub::new(16);
    let mut subscriber = hub.subscribe();
    let mut projection = TelemetryProjection::new(60);

    let observation = create_observation(1, 1_000_000);

    let event = projection
        .ingest_observation("station_cell_01", "acq_vision_01", observation.clone())
        .expect("First observation must emit initial frame");

    hub.publish(event);

    let received = subscriber.recv().await.expect("Subscriber receives event");

    assert_eq!(received.station_id(), "station_cell_01");
    assert_eq!(received.event_sequence(), 1);
    assert_eq!(
        received.observation_sequence().unwrap(),
        observation.observation_sequence
    );
}

/// 2. Architectural Invariant: 1kHz acquisition processing is completely decoupled from 60Hz UI sampling.
#[tokio::test]
async fn test_sampling_does_not_interfere_with_acquisition() {
    let hub = TelemetryHub::new(1024);
    let mut subscriber = hub.subscribe();
    let mut projection = TelemetryProjection::new(60);

    let mut raw_acquisition_count = 0;
    let mut telemetry_emitted_count = 0;

    // Simulate 1000 observations at 1kHz (1ms interval)
    for i in 1..=1000 {
        let obs = create_observation(i, i * 1_000_000);
        raw_acquisition_count += 1; // 1kHz domain acquisition loop processes 100% of samples

        if let Some(event) = projection.ingest_observation("st_01", "mod_01", obs) {
            hub.publish(event);
            telemetry_emitted_count += 1;
        }
    }

    assert_eq!(raw_acquisition_count, 1000);
    // At 60Hz downsampling, 1 second (1000ms) yields ~60-61 telemetry events
    assert!(
        (59..=62).contains(&telemetry_emitted_count),
        "Expected ~60 events for 1000ms at 60Hz, got {}",
        telemetry_emitted_count
    );

    // Verify subscriber actually received all emitted events
    let mut received_count = 0;
    while subscriber.try_recv().is_ok() {
        received_count += 1;
    }
    assert_eq!(received_count, telemetry_emitted_count);
}

/// 3. Architectural Invariant: Slow consumers experience Lagged drop without blocking publisher or acquisition.
#[tokio::test]
async fn test_slow_consumer_lagged_does_not_block_publisher() {
    let hub = TelemetryHub::new(16); // Small buffer capacity of 16
    let mut fast_sub = hub.subscribe();
    let mut slow_sub = hub.subscribe();
    let mut projection = TelemetryProjection::new(60);

    let mut fast_received = 0;

    // Generate enough observations to overflow the 16-slot broadcast buffer
    for i in 1..=1000 {
        let obs = create_observation(i, i * 1_000_000);
        if let Some(event) = projection.ingest_observation("st_01", "mod_01", obs) {
            // Publishing never blocks or fails even when slow_sub buffer overflows
            hub.publish(event);

            // Fast subscriber actively consumes events
            if fast_sub.try_recv().is_ok() {
                fast_received += 1;
            }
        }
    }

    // Slow subscriber receives RecvError::Lagged because it fell behind and did not consume
    let slow_res = slow_sub.recv().await;
    assert!(
        matches!(slow_res, Err(RecvError::Lagged(_))),
        "Slow subscriber should experience Lagged error"
    );

    // Fast subscriber successfully received events without being blocked by slow_sub
    assert!(fast_received > 0);
}

/// 4. Sequence Duality: event_sequence (1, 2, 3...) vs observation_sequence (1, 17, 34...)
#[tokio::test]
async fn test_sequence_duality_event_vs_observation_sequence() {
    let hub = TelemetryHub::new(1024);
    let mut subscriber = hub.subscribe();
    let mut projection = TelemetryProjection::new(60); // ~16.6ms sampling interval

    for i in 1..=100 {
        let obs = create_observation(i, i * 1_000_000);
        if let Some(event) = projection.ingest_observation("st_01", "mod_01", obs) {
            hub.publish(event);
        }
    }

    let mut expected_event_seq = 1;
    let mut prev_obs_seq = 0;

    while let Ok(event) = subscriber.try_recv() {
        assert_eq!(
            event.event_sequence(),
            expected_event_seq,
            "event_sequence must be strictly monotonic without gaps (1, 2, 3...)"
        );
        let obs_seq = event.observation_sequence().unwrap();
        assert!(
            obs_seq > prev_obs_seq,
            "observation_sequence must reflect actual raw sample sequence jumps"
        );

        prev_obs_seq = obs_seq;
        expected_event_seq += 1;
    }

    assert!(
        expected_event_seq > 5,
        "Should have received multiple projected events"
    );
}

/// 5. Station Filter Invariant: Hub publishes events containing station_id for client-side/adapter routing.
#[tokio::test]
async fn test_station_id_filtering_invariant() {
    let hub = TelemetryHub::new(16);
    let mut subscriber = hub.subscribe();
    let mut proj_cell1 = TelemetryProjection::new(60);
    let mut proj_cell2 = TelemetryProjection::new(60);

    let obs1 = create_observation(1, 1_000_000);
    let obs2 = create_observation(1, 1_000_000);

    if let Some(ev) = proj_cell1.ingest_observation("cell_01", "vision_01", obs1) {
        hub.publish(ev);
    }
    if let Some(ev) = proj_cell2.ingest_observation("cell_02", "vision_02", obs2) {
        hub.publish(ev);
    }

    let ev1 = subscriber.recv().await.unwrap();
    let ev2 = subscriber.recv().await.unwrap();

    let cell1_events: Vec<_> = vec![ev1, ev2]
        .into_iter()
        .filter(|e| e.station_id() == "cell_01")
        .collect();

    assert_eq!(cell1_events.len(), 1);
    assert_eq!(cell1_events[0].station_id(), "cell_01");
}
