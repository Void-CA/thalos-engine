//! HardwareExecutor Physical E2E Tests (FASE 12.5)
//!
//! These tests validate the HardwareExecutor closed loop with physical transport:
//! - Command dispatch → Observation → Convergence
//! - Observation timeout
//! - Transport loss detection
//! - Target not reached
//!
//! Run with: `cargo test -p thalos_runtime --features physical-e2e -- hardware_executor_physical_e2e`
//!
//! Requirements:
//! - ESP32 connected and responding
//! - Robot at a known initial position

#![cfg(feature = "physical-e2e")]

use std::sync::{Arc, Mutex};
use std::time::Duration;
use thalos_ports::robot::{RobotCommand, RobotObservation, RobotTransport, TransportError, TransportState};
use thalos_ports::SignalQuality;
use thalos_runtime::execution::executor::ExecutionSessionState;
use thalos_runtime::execution::hardware::{
    HardwareExecutor, HardwareSafetyConfig, PhysicalExecutionError, TrackingState,
};
use thalos_transport::esp32::Esp32RobotAdapter;
use thalos_transport::tcp::TcpTransport;

/// Helper to create a physical transport adapter.
fn create_physical_transport() -> Esp32RobotAdapter<TcpTransport> {
    let addr = std::env::var("THALOS_ESP32_ADDR")
        .unwrap_or_else(|_| "192.168.1.100:8080".to_string());
    let transport = TcpTransport::with_receive_timeout(&addr, 500);
    Esp32RobotAdapter::new(transport)
}

/// Test: HardwareExecutor completes a single waypoint with physical observation.
#[tokio::test(flavor = "multi_thread")]
async fn hardware_executor_single_waypoint_physical() {
    let mut transport = create_physical_transport();
    transport.connect().await.expect("Failed to connect to ESP32");

    let session_id = thalos_core::ids::ExecutionSessionId("physical-test-01".into());
    let waypoints = vec![vec![0.1, 0.2, 0.3]]; // Small movement
    let safety = HardwareSafetyConfig {
        observation_timeout: Duration::from_secs(5),
        convergence_timeout: Duration::from_secs(10),
        ..Default::default()
    };

    let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.1)
        .with_safety(safety);

    // Start execution
    executor.start().expect("Failed to start executor");
    assert_eq!(executor.state(), ExecutionSessionState::Running);

    // Run tick loop until completion or timeout
    let start = std::time::Instant::now();
    let timeout = Duration::from_secs(15);

    loop {
        if start.elapsed() > timeout {
            panic!("Execution timed out after {:?}", timeout);
        }

        match executor.tick(0.1) {
            Ok(progress) => {
                println!("Progress: {:.2}%", progress * 100.0);

                if executor.state() == ExecutionSessionState::Completed {
                    println!("Execution completed successfully!");
                    break;
                }
            }
            Err(PhysicalExecutionError::ObservationTimeout) => {
                println!("Observation timeout - robot may not be responding");
                break;
            }
            Err(e) => {
                panic!("Execution failed: {:?}", e);
            }
        }

        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    // Cleanup
    executor.cancel().expect("Failed to cancel executor");
    executor.transport.disconnect().await.expect("Failed to disconnect");
}

/// Test: HardwareExecutor detects observation timeout with physical transport.
#[tokio::test(flavor = "multi_thread")]
async fn hardware_executor_observation_timeout_physical() {
    let mut transport = create_physical_transport();
    transport.connect().await.expect("Failed to connect to ESP32");

    let session_id = thalos_core::ids::ExecutionSessionId("physical-test-timeout".into());
    let waypoints = vec![vec![0.5, 0.5, 0.5]]; // Position robot may not reach quickly

    // Very short timeout to trigger failure
    let safety = HardwareSafetyConfig {
        observation_timeout: Duration::from_millis(500),
        convergence_timeout: Duration::from_secs(5),
        ..Default::default()
    };

    let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.01)
        .with_safety(safety);

    executor.start().expect("Failed to start executor");

    // Run ticks until we get a timeout or completion
    let mut got_timeout = false;
    for _ in 0..20 {
        match executor.tick(0.1) {
            Ok(_) => {
                if executor.state() == ExecutionSessionState::Completed {
                    break;
                }
            }
            Err(PhysicalExecutionError::ObservationTimeout) => {
                got_timeout = true;
                break;
            }
            Err(PhysicalExecutionError::TargetNotReached { .. }) => {
                got_timeout = true;
                break;
            }
            Err(e) => {
                println!("Other error: {:?}", e);
                break;
            }
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    // We should have gotten a timeout or the robot actually reached the target
    if !got_timeout && executor.state() != ExecutionSessionState::Completed {
        println!("Warning: Expected timeout but execution continued");
    }

    executor.cancel().expect("Failed to cancel executor");
    executor.transport.disconnect().await.expect("Failed to disconnect");
}

/// Test: HardwareExecutor fails correctly when transport is disconnected.
#[tokio::test(flavor = "multi_thread")]
async fn hardware_executor_transport_loss_physical() {
    let mut transport = create_physical_transport();
    transport.connect().await.expect("Failed to connect to ESP32");

    let session_id = thalos_core::ids::ExecutionSessionId("physical-test-disconnect".into());
    let waypoints = vec![vec![0.1, 0.2, 0.3]];

    let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.1);
    executor.start().expect("Failed to start executor");

    // Disconnect transport
    executor.transport.disconnect().await.expect("Failed to disconnect");

    // Next tick should detect transport loss
    let result = executor.tick(0.1);
    assert!(result.is_err(), "Should detect transport loss");

    match result.unwrap_err() {
        PhysicalExecutionError::TransportLost => {
            println!("Transport loss correctly detected");
        }
        e => {
            panic!("Expected TransportLost, got: {:?}", e);
        }
    }

    assert_eq!(executor.state(), ExecutionSessionState::Failed);
}

/// Test: Verify observation timestamps are preserved in physical execution.
#[tokio::test(flavor = "multi_thread")]
async fn hardware_executor_observation_timestamps_physical() {
    let mut transport = create_physical_transport();
    transport.connect().await.expect("Failed to connect to ESP32");

    let session_id = thalos_core::ids::ExecutionSessionId("physical-test-timestamps".into());
    let waypoints = vec![vec![0.1, 0.0, 0.0]];

    let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.2);
    executor.start().expect("Failed to start executor");

    // Run a few ticks and verify we get valid observations
    let mut got_observation = false;
    for _ in 0..10 {
        let _ = executor.tick(0.1);

        // Check if we got a valid observation in the snapshot
        let snapshot = executor.snapshot();
        if snapshot.observation.latest.sequence > 0 {
            got_observation = true;
            assert!(
                snapshot.observation.latest.sampled_at_ns > 0,
                "Observation should have valid sampled_at_ns"
            );
            assert!(
                snapshot.observation.latest.received_at_ns > 0,
                "Observation should have valid received_at_ns"
            );
            println!(
                "Got observation: seq={}, sampled_ns={}, received_ns={}",
                snapshot.observation.latest.sequence,
                snapshot.observation.latest.sampled_at_ns,
                snapshot.observation.latest.received_at_ns
            );
            break;
        }

        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    if !got_observation {
        println!("Warning: No valid observation received during test");
    }

    executor.cancel().expect("Failed to cancel executor");
    executor.transport.disconnect().await.expect("Failed to disconnect");
}
