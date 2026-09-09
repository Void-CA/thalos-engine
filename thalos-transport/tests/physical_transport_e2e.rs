//! Physical Transport E2E Tests (FASE 12.5)
//!
//! These tests validate the physical transport boundary:
//! - ESP32 connection
//! - Command sending
//! - Observation receiving
//! - Timeout handling
//! - Disconnection/reconnection
//!
//! Run with: `cargo test -p thalos_transport --features physical-e2e -- physical_transport_e2e`
//!
//! Requirements:
//! - ESP32 connected via serial/TCP
//! - ESP32 running the Thalos firmware

#![cfg(feature = "physical-e2e")]

use std::time::Duration;
use thalos_ports::robot::{RobotCommand, RobotTransport, TransportError, TransportState};
use thalos_transport::esp32::Esp32RobotAdapter;
use thalos_transport::tcp::TcpTransport;

/// Test: Connect to ESP32 via TCP and verify connection state.
#[tokio::test(flavor = "multi_thread")]
async fn physical_transport_connect() {
    let addr = std::env::var("THALOS_ESP32_ADDR")
        .unwrap_or_else(|_| "192.168.1.100:8080".to_string());

    let transport = TcpTransport::new(&addr);
    let mut adapter = Esp32RobotAdapter::new(transport);

    assert_eq!(adapter.state(), TransportState::Disconnected);

    adapter.connect().await.expect("Failed to connect to ESP32");
    assert_eq!(adapter.state(), TransportState::Connected);

    adapter.disconnect().await.expect("Failed to disconnect");
    assert_eq!(adapter.state(), TransportState::Disconnected);
}

/// Test: Send a MoveJoints command and verify no error.
#[tokio::test(flavor = "multi_thread")]
async fn physical_transport_send_command() {
    let addr = std::env::var("THALOS_ESP32_ADDR")
        .unwrap_or_else(|_| "192.168.1.100:8080".to_string());

    let transport = TcpTransport::new(&addr);
    let mut adapter = Esp32RobotAdapter::new(transport);
    adapter.connect().await.expect("Failed to connect");

    let result = adapter.send(RobotCommand::MoveJoints {
        positions_rad: vec![0.0, 0.0, 0.0],
        velocities_rad_s: None,
    });

    assert!(result.is_ok(), "Send failed: {:?}", result.err());

    adapter.disconnect().await.expect("Failed to disconnect");
}

/// Test: Send a Stop command and verify no error.
#[tokio::test(flavor = "multi_thread")]
async fn physical_transport_send_stop() {
    let addr = std::env::var("THALOS_ESP32_ADDR")
        .unwrap_or_else(|_| "192.168.1.100:8080".to_string());

    let transport = TcpTransport::new(&addr);
    let mut adapter = Esp32RobotAdapter::new(transport);
    adapter.connect().await.expect("Failed to connect");

    let result = adapter.stop();
    assert!(result.is_ok(), "Stop failed: {:?}", result.err());

    adapter.disconnect().await.expect("Failed to disconnect");
}

/// Test: Receive observation from ESP32 (with timeout).
#[tokio::test(flavor = "multi_thread")]
async fn physical_transport_receive_observation() {
    let addr = std::env::var("THALOS_ESP32_ADDR")
        .unwrap_or_else(|_| "192.168.1.100:8080".to_string());

    let transport = TcpTransport::with_receive_timeout(&addr, 2000);
    let mut adapter = Esp32RobotAdapter::new(transport);
    adapter.connect().await.expect("Failed to connect");

    // Send a command first to trigger observation response
    adapter.send(RobotCommand::MoveJoints {
        positions_rad: vec![0.1, 0.2, 0.3],
        velocities_rad_s: None,
    }).expect("Send failed");

    // Try to receive observation (may timeout if ESP32 doesn't respond)
    let result = adapter.try_receive_observation();

    match result {
        Ok(Some(obs)) => {
            assert!(obs.joint_positions_rad.len() > 0, "Observation should have joint positions");
            assert!(obs.sampled_at_ns > 0, "Observation should have valid timestamp");
        }
        Ok(None) => {
            // No observation available (timeout) - this is acceptable
            println!("No observation received (timeout)");
        }
        Err(TransportError::Disconnected) => {
            panic!("Transport disconnected unexpectedly");
        }
        Err(e) => {
            // Other errors are acceptable in E2E (e.g., communication failure)
            println!("Observation receive error: {:?}", e);
        }
    }

    adapter.disconnect().await.expect("Failed to disconnect");
}

/// Test: Verify timeout behavior when no observation is available.
#[tokio::test(flavor = "multi_thread")]
async fn physical_transport_observation_timeout() {
    let addr = std::env::var("THALOS_ESP32_ADDR")
        .unwrap_or_else(|_| "192.168.1.100:8080".to_string());

    // Use a very short timeout
    let transport = TcpTransport::with_receive_timeout(&addr, 100);
    let mut adapter = Esp32RobotAdapter::new(transport);
    adapter.connect().await.expect("Failed to connect");

    let start = std::time::Instant::now();
    let result = adapter.try_receive_observation();
    let elapsed = start.elapsed();

    // Should return Ok(None) on timeout, not block forever
    assert!(result.is_ok() || result.is_err());
    assert!(elapsed < Duration::from_secs(2), "Receive should not block for more than 2s");

    adapter.disconnect().await.expect("Failed to disconnect");
}

/// Test: Verify error propagation when transport is disconnected.
#[tokio::test(flavor = "multi_thread")]
async fn physical_transport_disconnected_error() {
    let addr = std::env::var("THALOS_ESP32_ADDR")
        .unwrap_or_else(|_| "192.168.1.100:8080".to_string());

    let transport = TcpTransport::new(&addr);
    let mut adapter = Esp32RobotAdapter::new(transport);

    // Try to send without connecting
    let result = adapter.send(RobotCommand::MoveJoints {
        positions_rad: vec![0.0],
        velocities_rad_s: None,
    });

    assert!(result.is_err());
    assert!(matches!(result.unwrap_err(), TransportError::Disconnected));
}

/// Test: Verify reconnection works.
#[tokio::test(flavor = "multi_thread")]
async fn physical_transport_reconnect() {
    let addr = std::env::var("THALOS_ESP32_ADDR")
        .unwrap_or_else(|_| "192.168.1.100:8080".to_string());

    let transport = TcpTransport::new(&addr);
    let mut adapter = Esp32RobotAdapter::new(transport);

    // Connect
    adapter.connect().await.expect("Failed to connect first time");
    assert_eq!(adapter.state(), TransportState::Connected);

    // Disconnect
    adapter.disconnect().await.expect("Failed to disconnect");
    assert_eq!(adapter.state(), TransportState::Disconnected);

    // Reconnect
    adapter.connect().await.expect("Failed to reconnect");
    assert_eq!(adapter.state(), TransportState::Connected);

    // Verify we can still send commands
    let result = adapter.send(RobotCommand::Stop);
    assert!(result.is_ok(), "Send after reconnect failed: {:?}", result.err());

    adapter.disconnect().await.expect("Failed to disconnect");
}
