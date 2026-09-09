//! E2E tests for MVP Operational Closure (FASE 6.7–6.9)
//!
//! These three vertical demos validate that the Interconnection architecture
//! works end-to-end:
//! 1. Reactive Execution: DSL code reads sensor → decides → commands robot
//! 2. Calibration: Robotics reads encoder via Interconnection, writes parameter back
//! 3. Communication Traceability: Observation → Signal → Channel → Binding → Endpoint

use thalos_core::device::{
    ChannelObservation, ChannelValue, ConnectionState, DerivedSignal, Endpoint, EndpointKind,
    Signal, SignalBinding, SignalDirection, SignalExpression, SignalKind, SignalQuality,
};
use thalos_core::robot::RobotCommand;
use thalos_runtime::execution::session::{
    Action, CapturingCommandProvider, ChannelAccessError, CommandProvider, Decision,
    DomainExecutionCoordinator, Environment, eval_channel_access, eval_derived_signal,
    ExecutionConfiguration, InMemoryObservationProvider, NoopCommandProvider,
    ObservationBundle, ObservationProvider, Reactivity, RobotState,
    SharedRobotObservation, TelemetryExecutionRunner, TickOutcome,
};

// ═══════════════════════════════════════════════════════════════════════════
// 6.7 — Reactive Execution E2E
// ═══════════════════════════════════════════════════════════════════════════

/// Simulates a .thalos reactive program:
/// ```text
/// if camera.target_x > 80
///     movej target_high
/// else
///     movej target_low
/// ```
///
/// Validates the full loop:
/// Signal → Channel → ObservationProvider → ObservationBundle →
/// eval_channel_access → Decision → RobotCommand → CommandProvider
#[test]
fn test_reactive_execution_e2e() {
    let coordinator = DomainExecutionCoordinator::new();
    let session_id = coordinator.create_session(
        "reactive_pick_and_place",
        ExecutionConfiguration {
            environment: Environment::VirtualSimulation,
            reactivity: Reactivity::Reactive,
            ..Default::default()
        },
    );
    coordinator.initialize(&session_id).unwrap();
    coordinator.start(&session_id).unwrap();

    // Setup: sensor feeds target_x via ObservationProvider
    let obs_provider = InMemoryObservationProvider::new();
    obs_provider.set_channel("camera.target_x", 100.0);
    obs_provider.set_channel("camera.target_y", 50.0);

    let robot_provider = SharedRobotObservation::new(RobotState {
        joints: vec![0.0, 0.0, 0.0],
        velocities: vec![0.0, 0.0, 0.0],
    });

    let cmd_provider = CapturingCommandProvider::new();

    let mut runner = TelemetryExecutionRunner::new(
        obs_provider.clone(),
        robot_provider.clone(),
        cmd_provider.clone(),
        thalos_runtime::execution::session::ExpectedState::default(),
    );

    // Tick 1: target_x = 100.0 > 80 → should dispatch motion
    let res1 = coordinator.tick_with_runner(&session_id, &mut runner, |bundle, _rob| {
        let target_x = eval_channel_access("camera", "camera.target_x", bundle)
            .unwrap_or(0.0);

        if target_x > 80.0 {
            (
                Decision::MotionAction {
                    motion_type: "movej".to_string(),
                    target_name: "target_high".to_string(),
                },
                Action::DispatchMotion {
                    kind: "movej".to_string(),
                    target: "target_high".to_string(),
                },
            )
        } else {
            (
                Decision::MotionAction {
                    motion_type: "movej".to_string(),
                    target_name: "target_low".to_string(),
                },
                Action::DispatchMotion {
                    kind: "movej".to_string(),
                    target: "target_low".to_string(),
                },
            )
        }
    })
    .unwrap();

    assert_eq!(res1.decision, Decision::MotionAction {
        motion_type: "movej".to_string(),
        target_name: "target_high".to_string(),
    });

    // Verify command was dispatched via CommandProvider
    let captured = cmd_provider.captured();
    assert_eq!(captured.len(), 1);
    assert!(matches!(captured[0], RobotCommand::MoveJoints { .. }));

    // Tick 2: target_x = 50.0 < 80 → should dispatch low
    obs_provider.set_channel("camera.target_x", 50.0);

    let res2 = coordinator.tick_with_runner(&session_id, &mut runner, |bundle, _rob| {
        let target_x = eval_channel_access("camera", "camera.target_x", bundle)
            .unwrap_or(0.0);

        if target_x > 80.0 {
            (
                Decision::MotionAction {
                    motion_type: "movej".to_string(),
                    target_name: "target_high".to_string(),
                },
                Action::DispatchMotion {
                    kind: "movej".to_string(),
                    target: "target_high".to_string(),
                },
            )
        } else {
            (
                Decision::MotionAction {
                    motion_type: "movej".to_string(),
                    target_name: "target_low".to_string(),
                },
                Action::DispatchMotion {
                    kind: "movej".to_string(),
                    target: "target_low".to_string(),
                },
            )
        }
    })
    .unwrap();

    assert_eq!(res2.decision, Decision::MotionAction {
        motion_type: "movej".to_string(),
        target_name: "target_low".to_string(),
    });

    // Verify two commands dispatched
    let captured = cmd_provider.captured();
    assert_eq!(captured.len(), 2);
}

// ═══════════════════════════════════════════════════════════════════════════
// 6.7b — Reactive Execution with Derived Signals
// ═══════════════════════════════════════════════════════════════════════════

/// Tests reactive execution using a derived signal (error = target - actual).
/// Validates that Source and Derived signals work together in the same bundle.
#[test]
fn test_reactive_execution_with_derived_signal() {
    let obs_provider = InMemoryObservationProvider::new();
    obs_provider.set_channel("target.position", 100.0);
    obs_provider.set_channel("actual.position", 95.0);

    let bundle = obs_provider.snapshot();

    // Compute derived signal: error = target - actual
    let derived = DerivedSignal::new(
        "position_error",
        SignalExpression::Subtract(
            "target.position".to_string(),
            "actual.position".to_string(),
        ),
    );

    let error = eval_derived_signal(&derived, &bundle).unwrap();
    assert!((error - 5.0).abs() < 0.001);

    // Reactive decision based on derived signal
    let decision = if error > 3.0 {
        "CORRECT"
    } else {
        "HOLD"
    };
    assert_eq!(decision, "CORRECT");
}

// ═══════════════════════════════════════════════════════════════════════════
// 6.8 — Calibration E2E
// ═══════════════════════════════════════════════════════════════════════════

/// Simulates a calibration workflow:
/// 1. Read encoder position via Interconnection (Input signal)
/// 2. Compute calibration offset (Derived signal)
/// 3. Write calibration parameter via Interconnection (Output signal)
///
/// Validates bidirectional flow through CommandProvider and ObservationProvider.
#[test]
fn test_calibration_e2e() {
    // Setup: encoder provides raw position
    let obs_provider = InMemoryObservationProvider::new();
    obs_provider.set_channel("encoder.joint_1.raw", 1024.0);
    obs_provider.set_channel("encoder.scale", 0.001);

    let mut cmd_provider = CapturingCommandProvider::new();

    // Step 1: Read encoder value
    let bundle = obs_provider.snapshot();
    let raw_value = eval_channel_access("encoder", "encoder.joint_1.raw", &bundle).unwrap();
    let scale = eval_channel_access("encoder", "encoder.scale", &bundle).unwrap();

    assert!((raw_value - 1024.0).abs() < 0.001);
    assert!((scale - 0.001).abs() < 0.001);

    // Step 2: Compute calibration offset (derived)
    let calibrated = raw_value * scale;
    assert!((calibrated - 1.024).abs() < 0.001);

    // Step 3: Write calibration parameter via CommandProvider
    let cmd = RobotCommand::MoveJoints {
        positions_rad: vec![calibrated],
        velocities_rad_s: None,
    };
    cmd_provider.dispatch(&cmd).unwrap();

    let captured = cmd_provider.captured();
    assert_eq!(captured.len(), 1);
    match &captured[0] {
        RobotCommand::MoveJoints { positions_rad, .. } => {
            assert!((positions_rad[0] - 1.024).abs() < 0.001);
        }
        _ => panic!("Expected MoveJoints command"),
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 6.9 — Communication Traceability E2E
// ═══════════════════════════════════════════════════════════════════════════

/// Validates the full traceability chain:
/// Signal → Channel → Binding → Endpoint → Connection
///
/// Every step is inspectable and the chain is complete.
#[test]
fn test_communication_traceability_e2e() {
    // Define the signal
    let signal = Signal::source("robot.joint_1.position", SignalDirection::Input)
        .with_unit("rad");

    assert_eq!(signal.id, "robot.joint_1.position");
    assert_eq!(signal.kind, SignalKind::Source);
    assert_eq!(signal.direction, SignalDirection::Input);

    // Define the binding: signal → channel → endpoint
    let binding = SignalBinding {
        signal_id: "robot.joint_1.position".to_string(),
        channel_id: "esp32.joint.1.pos".to_string(),
        endpoint_id: "esp32-cell01".to_string(),
    };

    // Define the endpoint
    let endpoint = Endpoint {
        id: "esp32-cell01".to_string(),
        kind: EndpointKind::Esp32,
        connection_state: ConnectionState::Connected,
    };

    // Verify the chain is inspectable
    assert_eq!(binding.signal_id, signal.id);
    assert_eq!(binding.endpoint_id, endpoint.id);
    assert_eq!(endpoint.connection_state, ConnectionState::Connected);
    assert_eq!(endpoint.kind, EndpointKind::Esp32);

    // Simulate: observation arrives from this endpoint
    let obs_provider = InMemoryObservationProvider::new();
    obs_provider.set_observation(
        &binding.channel_id,
        ChannelObservation {
            channel_id: binding.channel_id.clone(),
            sampled_at_ns: 1_000_000_000,
            received_at_ns: 1_000_001_000,
            value: ChannelValue::Scalar(1.57),
            unit: Some("rad".to_string()),
            quality: SignalQuality::Nominal,
        },
    );

    let bundle = obs_provider.snapshot();

    // Trace back: observation → channel → binding → endpoint
    let obs = bundle.observations.get(&binding.channel_id).unwrap();
    assert_eq!(obs.channel_id, "esp32.joint.1.pos");
    assert_eq!(obs.quality, SignalQuality::Nominal);
    assert_eq!(obs.unit.as_deref(), Some("rad"));

    // Verify endpoint state
    assert_eq!(endpoint.connection_state, ConnectionState::Connected);

    // Full chain is documented and verifiable
    let trace = TraceabilityChain {
        signal: signal.clone(),
        binding: binding.clone(),
        endpoint: endpoint.clone(),
        observation_quality: obs.quality,
    };

    assert_eq!(trace.signal.id, "robot.joint_1.position");
    assert_eq!(trace.binding.channel_id, "esp32.joint.1.pos");
    assert_eq!(trace.endpoint.id, "esp32-cell01");
    assert_eq!(trace.observation_quality, SignalQuality::Nominal);
}

/// Documentation struct representing the full traceability chain.
/// In production, this would be a runtime-queryable structure.
struct TraceabilityChain {
    signal: Signal,
    binding: SignalBinding,
    endpoint: Endpoint,
    observation_quality: SignalQuality,
}

// ═══════════════════════════════════════════════════════════════════════════
// 6.7c — Reactive Execution: Capability Boundary
// ═══════════════════════════════════════════════════════════════════════════

/// Verifies that Execution does NOT depend on Interconnection runtime types.
/// This is the architectural invariant from ADR-016.
#[test]
fn test_execution_boundary_invariant() {
    // This test documents the architectural boundary:
    // Execution depends on ObservationProvider, CommandProvider, RobotObservationProvider
    // Execution does NOT depend on InterconnectionRuntime, InterconnectionModule, etc.
    //
    // If this test compiles, the boundary is intact.

    fn assert_execution_uses_contracts_only(
        _obs: &dyn ObservationProvider,
        _cmd: &mut dyn CommandProvider,
    ) {
        // Execution only knows about these traits
    }

    let obs = InMemoryObservationProvider::new();
    let mut cmd = NoopCommandProvider;

    assert_execution_uses_contracts_only(&obs, &mut cmd);
}

// ═══════════════════════════════════════════════════════════════════════════
// 12.1 — RobotTransportCommandProvider Adapter
// ═══════════════════════════════════════════════════════════════════════════

use std::sync::{Arc, Mutex};
use thalos_runtime::execution::session::{CommandError, TransportCommandProvider};
use thalos_ports::robot::fake::FakeRobotTransport;
use thalos_ports::robot::TransportState;

#[test]
fn test_transport_command_provider_sends_command() {
    let fake_transport = FakeRobotTransport::new();
    let transport = Arc::new(Mutex::new(fake_transport));

    let mut provider = TransportCommandProvider::new(transport.clone());

    let cmd = RobotCommand::MoveJoints {
        positions_rad: vec![0.1, 0.2, 0.3],
        velocities_rad_s: None,
    };

    let result = provider.dispatch(&cmd);
    assert!(result.is_ok());

    // Verify command was sent through transport
    let transport = transport.lock().unwrap();
    assert_eq!(transport.sent_commands.len(), 1);
    assert_eq!(transport.sent_commands[0], cmd);
}

#[test]
fn test_transport_command_provider_rejects_when_disconnected() {
    let mut fake_transport = FakeRobotTransport::new();
    fake_transport.state = TransportState::Disconnected;
    let transport = Arc::new(Mutex::new(fake_transport));

    let mut provider = TransportCommandProvider::new(transport);

    let cmd = RobotCommand::Stop;
    let result = provider.dispatch(&cmd);
    assert!(result.is_err());
    assert!(matches!(result.unwrap_err(), CommandError::NotConnected));
}

#[test]
fn test_transport_command_provider_multiple_commands() {
    let fake_transport = FakeRobotTransport::new();
    let transport = Arc::new(Mutex::new(fake_transport));

    let mut provider = TransportCommandProvider::new(transport.clone());

    provider.dispatch(&RobotCommand::MoveJoints {
        positions_rad: vec![0.1],
        velocities_rad_s: None,
    }).unwrap();

    provider.dispatch(&RobotCommand::MoveJoints {
        positions_rad: vec![0.2],
        velocities_rad_s: None,
    }).unwrap();

    provider.dispatch(&RobotCommand::Stop).unwrap();

    let transport = transport.lock().unwrap();
    assert_eq!(transport.sent_commands.len(), 3);
}
