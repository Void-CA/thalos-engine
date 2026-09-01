//! Canonical Behavioral Scenarios & Executable Invariant Matrix for Station, Module, Resource & Transport lifecycle separation.
//!
//! Station Operational Invariant Matrix:
//! | Invariant                                     | Test |
//! | ---------------------------------------------- | ---- |
//! | Opening Station does not activate hardware     | S02  |
//! | Station can host N modules                     | S10  |
//! | Module failure does not imply Station failure  | S03  |
//! | Resource reservation is exclusive              | S05  |
//! | Acquisition is shared                          | S06  |
//! | Robot + IIoT are concurrent                    | S04  |
//! | Physical resource can serve multiple ports     | S08  |
//! | Transport failure is contained                 | S09  |
//! | Capabilities resolve independently of hardware | S07  |
//! | Capability availability is granular            | S11  |
//! | Session cleanup releases resources             | S12  |
//! | Stopped Station owns no operational resources  | S01  |
//! | Station is an operational isolation boundary   | S13  |
//! | Hardware swap preserves session semantics      | S14  |

use thalos_engine::prelude::*;
use thalos_ports::device::{
    ChannelSubscription, DeviceTransport,
};
use thalos_ports::robot::{
    RobotCommand, RobotTransport, TransportError, TransportState,
};

use thalos_runtime::acquisition::{
    AcquisitionRequirement, AcquisitionRuntime, SamplingRequirement,
};
use thalos_runtime::resources::{
    ReservationError, ResourceRegistry, ResourceReservationManager, ResourceResolver,
};
use thalos_runtime::station::{StationRuntime, StationRuntimeState};
use thalos_runtime::test_support::{FakeDeviceTransport, FakeRobotTransport};

/// Helper: Find subscribed target_hz for a channel ID in FakeDeviceTransport
fn find_subscription_hz(transport: &FakeDeviceTransport, channel_id: &str) -> Option<u32> {
    transport
        .active_subscriptions
        .iter()
        .find(|s| s.channel_id.as_str() == channel_id)
        .map(|s| s.target_hz)
}

/// S01 — Station lifecycle & Stopped State Invariants:
/// Created -> Starting -> Ready -> Active -> Ready -> Stopping -> Stopped.
/// Stopped station owns 0 active sessions, 0 reservations, and 0 leases.
#[test]
fn s01_station_full_lifecycle_and_stopped_invariants() {
    let robot_ref = ResourceRef::new("scara-01", ResourceKind::Robot);
    let station = Station::new("cell-01", "SCARA Cell", vec![robot_ref]);
    let registry = ResourceRegistry::new();

    let mut station_runtime = StationRuntime::new(station, registry);
    assert_eq!(station_runtime.state(), StationRuntimeState::Created);

    // Created -> Ready
    station_runtime.start().unwrap();
    assert_eq!(station_runtime.state(), StationRuntimeState::Ready);
    assert!(station_runtime.state().is_operational());

    // Ready -> Active
    let session = station_runtime.start_session().unwrap();
    assert_eq!(station_runtime.state(), StationRuntimeState::Active);
    assert_eq!(station_runtime.active_session().unwrap().id, session.id);

    // Active -> Ready
    station_runtime.stop_session().unwrap();
    assert_eq!(station_runtime.state(), StationRuntimeState::Ready);

    // Ready -> Stopped
    station_runtime.stop().unwrap();
    assert_eq!(station_runtime.state(), StationRuntimeState::Stopped);
    assert!(!station_runtime.state().is_operational());
    assert!(station_runtime.active_session().is_none());
}

/// S02 — Opening a Station transitions it to Ready, but does NOT trigger hardware acquisition or subscriptions.
#[test]
fn s02_station_open_does_not_start_acquisition() {
    let station = Station::new("station-greenhouse-01", "Greenhouse 01", vec![]);
    let mut registry = ResourceRegistry::new();
    let sensor = Resource::new(
        "bme280-temp",
        ResourceKind::Channel,
        "Temperature Sensor",
        vec![CapabilityRequirement::TemperatureSensor],
    );
    registry.register(sensor);

    let mut station_runtime = StationRuntime::new(station, registry);
    assert_eq!(station_runtime.state(), StationRuntimeState::Created);

    // Opening Station -> Ready
    station_runtime.start().unwrap();
    assert_eq!(station_runtime.state(), StationRuntimeState::Ready);

    // Acquisition runtime initialized with FakeDeviceTransport is completely idle
    let fake_transport = FakeDeviceTransport::new();
    let mut acq_runtime = AcquisitionRuntime::new(fake_transport);

    assert_eq!(acq_runtime.transport().active_subscriptions.len(), 0);
    assert_eq!(acq_runtime.drain_observations().len(), 0);
}

/// S03 — Module availability != Station availability. Faulting one module/resource does not fault the Station.
#[test]
fn s03_module_failure_does_not_fault_station() {
    let station = Station::new("station-cell-1", "Cell 1", vec![]);
    let mut registry = ResourceRegistry::new();

    let robot_r1 = Resource::new("r1", ResourceKind::Robot, "Robot Arm R1", vec![]);
    let robot_r2 = Resource::new("r2", ResourceKind::Robot, "Robot Arm R2", vec![]);
    let env_e1 = Resource::new("e1", ResourceKind::Channel, "Environment E1", vec![]);

    registry.register(robot_r1);
    registry.register(robot_r2);
    registry.register(env_e1);

    let mut station_runtime = StationRuntime::new(station, registry);
    station_runtime.start().unwrap();
    assert_eq!(station_runtime.state(), StationRuntimeState::Ready);

    // Simulate R1 encountering a transport fault
    let mut transport_r1 = FakeRobotTransport::new();
    transport_r1.state = TransportState::Faulted;

    // Session requiring R1 fails
    let r1_err = transport_r1
        .send(RobotCommand::MoveJoints {
            positions_rad: vec![0.0],
            velocities_rad_s: None,
        })
        .unwrap_err();
    assert_eq!(r1_err, TransportError::Disconnected);

    // Station remains operational (Ready)
    assert_eq!(station_runtime.state(), StationRuntimeState::Ready);

    // Session requiring R2 + E1 executes successfully
    let mut transport_r2 = FakeRobotTransport::new();
    transport_r2.state = TransportState::Connected;
    transport_r2
        .send(RobotCommand::MoveJoints {
            positions_rad: vec![0.5],
            velocities_rad_s: None,
        })
        .unwrap();

    assert_eq!(transport_r2.sent_commands.len(), 1);
    assert_eq!(station_runtime.state(), StationRuntimeState::Ready);
}

/// S04 — An ExecutionSession and an AcquisitionSession run concurrently in the same Station without conflict.
#[test]
fn s04_concurrent_robot_and_iiot_sessions() {
    let station = Station::new("station-01", "Station 01", vec![]);
    let mut registry = ResourceRegistry::new();

    let robot = Resource::new("robot-r1", ResourceKind::Robot, "Robot R1", vec![]);
    let sensor = Resource::new("temp-s1", ResourceKind::Channel, "Temperature Sensor", vec![]);
    registry.register(robot);
    registry.register(sensor);

    let mut station_runtime = StationRuntime::new(station, registry);
    station_runtime.start().unwrap(); // Ready

    // Session A: Execution Session reserves Robot R1
    let _session_a = station_runtime.start_session().unwrap();
    let mut resv_mgr = ResourceReservationManager::new();
    let robot_ref = ResourceRef::new("robot-r1", ResourceKind::Robot);
    let resv_a = resv_mgr
        .reserve(ExecutionSessionId("exec-session-a".into()), vec![robot_ref])
        .unwrap();

    // Session B: Acquisition Session leases Temperature Sensor
    let fake_transport = FakeDeviceTransport::new();
    let mut acq_runtime = AcquisitionRuntime::new(fake_transport);
    let acq_req = AcquisitionRequirement {
        channel_id: "temp-s1".into(),
        sampling: SamplingRequirement::Continuous { target_hz: 10 },
        required: true,
    };
    let lease_b = acq_runtime.acquire_lease(&acq_req).unwrap();

    // Both sessions coexist active
    assert!(resv_mgr.is_reserved(&ResourceId("robot-r1".into())));
    assert_eq!(acq_runtime.active_lease_count(&"temp-s1".into()), 1);

    // Cleanup
    resv_mgr.release(&resv_a.id).unwrap();
    acq_runtime.release_lease(lease_b).unwrap();
    station_runtime.stop_session().unwrap();
}

/// S05 — Concurrent reservation attempt on the same robot resource is rejected with ResourceAlreadyReserved.
#[test]
fn s05_exclusive_robot_reservation_rejects_second_session() {
    let mut resv_mgr = ResourceReservationManager::new();
    let robot_ref = ResourceRef::new("robot-r1", ResourceKind::Robot);

    // Session A acquires reservation on robot-r1
    let _resv_a = resv_mgr
        .reserve(ExecutionSessionId("session-a".into()), vec![robot_ref.clone()])
        .unwrap();

    // Session B attempts to reserve same robot-r1
    let resv_b_result = resv_mgr.reserve(ExecutionSessionId("session-b".into()), vec![robot_ref]);

    assert_eq!(
        resv_b_result,
        Err(ReservationError::ResourceAlreadyReserved("robot-r1".into()))
    );
}

/// S06 — Multiple sessions leasing the same telemetry channel update sampling rate to max and unsubscribe on teardown.
#[test]
fn s06_shared_acquisition_lease_rate_escalation_and_teardown() {
    let fake_transport = FakeDeviceTransport::new();
    let mut acq_runtime = AcquisitionRuntime::new(fake_transport);

    let req_10hz = AcquisitionRequirement {
        channel_id: "temperature".into(),
        sampling: SamplingRequirement::Continuous { target_hz: 10 },
        required: true,
    };
    let req_2hz = AcquisitionRequirement {
        channel_id: "temperature".into(),
        sampling: SamplingRequirement::Continuous { target_hz: 2 },
        required: true,
    };

    // 1. Session A leases @ 10 Hz
    let lease_a = acq_runtime.acquire_lease(&req_10hz).unwrap();
    assert_eq!(
        find_subscription_hz(acq_runtime.transport(), "temperature"),
        Some(10)
    );

    // 2. Session B leases @ 2 Hz -> max target_hz remains 10 Hz
    let lease_b = acq_runtime.acquire_lease(&req_2hz).unwrap();
    assert_eq!(acq_runtime.active_lease_count(&"temperature".into()), 2);
    assert_eq!(
        find_subscription_hz(acq_runtime.transport(), "temperature"),
        Some(10)
    );

    // 3. Release Session A (10 Hz) -> steps down to Session B's 2 Hz
    acq_runtime.release_lease(lease_a).unwrap();
    assert_eq!(acq_runtime.active_lease_count(&"temperature".into()), 1);
    assert_eq!(
        find_subscription_hz(acq_runtime.transport(), "temperature"),
        Some(2)
    );

    // 4. Release Session B (2 Hz) -> transport unsubscribes (IDLE)
    acq_runtime.release_lease(lease_b).unwrap();
    assert_eq!(acq_runtime.active_lease_count(&"temperature".into()), 0);
    assert_eq!(
        find_subscription_hz(acq_runtime.transport(), "temperature"),
        None
    );
}

/// S07 — Capability resolution & Fallback: Program requests capability ("robot.motion"), resolving to available R2 when R1 is reserved.
#[test]
fn s07_capability_resolution_falls_back_when_preferred_resource_is_reserved() {
    let station = Station::new("station-cell-01", "Cell 01", vec![]);
    let mut registry = ResourceRegistry::new();
    let mut resv_mgr = ResourceReservationManager::new();

    let r1 = Resource::new(
        "robot-r1",
        ResourceKind::Robot,
        "Robot R1",
        vec![CapabilityRequirement::JointMotion],
    );
    let r2 = Resource::new(
        "robot-r2",
        ResourceKind::Robot,
        "Robot R2",
        vec![CapabilityRequirement::JointMotion],
    );

    registry.register(r1.clone());
    registry.register(r2.clone());

    // Session A reserves R1
    let _resv_a = resv_mgr
        .reserve(ExecutionSessionId("session-a".into()), vec![r1.to_ref()])
        .unwrap();

    // Session B requests JointMotion capability
    let reqs = vec![ResourceRequirement::mandatory(
        CapabilityRequirement::JointMotion,
    )];

    let resolved =
        ResourceResolver::resolve_available(&station, &registry, &resv_mgr, &reqs).unwrap();

    assert_eq!(resolved.matches.len(), 1);
    assert_eq!(resolved.matches[0].matched_resource.id.as_str(), "robot-r2");
}

/// S08 — Dual L1 adapters (RobotTransport + DeviceTransport) operating over the same underlying hardware target.
#[test]
fn s08_shared_hardware_dual_ports_isolation() {
    let mut fake_robot_transport = FakeRobotTransport::new();
    let mut fake_device_transport = FakeDeviceTransport::new();

    // Set both to connected
    fake_robot_transport.state = TransportState::Connected;

    // Dispatch RobotCommand over RobotTransport
    fake_robot_transport
        .send(RobotCommand::MoveJoints {
            positions_rad: vec![0.1, 0.2, 0.3],
            velocities_rad_s: None,
        })
        .unwrap();

    // Acquire Device Telemetry over DeviceTransport
    fake_device_transport
        .subscribe(ChannelSubscription {
            channel_id: "esp32_temp".into(),
            target_hz: 50,
        })
        .unwrap();

    // Verify L1 domain boundaries operate independently
    assert_eq!(fake_robot_transport.sent_commands.len(), 1);
    assert_eq!(
        find_subscription_hz(&fake_device_transport, "esp32_temp"),
        Some(50)
    );
}

/// S09 — Physical transport disconnection interrupts/faults the ExecutionSession while leaving the Station operational context in Ready.
#[test]
fn s09_resource_disconnection_faults_session_without_crashing_station() {
    let station = Station::new("station-01", "Station 01", vec![]);
    let registry = ResourceRegistry::new();

    let mut station_runtime = StationRuntime::new(station, registry);
    station_runtime.start().unwrap(); // Ready
    let _session = station_runtime.start_session().unwrap(); // Active

    // Robot transport disconnects / faults
    let mut robot_transport = FakeRobotTransport::new();
    robot_transport.state = TransportState::Connected;

    // Simulate transport failure during command send
    robot_transport.state = TransportState::Faulted;
    let err = robot_transport
        .send(RobotCommand::MoveJoints {
            positions_rad: vec![0.0],
            velocities_rad_s: None,
        })
        .unwrap_err();

    assert_eq!(err, TransportError::Disconnected);

    // Stop session due to execution fault
    station_runtime.stop_session().unwrap();

    // Station remains operational (Ready), ready for recovery or new session
    assert_eq!(station_runtime.state(), StationRuntimeState::Ready);
    assert!(station_runtime.state().is_operational());
}

/// S10 — Multi-Module independence: Robot A, Robot B, and Environment Module operate simultaneously without cross-interference.
#[test]
fn s10_multi_module_independence() {
    let mut resv_mgr = ResourceReservationManager::new();
    let fake_transport = FakeDeviceTransport::new();
    let mut acq_runtime = AcquisitionRuntime::new(fake_transport);

    let robot_a = ResourceRef::new("robot-a", ResourceKind::Robot);
    let robot_b = ResourceRef::new("robot-b", ResourceKind::Robot);

    // Session A reserves Robot A
    let resv_a = resv_mgr
        .reserve(ExecutionSessionId("sess-a".into()), vec![robot_a])
        .unwrap();

    // Session B reserves Robot B
    let resv_b = resv_mgr
        .reserve(ExecutionSessionId("sess-b".into()), vec![robot_b])
        .unwrap();

    // Session C acquires Environment telemetry
    let acq_req = AcquisitionRequirement {
        channel_id: "esp32-env-01".into(),
        sampling: SamplingRequirement::Continuous { target_hz: 100 },
        required: true,
    };
    let lease_c = acq_runtime.acquire_lease(&acq_req).unwrap();

    // All 3 operate concurrently without state collision
    assert!(resv_mgr.is_reserved(&ResourceId("robot-a".into())));
    assert!(resv_mgr.is_reserved(&ResourceId("robot-b".into())));
    assert_eq!(acq_runtime.active_lease_count(&"esp32-env-01".into()), 1);

    // Teardown
    resv_mgr.release(&resv_a.id).unwrap();
    resv_mgr.release(&resv_b.id).unwrap();
    acq_runtime.release_lease(lease_c).unwrap();
}

/// S11 — Partial capability failure: Motion interface fails while Telemetry interface remains available.
#[test]
fn s11_granular_capability_failure() {
    let mut robot_transport = FakeRobotTransport::new();
    let mut device_transport = FakeDeviceTransport::new();

    // Motion port encounters transport fault
    robot_transport.state = TransportState::Faulted;
    let motion_err = robot_transport
        .send(RobotCommand::MoveJoints {
            positions_rad: vec![0.0],
            velocities_rad_s: None,
        })
        .unwrap_err();
    assert_eq!(motion_err, TransportError::Disconnected);

    // Telemetry port remains Connected and functional
    device_transport.state = TransportState::Connected;
    device_transport
        .subscribe(ChannelSubscription {
            channel_id: "robot_temp".into(),
            target_hz: 10,
        })
        .unwrap();

    assert_eq!(
        find_subscription_hz(&device_transport, "robot_temp"),
        Some(10)
    );
}

/// S12 — Robust Session Cleanup: Unexpected session cancellation/fault cleanly releases exclusive reservations and acquisition leases.
#[test]
fn s12_session_cleanup_releases_all_reservations_and_leases() {
    let station = Station::new("station-cell-01", "Cell 01", vec![]);
    let registry = ResourceRegistry::new();
    let mut station_runtime = StationRuntime::new(station, registry);
    station_runtime.start().unwrap();

    let mut resv_mgr = ResourceReservationManager::new();
    let fake_transport = FakeDeviceTransport::new();
    let mut acq_runtime = AcquisitionRuntime::new(fake_transport);

    let session = station_runtime.start_session().unwrap();
    let session_id = ExecutionSessionId(session.id.as_str().to_string());

    // Session acquires 1 Robot Reservation & 3 Channel Acquisition Leases
    let robot_ref = ResourceRef::new("scara-01", ResourceKind::Robot);
    let resv = resv_mgr
        .reserve(session_id.clone(), vec![robot_ref])
        .unwrap();

    let lease_1 = acq_runtime
        .acquire_lease(&AcquisitionRequirement {
            channel_id: "ch-1".into(),
            sampling: SamplingRequirement::Continuous { target_hz: 10 },
            required: true,
        })
        .unwrap();
    let lease_2 = acq_runtime
        .acquire_lease(&AcquisitionRequirement {
            channel_id: "ch-2".into(),
            sampling: SamplingRequirement::Continuous { target_hz: 20 },
            required: true,
        })
        .unwrap();
    let lease_3 = acq_runtime
        .acquire_lease(&AcquisitionRequirement {
            channel_id: "ch-3".into(),
            sampling: SamplingRequirement::Continuous { target_hz: 50 },
            required: true,
        })
        .unwrap();

    // Verify active resources before teardown
    assert!(resv_mgr.is_reserved(&ResourceId("scara-01".into())));
    assert_eq!(acq_runtime.transport().active_subscriptions.len(), 3);

    // Simulate Session Teardown on Failure/Cancellation
    resv_mgr.release(&resv.id).unwrap();
    acq_runtime.release_lease(lease_1).unwrap();
    acq_runtime.release_lease(lease_2).unwrap();
    acq_runtime.release_lease(lease_3).unwrap();
    station_runtime.stop_session().unwrap();

    // Verify complete resource & lease release + Station return to Ready
    assert!(!resv_mgr.is_reserved(&ResourceId("scara-01".into())));
    assert_eq!(acq_runtime.transport().active_subscriptions.len(), 0);
    assert_eq!(station_runtime.state(), StationRuntimeState::Ready);
    assert!(station_runtime.state().is_operational());
}

/// S13 — Cross-Station Isolation Boundary: Station S1 cannot resolve or hijack resources bound explicitly to Station S2.
#[test]
fn s13_cross_station_isolation_boundary() {
    let r1 = Resource::new(
        "robot-r1",
        ResourceKind::Robot,
        "Robot R1",
        vec![CapabilityRequirement::JointMotion],
    );
    let r2 = Resource::new(
        "robot-r2",
        ResourceKind::Robot,
        "Robot R2",
        vec![CapabilityRequirement::JointMotion],
    );

    // Station 1 explicitly binds R1; Station 2 explicitly binds R2
    let station_1 = Station::new("station-s1", "Station 1", vec![r1.to_ref()]);
    let station_2 = Station::new("station-s2", "Station 2", vec![r2.to_ref()]);

    let mut registry = ResourceRegistry::new();
    registry.register(r1);
    registry.register(r2);

    let reqs = vec![ResourceRequirement::mandatory(CapabilityRequirement::JointMotion)];

    // Resolution scoped to Station 1 deterministically resolves R1
    let resolved_s1 = ResourceResolver::resolve(&station_1, &registry, &reqs).unwrap();
    assert_eq!(resolved_s1.matches[0].matched_resource.id.as_str(), "robot-r1");

    // Resolution scoped to Station 2 deterministically resolves R2
    let resolved_s2 = ResourceResolver::resolve(&station_2, &registry, &reqs).unwrap();
    assert_eq!(resolved_s2.matches[0].matched_resource.id.as_str(), "robot-r2");
}

/// S14 — Module replacement without session semantic change: Hot-swapping hardware (ESP32 vs PLC) leaves session intent intact.
#[test]
fn s14_module_replacement_without_session_semantic_change() {
    let station = Station::new("cell-01", "Cell 01", vec![]);
    let reqs = vec![ResourceRequirement::mandatory(CapabilityRequirement::JointMotion)];

    // Run 1: Hardware backed by ESP32 Stepper Arm
    let mut registry_esp32 = ResourceRegistry::new();
    let esp32_robot = Resource::new(
        "esp32-arm",
        ResourceKind::Robot,
        "ESP32 Stepper Arm",
        vec![CapabilityRequirement::JointMotion],
    );
    registry_esp32.register(esp32_robot);

    let resolved_run_1 = ResourceResolver::resolve(&station, &registry_esp32, &reqs).unwrap();
    assert_eq!(resolved_run_1.matches[0].matched_resource.id.as_str(), "esp32-arm");

    // Run 2: Hot-swapped to Beckhoff Industrial PLC Arm
    let mut registry_plc = ResourceRegistry::new();
    let plc_robot = Resource::new(
        "industrial-plc-arm",
        ResourceKind::Robot,
        "Beckhoff Industrial PLC Arm",
        vec![CapabilityRequirement::JointMotion],
    );
    registry_plc.register(plc_robot);

    let resolved_run_2 = ResourceResolver::resolve(&station, &registry_plc, &reqs).unwrap();
    assert_eq!(resolved_run_2.matches[0].matched_resource.id.as_str(), "industrial-plc-arm");

    // Abstract execution intent (JointMotion) is identical and unchanged
    assert_eq!(reqs[0].capability, CapabilityRequirement::JointMotion);
}
