//! ESP32 backend integration tests — STATUS polling, firmware-state mapping,
//! and SAMPLES collection, driven through a `FakeTransport` (no hardware).
//!
//! S2.x cover `robot_state()` polling + `map_firmware_state`; S3.x cover
//! execution-trace collection on completion. All responses are scripted.

use std::sync::Arc;
use std::time::Duration;

use thalos_engine::core::execution::plan::{
    ExecutionPlan, ExecutionSegment, ExecutionWaypoint, PlanInstruction,
};
use thalos_runtime::{
    ExecutionSession, RobotController, SessionStatus,
    backends::{esp32::Esp32Backend, transport::FakeTransport},
    state::robot_state::{MotionMode, RobotState},
};

/// Create a connected `Esp32Backend` over a FakeTransport that answers the
/// HELLO handshake.
async fn make_connected_backend() -> Esp32Backend {
    let transport = FakeTransport::new();
    let mut backend = Esp32Backend::new(Box::new(transport));
    backend.test_inject_response(b"HELLO 2 OK\n".to_vec()).await;
    backend.connect().await.expect("connect should succeed");
    assert!(backend.is_connected());
    backend
}

/// The 2-waypoint, 2-DOF plan the fixture executes — a single MoveJ segment
/// over 2.0 s.
fn two_dof_plan() -> ExecutionPlan {
    ExecutionPlan {
        waypoints: vec![
            ExecutionWaypoint {
                joints: vec![0.0, 0.0],
                timestamp: 0.0,
            },
            ExecutionWaypoint {
                joints: vec![1.0, 1.0],
                timestamp: 2.0,
            },
        ],
        segments: vec![ExecutionSegment {
            index: 0,
            planned_segment_index: 0,
            instruction: PlanInstruction::MoveJ,
            waypoint_range: 0..2,
        }],
        duration: 2.0,
    repeat_count: 1,
    }
}

/// Connect + execute a 2-waypoint, 2-DOF plan over 2.0s, so `plan_duration`
/// is stored on the backend (required for RUNNING → seconds progress).
async fn make_executing_backend() -> Esp32Backend {
    let mut backend = make_connected_backend().await;
    // Upload (v2 chunked, C): MANIFEST OK, SEGMENT OK — the 2 samples (dof=2
    // → chunk 64) form a trailing partial chunk → NO per-sample ACK; END_UPLOAD
    // READY; then EXECUTE OK.
    for _ in 0..2 {
        backend.test_inject_response(b"OK\n".to_vec()).await;
    }
    backend.test_inject_response(b"READY\n".to_vec()).await;
    // EXECUTE OK
    backend.test_inject_response(b"OK\n".to_vec()).await;
    backend
        .execute(two_dof_plan())
        .await
        .expect("execute should succeed");
    backend
}

/// Inject a STATUS response and poll once, sleeping past the 75ms poll TTL
/// first so a previous poll's cached state does not mask the new response.
async fn poll_status(backend: &Esp32Backend, line: &str) -> Arc<RobotState> {
    tokio::time::sleep(Duration::from_millis(80)).await;
    backend.test_inject_response(line.as_bytes().to_vec()).await;
    backend.robot_state().await
}

/// How many `STATUS` commands have been sent over the wire so far.
async fn status_polls_sent(backend: &Esp32Backend) -> usize {
    backend
        .test_sent_commands()
        .await
        .iter()
        .filter(|c| c.starts_with(b"STATUS"))
        .count()
}

// ── S2.1 RED: throttled STATUS polling with cache ─────────────────────────

#[tokio::test]
async fn robot_state_polls_then_serves_cached_within_ttl() {
    let backend = make_executing_backend().await;

    // First call: cache miss → polls STATUS, returns RUNNING-derived state.
    backend.test_inject_response(b"STATUS RUNNING 0.5 0.1 0.2\n".to_vec()).await;
    let first = backend.robot_state().await;
    assert_eq!(
        first.motion.mode,
        MotionMode::Moving,
        "RUNNING must map to Moving"
    );
    // progress stored as SECONDS: 0.5 * plan_duration(2.0) = 1.0
    assert!(
        (first.execution.progress - 1.0).abs() < 1e-9,
        "progress should be fraction*plan_duration = 1.0s, got {}",
        first.execution.progress
    );
    assert_eq!(first.joints.positions, vec![0.1, 0.2]);

    // Second call within the 75ms TTL: cached, no extra STATUS on the wire.
    let second = backend.robot_state().await;
    assert_eq!(status_polls_sent(&backend).await, 1, "only one STATUS poll");
    assert_eq!(second.execution.progress, first.execution.progress);
}

// ── S2.2 RED: firmware-state → runtime-state mapping (all states) ─────────

#[tokio::test]
async fn maps_idle_states_to_motion_idle() {
    let backend = make_executing_backend().await;

    let idle = poll_status(&backend, "STATUS IDLE").await;
    assert_eq!(idle.motion.mode, MotionMode::Idle);
    assert_eq!(idle.execution.progress, 0.0);

    let receiving = poll_status(&backend, "STATUS RECEIVING").await;
    assert_eq!(receiving.motion.mode, MotionMode::Idle);

    let ready = poll_status(&backend, "STATUS READY").await;
    assert_eq!(ready.motion.mode, MotionMode::Idle);
}

#[tokio::test]
async fn maps_running_to_moving_with_seconds_progress_and_joints() {
    let backend = make_executing_backend().await;

    let running = poll_status(&backend, "STATUS RUNNING 0.45 0.5 0.3").await;
    assert_eq!(running.motion.mode, MotionMode::Moving);
    // SECONDS semantics: 0.45 * 2.0s plan = 0.9s
    assert!(
        (running.execution.progress - 0.9).abs() < 1e-9,
        "got {}",
        running.execution.progress
    );
    assert_eq!(running.joints.positions, vec![0.5, 0.3]);
}

#[tokio::test]
async fn maps_error_to_estop() {
    let backend = make_executing_backend().await;

    let error = poll_status(&backend, "STATUS ERROR MOTOR_FAULT").await;
    assert_eq!(error.motion.mode, MotionMode::EStop, "ERROR → EStop");
    assert_eq!(error.execution.progress, 0.0);
}

// ── S2.3 RED: COMPLETED mapping ───────────────────────────────────────────

#[tokio::test]
async fn completed_maps_to_idle_with_progress_ge_1() {
    let backend = make_executing_backend().await;

    let completed = poll_status(&backend, "STATUS COMPLETED 5").await;
    assert_eq!(completed.motion.mode, MotionMode::Idle);
    assert!(
        completed.execution.progress >= 1.0,
        "COMPLETED progress must be >= 1.0 (plan_duration), got {}",
        completed.execution.progress
    );
}

// ── S2.4 RED: DTO progress convention (seconds / plan_duration) ──────────

/// The DTO mapper computes `progress = current_time / plan_duration`
/// (execution_session.rs). `ExecutionSession::derived(status, progress)`
/// stores the value in `current_time`, so feeding it SECONDS yields the
/// correct UI fraction when the caller divides by `plan_duration`.
#[test]
fn dto_progress_is_seconds_over_plan_duration() {
    let running = ExecutionSession::derived(SessionStatus::Running, 0.9);
    assert!(
        (running.progress(2.0) - 0.45).abs() < 1e-9,
        "0.9s / 2.0s plan = 0.45, got {}",
        running.progress(2.0)
    );

    let completed = ExecutionSession::derived(SessionStatus::Completed, 2.0);
    assert_eq!(completed.progress(2.0), 1.0);
}

// ── S3.1 RED: SAMPLES collection on COMPLETED ─────────────────────────────

#[tokio::test]
async fn completed_collects_samples_and_exposes_trace() {
    let backend = make_executing_backend().await;

    // Scripted completion: STATUS COMPLETED 5 → OK → 5 ts-first SAMPLE lines.
    backend.test_inject_response(b"STATUS COMPLETED 5\n".to_vec()).await;
    backend.test_inject_response(b"OK\n".to_vec()).await;
    for i in 0..5u64 {
        let ts = i * 500_000;
        let j = i as f64 * 0.1;
        backend
            .test_inject_response(format!("SAMPLE {ts} {j} 0.5\n").into_bytes())
            .await;
    }

    let state = backend.robot_state().await;
    // After collection the backend reports Idle with full progress.
    assert_eq!(state.motion.mode, MotionMode::Idle);
    assert!(state.execution.progress >= 1.0);

    // `SAMPLES 5` was sent over the wire.
    let sent = backend.test_sent_commands().await;
    assert!(
        sent.iter().any(|c| c.starts_with(b"SAMPLES 5")),
        "SAMPLES 5 must be sent"
    );

    // The collected trace is returned ONCE, then None (clear-on-collect).
    let trace = backend.take_execution_trace().await.expect("trace available");
    assert_eq!(trace.len(), 5);
    assert_eq!(trace[0].timestamp_us, 0);
    assert_eq!(trace[4].timestamp_us, 2_000_000);
    assert_eq!(trace[0].joints, vec![0.0, 0.5]);
    assert!(backend.take_execution_trace().await.is_none());
}

#[tokio::test]
async fn completed_zero_samples_skips_send_and_collect() {
    let backend = make_executing_backend().await;

    // S3.2+S3.3: firmware rejects `SAMPLES 0` as MALFORMED — the host MUST
    // NOT send it; completion is still handled.
    backend.test_inject_response(b"STATUS COMPLETED 0\n".to_vec()).await;

    let state = backend.robot_state().await;
    assert_eq!(state.motion.mode, MotionMode::Idle);
    assert!(state.execution.progress >= 1.0);

    let sent = backend.test_sent_commands().await;
    assert!(
        !sent.iter().any(|c| c.starts_with(b"SAMPLES")),
        "SAMPLES must never be sent for count <= 0"
    );
    assert!(backend.take_execution_trace().await.is_none());
}

// ── S3.3 (part of S3.1): collect-direction format is ts-first ─────────────
// Covered by completed_collects_samples_and_exposes_trace asserting
// timestamp_us / joints parsed from `SAMPLE <ts> <j0..jN>` lines.
