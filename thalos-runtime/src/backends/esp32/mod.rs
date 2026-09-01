//! ESP32 execution backend — connects the `RobotController` trait to an
//! ESP32 running the firmware-side execution engine.
//!
//! The backend performs a batch upload→execute→collect cycle against the
//! ESP32 via `Esp32Protocol`, which owns all text wire-format concerns.

pub mod device;
pub mod protocol;

pub use device::{ChannelBinding, Esp32DeviceAdapter};

use std::sync::Arc;

use async_trait::async_trait;

use crate::backends::controller::{BackendCapabilities, RobotController};
use crate::backends::transport::{Transport, TransportError};
use crate::error::ControllerError;
use crate::execution_boundary::manifest::{
    ExecutionManifest, ManifestInstruction, ManifestMetadata, ManifestSegment, TimedWaypoint,
};
use crate::execution_boundary::manifest_builder::ExecutionManifestBuilder;
use crate::execution_boundary::ExecutionSample;
use crate::session::execution_source::ExecutionSource;
use crate::state::robot_state::{MotionMode, RobotState};
use thalos_engine::core::execution::plan::{BuilderError, ExecutionPlan};

use protocol::{Esp32Protocol, FirmwareState, ProtocolError};

/// ESP32 hardware backend.
///
/// Implements `RobotController` by delegating all wire communication to
/// `Esp32Protocol`. The protocol tracks firmware state and handles text
/// encoding/decoding.
///
/// Interior mutability: `robot_state(&self)` must poll STATUS and collect
/// samples through `&mut Esp32Protocol`, so the protocol lives behind a
/// `tokio::sync::Mutex`. Polled states are cached for a 75ms TTL so the UI's
/// ~60Hz tick loop does not hammer the wire.
pub struct Esp32Backend {
    protocol: tokio::sync::Mutex<Option<Esp32Protocol>>,
    connected: std::sync::atomic::AtomicBool,
    /// RES-02: consecutive `robot_state` poll failures — after 3 the
    /// connection is declared lost (`connected` cleared) so the next
    /// tick/snapshot surfaces a connection problem instead of freezing.
    consecutive_poll_failures: std::sync::atomic::AtomicU32,
    /// Total trajectory duration (seconds) of the current execution — set by
    /// `execute()`, reset on `disconnect()`. Used to convert the firmware's
    /// 0..1 progress fraction into SECONDS (R2.4/R2.5 pinned decision).
    plan_duration: f64,
    /// Throttled poll cache: last polled state + poll timestamp (75ms TTL).
    cached_state: tokio::sync::Mutex<Option<(std::time::Instant, Arc<RobotState>)>>,
    /// Samples collected on COMPLETED, consumed once by `take_execution_trace`.
    collected_samples: tokio::sync::Mutex<Option<Vec<ExecutionSample>>>,
    /// Last firmware status transition logged (0=idle/unknown, 1=RUNNING,
    /// 2=COMPLETED, 3=ERROR) — dedup so STATUS polls only log on change,
    /// giving an explicit `RUNNING → COMPLETED` trace in the logs (PR-0).
    last_status_logged: std::sync::atomic::AtomicU8,
}

impl Esp32Backend {
    /// Create a new `Esp32Backend` over the given transport.
    ///
    /// The transport is wrapped in an `Esp32Protocol` with the expected
    /// protocol version (currently 1). The handshake is not performed
    /// until `connect()` is called.
    pub fn new(transport: Box<dyn Transport>) -> Self {
        Self {
            // Protocol v2 (C): chunked upload ACK + 460800 baud. A stale v1
            // firmware fails the handshake (VERSION_MISMATCH) before upload.
            protocol: tokio::sync::Mutex::new(Some(Esp32Protocol::new(transport, 2))),
            connected: std::sync::atomic::AtomicBool::new(false),
            consecutive_poll_failures: std::sync::atomic::AtomicU32::new(0),
            plan_duration: 0.0,
            cached_state: tokio::sync::Mutex::new(None),
            collected_samples: tokio::sync::Mutex::new(None),
            last_status_logged: std::sync::atomic::AtomicU8::new(0),
        }
    }

    /// Get a mutable reference to the protocol, if connected.
    ///
    /// `&mut self` callers use `tokio::sync::Mutex::get_mut` (no await needed);
    /// `&self` callers (`robot_state`, `take_execution_trace`) lock instead.
    fn protocol_mut(&mut self) -> Result<&mut Esp32Protocol, ControllerError> {
        self.protocol
            .get_mut()
            .as_mut()
            .ok_or(ControllerError::NotConnected)
    }

    /// Map a firmware state to a runtime [`RobotState`] — the single source
    /// of truth for the firmware → runtime mapping (design decision table).
    ///
    /// | Firmware | motion.mode | execution.progress | joints |
    /// |---|---|---|---|
    /// | IDLE / RECEIVING / READY | Idle | 0.0 | [] |
    /// | RUNNING (→ Executing) | Moving | fraction × plan_duration (SECONDS) | commanded |
    /// | COMPLETED | Idle | plan_duration, or 1.0 if < 1.0s | last commanded joints from cached RUNNING, else [] |
    /// | ERROR | EStop | 0.0 | [] |
    async fn map_firmware_state(&self, fs: &FirmwareState) -> RobotState {
        let mut state = RobotState::default();
        match fs {
            FirmwareState::Idle | FirmwareState::Receiving | FirmwareState::Ready => {
                state.motion.mode = MotionMode::Idle;
                state.execution.progress = 0.0;
            }
            FirmwareState::Executing { progress, joints } => {
                state.motion.mode = MotionMode::Moving;
                // R2.4/R2.5 (pinned): progress is SECONDS (fraction × plan_duration)
                // so the DTO mapper (current_time / plan_duration) yields the
                // correct 0..1 fraction on the wire.
                state.execution.progress = progress * self.plan_duration;
                state.joints.positions = joints.clone();
            }
            FirmwareState::Completed { .. } => {
                state.motion.mode = MotionMode::Idle;
                // COMPLETED → full progress. For plans ≥ 1s this is
                // plan_duration (seconds); short plans (< 1s) map to 1.0 so
                // completion detection (`progress >= 1.0`) still fires.
                state.execution.progress = if self.plan_duration >= 1.0 {
                    self.plan_duration
                } else {
                    1.0
                };
            }
            FirmwareState::Error(_) => {
                // ERROR → EStop so the existing `EStop → Failed` path in
                // session_from_robot_state works unchanged.
                state.motion.mode = MotionMode::EStop;
                state.execution.progress = 0.0;
            }
        }
        state
    }

    /// Map a protocol-layer failure to a `ControllerError` (R4-001): a
    /// transport that reports `Disconnected` means the device vanished
    /// mid-operation → `ConnectionLost` (so the frontend offers Reconectar);
    /// everything else stays a generic `Protocol` error.
    fn map_protocol_error(context: &str, e: ProtocolError) -> ControllerError {
        match e {
            ProtocolError::Transport(TransportError::Disconnected) => {
                ControllerError::ConnectionLost
            }
            other => ControllerError::Protocol(format!("{context}: {other}")),
        }
    }

    /// Map a pure-chain builder rejection to a graceful `ControllerError`,
    /// preserving the firmware-parity diagnostic CODE in the message.
    ///
    /// R1-1 (CRITICAL): the builder rejects plans the old structural
    /// `validate_manifest` passed — out-of-envelope positions (INVALID_JOINT)
    /// and implied velocities above the firmware ceilings (VELOCITY_EXCEEDED).
    /// The rejection MUST surface as a graceful `InvalidManifest` (→ 4xx at
    /// the API), NEVER a panic. The frontend error-UX keys on the machine-
    /// readable code, so `Validation(code)` keeps the code verbatim.
    fn map_builder_error(e: BuilderError) -> ControllerError {
        match e {
            BuilderError::Validation(code) => ControllerError::InvalidManifest(format!(
                "plan rejected by the firmware-parity validator: {code}"
            )),
            BuilderError::DedupConflict { index, t } => ControllerError::InvalidManifest(format!(
                "duplicate timestamp {t} with different positions at waypoint {index}"
            )),
        }
    }
}

#[async_trait]
impl RobotController for Esp32Backend {
    async fn connect(&mut self) -> Result<(), ControllerError> {
        if self.is_connected() {
            return Err(ControllerError::AlreadyConnected);
        }

        let protocol = self.protocol_mut()?;

        protocol.handshake().await.map_err(|e| {
            let mapped = Self::map_protocol_error("handshake failed", e);
            tracing::error!(error = %mapped, "ESP32 handshake failed");
            mapped
        })?;

        self.connected
            .store(true, std::sync::atomic::Ordering::SeqCst);
        // RES-02: a fresh connection resets the poll-failure streak.
        self.consecutive_poll_failures
            .store(0, std::sync::atomic::Ordering::SeqCst);
        tracing::info!("ESP32 connected (handshake OK)");
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), ControllerError> {
        self.connected
            .store(false, std::sync::atomic::Ordering::SeqCst);
        self.plan_duration = 0.0;
        // Stale poll cache / collected samples must not leak across connects.
        *self.cached_state.lock().await = None;
        *self.collected_samples.lock().await = None;
        self.last_status_logged
            .store(0, std::sync::atomic::Ordering::SeqCst);
        if let Some(protocol) = self.protocol.get_mut().as_mut() {
            let _ = protocol.stop().await;
        }
        Ok(())
    }

    fn is_connected(&self) -> bool {
        self.connected.load(std::sync::atomic::Ordering::SeqCst)
    }

    async fn execute(
        &mut self,
        plan: ExecutionPlan,
    ) -> Result<(), ControllerError> {
        if !self.is_connected() {
            return Err(ControllerError::NotConnected);
        }

        let total_samples = plan.waypoints.len();
        // Store the plan duration so STATUS polls can map fraction → seconds.
        self.plan_duration = plan.duration;

        // DEGENERATE TRUNCATION GUARD (legacy shim parity): a duration whose
        // µs value truncates below (N-1) — `(duration * 1e6) as u64`
        // (TRUNCATION, NOT round) — collapses every reconstructed gap to
        // 0 µs. The pure builder CANNOT represent N distinct sub-µs
        // timestamps: it either returns `Err(DedupConflict)` (equal
        // timestamp, different joints) or silently collapses N distinct
        // commanded waypoints when joints are bit-equal. Legacy behavior was
        // total: an N-sample manifest with all `dt_us = 0` (`duration_us` =
        // 0) that the firmware validator accepts (timing diff 0 <= 1000 µs
        // floor). Bypass the builder and reproduce that output exactly.
        //
        // M3 (ADR-3/ADR-5): this all-dt_us==0 output is NOT an instant-jump
        // plan. dt_us==0 makes physical velocity v = Δq/Δt UNDEFINED — the
        // manifest carries NO timing claim the executor could read as a jump.
        // Velocity-bounding is FIRMWARE-AUTHORITATIVE: the executor controls
        // advancement as max_velocity × elapsed_real_time and steps at most
        // one dt_us==0 waypoint per update (PROTOCOL SEMANTICS). The backend
        // never infers host velocity from Δq over a zero dt.
        let manifest = if total_samples > 1 && {
            let duration_us = (plan.duration * 1_000_000.0) as u64;
            duration_us < (total_samples - 1) as u64
        } {
            ExecutionManifest {
                metadata: ManifestMetadata {
                    dof_count: plan.waypoints.first().map(|w| w.joints.len()).unwrap_or(0),
                    total_samples,
                    duration_us: 0,
                    repeat_count: plan.repeat_count,
                },
                segments: vec![ManifestSegment {
                    index: 0,
                    instruction: ManifestInstruction::MoveJ,
                    sample_start: 0,
                    sample_count: total_samples,
                }],
                samples: plan
                    .waypoints
                    .iter()
                    .map(|wp| TimedWaypoint {
                        joints: wp.joints.clone(),
                        dt_us: 0,
                    })
                    .collect(),
            }
        } else {
            // The real-timestamp pure chain: absolute timestamps become
            // per-gap dt_us with the REAL dt (no even-spacing reconstruction),
            // and the firmware-parity validation (INVALID_JOINT /
            // VELOCITY_EXCEEDED / TIMING_INVALID / …) runs inside build().
            // R1-1 (CRITICAL): a rejection surfaces as a graceful
            // `InvalidManifest` (→ 4xx at the API), never a panic. No wire
            // traffic has happened yet. Fail loud, reject-not-clamp — never
            // silent clamp/mutation of the commanded plan.
            ExecutionManifestBuilder::build(&plan).map_err(|e| {
                tracing::error!(
                    error = %e,
                    waypoints = plan.waypoints.len(),
                    duration_s = plan.duration,
                    "ESP32 execute rejected by manifest builder (no wire traffic)"
                );
                Self::map_builder_error(e)
            })?
        };

        let protocol = self.protocol_mut()?;

        // Upload → READY. One NOT_IDLE recovery: a stale firmware state
        // (READY/EXECUTING/ERROR left over from a previous session) rejects
        // MANIFEST — STOP resets the firmware to IDLE, then retry once.
        // Observed on real hardware after a failed connect left the device
        // in a non-IDLE state.
        let mut upload = protocol.upload_manifest(&manifest).await;
        if let Err(ProtocolError::EspError(reason)) = &upload {
            if reason.trim() == "NOT_IDLE" {
                tracing::warn!(reason = %reason, "manifest rejected NOT_IDLE — STOP-resetting the firmware and retrying");
                protocol.stop().await.ok(); // consumes its response; firmware → IDLE
                upload = protocol.upload_manifest(&manifest).await;
            }
        }
        upload
            .map_err(|e| {
                let mapped = Self::map_protocol_error("upload failed", e);
                tracing::error!(error = %mapped, waypoints = plan.waypoints.len(), "ESP32 manifest upload failed");
                mapped
            })?;
        tracing::info!(
            waypoints = plan.waypoints.len(),
            duration_s = plan.duration,
            "ESP32 manifest uploaded (READY)"
        );

        // Execute → OK
        protocol.start_execution().await.map_err(|e| {
            let mapped = Self::map_protocol_error("execute failed", e);
            tracing::error!(error = %mapped, "ESP32 start_execution failed");
            mapped
        })?;
        tracing::info!("ESP32 execution started (EXECUTE OK)");

        // Return immediately per RobotController contract
        Ok(())
    }

    async fn stop(&mut self) -> Result<(), ControllerError> {
        if !self.is_connected() {
            return Err(ControllerError::NotConnected);
        }
        let protocol = self.protocol_mut()?;
        protocol
            .stop()
            .await
            .map_err(|e| Self::map_protocol_error("stop failed", e))?;
        Ok(())
    }

    /// Live state via a throttled STATUS poll (75ms TTL cache).
    ///
    /// Infallible: poll errors fall back to the cached state, else a default
    /// state. A not-connected backend returns a default state immediately.
    async fn robot_state(&self) -> Arc<RobotState> {
        const POLL_TTL: std::time::Duration = std::time::Duration::from_millis(75);

        // Cache hit within TTL → no wire traffic (UI ticks at ~60Hz).
        {
            let cached = self.cached_state.lock().await;
            if let Some((at, state)) = cached.as_ref() {
                if at.elapsed() < POLL_TTL {
                    return state.clone();
                }
            }
        }

        if !self.is_connected() {
            return Arc::new(RobotState::default());
        }

        let poll_result = {
            let mut guard = self.protocol.lock().await;
            match guard.as_mut() {
                Some(protocol) => protocol.query_status().await,
                None => {
                    return Arc::new(RobotState::default());
                }
            }
        };

        let state = match poll_result {
            Ok(fs) => {
                // A successful poll breaks the failure streak (RES-02).
                self.consecutive_poll_failures
                    .store(0, std::sync::atomic::Ordering::SeqCst);
                // Log firmware status TRANSITIONS (dedup) — the PR-0 evidence
                // of the RUNNING → COMPLETED cycle in the integration logs.
                let tag: u8 = match &fs {
                    FirmwareState::Executing { .. } => 1,
                    FirmwareState::Completed { .. } => 2,
                    FirmwareState::Error(_) => 3,
                    _ => 0,
                };
                if tag != 0 {
                    let prev = self
                        .last_status_logged
                        .swap(tag, std::sync::atomic::Ordering::SeqCst);
                    if prev != tag {
                        match tag {
                            1 => tracing::info!("firmware status: RUNNING"),
                            2 => tracing::info!("firmware status: COMPLETED"),
                            _ => tracing::info!("firmware status: ERROR"),
                        }
                    }
                }
                // On COMPLETED, collect the recorded samples (S3.5). Guard on
                // `sample_count > 0`: the firmware rejects `SAMPLES 0` as
                // MALFORMED (protocol.cpp), so the host must never send it.
                if let FirmwareState::Completed { sample_count } = &fs {
                    if *sample_count > 0 {
                        let mut guard = self.protocol.lock().await;
                        if let Some(protocol) = guard.as_mut() {
                            if let Ok(samples) =
                                protocol.collect_samples(*sample_count as usize).await
                            {
                                *self.collected_samples.lock().await = Some(samples);
                            }
                        }
                    }
                }

                let mut state = self.map_firmware_state(&fs).await;
                // COMPLETED: carry over the last commanded joints from the
                // previous cached RUNNING state, if any (design table: "last
                // commanded").
                if matches!(fs, FirmwareState::Completed { .. }) {
                    if let Some((_, cached)) = self.cached_state.lock().await.as_ref() {
                        state.joints.positions = cached.joints.positions.clone();
                    }
                }
                let state = Arc::new(state);
                *self.cached_state.lock().await = Some((std::time::Instant::now(), state.clone()));
                state
            }
            // Poll error (timeout / disconnected) → cached state, else default.
            // RES-02: after 3 CONSECUTIVE failures clear `connected` so the
            // next tick/snapshot surfaces a connection problem instead of a
            // frozen stale Running state; this call still serves the cache.
            Err(e) => {
                let failures = self
                    .consecutive_poll_failures
                    .fetch_add(1, std::sync::atomic::Ordering::SeqCst)
                    + 1;
                tracing::debug!(error = %e, failures, "ESP32 STATUS poll failed");
                if failures >= 3 {
                    self.connected
                        .store(false, std::sync::atomic::Ordering::SeqCst);
                    tracing::warn!("ESP32 marked disconnected after 3 consecutive poll failures");
                }
                let cached = self.cached_state.lock().await;
                match cached.as_ref() {
                    Some((_, state)) => state.clone(),
                    None => Arc::new(RobotState::default()),
                }
            }
        };

        state
    }

    /// Take the collected execution samples (SAMPLES) exactly once.
    ///
    /// The scene service drains this after completion detection; `mem::take`
    /// clears the buffer so a subsequent call returns `None`.
    async fn take_execution_trace(&self) -> Option<Vec<ExecutionSample>> {
        let mut guard = self.collected_samples.lock().await;
        std::mem::take(&mut *guard)
    }

    fn capabilities(&self) -> BackendCapabilities {
        BackendCapabilities {
            firmware_repeat: true,
            ..BackendCapabilities::minimal()
        }
    }

    /// The ESP32 is a real hardware execution backend: report `Hardware` so the
    /// UI execution-source badge reflects the actual controller instead of the
    /// `Simulation` default (review fix R4-001).
    fn execution_source(&self) -> ExecutionSource {
        ExecutionSource::Hardware
    }
}

// ── Test helpers (for integration tests; always available but test-only by contract) ──

impl Esp32Backend {
    /// Expose the protocol's sent commands for integration test verification.
    ///
    /// # Contract
    ///
    /// This method is intended for integration tests ONLY. It provides access
    /// to the raw wire commands sent by the backend for verification purposes.
    /// Production code MUST NOT depend on this method.
    pub async fn test_sent_commands(&self) -> Vec<Vec<u8>> {
        let guard = self.protocol.lock().await;
        guard
            .as_ref()
            .map(|p| p.test_sent_commands())
            .unwrap_or_default()
    }

    /// Expose the protocol for integration test response injection.
    ///
    /// # Contract
    ///
    /// This method is intended for integration tests ONLY. It allows
    /// pre-loading response data into the underlying transport for
    /// simulating firmware interactions.
    pub async fn test_inject_response(&self, data: Vec<u8>) {
        let guard = self.protocol.lock().await;
        if let Some(protocol) = guard.as_ref() {
            protocol.test_inject_response(data);
        }
    }
}

// ═════════════════════════════════════════════════════════════════════
// TESTS
// ═════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backends::transport::{FakeTransport, TransportError};
    use thalos_engine::core::execution::plan::{
        ExecutionPlan, ExecutionSegment, ExecutionWaypoint, PlanInstruction,
    };

    /// Helper: create a connected Esp32Backend with a FakeTransport that
    /// will respond with HELLO 2 OK on the first handshake (protocol v2, C).
    async fn make_connected_backend(transport: FakeTransport) -> Esp32Backend {
        let mut backend = Esp32Backend::new(Box::new(transport));
        // Inject the HELLO response BEFORE connect
        backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_inject_response(b"HELLO 2 OK\n".to_vec());
        backend.connect().await.expect("connect should succeed");
        assert!(backend.is_connected());
        backend
    }

    /// Build a single-segment MoveJ `ExecutionPlan` from raw waypoints and a
    /// duration — the even-spacing reconstruction used by the legacy shim,
    /// kept as the migration fixture for pre-existing tests.
    fn plan_of(waypoints: Vec<Vec<f64>>, duration: f64) -> ExecutionPlan {
        let n = waypoints.len();
        let duration_us = (duration * 1_000_000.0) as u64;
        let dt_per_sample = if n > 1 {
            duration_us / (n - 1) as u64
        } else {
            0
        };
        ExecutionPlan {
            waypoints: waypoints
                .iter()
                .enumerate()
                .map(|(i, joints)| ExecutionWaypoint {
                    joints: joints.clone(),
                    timestamp: i as f64 * dt_per_sample as f64 / 1_000_000.0,
                })
                .collect(),
            segments: vec![ExecutionSegment {
                index: 0,
                planned_segment_index: 0,
                instruction: PlanInstruction::MoveJ,
                waypoint_range: 0..n,
            }],
            duration,
            repeat_count: 1,
        }
    }

    /// Inject the response sequence for a full upload→execute against a plan
    /// (protocol v2, C): OK (MANIFEST), OK (SEGMENT), ONE OK per COMPLETE
    /// SAMPLE chunk (`n / chunk` — the trailing partial chunk gets no ACK),
    /// READY (END_UPLOAD), OK (EXECUTE). The chunk is derived exactly like
    /// `upload_manifest` (same 3072-byte RX-buffer invariant).
    async fn inject_full_upload(backend: &Esp32Backend, plan: &ExecutionPlan) {
        let dof = plan.waypoints.first().map(|w| w.joints.len()).unwrap_or(2);
        let max_line = 19 + 10 * dof;
        let chunk = (3072usize / max_line.max(1)).clamp(1, 64);
        let full_chunks = plan.waypoints.len() / chunk;
        let protocol = backend.protocol.lock().await;
        let p = protocol.as_ref().unwrap();
        // MANIFEST + SEGMENT(s) — one OK each.
        p.test_inject_response(b"OK\n".to_vec());
        p.test_inject_response(b"OK\n".to_vec());
        // One OK per COMPLETE SAMPLE chunk.
        for _ in 0..full_chunks {
            p.test_inject_response(b"OK\n".to_vec());
        }
        p.test_inject_response(b"READY\n".to_vec());
        p.test_inject_response(b"OK\n".to_vec());
    }

    /// Flatten the recorded `send()` buffers into individual wire LINES.
    /// Protocol v2 batching (C) merges many SAMPLE lines into one `send()`, so
    /// per-send buffers no longer map 1:1 to protocol lines.
    async fn sent_lines(backend: &Esp32Backend) -> Vec<String> {
        let sent = backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_sent_commands();
        sent.iter()
            .flat_map(|c| {
                String::from_utf8_lossy(c)
                    .lines()
                    .map(|s| s.to_string())
                    .collect::<Vec<_>>()
            })
            .collect()
    }

    /// Regression (a) — limit-to-limit at the velocity ceiling MUST PASS
    /// (no false VELOCITY_EXCEEDED). A trapezoidal plan whose cruise is
    /// EXACTLY the 1.0 rad/s ceiling: the legacy even-spacing shim
    /// reconstructed dt = duration_us/(N-1) and read the 10 ms cruise gaps
    /// as ~1.0017 rad/s (false positive); the real-timestamp chain reads the
    /// true 10 ms dt → exactly 1.0 rad/s → accepted.
    #[tokio::test]
    async fn execute_accepts_velocity_ceiling_trapezoid() {
        let transport = FakeTransport::new();
        let mut backend = make_connected_backend(transport).await;

        // Trapezoid on the base joint: 1.6 ms ramp samples, 10 ms cruise
        // samples. Cruise Δq = 1.0 rad/s × 10 ms = 0.01 rad per gap.
        let plan = ExecutionPlan {
        repeat_count: 1,
            waypoints: vec![
                ExecutionWaypoint { joints: vec![0.0, 0.0], timestamp: 0.0 },
                ExecutionWaypoint { joints: vec![0.0008, 0.0], timestamp: 0.0016 },
                ExecutionWaypoint { joints: vec![0.0024, 0.0], timestamp: 0.0032 },
                ExecutionWaypoint { joints: vec![0.0124, 0.0], timestamp: 0.0132 },
                ExecutionWaypoint { joints: vec![0.0224, 0.0], timestamp: 0.0232 },
                ExecutionWaypoint { joints: vec![0.0324, 0.0], timestamp: 0.0332 },
                ExecutionWaypoint { joints: vec![0.0332, 0.0], timestamp: 0.0348 },
                ExecutionWaypoint { joints: vec![0.0336, 0.0], timestamp: 0.0364 },
            ],
            segments: vec![ExecutionSegment {
                index: 0,
                planned_segment_index: 0,
                instruction: PlanInstruction::MoveJ,
                waypoint_range: 0..8,
            }],
            duration: 0.0364,
        };

        // Direct pure-chain proof: the REAL per-gap dt keeps every implied
        // velocity at or below the 1.0 rad/s ceiling.
        let manifest = ExecutionManifestBuilder::build(&plan)
            .expect("cruise exactly at the ceiling must build");
        let dt: Vec<u32> = manifest.samples.iter().map(|s| s.dt_us).collect();
        assert_eq!(
            dt,
            vec![0, 1_600, 1_600, 10_000, 10_000, 10_000, 1_600, 1_600],
            "real per-gap dt must reach the manifest"
        );

        // End-to-end: execute() accepts the plan (no false VELOCITY_EXCEEDED)
        // and uploads it — a connected backend with a valid trapezoid.
        let transport = FakeTransport::new();
        let mut backend = make_connected_backend(transport).await;
        inject_full_upload(&backend, &plan).await;
        backend
            .execute(plan)
            .await
            .expect("velocity-ceiling trapezoid must execute");
        assert!(backend.is_connected());
    }

    /// Regression (b) — a GENUINE exceedance with real dt MUST still be
    /// rejected: base joint Δq = 0.02 rad over a real 10 ms gap = 2.0 rad/s,
    /// double the 1.0 rad/s ceiling. The message must carry the
    /// VELOCITY_EXCEEDED diagnostic code.
    #[tokio::test]
    async fn execute_rejects_genuine_velocity_exceedance_with_real_dt() {
        let transport = FakeTransport::new();
        let mut backend = make_connected_backend(transport).await;

        // Real timestamps: one 10 ms gap, Δq = 0.02 → 2.0 rad/s > 1.0.
        let plan = ExecutionPlan {
        repeat_count: 1,
            waypoints: vec![
                ExecutionWaypoint { joints: vec![0.0, 0.0], timestamp: 0.0 },
                ExecutionWaypoint { joints: vec![0.02, 0.0], timestamp: 0.01 },
            ],
            segments: vec![ExecutionSegment {
                index: 0,
                planned_segment_index: 0,
                instruction: PlanInstruction::MoveJ,
                waypoint_range: 0..2,
            }],
            duration: 0.01,
        };

        let result = backend.execute(plan).await;
        match result {
            Ok(()) => panic!("a 2.0 rad/s gap must be rejected, not executed"),
            Err(ControllerError::InvalidManifest(msg)) => {
                assert!(
                    msg.contains("VELOCITY_EXCEEDED"),
                    "rejection must carry the VELOCITY_EXCEEDED code: {msg}"
                );
            }
            Err(other) => panic!("expected InvalidManifest, got {other:?}"),
        }

        // Rejected BEFORE wire traffic: only HELLO from connect was sent.
        assert!(backend.is_connected());
        let sent = backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_sent_commands();
        assert_eq!(sent.len(), 1, "only HELLO from connect — no upload traffic");
        assert_eq!(String::from_utf8(sent[0].clone()).unwrap(), "HELLO 2\n");
    }

    /// Regression (c) — the degenerate TRUNCATION guard: N > 1 waypoints and
    /// a TRUNCATED duration_us `(duration * 1e6) as u64` smaller than (N-1)
    /// produces the all-dt_us==0 manifest (duration_us: 0, single MoveJ
    /// segment, every dt_us 0) — bypassing the builder, which cannot
    /// represent N distinct sub-µs timestamps. This is the MIGRATED legacy
    /// `degenerate_zero_dt_manifest_has_no_instant_jump_timing_claim` test
    /// against the new execute(plan) path.
    #[tokio::test]
    async fn execute_degenerate_truncation_produces_all_zero_dt_manifest() {
        let transport = FakeTransport::new();
        let mut backend = make_connected_backend(transport).await;

        // 3 waypoints over 1.5 µs: trunc(1.5) = 1 µs < (3-1) = 2 gaps → guard.
        let waypoints = vec![
            vec![0.0, 0.0, 0.0, 0.01],
            vec![0.5, 0.5, 0.5, 0.02],
            vec![1.0, 1.0, 1.0, 0.03],
        ];
        let plan = plan_of(waypoints.clone(), 1.5e-6);
        inject_full_upload(&backend, &plan).await;

        backend
            .execute(plan)
            .await
            .expect("sub-microsecond duration must take the degenerate branch");

        // The wire manifest is the all-zero-dt output: MANIFEST ... 0 and
        // every SAMPLE line ends with dt_us = 0 (no timing claim).
        let lines = sent_lines(&backend).await;
        let manifest_line = lines
            .iter()
            .find(|l| l.starts_with("MANIFEST"))
            .expect("MANIFEST must have been sent");
        let parts: Vec<&str> = manifest_line.trim().split_whitespace().collect();
        assert_eq!(parts[0], "MANIFEST");
        assert_eq!(parts[1], "4", "DOF preserved");
        assert_eq!(parts[2], "3", "all 3 samples preserved");
        assert_eq!(
            parts[3], "0",
            "degenerate manifest declares NO duration (duration_us = 0)"
        );

        let sample_lines: Vec<&String> = lines.iter().filter(|l| l.starts_with("SAMPLE")).collect();
        assert_eq!(sample_lines.len(), 3);
        for (i, line) in sample_lines.iter().enumerate() {
            let tokens: Vec<&str> = line.trim().split_whitespace().collect();
            assert_eq!(
                tokens.last().unwrap(),
                &"0",
                "sample {i} must carry dt_us = 0 (no instant-jump timing claim)"
            );
            // Every commanded joint preserved — nothing collapsed.
            assert_eq!(tokens[1], format!("{:.6}", waypoints[i][0]));
        }
    }

    /// Test transport that answers the HELLO handshake once, then reports the
    /// device disconnected on every subsequent `receive` (mid-operation drop).
    struct DisconnectAfterHandshake {
        handshaken: std::sync::atomic::AtomicBool,
    }

    impl DisconnectAfterHandshake {
        fn new() -> Self {
            Self {
                handshaken: std::sync::atomic::AtomicBool::new(false),
            }
        }
    }

    #[async_trait]
    impl Transport for DisconnectAfterHandshake {
        async fn connect(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
        async fn disconnect(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
        async fn send(&mut self, _data: &[u8]) -> Result<(), TransportError> {
            Ok(())
        }
        async fn receive(&mut self) -> Result<Vec<u8>, TransportError> {
            if !self
                .handshaken
                .swap(true, std::sync::atomic::Ordering::SeqCst)
            {
                Ok(b"HELLO 2 OK\n".to_vec())
            } else {
                Err(TransportError::Disconnected)
            }
        }
    }

    /// Test transport that is disconnected from the start — every receive
    /// reports the transport lost.
    struct AlwaysDisconnected;

    #[async_trait]
    impl Transport for AlwaysDisconnected {
        async fn connect(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
        async fn disconnect(&mut self) -> Result<(), TransportError> {
            Ok(())
        }
        async fn send(&mut self, _data: &[u8]) -> Result<(), TransportError> {
            Ok(())
        }
        async fn receive(&mut self) -> Result<Vec<u8>, TransportError> {
            Err(TransportError::Disconnected)
        }
    }

    /// R4-001: a transport that reports `Disconnected` mid-operation must
    /// surface as `ControllerError::ConnectionLost` (not a generic Protocol
    /// error) so the execution flow can offer the Reconectar CTA.
    #[tokio::test]
    async fn connect_with_disconnected_transport_returns_connection_lost() {
        let mut backend = Esp32Backend::new(Box::new(AlwaysDisconnected));
        let err = backend.connect().await.unwrap_err();
        assert_eq!(err, ControllerError::ConnectionLost);
    }

    /// R4-001: same contract for `execute` — the device dropping during
    /// upload/execute reports `ConnectionLost`, not `Protocol`.
    #[tokio::test]
    async fn execute_with_disconnected_transport_returns_connection_lost() {
        let mut backend = Esp32Backend::new(Box::new(DisconnectAfterHandshake::new()));
        backend.connect().await.expect("handshake should succeed");
        let err = backend
            .execute(plan_of(vec![vec![0.0, 0.0], vec![1.0, 1.0]], 1.0))
            .await
            .unwrap_err();
        assert_eq!(err, ControllerError::ConnectionLost);
    }

    /// RES-02 (RED): N consecutive poll failures must clear `connected` so
    /// the next tick/snapshot surfaces a connection problem instead of
    /// serving the stale cached state (or default) forever with the session
    /// stuck Running.
    #[tokio::test]
    async fn consecutive_poll_failures_clear_connected() {
        let mut backend = Esp32Backend::new(Box::new(DisconnectAfterHandshake::new()));
        backend.connect().await.expect("handshake succeeds");
        assert!(backend.is_connected());

        // Each poll sleeps past the 75ms cache TTL so it actually hits the wire.
        for _ in 0..3 {
            tokio::time::sleep(std::time::Duration::from_millis(80)).await;
            let _ = backend.robot_state().await;
        }
        assert!(
            !backend.is_connected(),
            "3 consecutive poll failures must clear connected"
        );
    }

    // ── Task 2.5: RED — full upload→execute→collect cycle ────────────

    #[tokio::test]
    async fn full_cycle_with_fake_transport() {
        let transport = FakeTransport::new();
        let mut backend = make_connected_backend(transport).await;

        // Inject responses for the full upload→execute flow (protocol v2, C):
        // MANIFEST OK, SEGMENT OK, NO chunk ACK (2 samples < chunk 64 → the
        // trailing partial chunk gets no ACK; END_UPLOAD confirms it), READY,
        // EXECUTE OK.
        backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_inject_response(b"OK\n".to_vec()); // MANIFEST
        backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_inject_response(b"OK\n".to_vec()); // SEGMENT
        backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_inject_response(b"READY\n".to_vec()); // END_UPLOAD
        backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_inject_response(b"OK\n".to_vec()); // EXECUTE

        // Execute with simple waypoints
        let waypoints = vec![vec![0.0, 0.0], vec![1.0, 1.0]];
        backend
            .execute(plan_of(waypoints, 1.0))
            .await
            .expect("execute should succeed");
        assert!(backend.is_connected());

        // Verify commands were sent
        let sent = backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_sent_commands();
        assert!(!sent.is_empty(), "commands should have been sent");

        // HELLO was first (from connect)
        assert_eq!(String::from_utf8(sent[0].clone()).unwrap(), "HELLO 2\n");

        // Check MANIFEST was sent
        let has_manifest = sent.iter().any(|c| c.starts_with(b"MANIFEST"));
        assert!(has_manifest, "MANIFEST should have been sent");

        // Check EXECUTE was sent
        let has_execute = sent.iter().any(|c| c.starts_with(b"EXECUTE"));
        assert!(has_execute, "EXECUTE should have been sent");
    }

    /// v2 (C): a multi-chunk upload consumes exactly ONE ACK per SAMPLE chunk
    /// — not one per line. DOF=6 → chunk = 3072/(19+60) = 38; 150 samples →
    /// ceil(150/38) = 4 chunk ACKs. The MANIFEST line declares the chunk so
    /// the firmware counts the batch boundaries.
    #[tokio::test]
    async fn execute_consumes_one_ack_per_sample_chunk() {
        use std::sync::atomic::Ordering;

        let transport = FakeTransport::new();
        let mut backend = make_connected_backend(transport).await;

        let dof = 6usize;
        let n = 150usize;
        let waypoints: Vec<Vec<f64>> = (0..n)
            .map(|i| {
                let t = i as f64 * 0.0001; // tiny amplitude — inside the envelope
                vec![t; dof]
            })
            .collect();
        let plan = plan_of(waypoints, 1.0);

        // Inject: OK (MANIFEST), OK (SEGMENT), 3 chunk ACKs (150/38 = 3 FULL
        // chunks — the trailing 36 samples get no ACK), READY, OK.
        let protocol = backend.protocol.lock().await;
        let p = protocol.as_ref().unwrap();
        p.test_inject_response(b"OK\n".to_vec());
        p.test_inject_response(b"OK\n".to_vec());
        for _ in 0..3 {
            p.test_inject_response(b"OK\n".to_vec());
        }
        p.test_inject_response(b"READY\n".to_vec());
        p.test_inject_response(b"OK\n".to_vec());
        drop(protocol);

        backend
            .execute(plan)
            .await
            .expect("multi-chunk upload must succeed");
        assert!(backend.is_connected());

        // The MANIFEST line declares the derived chunk (3072 / max_line(6)).
        let lines = sent_lines(&backend).await;
        let manifest = lines
            .iter()
            .find(|l| l.starts_with("MANIFEST"))
            .expect("MANIFEST sent");
        let parts: Vec<&str> = manifest.trim().split_whitespace().collect();
        assert_eq!(parts[4], "38", "chunk derived from DOF=6: 3072/79 = 38");
        let sample_lines = lines.iter().filter(|l| l.starts_with("SAMPLE")).count();
        assert_eq!(sample_lines, n, "all SAMPLE lines hit the wire (batched)");
        assert!(backend.is_connected());
    }

    #[tokio::test]
    async fn double_connect_rejected() {
        let transport = FakeTransport::new();
        let mut backend = Esp32Backend::new(Box::new(transport));
        backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_inject_response(b"HELLO 2 OK\n".to_vec());
        backend.connect().await.expect("first connect");

        let err = backend.connect().await.unwrap_err();
        assert_eq!(err, ControllerError::AlreadyConnected);
    }

    #[tokio::test]
    async fn execute_requires_connection() {
        let transport = FakeTransport::new();
        let mut backend = Esp32Backend::new(Box::new(transport));

        let err = backend
            .execute(plan_of(vec![vec![0.0]], 1.0))
            .await
            .unwrap_err();
        assert_eq!(err, ControllerError::NotConnected);
    }

    // ── Task 2.9: RED — invalid manifest rejected before wire traffic ──

    #[tokio::test]
    async fn empty_waypoints_rejected_before_wire_traffic() {
        let transport = FakeTransport::new();
        // Inject HELLO response BUT NOT any manifest responses
        let mut backend = Esp32Backend::new(Box::new(transport));
        backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_inject_response(b"HELLO 2 OK\n".to_vec());
        backend.connect().await.expect("connect");

        let result = backend.execute(plan_of(vec![], 1.0)).await;

        assert!(result.is_err());
        match result.unwrap_err() {
            ControllerError::InvalidManifest(msg) => {
                assert!(!msg.is_empty(), "should have a descriptive message");
            }
            other => panic!("Expected InvalidManifest, got {other:?}"),
        }

        // Verify NO upload commands were sent over the transport
        // Only the 1 HELLO from connect should exist
        let sent = backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_sent_commands();
        assert_eq!(sent.len(), 1, "only HELLO should have been sent");
        assert_eq!(String::from_utf8(sent[0].clone()).unwrap(), "HELLO 2\n");
    }

    /// A zero-duration plan is the degenerate case the truncation guard owns
    /// — it is NOT a "duration must be positive" rejection anymore (that
    /// structural check lived in the removed `validate_manifest`). `execute`
    /// builds the all-dt_us==0 manifest and uploads it; the scene's `has_wps`
    /// guard is what stops zero-duration plans from reaching the wire.
    #[tokio::test]
    async fn zero_duration_plan_uploads_via_degenerate_guard() {
        let transport = FakeTransport::new();
        let mut backend = make_connected_backend(transport).await;
        // 2 waypoints, duration 0 → trunc(0) = 0 < (2-1) → degenerate branch.
        let plan = plan_of(vec![vec![0.0, 0.0], vec![1.0, 0.0]], 0.0);
        inject_full_upload(&backend, &plan).await;

        backend
            .execute(plan)
            .await
            .expect("zero-duration multi-waypoint plan takes the degenerate branch");

        let sent = backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_sent_commands();
        let manifest_line = sent
            .iter()
            .find(|c| c.starts_with(b"MANIFEST"))
            .expect("MANIFEST must have been sent");
        let manifest_text = String::from_utf8_lossy(manifest_line).to_string();
        let parts: Vec<&str> = manifest_text.trim().split_whitespace().collect();
        assert_eq!(parts[3], "0", "degenerate manifest declares duration_us = 0");
        // Every SAMPLE line ends with dt_us = 0.
        for c in sent.iter().filter(|c| c.starts_with(b"SAMPLE")) {
            let line_text = String::from_utf8_lossy(c).to_string();
            let tokens: Vec<&str> = line_text.trim().split_whitespace().collect();
            assert_eq!(tokens.last().unwrap(), &"0", "dt_us must be 0");
        }
    }

    #[tokio::test]
    async fn inconsistent_dof_rejected_before_wire_traffic() {
        let transport = FakeTransport::new();
        let mut backend = Esp32Backend::new(Box::new(transport));
        backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_inject_response(b"HELLO 2 OK\n".to_vec());
        backend.connect().await.expect("connect");

        let result = backend
            .execute(plan_of(vec![vec![0.0, 0.0], vec![1.0]], 1.0))
            .await;

        assert!(result.is_err());
        match result.unwrap_err() {
            ControllerError::InvalidManifest(msg) => {
                assert!(msg.contains("DOF"), "message should mention DOF: {msg}");
            }
            other => panic!("Expected InvalidManifest, got {other:?}"),
        }

        // Only HELLO was sent
        assert_eq!(
            backend
                .protocol
                .lock()
                .await
                .as_ref()
                .unwrap()
                .test_sent_commands()
                .len(),
            1
        );
    }

    // ── R1-1 (CRITICAL review finding): out-of-envelope plans MUST be ──
    // ── rejected gracefully by execute() — NEVER panic.               ──

    /// R1-1 (CRITICAL, deterministic): P2 added INVALID_JOINT/
    /// VELOCITY_EXCEEDED checks to `ExecutionManifestBuilder::validate`, but
    /// `execute()`'s deprecated `build_manifest` shim `.expect()` PANICKED
    /// when the pure builder rejected the plan — a DoS on the live
    /// start-execution path. A movej whose implied velocity exceeds the
    /// firmware SAFETY_ENVELOPE ceiling (base 1.0 rad over 0.2 s =
    /// 5.0 rad/s > 1.0 rad/s) passes the API planner (PhysicalEnvelope
    /// ceiling 25 rad/s) and previously crashed the backend. It MUST now
    /// surface as `ControllerError::InvalidManifest` (→ HTTP 400
    /// `invalid_manifest`) with the VELOCITY_EXCEEDED diagnostic — fail
    /// loud, reject-not-clamp, no wire traffic.
    #[tokio::test]
    async fn execute_rejects_out_of_envelope_velocity_without_panic() {
        let transport = FakeTransport::new();
        let mut backend = make_connected_backend(transport).await;

        // Base 1.0 rad over 0.2 s = 5.0 rad/s implied velocity — inside the
        // planner envelope (25 rad/s), outside the firmware envelope (1.0).
        let result = backend
            .execute(plan_of(vec![vec![0.0, 0.0], vec![1.0, 0.0]], 0.2))
            .await;

        match result {
            Ok(()) => panic!("out-of-envelope velocity plan must be rejected, not executed"),
            Err(ControllerError::InvalidManifest(msg)) => {
                assert!(
                    msg.contains("VELOCITY_EXCEEDED"),
                    "rejection must name the VELOCITY_EXCEEDED diagnostic: {msg}"
                );
            }
            Err(other) => panic!("expected InvalidManifest, got {other:?}"),
        }

        // Rejected BEFORE wire traffic: still connected, only the HELLO from
        // connect was sent.
        assert!(backend.is_connected());
        let sent = backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_sent_commands();
        assert_eq!(sent.len(), 1, "only HELLO from connect — no upload traffic");
        assert_eq!(String::from_utf8(sent[0].clone()).unwrap(), "HELLO 2\n");
    }

    /// R1-1 (CRITICAL): an out-of-envelope POSITION plan (base at 4.0 rad —
    /// outside the firmware ±1.5708 rad envelope) previously PANICKED the
    /// shim's `.expect()` (INVALID_JOINT). It MUST now be rejected
    /// gracefully with the INVALID_JOINT diagnostic — never clamped, never
    /// executed, no wire traffic.
    #[tokio::test]
    async fn execute_rejects_out_of_envelope_position_without_panic() {
        let transport = FakeTransport::new();
        let mut backend = make_connected_backend(transport).await;

        // Base 4.0 rad — the planner accepts it (URDF planning envelope),
        // the firmware SafetyEnvelope rejects it.
        let result = backend
            .execute(plan_of(vec![vec![0.0, 0.0], vec![4.0, 0.0]], 1.0))
            .await;

        match result {
            Ok(()) => panic!("out-of-envelope position plan must be rejected, not executed"),
            Err(ControllerError::InvalidManifest(msg)) => {
                assert!(
                    msg.contains("INVALID_JOINT"),
                    "rejection must name the INVALID_JOINT diagnostic: {msg}"
                );
            }
            Err(other) => panic!("expected InvalidManifest, got {other:?}"),
        }

        assert!(backend.is_connected());
        let sent = backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_sent_commands();
        assert_eq!(sent.len(), 1, "only HELLO from connect — no upload traffic");
    }

    // ── Additional backend tests ─────────────────────────────────────

    #[tokio::test]
    async fn disconnect_sends_stop_and_clears_connected() {
        let transport = FakeTransport::new();
        let mut backend = Esp32Backend::new(Box::new(transport));
        backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_inject_response(b"HELLO 2 OK\n".to_vec());
        backend.connect().await.expect("connect");
        assert!(backend.is_connected());

        backend
            .disconnect()
            .await
            .expect("disconnect should succeed");
        assert!(!backend.is_connected());
    }

    #[tokio::test]
    async fn capabilities_are_minimal() {
        let transport = FakeTransport::new();
        let backend = Esp32Backend::new(Box::new(transport));

        let caps = backend.capabilities();
        assert!(!caps.pause);
        assert!(!caps.resume);
        assert!(!caps.io);
        assert!(!caps.gripper);
        assert!(!caps.streaming);
        // v3: the ESP32 REPEATS INTERNALLY (manifest repeat_count, loops
        // back-to-back) — the ONLY backend with firmware-side repeat.
        assert!(caps.firmware_repeat);
    }

    /// v3: a firmware-side `Repeat { count }` plan uploads ONCE with the
    /// `repeat_count` in the MANIFEST (5th field) — NO re-upload per pass.
    #[tokio::test]
    async fn execute_repeat_uploads_once_with_count_in_manifest() {
        let transport = FakeTransport::new();
        let mut backend = make_connected_backend(transport).await;

        let mut plan = plan_of(
            vec![vec![0.0, 0.0], vec![0.5, 0.3]],
            1.0,
        );
        plan.repeat_count = 3;

        // Inject: MANIFEST OK, SEGMENT OK, no chunk ACK (2 samples < chunk),
        // READY, EXECUTE OK — the host uploads ONCE regardless of repeat.
        let protocol = backend.protocol.lock().await;
        let p = protocol.as_ref().unwrap();
        p.test_inject_response(b"OK\n".to_vec());
        p.test_inject_response(b"OK\n".to_vec());
        p.test_inject_response(b"READY\n".to_vec());
        p.test_inject_response(b"OK\n".to_vec());
        drop(protocol);

        backend
            .execute(plan)
            .await
            .expect("single upload with repeat_count must succeed");
        assert!(backend.is_connected());

        // The MANIFEST line carries the repeat count in the 5th field.
        let lines = sent_lines(&backend).await;
        let manifest = lines
            .iter()
            .find(|l| l.starts_with("MANIFEST"))
            .expect("MANIFEST sent");
        let parts: Vec<&str> = manifest.trim().split_whitespace().collect();
        assert_eq!(parts[4], "64", "chunk");
        assert_eq!(parts[5], "3", "repeat_count in MANIFEST");
        // EXACTLY the one upload — no second manifest for the next pass.
        let manifests = lines.iter().filter(|l| l.starts_with("MANIFEST")).count();
        assert_eq!(manifests, 1, "one upload for the whole Repeat");
    }

    #[test]
    fn execution_source_reports_hardware() {
        // The ESP32 is a real hardware execution backend — the UI badge must
        // reflect that, not the Simulation default (review fix R4-001).
        let transport = FakeTransport::new();
        let backend = Esp32Backend::new(Box::new(transport));
        assert_eq!(backend.execution_source(), ExecutionSource::Hardware);
    }

    /// Robustness regression (real hardware): a stale serial buffer (boot
    /// bytes / leftover from a previous session) can make the first HELLO
    /// read return garbage. The handshake retries once and succeeds.
    #[tokio::test]
    async fn handshake_survives_stale_buffer_line() {
        let transport = FakeTransport::new();
        let mut backend = Esp32Backend::new(Box::new(transport));
        // First read → stale garbage (observed: "0.000000 0.000000");
        // retry read → the real handshake response.
        backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_inject_response(b"0.000000 0.000000\n".to_vec());
        backend
            .protocol
            .lock()
            .await
            .as_ref()
            .unwrap()
            .test_inject_response(b"HELLO 2 OK\n".to_vec());

        backend
            .connect()
            .await
            .expect("handshake retry must recover from a stale buffer line");
        assert!(backend.is_connected());
    }

    /// Robustness regression (real hardware): a stale firmware state
    /// (READY/EXECUTING/ERROR from a previous session) rejects MANIFEST with
    /// NOT_IDLE. The backend STOP-resets the device (→ IDLE) and retries the
    /// upload once — no manual device reset needed.
    #[tokio::test]
    async fn upload_recovers_from_not_idle_with_stop_and_retry() {
        let transport = FakeTransport::new();
        let mut backend = make_connected_backend(transport).await;
        {
            let protocol = backend.protocol.lock().await;
            let p = protocol.as_ref().unwrap();
            // First upload: MANIFEST rejected because the firmware is not IDLE.
            p.test_inject_response(b"ERROR NOT_IDLE\n".to_vec());
            // Recovery STOP response (consumed by protocol.stop()).
            p.test_inject_response(b"OK\n".to_vec());
            // Retry upload (v2): MANIFEST → OK, SEGMENT → OK, then the 2
            // samples (chunk 64) form a trailing partial chunk → NO chunk ACK;
            // END_UPLOAD → READY.
            p.test_inject_response(b"OK\n".to_vec());
            p.test_inject_response(b"OK\n".to_vec());
            p.test_inject_response(b"READY\n".to_vec());
            // EXECUTE.
            p.test_inject_response(b"OK\n".to_vec());
        }

        backend
            .execute(plan_of(vec![vec![0.0, 0.0], vec![1.0, 1.0]], 1.0))
            .await
            .expect("upload must recover from NOT_IDLE with a STOP + retry");
        assert!(backend.is_connected());
    }
}
