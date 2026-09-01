//! ESP Simulator — deterministic wire-verification instrument for the
//! CURRENT ESP32 protocol.
//!
//! This is NOT a second firmware and NOT a kinematics simulation. It is a
//! fake ESP32 that speaks EXACTLY the current wire protocol (see
//! `docs/architecture/protocol/esp32-execution.md`, `backends/esp32/protocol.rs` for the
//! host side, `firmware/esp32/src/protocol.cpp` for the reference firmware)
//! and produces deterministic states, responses, and execution samples so
//! the host backend (`Esp32Backend` over `TcpTransport`) can be exercised
//! end-to-end without hardware.
//!
//! It is a plain TCP listener speaking line-by-line text (`\n`), one
//! simulated device per connection. The state machine mirrors the firmware:
//! `IDLE → RECEIVING → READY → EXECUTING → COMPLETED | ERROR`, with the same
//! out-of-order rejections (`NOT_READY`, `NOT_RECEIVING`, `NOT_IDLE`,
//! `NOT_ACTIVE`, `NOT_AVAILABLE`, `UNKNOWN_COMMAND`, `MALFORMED_*`).
//!
//! Determinism: every connection starts from the same state and execution
//! progress is driven by the number of `STATUS` polls received (never by the
//! wall clock), so the same input sequence always produces the same output.
//!
//! # Scenarios
//!
//! | Scenario | Behavior after `EXECUTE` |
//! |----------|---------------------------|
//! | `happy`  | `OK`, then `STATUS RUNNING 0.0→1.0` across polls, then `STATUS COMPLETED <N>`, and `SAMPLES <N>` returns N recorded samples. |
//! | `error`  | `OK`, two `STATUS RUNNING`, then `STATUS ERROR MOTOR_STALL`. |
//! | `silence`| `OK`, then the device stops responding to anything (the host must detect the receive timeout). |
//!
//! `happy` records `--samples` execution samples (default 10), generated
//! deterministically from the manifest metadata (`dof_count`, `duration_us`).
//!
//! # Usage — standalone CLI
//!
//! ```bash
//! # Terminal 1 — start the simulator (default scenario: happy)
//! cargo run --example esp-simulator
//! cargo run --example esp-simulator -- --scenario error
//! cargo run --example esp-simulator -- --scenario silence --port 7001
//!
//! # Terminal 2 — drive it manually (host side)
//! nc 127.0.0.1 7000
//! HELLO 1
//! MANIFEST 2 3 2000000
//! SEGMENT 0 movej 0 3
//! SAMPLE 0.0 0.0 0
//! SAMPLE 0.5 0.3 1000000
//! SAMPLE 1.0 0.5 1000000
//! END_UPLOAD
//! EXECUTE
//! STATUS
//! STATUS
//! STATUS
//! SAMPLES 10
//! ```
//!
//! Or from the Rust host, connect the real backend:
//!
//! ```rust,ignore
//! let mut transport = TcpTransport::new("127.0.0.1:7000");
//! transport.connect().await?; // open the socket BEFORE the backend handshake
//! let backend = Esp32Backend::new(Box::new(transport));
//! ```
//!
//! # In-process use (tests)
//!
//! The whole core is `pub` so an integration test can run the simulator
//! inside the test process — no subprocess, no fixed port:
//!
//! ```rust,ignore
//! let mut server = esp_simulator::start_listener("127.0.0.1:0", SimConfig::default())?;
//! let addr = server.addr(); // ephemeral port, already bound
//! // ...drive with Esp32Backend over TcpTransport...
//! server.stop(); // stops the accept loop and joins its thread
//! ```
//!
//! # CLI
//!
//! ```text
//! esp-simulator [--scenario happy|error|silence] [--port <u16>] [--samples <usize>]
//! ```
//!
//! Every flag also has an environment variable fallback: `ESP_SIM_SCENARIO`,
//! `ESP_SIM_PORT`, `ESP_SIM_SAMPLES`. CLI args win over the environment; the
//! scenario defaults to `happy`, the port to `7000`, samples per run to `10`.

// The file is compiled in TWO contexts: as a standalone example binary (where
// the test-only `start_listener`/`SimServer` surface is unused) and included
// verbatim as a module in `tests/esp32_simulator_e2e.rs` (where the CLI entry
// points `main`/`parse_args` are unused). No single context reaches the whole
// surface, so dead_code is allowed at module level.
#![allow(dead_code)]

use std::io::{BufRead, BufReader, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

/// Protocol version this device announces in the HELLO handshake.
pub const PROTOCOL_VERSION: u32 = 1;

/// Number of `STATUS RUNNING` progress steps before a `happy` run completes
/// (progress goes 0.0 → 1.0 across `samples_per_run + 1` polls, then one
/// `STATUS COMPLETED <N>`).
pub const RUNNING_STEPS_DIVISOR: usize = 10;

/// Deterministic behavior scenario, selected via CLI or environment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scenario {
    /// Normal cycle: RUNNING ramp → COMPLETED <N> → collectable samples.
    Happy,
    /// RUNNING twice, then `STATUS ERROR MOTOR_STALL` (→ EStop on the host).
    Error,
    /// OK to EXECUTE, then the device never responds again.
    Silence,
}

impl Scenario {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "happy" => Some(Scenario::Happy),
            "error" => Some(Scenario::Error),
            "silence" => Some(Scenario::Silence),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Scenario::Happy => "happy",
            Scenario::Error => "error",
            Scenario::Silence => "silence",
        }
    }
}

/// Deterministic device configuration for a simulated connection.
#[derive(Debug, Clone, Copy)]
pub struct SimConfig {
    pub scenario: Scenario,
    /// Number of execution samples a `happy` run records (collected via
    /// `SAMPLES <n>` after `COMPLETED`).
    pub samples_per_run: usize,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            scenario: Scenario::Happy,
            samples_per_run: 10,
        }
    }
}

/// Firmware state machine — mirrors `protocol.cpp`'s `state_`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Idle,
    Receiving,
    Ready,
    Executing,
    Completed,
    Error,
}

/// Manifest metadata parsed from the `MANIFEST` command.
#[derive(Debug, Clone, Copy)]
pub struct ManifestMeta {
    dof_count: usize,
    total_samples: usize,
    duration_us: u64,
    /// Firmware-side repeat count (v3): the executor loops the trajectory
    /// `repeat_count` times back-to-back (default 1).
    repeat_count: usize,
}

impl Default for ManifestMeta {
    fn default() -> Self {
        Self {
            dof_count: 0,
            total_samples: 0,
            duration_us: 0,
            repeat_count: 1,
        }
    }
}

/// A `SEGMENT` line, stored so `END_UPLOAD` validation can mirror the
/// firmware validator's `SEGMENT_ORDER` / `SEGMENT_COVERAGE` checks.
#[derive(Debug, Clone)]
pub struct Segment {
    index: usize,
    sample_start: usize,
    sample_count: usize,
}

/// An uploaded waypoint (`SAMPLE <j0..jN> <dt_us>` in the upload direction).
#[derive(Debug, Clone)]
pub struct Waypoint {
    joints: Vec<f64>,
    dt_us: u32,
}

/// A recorded execution sample, collected after `COMPLETED` via
/// `SAMPLES <count>` — timestamp-first, like the firmware.
#[derive(Debug, Clone)]
pub struct RecordedSample {
    timestamp_us: u64,
    joints: Vec<f64>,
}

/// Per-connection simulated device state.
pub struct SimState {
    scenario: Scenario,
    samples_per_run: usize,
    state: State,
    error_reason: String,
    meta: ManifestMeta,
    segments: Vec<Segment>,
    uploaded: Vec<Waypoint>,
    recorded: Vec<RecordedSample>,
    /// `STATUS` polls since the last `EXECUTE` — drives deterministic progress.
    exec_step: usize,
    /// `silence` scenario: after `EXECUTE` the device stops responding.
    silent: bool,
    /// Chunked-ACK batch size (v2, C) — ACK one `OK` per N samples.
    chunk_size: usize,
    /// Samples received since the last chunk ACK.
    samples_since_ack: usize,
}

impl SimState {
    pub fn new(scenario: Scenario, samples_per_run: usize) -> Self {
        Self {
            scenario,
            samples_per_run: samples_per_run.max(1),
            state: State::Idle,
            error_reason: String::new(),
            meta: ManifestMeta::default(),
            segments: Vec::new(),
            uploaded: Vec::new(),
            recorded: Vec::new(),
            exec_step: 0,
            silent: false,
            chunk_size: 1,
            samples_since_ack: 0,
        }
    }

    // ── Command handlers (mirror protocol.cpp) ──────────────────────────

    pub fn handle_hello(&mut self, parts: &[&str]) -> String {
        match parts.get(1).and_then(|s| s.parse::<u32>().ok()) {
            // v2 (C): validate the version — a stale v1 host fails the
            // handshake BEFORE upload traffic (mirrors protocol.cpp).
            Some(version) if version == 2 => format!("HELLO {version} OK\n"),
            Some(_) => self.set_error("VERSION_MISMATCH"),
            None => self.set_error("MALFORMED_HELLO"),
        }
    }

    pub fn handle_manifest(&mut self, parts: &[&str]) -> String {
        if self.state != State::Idle && self.state != State::Completed {
            return self.set_error("NOT_IDLE");
        }
        let dof = parts.get(1).and_then(|s| s.parse::<usize>().ok());
        let total = parts.get(2).and_then(|s| s.parse::<usize>().ok());
        let dur = parts.get(3).and_then(|s| s.parse::<u64>().ok());
        // v2 (C): optional 4th field = chunked-ACK batch size (default 1).
        let chunk = parts
            .get(4)
            .and_then(|s| s.parse::<usize>().ok())
            .unwrap_or(1);
        // v3 (firmware-side repeat): optional 5th field = pass count (default 1).
        let repeat = parts
            .get(5)
            .and_then(|s| s.parse::<usize>().ok())
            .unwrap_or(1);
        let (Some(dof), Some(total), Some(dur)) = (dof, total, dur) else {
            return self.set_error("MALFORMED_MANIFEST");
        };
        if dof == 0 || total == 0 || dur == 0 || chunk == 0 || repeat == 0 {
            return self.set_error("INVALID_MANIFEST");
        }
        self.meta = ManifestMeta {
            dof_count: dof,
            total_samples: total,
            duration_us: dur,
            repeat_count: repeat,
        };
        self.chunk_size = chunk;
        self.samples_since_ack = 0;
        self.segments.clear();
        self.uploaded.clear();
        self.state = State::Receiving;
        "OK\n".to_string()
    }

    pub fn handle_segment(&mut self, parts: &[&str]) -> String {
        if self.state != State::Receiving {
            return self.set_error("NOT_RECEIVING");
        }
        let idx = parts.get(1).and_then(|s| s.parse::<usize>().ok());
        let start = parts.get(3).and_then(|s| s.parse::<usize>().ok());
        let count = parts.get(4).and_then(|s| s.parse::<usize>().ok());
        let (Some(idx), Some(start), Some(count)) = (idx, start, count) else {
            return self.set_error("MALFORMED_SEGMENT");
        };
        // Instruction (parts[2]) is accepted verbatim: the firmware stores an
        // UNKNOWN instruction type rather than rejecting the line.
        self.segments.push(Segment {
            index: idx,
            sample_start: start,
            sample_count: count,
        });
        "OK\n".to_string()
    }

    pub fn handle_sample(&mut self, parts: &[&str]) -> String {
        if self.state != State::Receiving {
            return self.set_error("NOT_RECEIVING");
        }
        if parts.len() < 3 {
            return self.set_error("MALFORMED_SAMPLE");
        }
        // Expected tokens: "SAMPLE" + dof_count joints + dt_us.
        let expected = self.meta.dof_count + 2;
        if parts.len() != expected {
            return self.set_error("DOF_MISMATCH");
        }
        let joints: Result<Vec<f64>, _> = parts[1..parts.len() - 1]
            .iter()
            .map(|s| s.parse::<f64>())
            .collect();
        let joints = match joints {
            Ok(j) => j,
            Err(_) => return self.set_error("MALFORMED_SAMPLE"),
        };
        let dt_us = match parts[parts.len() - 1].parse::<u32>() {
            Ok(d) => d,
            Err(_) => return self.set_error("MALFORMED_SAMPLE"),
        };
        self.uploaded.push(Waypoint { joints, dt_us });
        // v2 (C): chunked ACK — one OK per chunk (mirror protocol.cpp).
        self.samples_since_ack += 1;
        if self.samples_since_ack >= self.chunk_size {
            self.samples_since_ack = 0;
            "OK\n".to_string()
        } else {
            String::new()
        }
    }

    pub fn handle_end_upload(&mut self) -> String {
        if self.state != State::Receiving {
            return self.set_error("NOT_RECEIVING");
        }
        match self.validate_upload() {
            Ok(()) => {
                self.state = State::Ready;
                "READY\n".to_string()
            }
            Err(reason) => self.set_error(reason),
        }
    }

    pub fn handle_execute(&mut self) -> String {
        if self.state != State::Ready {
            return self.set_error("NOT_READY");
        }
        self.state = State::Executing;
        self.exec_step = 0;
        if self.scenario == Scenario::Silence {
            // ACK the EXECUTE, then go deaf so the host's next receive times out.
            self.silent = true;
        }
        "OK\n".to_string()
    }

    pub fn handle_stop(&mut self) -> String {
        if self.state == State::Idle || self.state == State::Receiving {
            return self.set_error("NOT_ACTIVE");
        }
        self.reset();
        "OK\n".to_string()
    }

    pub fn handle_status(&mut self) -> String {
        match self.state {
            State::Idle => "STATUS IDLE\n".to_string(),
            State::Receiving => "STATUS RECEIVING\n".to_string(),
            State::Ready => "STATUS READY\n".to_string(),
            State::Executing => self.status_while_executing(),
            State::Completed => format!("STATUS COMPLETED {}\n", self.recorded.len()),
            State::Error => format!("STATUS ERROR {}\n", self.error_reason),
        }
    }

    pub fn handle_samples(&mut self, parts: &[&str]) -> String {
        // Valid from IDLE / READY / COMPLETED — same as protocol.cpp.
        if !matches!(self.state, State::Idle | State::Ready | State::Completed) {
            return self.set_error("NOT_AVAILABLE");
        }
        let count = parts.get(1).and_then(|s| s.parse::<usize>().ok());
        let Some(count) = count.filter(|&c| c > 0) else {
            return self.set_error("MALFORMED");
        };
        let to_send = count.min(self.recorded.len());
        let mut out = "OK\n".to_string();
        for sample in &self.recorded[..to_send] {
            let joints = sample
                .joints
                .iter()
                .map(|j| format!("{:.6}", j))
                .collect::<Vec<_>>()
                .join(" ");
            out.push_str(&format!("SAMPLE {} {joints}\n", sample.timestamp_us));
        }
        // Clear the buffer after collection — frees RAM on the real device.
        self.recorded.clear();
        out
    }

    // ── Scenario-driven status while EXECUTING ──────────────────────────

    fn status_while_executing(&mut self) -> String {
        match self.scenario {
            Scenario::Happy => {
                // Overall progress across ALL passes (v3 firmware-side repeat):
                // `repeat_count × progress_steps` polls from 0.0 → 1.0, then
                // COMPLETED. The joints ramp per-pass (within-pass progress).
                let steps = self.progress_steps();
                let total_steps = steps * self.meta.repeat_count.max(1);
                if self.exec_step > total_steps {
                    self.recorded = self.generate_recorded_samples();
                    self.state = State::Completed;
                    return format!("STATUS COMPLETED {}\n", self.recorded.len());
                }
                let within = (self.exec_step % steps.max(1)) as f64 / steps.max(1) as f64;
                let progress = self.exec_step as f64 / total_steps as f64;
                self.exec_step += 1;
                self.format_running_with(progress, within)
            }
            Scenario::Error => match self.exec_step {
                0 => {
                    self.exec_step = 1;
                    self.format_running(0.0)
                }
                1 => {
                    self.exec_step = 2;
                    self.format_running(0.5)
                }
                _ => self.set_error("MOTOR_STALL"),
            },
            Scenario::Silence => {
                // Unreachable: `handle_command` short-circuits once silent.
                String::new()
            }
        }
    }

    fn format_running(&self, progress: f64) -> String {
        self.format_running_with(progress, progress)
    }

    /// `STATUS RUNNING <overall> <joints>` — joints driven by the per-pass
    /// progress (v3 repeat: the joints repeat each pass while the overall
    /// progress spans all passes).
    fn format_running_with(&self, overall: f64, within_pass: f64) -> String {
        let joints = self.joints_at(within_pass);
        format!("STATUS RUNNING {:.4} {joints}\n", overall)
    }

    /// Progress steps for the `happy` ramp: `RUNNING_STEPS_DIVISOR` steps
    /// regardless of `samples_per_run` keeps the run short and deterministic.
    fn progress_steps(&self) -> usize {
        RUNNING_STEPS_DIVISOR
    }

    /// Commanded joints at a given progress fraction: a uniform ramp across
    /// the manifest's DOF. `joint = progress * (j+1) / dof` stays in [0,1].
    fn joints(&self, progress: f64) -> Vec<f64> {
        let dof = self.meta.dof_count;
        (0..dof)
            .map(|j| progress * (j as f64 + 1.0) / dof.max(1) as f64)
            .collect()
    }

    fn joints_at(&self, progress: f64) -> String {
        self.joints(progress)
            .iter()
            .map(|j| format!("{:.6}", j))
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// Build the `--samples` recorded execution samples deterministically
    /// from the manifest metadata (no kinematics, no timing simulation).
    /// Mirrors the REAL firmware's BOUNDED trace contract (bounded-reusable
    /// buffer, trace_scope = last_iteration): all passes RUN, but only the
    /// LAST pass is retained, with sample timestamps offset by the accumulated
    /// pass durations so the trace is monotonic. sample_count is never
    /// repeat_count × waypoints — it is exactly one pass.
    fn generate_recorded_samples(&self) -> Vec<RecordedSample> {
        let n = self.samples_per_run;
        let step_us = self.meta.duration_us / n as u64;
        let last_pass = self.meta.repeat_count.max(1).saturating_sub(1) as u64;
        let base = last_pass * self.meta.duration_us;
        (0..n)
            .map(|i| {
                let progress = if n > 1 { i as f64 / (n - 1) as f64 } else { 0.0 };
                RecordedSample {
                    timestamp_us: base + i as u64 * step_us,
                    joints: self.joints(progress),
                }
            })
            .collect()
    }

    // ── END_UPLOAD validation (mirrors validator.cpp check order) ───────

    fn validate_upload(&self) -> Result<(), &'static str> {
        if self.uploaded.is_empty() {
            return Err("EMPTY_MANIFEST");
        }
        if self
            .uploaded
            .iter()
            .any(|w| w.joints.len() != self.meta.dof_count)
        {
            return Err("DOF_MISMATCH");
        }
        if self.uploaded.len() != self.meta.total_samples {
            return Err("WAYPOINT_COUNT");
        }
        if !self.segments_ordered() {
            return Err("SEGMENT_ORDER");
        }
        if !self.segments_cover_all_samples() {
            return Err("SEGMENT_COVERAGE");
        }
        if !self.timing_valid() {
            return Err("TIMING_INVALID");
        }
        Ok(())
    }

    fn segments_ordered(&self) -> bool {
        self.segments
            .windows(2)
            .all(|w| w[1].index > w[0].index)
    }

    fn segments_cover_all_samples(&self) -> bool {
        let mut next = 0usize;
        for seg in &self.segments {
            if seg.sample_start > next {
                return false; // gap
            }
            let end = seg.sample_start + seg.sample_count;
            if end > next {
                next = end;
            }
        }
        next == self.meta.total_samples
    }

    /// Total accumulated dt must match the declared duration within 1%
    /// (min 1000 µs) — mirrors `check_timing_integrity`.
    fn timing_valid(&self) -> bool {
        let accumulated: u64 = self.uploaded.iter().map(|w| w.dt_us as u64).sum();
        let declared = self.meta.duration_us;
        let diff = accumulated.abs_diff(declared);
        let tolerance = (declared / 100).max(1000);
        diff <= tolerance
    }

    // ── Internal helpers ─────────────────────────────────────────────────

    fn set_error(&mut self, reason: &str) -> String {
        self.state = State::Error;
        self.error_reason = reason.to_string();
        format!("ERROR {reason}\n")
    }

    fn reset(&mut self) {
        self.state = State::Idle;
        self.error_reason.clear();
        self.meta = ManifestMeta::default();
        self.segments.clear();
        self.uploaded.clear();
        self.recorded.clear();
        self.exec_step = 0;
        self.silent = false;
        self.chunk_size = 1;
        self.samples_since_ack = 0;
    }
}

/// Dispatch a single input line. Returns the response text (empty when the
/// device is intentionally silent).
pub fn handle_command(state: &mut SimState, line: &str) -> String {
    if state.silent {
        return String::new();
    }
    let trimmed = line.trim();
    if trimmed.is_empty() {
        return String::new();
    }
    let parts: Vec<&str> = trimmed.split_whitespace().collect();
    match parts[0] {
        "HELLO" => state.handle_hello(&parts),
        "MANIFEST" => state.handle_manifest(&parts),
        "SEGMENT" => state.handle_segment(&parts),
        "SAMPLE" => state.handle_sample(&parts),
        "END_UPLOAD" => state.handle_end_upload(),
        "EXECUTE" => state.handle_execute(),
        "STOP" => state.handle_stop(),
        "STATUS" => state.handle_status(),
        "SAMPLES" => state.handle_samples(&parts),
        _ => state.set_error("UNKNOWN_COMMAND"),
    }
}

/// Serve a single accepted connection until the client closes it (or the
/// device goes silent and the client walks away).
pub fn serve_connection(mut stream: TcpStream, config: SimConfig) {
    let addr = stream.peer_addr().unwrap();
    eprintln!("[esp-sim] connected: {addr}");

    let mut state = SimState::new(config.scenario, config.samples_per_run);
    let reader = BufReader::new(stream.try_clone().unwrap());

    for line in reader.lines() {
        match line {
            Ok(line) => {
                let trimmed = line.trim();
                if trimmed.is_empty() {
                    continue;
                }
                eprintln!("[esp-sim] >> {trimmed}");
                let response = handle_command(&mut state, trimmed);
                if response.is_empty() {
                    eprintln!("[esp-sim] (silent) not responding");
                    continue;
                }
                eprintln!("[esp-sim] << {}", response.trim());
                if let Err(e) = stream.write_all(response.as_bytes()) {
                    eprintln!("[esp-sim] write error: {e}");
                    break;
                }
                if let Err(e) = stream.flush() {
                    eprintln!("[esp-sim] flush error: {e}");
                    break;
                }
            }
            Err(e) => {
                eprintln!("[esp-sim] read error: {e}");
                break;
            }
        }
    }
    eprintln!("[esp-sim] disconnected: {addr}");
}

/// Serve incoming connections forever (blocking). One thread per connection,
/// so a slow or silent client never blocks other clients. Returns when the
/// listener fails.
pub fn serve_forever(listener: TcpListener, config: SimConfig) -> std::io::Result<()> {
    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                thread::spawn(move || serve_connection(stream, config));
            }
            Err(e) => {
                eprintln!("[esp-sim] accept error: {e}");
            }
        }
    }
    Ok(())
}

/// Handle to an in-process simulator: the bound address plus a handle to the
/// background accept thread (see [`start_listener`]).
pub struct SimServer {
    addr: SocketAddr,
    stop: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
}

impl SimServer {
    /// The address the simulator is actually listening on (the ephemeral
    /// port is resolved at bind time).
    pub fn addr(&self) -> SocketAddr {
        self.addr
    }

    /// Stop the accept loop and join the background thread. Subsequent
    /// connections are refused; already-open connections are left to the
    /// client to close.
    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for SimServer {
    /// Best-effort cleanup on early return / panic: flag the accept loop to
    /// exit. The thread ends within a few milliseconds; if the process is
    /// already exiting, the OS reclaims it.
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// Bind a listener (pass `"127.0.0.1:0"` for an ephemeral port) and run the
/// accept loop on a background thread. Returns a [`SimServer`] carrying the
/// actually-bound address so the caller can point a `TcpTransport` at it.
pub fn start_listener<A: ToSocketAddrs>(
    addr: A,
    config: SimConfig,
) -> std::io::Result<SimServer> {
    let listener = TcpListener::bind(addr)?;
    let addr = listener.local_addr()?;
    let stop = Arc::new(AtomicBool::new(false));
    let stop_flag = Arc::clone(&stop);
    let handle = thread::spawn(move || accept_loop(listener, config, stop_flag));
    Ok(SimServer {
        addr,
        stop,
        handle: Some(handle),
    })
}

/// Accept loop for the stoppable [`start_listener`] server: non-blocking
/// accepts so the stop flag is observed between connections.
fn accept_loop(listener: TcpListener, config: SimConfig, stop: Arc<AtomicBool>) {
    let _ = listener.set_nonblocking(true);
    while !stop.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, _)) => {
                thread::spawn(move || serve_connection(stream, config));
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(5));
            }
            Err(e) => {
                eprintln!("[esp-sim] accept error: {e}");
                break;
            }
        }
    }
    eprintln!("[esp-sim] listener stopped");
}

/// Parse `--flag value` pairs, falling back to `ESP_SIM_*` environment
/// variables. CLI args win over the environment.
fn parse_args() -> (Scenario, u16, usize) {
    let scenario = std::env::var("ESP_SIM_SCENARIO")
        .ok()
        .and_then(|s| Scenario::parse(&s))
        .unwrap_or(Scenario::Happy);
    let port = std::env::var("ESP_SIM_PORT")
        .ok()
        .and_then(|s| s.parse::<u16>().ok())
        .unwrap_or(7000);
    let samples = std::env::var("ESP_SIM_SAMPLES")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(10);

    let mut scenario = scenario;
    let mut port = port;
    let mut samples = samples;

    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < args.len() {
        let mut take_value = |key: &str| -> Option<&str> {
            if args[i] == key {
                let v = args.get(i + 1).map(|s| s.as_str());
                if v.is_some() {
                    i += 1;
                }
                v
            } else {
                None
            }
        };
        if let Some(v) = take_value("--scenario") {
            match Scenario::parse(v) {
                Some(s) => scenario = s,
                None => {
                    eprintln!("[esp-sim] invalid scenario '{v}' (expected happy|error|silence)");
                    std::process::exit(2);
                }
            }
        } else if let Some(v) = take_value("--port") {
            match v.parse::<u16>() {
                Ok(p) => port = p,
                Err(_) => {
                    eprintln!("[esp-sim] invalid port '{v}'");
                    std::process::exit(2);
                }
            }
        } else if let Some(v) = take_value("--samples") {
            match v.parse::<usize>() {
                Ok(s) if s > 0 => samples = s,
                _ => {
                    eprintln!("[esp-sim] invalid samples '{v}' (expected positive integer)");
                    std::process::exit(2);
                }
            }
        } else if args[i] == "--help" || args[i] == "-h" {
            print_usage();
            std::process::exit(0);
        } else {
            eprintln!("[esp-sim] unknown argument: {}", args[i]);
            print_usage();
            std::process::exit(2);
        }
        i += 1;
    }

    (scenario, port, samples)
}

fn print_usage() {
    println!(
        "ESP Simulator — deterministic ESP32 wire-verification instrument\n\n\
         Usage:\n  \
         esp-simulator [--scenario happy|error|silence] [--port <u16>] [--samples <usize>]\n\n\
         Environment fallbacks (CLI wins):\n  \
         ESP_SIM_SCENARIO, ESP_SIM_PORT, ESP_SIM_SAMPLES\n\n\
         Connect the host with TcpTransport (e.g. \"127.0.0.1:7000\")."
    );
}

fn main() {
    let (scenario, port, samples) = parse_args();
    let config = SimConfig {
        scenario,
        samples_per_run: samples,
    };
    let listener = TcpListener::bind(("127.0.0.1", port)).expect("bind failed");
    eprintln!(
        "[esp-sim] protocol v{PROTOCOL_VERSION} listening on 127.0.0.1:{port} scenario={} samples_per_run={}",
        scenario.as_str(),
        samples
    );

    serve_forever(listener, config).expect("listener failed");
}
