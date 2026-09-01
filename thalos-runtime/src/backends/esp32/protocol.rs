//! ESP32 protocol codec — text wire format for ESP32 communication.
//!
//! Defines the text protocol shared between the Rust host and C++ firmware:
//!
//! ```text
//! HOST → ESP                ESP → HOST
//! ─────────────────────────────────────
//! HELLO <ver>               HELLO <ver> OK
//! MANIFEST <dof> <N> <dur>  OK
//! SEGMENT <i> <instr> ...
//! SAMPLE <j0> <j1> .. <dt>  OK
//! END_UPLOAD                READY | ERROR <reason>
//! EXECUTE                   OK | ERROR <reason>
//! STOP                      OK
//! STATUS                    STATUS RUNNING | COMPLETED | ERROR <reason>
//! SAMPLES <count>           OK
//! SAMPLE <ts> <j0> <j1> ..  (×count, implicit)
//! ```

use crate::backends::transport::{Transport, TransportError};
use crate::execution_boundary::manifest::{ExecutionManifest, ManifestInstruction};
use crate::execution_boundary::sample::ExecutionSample;

/// Errors from the ESP32 protocol layer.
#[derive(Debug, thiserror::Error)]
pub enum ProtocolError {
    #[error("transport error: {0}")]
    Transport(#[from] TransportError),

    #[error("protocol error: {0}")]
    Protocol(String),

    #[error("unexpected response: {0}")]
    UnexpectedResponse(String),

    #[error("malformed response: {0}")]
    MalformedResponse(String),

    #[error("version mismatch: expected {expected}, got {actual}")]
    VersionMismatch { expected: u32, actual: u32 },

    #[error("ESP error: {0}")]
    EspError(String),
}

/// Firmware-side execution state as tracked by the host protocol codec.
///
/// The wire token stays `RUNNING` (firmware never emits `EXECUTING`); the
/// host maps `STATUS RUNNING <progress> <j0..jN>` to `Executing` internally.
#[derive(Debug, Clone, PartialEq)]
pub enum FirmwareState {
    /// No manifest loaded, idle.
    Idle,
    /// Receiving manifest data (MANIFEST / SEGMENT / SAMPLE commands).
    Receiving,
    /// Manifest uploaded and validated, ready to execute.
    Ready,
    /// Execution in progress — carries the progress fraction (0..1) and the
    /// commanded joint positions reported by `STATUS RUNNING`.
    Executing { progress: f64, joints: Vec<f64> },
    /// Execution finished — carries how many recorded samples the host can
    /// collect via `SAMPLES <count>`.
    Completed { sample_count: u32 },
    /// Firmware error state with a human-readable reason.
    Error(String),
}

/// Internal parsed representation of an ESP32 response line.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum ParsedResponse {
    Ok,
    Ready,
    HandshakeOk(u32),
    Error(String),
    StatusIdle,
    StatusReceiving,
    StatusReady,
    /// `STATUS RUNNING <progress> <j0..jN>` — wire token RUNNING.
    StatusRunning { progress: f64, joints: Vec<f64> },
    /// `STATUS COMPLETED <count>`.
    StatusCompleted { sample_count: u32 },
    Sample(ExecutionSample),
}

/// Maximum number of SAMPLE response lines the host will read after
/// `SAMPLES <count>` (RISK-3): bounds allocation + read loop when the
/// firmware reports a bogus count.
const MAX_SAMPLES: usize = 100_000;

/// Cap a firmware-supplied sample count before allocating the sample
/// buffer (RISK-3) — a malicious/buggy count must not drive an unbounded
/// `Vec::with_capacity` + read loop.
fn cap_sample_count(count: usize) -> usize {
    count.min(MAX_SAMPLES)
}

/// ESP32 protocol codec.
///
/// Wraps a [`Transport`] and provides protocol-level operations:
/// handshake, manifest upload, execute, status query, and sample
/// collection. Owns all text wire-format concerns.
pub struct Esp32Protocol {
    transport: Box<dyn Transport>,
    firmware_state: FirmwareState,
    firmware_version: u32,
    expected_version: u32,
}

impl Esp32Protocol {
    /// Create a new protocol codec over the given transport.
    ///
    /// `expected_version` is the protocol version the host expects the
    /// firmware to announce during the HELLO handshake.
    pub fn new(transport: Box<dyn Transport>, expected_version: u32) -> Self {
        Self {
            transport,
            firmware_state: FirmwareState::Idle,
            firmware_version: 0,
            expected_version,
        }
    }

    /// Format a protocol command line and append a newline.
    fn format_line(args: &[&str]) -> Vec<u8> {
        let mut line = args.join(" ");
        line.push('\n');
        line.into_bytes()
    }

    /// Format a SAMPLE line from joint positions and delta time.
    ///
    /// Output: `SAMPLE <j0> <j1> ... <dt_us>\n`
    fn format_sample_line(joints: &[f64], dt_us: u32) -> Vec<u8> {
        let mut parts = vec!["SAMPLE".to_string()];
        for j in joints {
            // Format with enough precision for round-trip parsing
            parts.push(format!("{:.6}", j));
        }
        parts.push(dt_us.to_string());
        let line = parts.join(" ") + "\n";
        line.into_bytes()
    }

    /// Parse a single response line from the ESP32 firmware.
    ///
    /// Returns a [`ParsedResponse`] representing the firmware's reply.
    pub(crate) fn parse_response(line: &str) -> Result<ParsedResponse, ProtocolError> {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            return Err(ProtocolError::MalformedResponse("empty line".into()));
        }
        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        if parts.is_empty() {
            return Err(ProtocolError::MalformedResponse("empty line".into()));
        }

        match parts[0] {
            "HELLO" => {
                if parts.len() >= 3 && parts[2] == "OK" {
                    let version: u32 = parts[1]
                        .parse()
                        .map_err(|_| ProtocolError::MalformedResponse(line.to_string()))?;
                    Ok(ParsedResponse::HandshakeOk(version))
                } else {
                    Err(ProtocolError::MalformedResponse(line.to_string()))
                }
            }
            "OK" => Ok(ParsedResponse::Ok),
            "READY" => Ok(ParsedResponse::Ready),
            "ERROR" => {
                let reason = if parts.len() > 1 {
                    parts[1..].join(" ")
                } else {
                    "unknown".into()
                };
                Ok(ParsedResponse::Error(reason))
            }
            "STATUS" => {
                if parts.len() < 2 {
                    return Err(ProtocolError::MalformedResponse(line.to_string()));
                }
                match parts[1] {
                    "IDLE" => Ok(ParsedResponse::StatusIdle),
                    "RECEIVING" => Ok(ParsedResponse::StatusReceiving),
                    "READY" => Ok(ParsedResponse::StatusReady),
                    "RUNNING" => {
                        // STATUS RUNNING [<progress> <j0> <j1> ... <jN>]
                        // RES-01: legacy HELLO-v1 firmware emits a BARE
                        // RUNNING — progress defaults to 0.0, joints to
                        // empty; strict validation applies only to fields
                        // actually present.
                        let progress: f64 = match parts.get(2) {
                            Some(raw) => raw.parse().map_err(|_| {
                                ProtocolError::MalformedResponse(line.to_string())
                            })?,
                            None => 0.0,
                        };
                        // RISK-2: nan/inf/1e999 parse successfully as f64 and
                        // would panic `Duration::from_secs_f64` after the
                        // ×plan_duration mapping — reject non-finite values.
                        if !progress.is_finite() {
                            return Err(ProtocolError::MalformedResponse(line.to_string()));
                        }
                        let joints = parts
                            .get(3..)
                            .unwrap_or(&[])
                            .iter()
                            .map(|s| {
                                s.parse::<f64>()
                                    .map_err(|_| ProtocolError::MalformedResponse(line.to_string()))
                            })
                            .collect::<Result<Vec<f64>, _>>()?;
                        Ok(ParsedResponse::StatusRunning { progress, joints })
                    }
                    "COMPLETED" => {
                        // STATUS COMPLETED [<count>] — RES-01: the count is
                        // optional for legacy firmware; defaults to 0.
                        let sample_count: u32 = match parts.get(2) {
                            Some(raw) => raw.parse().map_err(|_| {
                                ProtocolError::MalformedResponse(line.to_string())
                            })?,
                            None => 0,
                        };
                        Ok(ParsedResponse::StatusCompleted { sample_count })
                    }
                    "ERROR" => {
                        // STATUS ERROR <reason> — the ONLY token that maps to
                        // a firmware error (REL-03): unknown tokens must be
                        // MalformedResponse, never an implicit EStop.
                        let reason = if parts.len() > 2 {
                            parts[2..].join(" ")
                        } else {
                            "unknown".into()
                        };
                        Ok(ParsedResponse::Error(reason))
                    }
                    _other => Err(ProtocolError::MalformedResponse(line.to_string())),
                }
            }
            "SAMPLE" => {
                // SAMPLE <ts_us> <j0> <j1> ... <jN>  (collect direction, ts-first)
                if parts.len() < 3 {
                    return Err(ProtocolError::MalformedResponse(line.to_string()));
                }
                let timestamp: u64 = parts[1]
                    .parse()
                    .map_err(|_| ProtocolError::MalformedResponse(line.to_string()))?;
                let joints = parts[2..]
                    .iter()
                    .map(|s| {
                        s.parse::<f64>()
                            .map_err(|_| ProtocolError::MalformedResponse(line.to_string()))
                    })
                    .collect::<Result<Vec<f64>, _>>()?;
                Ok(ParsedResponse::Sample(ExecutionSample {
                    timestamp_us: timestamp,
                    joints,
                }))
            }
            _ => Err(ProtocolError::UnexpectedResponse(line.to_string())),
        }
    }

    /// Encode an [`ExecutionManifest`] into a list of text command lines
    /// ready to send over the transport.
    ///
    /// The returned vector contains the following lines, in order:
    /// 1. `MANIFEST <dof> <N> <dur_us> <chunk>\n` — the 4th field (v2, C) is
    ///    the chunked-ACK batch size the firmware ACKs once per batch.
    /// 2. `SEGMENT <idx> <instr> <start> <count>\n` (one per segment)
    /// 3. `SAMPLE <j0> ... <dt_us>\n` (one per sample)
    /// 4. `END_UPLOAD\n`
    pub fn encode_manifest(manifest: &ExecutionManifest, chunk: usize) -> Vec<Vec<u8>> {
        let mut lines = Vec::new();

        // MANIFEST <dof> <N> <dur_us> <chunk> <repeat> (v3: repeat = firmware
        // side pass count, default 1).
        lines.push(Self::format_line(&[
            "MANIFEST",
            &manifest.metadata.dof_count.to_string(),
            &manifest.metadata.total_samples.to_string(),
            &manifest.metadata.duration_us.to_string(),
            &chunk.to_string(),
            &manifest.metadata.repeat_count.to_string(),
        ]));

        // SEGMENT <idx> <instruction> <start> <count>
        for seg in &manifest.segments {
            let instr = match seg.instruction {
                ManifestInstruction::MoveJ => "movej",
                ManifestInstruction::MoveL => "movel",
            };
            lines.push(Self::format_line(&[
                "SEGMENT",
                &seg.index.to_string(),
                instr,
                &seg.sample_start.to_string(),
                &seg.sample_count.to_string(),
            ]));
        }

        // SAMPLE <j0> <j1> ... <dt_us>
        for sample in &manifest.samples {
            lines.push(Self::format_sample_line(&sample.joints, sample.dt_us));
        }

        // END_UPLOAD
        lines.push(Self::format_line(&["END_UPLOAD"]));

        lines
    }

    /// Perform the HELLO version handshake.
    ///
    /// Sends `HELLO <expected_version>` and expects `HELLO <ver> OK`.
    /// Returns an error if the version does not match.
    pub async fn handshake(&mut self) -> Result<(), ProtocolError> {
        // Retry-once: a stale serial buffer (boot ROM bytes / leftovers from a
        // previous session) can make the first read return garbage instead of
        // the handshake response. Consuming that line and re-sending HELLO
        // yields the real response — observed on real hardware (first connect
        // read `0.000000 0.000000`, the retry succeeded).
        let mut last_error: Option<ProtocolError> = None;
        for _attempt in 0..2 {
            let cmd = Self::format_line(&["HELLO", &self.expected_version.to_string()]);
            self.transport.send(&cmd).await?;
            let response = self.transport.receive().await?;
            let line = String::from_utf8(response)
                .map_err(|e| ProtocolError::MalformedResponse(format!("invalid UTF-8: {e}")))?;

            match Self::parse_response(&line) {
                Ok(ParsedResponse::HandshakeOk(version)) => {
                    if version != self.expected_version {
                        return Err(ProtocolError::VersionMismatch {
                            expected: self.expected_version,
                            actual: version,
                        });
                    }
                    self.firmware_version = version;
                    self.firmware_state = FirmwareState::Idle;
                    return Ok(());
                }
                Ok(other) => {
                    last_error =
                        Some(ProtocolError::UnexpectedResponse(format!("{other:?}")));
                }
                Err(e) => last_error = Some(e),
            }
        }
        Err(last_error.unwrap_or_else(|| {
            ProtocolError::UnexpectedResponse("no handshake response".into())
        }))
    }

    /// Upload a manifest to the ESP32 (protocol v2, C).
    ///
    /// Sends MANIFEST and SEGMENT lines expecting an `OK` each, then SAMPLE
    /// lines in BATCHES of `chunk` expecting ONE `OK` per batch (instead of
    /// one per line — the v2 latency fix), then `END_UPLOAD` expecting
    /// `READY` or `ERROR <reason>`.
    ///
    /// The chunk size is derived from the DOF so a full batch of encoded
    /// SAMPLE lines fits the firmware RX buffer (4096) with margin:
    /// `chunk × max_line ≤ 3072`. max_line = "SAMPLE " + DOF×(space+9) + dt_u32 + '\n'.
    pub async fn upload_manifest(
        &mut self,
        manifest: &ExecutionManifest,
    ) -> Result<(), ProtocolError> {
        self.firmware_state = FirmwareState::Receiving;

        // Protocol desync defense: drain stale lines left in the buffer from
        // a prior STATUS poll (or a cancelled read). Without this, the first
        // upload response read could return a leftover fragment ("unexpected
        // response: 0.000000 0.000000" real repro) instead of the firmware's
        // `OK`. Real transports drain; fakes are a no-op.
        self.transport.drain().await?;

        let dof = manifest.metadata.dof_count as usize;
        let max_line = 19 + 10 * dof; // upper bound per SAMPLE line (bytes)
        let chunk = (3072usize / max_line.max(1)).clamp(1, 64);

        let lines = Self::encode_manifest(manifest, chunk);

        // MANIFEST (first line) — expect OK. A firmware `ERROR <reason>`
        // (e.g. NOT_IDLE from a stale state) surfaces here as EspError so the
        // caller can STOP+retry (recovery in Esp32Backend::execute).
        Self::send_expect_ok(&mut *self.transport, &lines[0]).await?;

        // SEGMENT lines — each answered OK (few lines, keep the per-line ACK).
        let mut idx = 1usize;
        let segment_end = idx + manifest.segments.len();
        while idx < segment_end {
            Self::send_expect_ok(&mut *self.transport, &lines[idx]).await?;
            idx += 1;
        }

        // SAMPLE lines — ONE send per COMPLETE chunk (batched write: a USB-serial
        // `send()` costs ~5-7ms per call on real adapters — 1228 individual
        // sends were the REAL post-v2 bottleneck, not the ACK round-trips), and
        // ONE OK per chunk. A trailing partial chunk is sent batched too, with
        // no per-chunk ACK; END_UPLOAD confirms it.
        let sample_end = segment_end + manifest.samples.len();
        let mut sent_in_chunk = 0usize;
        let mut batch = Vec::new();
        while idx < sample_end {
            batch.extend_from_slice(&lines[idx]);
            idx += 1;
            sent_in_chunk += 1;
            if sent_in_chunk >= chunk {
                self.transport.send(&batch).await?;
                batch.clear();
                Self::expect_ok_response(&mut *self.transport).await?;
                sent_in_chunk = 0;
            }
        }
        if !batch.is_empty() {
            self.transport.send(&batch).await?;
        }

        // Send END_UPLOAD — expect READY or ERROR
        let end = &lines[lines.len() - 1];
        self.transport.send(end).await?;
        let response = self.transport.receive().await?;
        let line = String::from_utf8(response)
            .map_err(|e| ProtocolError::MalformedResponse(format!("invalid UTF-8: {e}")))?;
        match Self::parse_response(&line)? {
            ParsedResponse::Ready => {
                self.firmware_state = FirmwareState::Ready;
                Ok(())
            }
            ParsedResponse::Error(reason) => Err(ProtocolError::EspError(reason)),
            other => Err(ProtocolError::UnexpectedResponse(format!("{other:?}"))),
        }
    }

    /// Send one line and consume exactly one response, mapping `OK` → Ok,
    /// `ERROR <reason>` → EspError, anything else → UnexpectedResponse.
    async fn send_expect_ok(
        transport: &mut dyn Transport,
        cmd: &[u8],
    ) -> Result<(), ProtocolError> {
        transport.send(cmd).await?;
        Self::expect_ok_response(transport).await
    }

    /// Consume exactly one response line with the OK/ERROR/Unexpected mapping
    /// (shared by MANIFEST, SEGMENT and each SAMPLE chunk).
    async fn expect_ok_response(transport: &mut dyn Transport) -> Result<(), ProtocolError> {
        let response = transport.receive().await?;
        let line = String::from_utf8(response)
            .map_err(|e| ProtocolError::MalformedResponse(format!("invalid UTF-8: {e}")))?;
        match Self::parse_response(&line)? {
            ParsedResponse::Ok => Ok(()),
            ParsedResponse::Error(reason) => Err(ProtocolError::EspError(reason)),
            other => Err(ProtocolError::UnexpectedResponse(format!("{other:?}"))),
        }
    }

    /// Start execution on the ESP32.
    ///
    /// Sends `EXECUTE` and expects `OK` or `ERROR <reason>`.
    pub async fn start_execution(&mut self) -> Result<(), ProtocolError> {
        self.transport
            .send(&Self::format_line(&["EXECUTE"]))
            .await?;
        let response = self.transport.receive().await?;
        let line = String::from_utf8(response)
            .map_err(|e| ProtocolError::MalformedResponse(format!("invalid UTF-8: {e}")))?;
        match Self::parse_response(&line)? {
            ParsedResponse::Ok => {
                self.firmware_state = FirmwareState::Executing {
                    progress: 0.0,
                    joints: Vec::new(),
                };
                Ok(())
            }
            ParsedResponse::Error(reason) => Err(ProtocolError::EspError(reason)),
            other => Err(ProtocolError::UnexpectedResponse(format!("{other:?}"))),
        }
    }

    /// Query the current execution status.
    ///
    /// Sends `STATUS` and parses the response into a [`FirmwareState`]. The
    /// wire token `RUNNING` is mapped internally to `Executing { progress, joints }`.
    pub async fn query_status(&mut self) -> Result<FirmwareState, ProtocolError> {
        self.transport.send(&Self::format_line(&["STATUS"])).await?;
        let response = self.transport.receive().await?;
        let line = String::from_utf8(response)
            .map_err(|e| ProtocolError::MalformedResponse(format!("invalid UTF-8: {e}")))?;
        match Self::parse_response(&line)? {
            ParsedResponse::StatusIdle => Ok(FirmwareState::Idle),
            ParsedResponse::StatusReceiving => Ok(FirmwareState::Receiving),
            ParsedResponse::StatusReady => Ok(FirmwareState::Ready),
            ParsedResponse::StatusRunning { progress, joints } => {
                self.firmware_state = FirmwareState::Executing {
                    progress,
                    joints: joints.clone(),
                };
                Ok(FirmwareState::Executing { progress, joints })
            }
            ParsedResponse::StatusCompleted { sample_count } => {
                self.firmware_state = FirmwareState::Idle;
                Ok(FirmwareState::Completed { sample_count })
            }
            // `STATUS ERROR <reason>` — a real firmware error state, NOT a
            // transport/protocol failure. Return it so the backend maps it to
            // EStop/Failed (design: ERROR → EStop).
            ParsedResponse::Error(reason) => Ok(FirmwareState::Error(reason)),
            other => Err(ProtocolError::UnexpectedResponse(format!("{other:?}"))),
        }
    }

    /// Collect execution samples from the ESP32.
    ///
    /// Sends `SAMPLES <count>`, expects `OK`, then reads exactly
    /// `count` `SAMPLE` response lines.
    pub async fn collect_samples(
        &mut self,
        count: usize,
    ) -> Result<Vec<ExecutionSample>, ProtocolError> {
        // RISK-3: cap the firmware-supplied count BEFORE allocation so a
        // bogus value cannot drive an unbounded with_capacity + read loop.
        let count = cap_sample_count(count);
        self.transport
            .send(&Self::format_line(&["SAMPLES", &count.to_string()]))
            .await?;
        let response = self.transport.receive().await?;
        let line = String::from_utf8(response)
            .map_err(|e| ProtocolError::MalformedResponse(format!("invalid UTF-8: {e}")))?;
        match Self::parse_response(&line)? {
            ParsedResponse::Ok => {}
            other => return Err(ProtocolError::UnexpectedResponse(format!("{other:?}"))),
        }

        let mut samples = Vec::with_capacity(count);
        for _ in 0..count {
            let resp = self.transport.receive().await?;
            let line = String::from_utf8(resp)
                .map_err(|e| ProtocolError::MalformedResponse(format!("invalid UTF-8: {e}")))?;
            match Self::parse_response(&line)? {
                ParsedResponse::Sample(sample) => samples.push(sample),
                other => return Err(ProtocolError::UnexpectedResponse(format!("{other:?}"))),
            }
        }
        Ok(samples)
    }

    /// Send a STOP command to the ESP32.
    pub async fn stop(&mut self) -> Result<(), ProtocolError> {
        self.transport.send(&Self::format_line(&["STOP"])).await?;
        // Consume the response (OK or NOT_ACTIVE) so the stream stays aligned —
        // a caller that immediately sends MANIFEST after a recovery STOP must
        // not read the STOP response as the MANIFEST reply.
        let _ = self.transport.receive().await;
        self.firmware_state = FirmwareState::Idle;
        Ok(())
    }

    /// The current firmware state as tracked by the host.
    pub fn firmware_state(&self) -> FirmwareState {
        self.firmware_state.clone()
    }

    /// The firmware protocol version, if the handshake completed.
    pub fn firmware_version(&self) -> u32 {
        self.firmware_version
    }

    /// Whether the protocol has completed a handshake.
    pub fn is_connected(&self) -> bool {
        self.firmware_version > 0
    }
}

// ═════════════════════════════════════════════════════════════════════
// Test helpers — always compiled so integration tests can use them.
// Assumes the inner transport IS a `FakeTransport`.
// ═════════════════════════════════════════════════════════════════════

impl Esp32Protocol {
    /// Access the FakeTransport's sent commands for test assertions.
    ///
    /// # Safety
    ///
    /// This assumes the inner transport IS a `FakeTransport`. Only call
    /// from tests where you created one.
    pub fn test_sent_commands(&self) -> Vec<Vec<u8>> {
        unsafe {
            let transport_ref: &dyn Transport = &*self.transport;
            let fake_ptr: *const crate::backends::transport::FakeTransport = transport_ref
                as *const dyn Transport
                as *const crate::backends::transport::FakeTransport;
            (*fake_ptr).sent_commands()
        }
    }

    /// Inject a response into the FakeTransport for scripted testing.
    ///
    /// # Safety
    ///
    /// This assumes the inner transport IS a `FakeTransport`.
    pub fn test_inject_response(&self, data: Vec<u8>) {
        unsafe {
            let transport_ref: &dyn Transport = &*self.transport;
            let fake_ptr: *const crate::backends::transport::FakeTransport = transport_ref
                as *const dyn Transport
                as *const crate::backends::transport::FakeTransport;
            // FakeTransport methods take &self (interior mutability)
            // so we need &mut. But we have *const. Use cast to *mut.
            let fake_mut = fake_ptr as *mut crate::backends::transport::FakeTransport;
            (*fake_mut).inject_response(data);
        }
    }
}

// ═════════════════════════════════════════════════════════════════════
// TESTS
// ═════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backends::transport::FakeTransport;
    use crate::execution_boundary::manifest::{
        ExecutionManifest, ManifestInstruction, ManifestMetadata, ManifestSegment, TimedWaypoint,
    };

    // ── Helpers ────────────────────────────────────────────────────────

    fn sample_manifest() -> ExecutionManifest {
        ExecutionManifest {
            metadata: ManifestMetadata {
                dof_count: 2,
                total_samples: 3,
                duration_us: 1_000_000,
                repeat_count: 1,
            },
            segments: vec![ManifestSegment {
                index: 0,
                instruction: ManifestInstruction::MoveJ,
                sample_start: 0,
                sample_count: 3,
            }],
            samples: vec![
                TimedWaypoint {
                    joints: vec![0.0, 0.0],
                    dt_us: 0,
                },
                TimedWaypoint {
                    joints: vec![0.5, 0.3],
                    dt_us: 500_000,
                },
                TimedWaypoint {
                    joints: vec![1.0, 0.5],
                    dt_us: 500_000,
                },
            ],
        }
    }

    fn multi_segment_manifest() -> ExecutionManifest {
        ExecutionManifest {
            metadata: ManifestMetadata {
                dof_count: 3,
                total_samples: 5,
                duration_us: 2_000_000,
                repeat_count: 1,
            },
            segments: vec![
                ManifestSegment {
                    index: 0,
                    instruction: ManifestInstruction::MoveJ,
                    sample_start: 0,
                    sample_count: 2,
                },
                ManifestSegment {
                    index: 1,
                    instruction: ManifestInstruction::MoveL,
                    sample_start: 2,
                    sample_count: 3,
                },
            ],
            samples: vec![
                TimedWaypoint {
                    joints: vec![0.0, 0.0, 0.0],
                    dt_us: 0,
                },
                TimedWaypoint {
                    joints: vec![0.2, 0.1, 0.0],
                    dt_us: 500_000,
                },
                TimedWaypoint {
                    joints: vec![0.2, 0.1, 0.0],
                    dt_us: 0,
                },
                TimedWaypoint {
                    joints: vec![0.5, 0.4, 0.3],
                    dt_us: 500_000,
                },
                TimedWaypoint {
                    joints: vec![0.8, 0.6, 0.5],
                    dt_us: 1_000_000,
                },
            ],
        }
    }

    // ── Task 2.1: RED — encode manifest to text lines ────────────────

    #[test]
    fn encode_single_segment_manifest() {
        let manifest = sample_manifest();
        let lines = Esp32Protocol::encode_manifest(&manifest, 64);

        // MANIFEST + 1 SEGMENT + 3 SAMPLES + END_UPLOAD = 6 lines
        assert_eq!(lines.len(), 6);

        // MANIFEST <dof> <N> <dur_us> <chunk> (v2 chunked ACK, C)
        assert_eq!(
            String::from_utf8(lines[0].clone()).unwrap(),
            "MANIFEST 2 3 1000000 64 1\n"
        );

        // SEGMENT 0 movej 0 3
        assert_eq!(
            String::from_utf8(lines[1].clone()).unwrap(),
            "SEGMENT 0 movej 0 3\n"
        );

        // SAMPLE lines
        assert!(
            String::from_utf8(lines[2].clone())
                .unwrap()
                .starts_with("SAMPLE")
        );
        assert!(
            String::from_utf8(lines[3].clone())
                .unwrap()
                .starts_with("SAMPLE")
        );
        assert!(
            String::from_utf8(lines[4].clone())
                .unwrap()
                .starts_with("SAMPLE")
        );

        // END_UPLOAD
        assert_eq!(String::from_utf8(lines[5].clone()).unwrap(), "END_UPLOAD\n");
    }

    #[test]
    fn encode_multi_segment_manifest() {
        let manifest = multi_segment_manifest();
        let lines = Esp32Protocol::encode_manifest(&manifest, 62);

        // MANIFEST + 2 SEGMENTS + 5 SAMPLES + END_UPLOAD = 9 lines
        assert_eq!(lines.len(), 9);

        assert_eq!(
            String::from_utf8(lines[0].clone()).unwrap(),
            "MANIFEST 3 5 2000000 62 1\n"
        );

        assert_eq!(
            String::from_utf8(lines[1].clone()).unwrap(),
            "SEGMENT 0 movej 0 2\n"
        );
        assert_eq!(
            String::from_utf8(lines[2].clone()).unwrap(),
            "SEGMENT 1 movel 2 3\n"
        );

        // Last line is END_UPLOAD
        assert_eq!(String::from_utf8(lines[8].clone()).unwrap(), "END_UPLOAD\n");
    }

    #[test]
    fn encode_sample_lines_include_joints_and_dt() {
        let manifest = sample_manifest();
        let lines = Esp32Protocol::encode_manifest(&manifest, 64);

        // Sample 0: joints=[0.0, 0.0], dt_us=0
        let sample0 = String::from_utf8(lines[2].clone()).unwrap();
        assert!(sample0.starts_with("SAMPLE "));
        assert!(sample0.ends_with("0\n")); // dt_us=0 at end

        // Sample 1: joints=[0.5, 0.3], dt_us=500000
        let sample1 = String::from_utf8(lines[3].clone()).unwrap();
        assert!(sample1.starts_with("SAMPLE "));
        assert!(sample1.ends_with("500000\n"));
    }

    #[test]
    fn encode_empty_manifest_still_produces_manifest_line() {
        let manifest = ExecutionManifest {
            metadata: ManifestMetadata {
                dof_count: 0,
                total_samples: 0,
                duration_us: 0,
                repeat_count: 1,
            },
            segments: vec![],
            samples: vec![],
        };
        let lines = Esp32Protocol::encode_manifest(&manifest, 64);

        // MANIFEST line + END_UPLOAD (no SEGMENT or SAMPLE lines)
        assert_eq!(lines.len(), 2);
        assert_eq!(
            String::from_utf8(lines[0].clone()).unwrap(),
            "MANIFEST 0 0 0 64 1\n"
        );
        assert_eq!(String::from_utf8(lines[1].clone()).unwrap(), "END_UPLOAD\n");
    }

    // ── Task 2.3: RED — decode SAMPLE lines → ExecutionSample ────────

    #[test]
    fn parse_sample_line_with_two_joints() {
        // S1.1: collect-direction SAMPLE is timestamp-FIRST (firmware emits
        // `SAMPLE <ts_us> <j0..jN>`, protocol doc line 113).
        let line = "SAMPLE 1000 0.0 0.5\n";
        let parsed = Esp32Protocol::parse_response(line).unwrap();

        match parsed {
            ParsedResponse::Sample(sample) => {
                assert_eq!(sample.timestamp_us, 1000);
                assert_eq!(sample.joints.len(), 2);
                assert!((sample.joints[0] - 0.0).abs() < 1e-9);
                assert!((sample.joints[1] - 0.5).abs() < 1e-9);
            }
            other => panic!("Expected Sample, got {other:?}"),
        }
    }

    #[test]
    fn parse_sample_line_with_six_joints() {
        let line = "SAMPLE 5000000 0.1 0.2 0.3 0.4 0.5 0.6\n";
        let parsed = Esp32Protocol::parse_response(line).unwrap();

        match parsed {
            ParsedResponse::Sample(sample) => {
                assert_eq!(sample.timestamp_us, 5_000_000);
                assert_eq!(sample.joints.len(), 6);
                assert!((sample.joints[0] - 0.1).abs() < 1e-9);
                assert!((sample.joints[5] - 0.6).abs() < 1e-9);
            }
            other => panic!("Expected Sample, got {other:?}"),
        }
    }

    /// S1.1 — the exact spec scenario: firmware emits
    /// `SAMPLE 1000000 0.5 0.3 0.1 -0.1 0.0 0.0` and the host must parse
    /// timestamp-first (currently the parser reads ts-LAST → RED).
    #[test]
    fn parse_sample_ts_first_spec_scenario() {
        let line = "SAMPLE 1000000 0.5 0.3 0.1 -0.1 0.0 0.0\n";
        let parsed = Esp32Protocol::parse_response(line).unwrap();

        match parsed {
            ParsedResponse::Sample(sample) => {
                assert_eq!(sample.timestamp_us, 1_000_000);
                assert_eq!(sample.joints, vec![0.5, 0.3, 0.1, -0.1, 0.0, 0.0]);
            }
            other => panic!("Expected Sample, got {other:?}"),
        }
    }

    #[test]
    fn parse_sample_line_zero_timestamp() {
        let line = "SAMPLE 0 1.0 2.0\n";
        let parsed = Esp32Protocol::parse_response(line).unwrap();

        match parsed {
            ParsedResponse::Sample(sample) => {
                assert_eq!(sample.timestamp_us, 0);
                assert_eq!(sample.joints, vec![1.0, 2.0]);
            }
            other => panic!("Expected Sample, got {other:?}"),
        }
    }

    #[test]
    fn parse_sample_line_malformed_rejected() {
        let line = "SAMPLE abc 1000\n";
        let result = Esp32Protocol::parse_response(line);
        assert!(result.is_err());
        match result.unwrap_err() {
            ProtocolError::MalformedResponse(_) => {} // expected
            other => panic!("Expected MalformedResponse, got {other}"),
        }
    }

    #[test]
    fn parse_sample_line_too_short_rejected() {
        let line = "SAMPLE\n";
        let result = Esp32Protocol::parse_response(line);
        assert!(result.is_err());
    }

    // ── Task 2.7: RED — version mismatch handshake rejected ──────────

    #[tokio::test]
    async fn handshake_version_mismatch_rejected() {
        let mut transport = FakeTransport::new();
        transport.inject_response(b"HELLO 2 OK\n".to_vec());
        transport.connect().await.unwrap();

        let mut protocol = Esp32Protocol::new(Box::new(transport), 1);
        let result = protocol.handshake().await;

        assert!(result.is_err());
        match result.unwrap_err() {
            ProtocolError::VersionMismatch { expected, actual } => {
                assert_eq!(expected, 1);
                assert_eq!(actual, 2);
            }
            other => panic!("Expected VersionMismatch, got {other}"),
        }
    }

    #[tokio::test]
    async fn handshake_version_match_succeeds() {
        let mut transport = FakeTransport::new();
        transport.inject_response(b"HELLO 1 OK\n".to_vec());
        transport.connect().await.unwrap();

        let mut protocol = Esp32Protocol::new(Box::new(transport), 1);
        let result = protocol.handshake().await;

        assert!(result.is_ok());
        assert_eq!(protocol.firmware_version(), 1);
        assert!(protocol.is_connected());
    }

    /// A firmware line containing invalid UTF-8 bytes must surface as
    /// `ProtocolError::MalformedResponse` — never a panic or a mis-parse.
    #[tokio::test]
    async fn handshake_with_invalid_utf8_returns_malformed_response() {
        let mut transport = FakeTransport::new();
        // 0xFF is not a valid UTF-8 byte; the rest is a well-formed HELLO reply.
        transport.inject_response(vec![
            b'H', 0xFF, b'E', b'L', b'L', b'O', b' ', b'1', b' ', b'O', b'K', b'\n',
        ]);
        transport.connect().await.unwrap();

        let mut protocol = Esp32Protocol::new(Box::new(transport), 1);
        let result = protocol.handshake().await;

        match result.unwrap_err() {
            ProtocolError::MalformedResponse(msg) => {
                assert!(msg.contains("UTF-8"), "error must mention UTF-8, got: {msg}");
            }
            other => panic!("Expected MalformedResponse, got {other}"),
        }
    }

    // ── Task 2.11: RED — unexpected response triggers protocol error ──

    #[tokio::test]
    async fn unexpected_response_triggers_protocol_error() {
        let mut transport = FakeTransport::new();
        // When we send EXECUTE, FakeTransport returns "READY" instead of "OK"
        transport.inject_response(b"READY\n".to_vec());
        transport.connect().await.unwrap();

        let mut protocol = Esp32Protocol::new(Box::new(transport), 1);
        let result = protocol.start_execution().await;

        assert!(result.is_err());
        match result.unwrap_err() {
            ProtocolError::UnexpectedResponse(msg) => {
                // Debug format of ParsedResponse::Ready is "Ready"
                assert!(msg.contains("Ready"), "msg should mention Ready: {msg}");
            }
            other => panic!("Expected UnexpectedResponse, got {other}"),
        }
    }

    #[tokio::test]
    async fn execute_with_error_response() {
        let mut transport = FakeTransport::new();
        transport.inject_response(b"ERROR NOT_READY\n".to_vec());
        transport.connect().await.unwrap();

        let mut protocol = Esp32Protocol::new(Box::new(transport), 1);
        let result = protocol.start_execution().await;

        assert!(result.is_err());
        match result.unwrap_err() {
            ProtocolError::EspError(reason) => {
                assert_eq!(reason, "NOT_READY");
            }
            other => panic!("Expected EspError, got {other}"),
        }
    }

    #[tokio::test]
    async fn upload_manifest_rejected_with_esp_error() {
        let mut transport = FakeTransport::new();
        // sample_manifest() has: MANIFEST + 1 SEGMENT + 3 SAMPLES + END_UPLOAD.
        // v2 (C): 3 samples < chunk 64 → NO per-sample ACKs; END_UPLOAD is the
        // next response consumed.
        transport.inject_response(b"OK\n".to_vec()); // MANIFEST
        transport.inject_response(b"OK\n".to_vec()); // SEGMENT
        transport.inject_response(b"ERROR DOF_MISMATCH\n".to_vec()); // END_UPLOAD
        transport.connect().await.unwrap();

        let manifest = sample_manifest();
        let mut protocol = Esp32Protocol::new(Box::new(transport), 1);
        let result = protocol.upload_manifest(&manifest).await;

        assert!(result.is_err());
        match result.unwrap_err() {
            ProtocolError::EspError(reason) => {
                assert_eq!(reason, "DOF_MISMATCH");
            }
            other => panic!("Expected EspError, got {other}"),
        }
    }

    #[tokio::test]
    async fn upload_manifest_full_success() {
        let mut transport = FakeTransport::new();
        // MANIFEST → OK, SEGMENT → OK; the 3 samples (dof=2 → chunk 64) form a
        // trailing partial chunk → NO per-sample ACKs; END_UPLOAD → READY.
        transport.inject_response(b"OK\n".to_vec()); // MANIFEST
        transport.inject_response(b"OK\n".to_vec()); // SEGMENT
        transport.inject_response(b"READY\n".to_vec()); // END_UPLOAD
        transport.connect().await.unwrap();

        let manifest = sample_manifest();
        let mut protocol = Esp32Protocol::new(Box::new(transport), 1);
        let result = protocol.upload_manifest(&manifest).await;

        assert!(result.is_ok());
        assert_eq!(protocol.firmware_state(), FirmwareState::Ready);
    }

    // ── Additional parse_response tests ──────────────────────────────

    #[test]
    fn parse_ok_response() {
        let parsed = Esp32Protocol::parse_response("OK\n").unwrap();
        assert_eq!(parsed, ParsedResponse::Ok);
    }

    #[test]
    fn parse_ready_response() {
        let parsed = Esp32Protocol::parse_response("READY\n").unwrap();
        assert_eq!(parsed, ParsedResponse::Ready);
    }

    #[test]
    fn parse_handshake_ok() {
        let parsed = Esp32Protocol::parse_response("HELLO 1 OK\n").unwrap();
        assert_eq!(parsed, ParsedResponse::HandshakeOk(1));
    }

    #[test]
    fn parse_error_response() {
        let parsed = Esp32Protocol::parse_response("ERROR DOF_MISMATCH\n").unwrap();
        match parsed {
            ParsedResponse::Error(reason) => assert_eq!(reason, "DOF_MISMATCH"),
            other => panic!("Expected Error, got {other:?}"),
        }
    }

    #[test]
    fn parse_status_running() {
        // S1.2: EXECUTING payload — `STATUS RUNNING <progress> <j0..jN>`.
        // Wire token stays RUNNING; the host maps it to Executing internally.
        let parsed = Esp32Protocol::parse_response("STATUS RUNNING 0.45 0.5 0.3 0.1 -0.1 0.0 0.0\n")
            .unwrap();
        match parsed {
            ParsedResponse::StatusRunning { progress, joints } => {
                assert!((progress - 0.45).abs() < 1e-9);
                assert_eq!(joints, vec![0.5, 0.3, 0.1, -0.1, 0.0, 0.0]);
            }
            other => panic!("Expected StatusRunning, got {other:?}"),
        }
    }

    #[test]
    fn parse_status_completed() {
        // S1.2/S3.1: `STATUS COMPLETED <count>` — how many samples to request.
        let parsed = Esp32Protocol::parse_response("STATUS COMPLETED 5\n").unwrap();
        match parsed {
            ParsedResponse::StatusCompleted { sample_count } => assert_eq!(sample_count, 5),
            other => panic!("Expected StatusCompleted, got {other:?}"),
        }
    }

    // ── S1.3 RED: STATUS full-state parse (IDLE/RECEIVING/READY + ERROR) ──

    #[test]
    fn parse_status_idle() {
        let parsed = Esp32Protocol::parse_response("STATUS IDLE\n").unwrap();
        assert_eq!(parsed, ParsedResponse::StatusIdle);
    }

    #[test]
    fn parse_status_receiving() {
        let parsed = Esp32Protocol::parse_response("STATUS RECEIVING\n").unwrap();
        assert_eq!(parsed, ParsedResponse::StatusReceiving);
    }

    #[test]
    fn parse_status_ready() {
        let parsed = Esp32Protocol::parse_response("STATUS READY\n").unwrap();
        assert_eq!(parsed, ParsedResponse::StatusReady);
    }

    #[test]
    fn parse_status_error_with_reason() {
        let parsed = Esp32Protocol::parse_response("STATUS ERROR MOTOR_FAULT\n").unwrap();
        match parsed {
            ParsedResponse::Error(reason) => assert_eq!(reason, "MOTOR_FAULT"),
            other => panic!("Expected Error, got {other:?}"),
        }
    }

    // ── Review correction (RES-01 / RISK-2 / REL-03) ────────────────────

    /// RES-01: legacy HELLO-v1 firmware emits a BARE `STATUS RUNNING` (no
    /// progress / joints payload). The parser must accept it — progress
    /// defaults to 0.0, joints to empty — instead of failing every poll.
    #[test]
    fn parse_status_running_bare_token_is_lenient() {
        let parsed = Esp32Protocol::parse_response("STATUS RUNNING\n").unwrap();
        match parsed {
            ParsedResponse::StatusRunning { progress, joints } => {
                assert_eq!(progress, 0.0);
                assert!(joints.is_empty());
            }
            other => panic!("Expected StatusRunning, got {other:?}"),
        }
    }

    /// RES-01: same leniency for a bare `STATUS COMPLETED` — sample_count
    /// defaults to 0.
    #[test]
    fn parse_status_completed_bare_token_is_lenient() {
        let parsed = Esp32Protocol::parse_response("STATUS COMPLETED\n").unwrap();
        match parsed {
            ParsedResponse::StatusCompleted { sample_count } => assert_eq!(sample_count, 0),
            other => panic!("Expected StatusCompleted, got {other:?}"),
        }
    }

    /// RISK-2: a non-finite progress (nan / inf / 1e999) would multiply by
    /// plan_duration in `map_firmware_state` and then panic the tick in
    /// `Duration::from_secs_f64`. It must be rejected at parse time.
    #[test]
    fn parse_status_running_rejects_non_finite_progress() {
        for line in [
            "STATUS RUNNING nan 0 0\n",
            "STATUS RUNNING inf 0 0\n",
            "STATUS RUNNING 1e999 0 0\n",
        ] {
            let result = Esp32Protocol::parse_response(line);
            assert!(
                matches!(result, Err(ProtocolError::MalformedResponse(_))),
                "{line:?} must be MalformedResponse, got {result:?}"
            );
        }
    }

    /// REL-03 / RES-06: an UNKNOWN STATUS token must be MalformedResponse —
    /// only the literal `ERROR` token is a firmware error (→ EStop). Any
    /// other token silently mapping to Error would EStop a healthy run.
    #[test]
    fn parse_status_unknown_token_is_malformed_not_error() {
        let result = Esp32Protocol::parse_response("STATUS FOO\n");
        assert!(
            matches!(result, Err(ProtocolError::MalformedResponse(_))),
            "STATUS FOO must be MalformedResponse, got {result:?}"
        );
    }

    /// RISK-3: the firmware-supplied `count` drives `Vec::with_capacity` +
    /// the read loop in `collect_samples` — a bogus huge count must be
    /// capped at MAX_SAMPLES before any allocation.
    #[test]
    fn collect_samples_caps_firmware_supplied_count() {
        let bogus = u32::MAX as usize;
        assert_eq!(
            cap_sample_count(bogus),
            MAX_SAMPLES,
            "bogus count must be capped to MAX_SAMPLES"
        );
        assert_eq!(cap_sample_count(5), 5, "small counts pass through");
    }

    #[test]
    fn parse_unknown_command_response() {
        let result = Esp32Protocol::parse_response("BOGUS\n");
        assert!(result.is_err());
        match result.unwrap_err() {
            ProtocolError::UnexpectedResponse(msg) => {
                assert!(msg.contains("BOGUS"));
            }
            other => panic!("Expected UnexpectedResponse, got {other}"),
        }
    }
}
