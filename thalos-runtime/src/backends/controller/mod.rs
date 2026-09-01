pub mod simulation;

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;

use thalos_engine::core::execution::plan::ExecutionPlan;
use thalos_engine::core::execution::runtime::RuntimeProgram;

use crate::error::ControllerError;
use crate::execution_boundary::ExecutionSample;
use crate::session::execution_source::ExecutionSource;
use crate::state::robot_state::RobotState;

/// Descriptor of a backend's capabilities — consumed by the UI to
/// enable/disable buttons.
#[derive(Clone, Debug, PartialEq)]
pub struct BackendCapabilities {
    pub pause: bool,
    pub resume: bool,
    pub io: bool,
    pub gripper: bool,
    pub streaming: bool,
    /// v3: the backend repeats the trajectory INTERNALLY (`repeat_count` in
    /// the manifest) — the host never re-executes between passes and derives
    /// the iteration from the overall progress. Only the ESP32 backend sets it.
    pub firmware_repeat: bool,
}

impl BackendCapabilities {
    /// Full capabilities — all features supported.
    pub fn full() -> Self {
        Self {
            pause: true,
            resume: true,
            io: true,
            gripper: true,
            streaming: true,
            firmware_repeat: false,
        }
    }

    /// Minimal capabilities — only execution and stop.
    pub fn minimal() -> Self {
        Self {
            pause: false,
            resume: false,
            io: false,
            gripper: false,
            streaming: false,
            firmware_repeat: false,
        }
    }
}

/// Async contract between the Thalos runtime and any controller
/// implementation (simulated, ROS2, serial, EtherCAT, etc.).
///
/// Represents a **controller**, not just a motion backend: it owns
/// the connection, executes trajectories, exposes live state, and
/// (optionally) controls peripherals such as I/O ports and grippers.
///
/// The runtime speaks ONLY to this trait — all backends implement it.
///
/// Device I/O methods have default implementations that return
/// `Err(ControllerError::UnsupportedCapability)`. Backends that
/// support a given operation MUST override the default.
#[async_trait]
pub trait RobotController: Send + Sync {
    /// Open the connection to the robot. Idempotent: calling
    /// `connect` on an already-connected controller returns
    /// `Err(ControllerError::AlreadyConnected)`.
    async fn connect(&mut self) -> Result<(), ControllerError>;

    /// Close the connection. Safe to call when already disconnected.
    async fn disconnect(&mut self) -> Result<(), ControllerError>;

    /// Whether the controller is currently connected.
    fn is_connected(&self) -> bool;

    /// Accept an execution plan and begin execution. Returns immediately —
    /// does NOT block until the trajectory completes. Progress is
    /// observable via `robot_state()`.
    ///
    /// `plan`: the execution IR — ordered waypoints with absolute
    /// timestamps (seconds), 1:1 segments, and the total duration.
    async fn execute(&mut self, plan: ExecutionPlan) -> Result<(), ControllerError>;

    /// Stop the current execution immediately. Always supported.
    async fn stop(&mut self) -> Result<(), ControllerError> {
        Err(ControllerError::UnsupportedCapability)
    }

    /// Pause execution. Requires `BackendCapabilities::pause`.
    async fn pause(&mut self) -> Result<(), ControllerError> {
        Err(ControllerError::UnsupportedCapability)
    }

    /// Resume a paused execution. Requires `BackendCapabilities::resume`.
    async fn resume(&mut self) -> Result<(), ControllerError> {
        Err(ControllerError::UnsupportedCapability)
    }

    /// Advance simulation time by `dt` seconds.
    ///
    /// Simulation backends implement this to interpolate along the trajectory.
    /// Real hardware backends return `Err(UnsupportedCapability)` — time is real.
    async fn advance(&self, _dt: f64) -> Result<(), ControllerError> {
        Err(ControllerError::UnsupportedCapability)
    }

    /// Load the `RuntimeProgram` (absolute `at_time` events) that `advance`
    /// should dispatch during execution.
    ///
    /// Called at schedule time, before `execute`. Simulation backends store
    /// the program and fire `SetOutput`/`Delay` at their absolute times
    /// (tick-driven dispatch). Hardware backends may ignore it — the default
    /// is a no-op.
    async fn load_runtime_program(
        &mut self,
        _program: RuntimeProgram,
    ) -> Result<(), ControllerError> {
        Ok(())
    }

    /// Seek to a position (fraction 0.0–1.0) in the current trajectory.
    ///
    /// Only meaningful for replay/simulation backends.
    /// Real hardware backends return `Err(UnsupportedCapability)`.
    async fn seek(&self, _position: f64) -> Result<(), ControllerError> {
        Err(ControllerError::UnsupportedCapability)
    }

    /// Live state of the robot, as an `Arc` for cheap sharing.
    async fn robot_state(&self) -> Arc<RobotState>;

    /// Take the execution trace (hardware-collected samples) if available.
    ///
    /// Default: `None` — simulation/playback backends have no hardware
    /// samples. `Esp32Backend` overrides this to return the SAMPLES collected
    /// on completion, exactly once (clear-on-take).
    async fn take_execution_trace(&self) -> Option<Vec<ExecutionSample>> {
        None
    }

    /// Static capabilities descriptor.
    fn capabilities(&self) -> BackendCapabilities;

    /// Execution origin this controller represents (R4-001). Defaults to
    /// Simulation; hardware controllers (Esp32) report Hardware so the
    /// execution badge reflects the ACTIVE backend instead of always
    /// "Simulation". Informational only — never gates the execution flow.
    fn execution_source(&self) -> ExecutionSource {
        ExecutionSource::Simulation
    }

    // ── Device I/O — defaults return UnsupportedCapability ──

    /// Set a digital output port.
    async fn set_io(&mut self, _port: u32, _value: bool) -> Result<(), ControllerError> {
        Err(ControllerError::UnsupportedCapability)
    }

    /// Wait for a digital input to reach a specific value, with timeout.
    async fn wait_input(
        &mut self,
        _port: u32,
        _value: bool,
        _timeout: Duration,
    ) -> Result<bool, ControllerError> {
        Err(ControllerError::UnsupportedCapability)
    }

    /// Set gripper position.
    async fn set_gripper(&mut self, _position: f64) -> Result<(), ControllerError> {
        Err(ControllerError::UnsupportedCapability)
    }
}

// ═════════════════════════════════════════════════════════════════════
// TESTS
// ═════════════════════════════════════════════════════════════════════

#[cfg(test)]
pub mod tests {
    use super::*;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use thalos_engine::core::execution::plan::{ExecutionSegment, ExecutionWaypoint, PlanInstruction};

    pub struct MockController {
        pub connected: AtomicBool,
        pub connect_count: AtomicUsize,
        pub disconnect_count: AtomicUsize,
        pub executed: AtomicBool,
        pub paused: AtomicBool,
        pub capabilities: BackendCapabilities,
        /// Reported `ExecutionSource` (R4-001) — lets tests simulate a
        /// hardware controller without a real transport.
        pub source: ExecutionSource,
        /// Optional `execute` failure to inject (R4-001): when set, `execute`
        /// returns this error instead of succeeding — tests exercise the
        /// ConnectionLost propagation path without a real device.
        pub execute_error: Option<ControllerError>,
        /// Optional `advance` failure to inject (R4-001): when set, `advance`
        /// returns this error instead of succeeding.
        pub advance_error: Option<ControllerError>,
        /// Optional execution trace to return from `take_execution_trace`
        /// (S3.6) — lets scene tests exercise the hardware-trace drain
        /// without a real device.
        pub execution_trace: Option<Vec<ExecutionSample>>,
        /// Optional `robot_state` override (review correction) — when set,
        /// `robot_state` returns this state instead of the default, letting
        /// scene tests simulate Moving/EStop/Completed states without a
        /// real device.
        pub state: Option<RobotState>,
        /// Number of `execute` calls (repeat orchestration): each iteration
        /// completion re-executes the plan, so the gate loop increments this.
        pub execute_count: AtomicUsize,
        /// Number of `take_execution_trace` calls — the repeat gate must drain
        /// the hardware samples EXACTLY once (final iteration, NF3) and never
        /// on intermediate iterations or failures (S2).
        pub take_trace_calls: AtomicUsize,
        /// Last `ExecutionPlan` passed to `execute` — captured so tests can
        /// assert the real timestamps/segments that reached the controller
        /// (esp32-execute-real-timestamps migration).
        pub last_plan: std::sync::Mutex<Option<ExecutionPlan>>,
    }

    impl MockController {
        pub fn new() -> Self {
            Self {
                connected: AtomicBool::new(false),
                connect_count: AtomicUsize::new(0),
                disconnect_count: AtomicUsize::new(0),
                executed: AtomicBool::new(false),
                paused: AtomicBool::new(false),
                capabilities: BackendCapabilities::full(),
                source: ExecutionSource::Simulation,
                execute_error: None,
                advance_error: None,
                execution_trace: None,
                state: None,
                execute_count: AtomicUsize::new(0),
                take_trace_calls: AtomicUsize::new(0),
                last_plan: std::sync::Mutex::new(None),
            }
        }
    }

    /// Build an even-spaced single-segment `ExecutionPlan` — the trait-test
    /// fixture for the `execute(plan)` contract.
    fn test_plan(waypoints: Vec<Vec<f64>>, duration: f64) -> ExecutionPlan {
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
            program_id: None,
            program_revision: None,
            source_fingerprint: None,
            robot_id: None,
        }
    }

    #[async_trait]
    impl RobotController for MockController {
        async fn connect(&mut self) -> Result<(), ControllerError> {
            if self.connected.load(Ordering::SeqCst) {
                return Err(ControllerError::AlreadyConnected);
            }
            self.connected.store(true, Ordering::SeqCst);
            self.connect_count.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }

        async fn disconnect(&mut self) -> Result<(), ControllerError> {
            self.connected.store(false, Ordering::SeqCst);
            self.disconnect_count.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }

        fn is_connected(&self) -> bool {
            self.connected.load(Ordering::SeqCst)
        }

        async fn execute(
            &mut self,
            plan: ExecutionPlan,
        ) -> Result<(), ControllerError> {
            if !self.connected.load(Ordering::SeqCst) {
                return Err(ControllerError::NotConnected);
            }
            if let Some(ref err) = self.execute_error {
                return Err(err.clone());
            }
            *self.last_plan.lock().unwrap() = Some(plan);
            self.executed.store(true, Ordering::SeqCst);
            self.execute_count.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }

        async fn stop(&mut self) -> Result<(), ControllerError> {
            self.executed.store(false, Ordering::SeqCst);
            Ok(())
        }

        async fn pause(&mut self) -> Result<(), ControllerError> {
            if !self.capabilities.pause {
                return Err(ControllerError::UnsupportedCapability);
            }
            self.paused.store(true, Ordering::SeqCst);
            Ok(())
        }

        async fn resume(&mut self) -> Result<(), ControllerError> {
            if !self.capabilities.resume {
                return Err(ControllerError::UnsupportedCapability);
            }
            self.paused.store(false, Ordering::SeqCst);
            Ok(())
        }

        async fn advance(&self, _dt: f64) -> Result<(), ControllerError> {
            if let Some(ref err) = self.advance_error {
                return Err(err.clone());
            }
            Ok(())
        }

        async fn robot_state(&self) -> Arc<RobotState> {
            Arc::new(self.state.clone().unwrap_or_default())
        }

        async fn take_execution_trace(&self) -> Option<Vec<ExecutionSample>> {
            self.take_trace_calls.fetch_add(1, Ordering::SeqCst);
            self.execution_trace.clone()
        }

        fn capabilities(&self) -> BackendCapabilities {
            self.capabilities.clone()
        }

        fn execution_source(&self) -> ExecutionSource {
            self.source.clone()
        }
    }

    #[tokio::test]
    async fn test_mock_connect_disconnect() {
        let mut ctrl = MockController::new();
        assert!(!ctrl.is_connected());

        ctrl.connect().await.unwrap();
        assert!(ctrl.is_connected());

        ctrl.disconnect().await.unwrap();
        assert!(!ctrl.is_connected());
    }

    #[tokio::test]
    async fn test_double_connect_rejected() {
        let mut ctrl = MockController::new();
        ctrl.connect().await.unwrap();
        let err = ctrl.connect().await.unwrap_err();
        assert_eq!(err, ControllerError::AlreadyConnected);
    }

    #[tokio::test]
    async fn test_execute_requires_connection() {
        let mut ctrl = MockController::new();
        let err = ctrl.execute(test_plan(vec![], 0.0)).await.unwrap_err();
        assert_eq!(err, ControllerError::NotConnected);
    }

    #[tokio::test]
    async fn test_execute_pause_resume_stop_flow() {
        let mut ctrl = MockController::new();
        ctrl.connect().await.unwrap();

        ctrl.execute(test_plan(vec![vec![0.0]], 1.0)).await.unwrap();
        assert!(ctrl.executed.load(Ordering::SeqCst));

        ctrl.pause().await.unwrap();
        assert!(ctrl.paused.load(Ordering::SeqCst));

        ctrl.resume().await.unwrap();
        assert!(!ctrl.paused.load(Ordering::SeqCst));

        ctrl.stop().await.unwrap();
        assert!(!ctrl.executed.load(Ordering::SeqCst));
    }

    /// The `execute(plan)` contract: the controller receives the full
    /// `ExecutionPlan` — waypoints with absolute timestamps, 1:1 segments,
    /// and the total duration — not a bare `(waypoints, duration)` pair.
    #[tokio::test]
    async fn test_execute_consumes_execution_plan() {
        let mut ctrl = MockController::new();
        ctrl.connect().await.unwrap();

        let plan = test_plan(vec![vec![0.0, 0.0], vec![0.5, 0.3], vec![1.0, 0.5]], 2.0);
        ctrl.execute(plan.clone()).await.unwrap();

        let captured = ctrl
            .last_plan
            .lock()
            .unwrap()
            .clone()
            .expect("execute must capture the plan");
        assert_eq!(captured, plan, "the exact plan must reach the controller");
        assert_eq!(captured.waypoints[0].timestamp, 0.0);
        assert_eq!(captured.waypoints[1].timestamp, 1.0);
        assert_eq!(captured.waypoints[2].timestamp, 2.0);
        assert_eq!(captured.duration, 2.0);
        assert_eq!(captured.segments.len(), 1);
        assert_eq!(captured.segments[0].waypoint_range, 0..3);
    }

    #[tokio::test]
    async fn test_mock_returns_robot_state() {
        let ctrl = MockController::new();
        let state = ctrl.robot_state().await;
        assert_eq!(state.revision, 0);
    }
}
