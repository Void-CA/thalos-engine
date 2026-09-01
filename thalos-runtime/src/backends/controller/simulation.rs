use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use arc_swap::ArcSwap;
use async_trait::async_trait;
use tokio::sync::RwLock;

use thalos_engine::core::execution::plan::ExecutionPlan;
use thalos_engine::core::execution::runtime::{RuntimeAction, RuntimeEvent, RuntimeProgram};

use crate::backends::controller::{BackendCapabilities, RobotController};
use crate::error::ControllerError;
use crate::plan::{ExecutionSession, SessionStatus};
use crate::state::robot_state::{
    Diagnostics, ExecutionState, JointState, MotionMode, MotionState, RobotState,
};

/// Simulation backend — the default controller when no hardware is connected.
///
/// Advances trajectories by interpolating waypoints linearly against
/// an internal clock. Publishes an `ArcSwap<RobotState>` that readers
/// can load cheaply without lock contention.
///
/// # Runtime event dispatch (PR 3)
///
/// The controller carries an optional `RuntimeProgram` loaded at schedule
/// time. Two clocks drive execution:
///
/// - `clock_time` — execution wall time, always advances per tick.
/// - `traj_time` — trajectory interpolation time, frozen while a `Delay`
///   event is active (the robot stays still) and resumed from the held
///   joints once the delay elapses.
///
/// Each tick: advance `clock_time`, dispatch events whose absolute `at_time`
/// has been reached (Delay freezes the trajectory; SetOutput is recorded),
/// then interpolate the trajectory only when no delay is active.
pub struct SimulationController {
    connected: AtomicBool,
    waypoints: RwLock<Vec<Vec<f64>>>,
    duration: RwLock<f64>,
    execution: RwLock<ExecutionSession>,
    dof: usize,
    state: ArcSwap<RobotState>,
    /// Runtime events (sorted by absolute `at_time`), loaded via
    /// `load_runtime_program` at schedule time. `None` = no event dispatch.
    program: RwLock<Option<RuntimeProgram>>,
    /// Execution wall clock — always advances.
    clock_time: RwLock<Duration>,
    /// Trajectory interpolation clock — frozen during active delays.
    traj_time: RwLock<Duration>,
    /// When the current delay ends (clock domain); `None` = no active delay.
    delay_until: RwLock<Option<Duration>>,
    /// Index of the next event to dispatch (`program` is sorted by at_time).
    event_cursor: RwLock<usize>,
    /// Events already dispatched — observability for tests/telemetry.
    dispatched: RwLock<Vec<RuntimeEvent>>,
}

impl SimulationController {
    pub fn new(dof: usize) -> Self {
        let initial = RobotState::default();
        Self {
            connected: AtomicBool::new(false),
            waypoints: RwLock::new(Vec::new()),
            duration: RwLock::new(0.0),
            execution: RwLock::new(ExecutionSession::new("sim")),
            dof,
            state: ArcSwap::new(Arc::new(initial)),
            program: RwLock::new(None),
            clock_time: RwLock::new(Duration::ZERO),
            traj_time: RwLock::new(Duration::ZERO),
            delay_until: RwLock::new(None),
            event_cursor: RwLock::new(0),
            dispatched: RwLock::new(Vec::new()),
        }
    }

    /// Reinitialize the controller for a different robot DOF.
    ///
    /// Called when the user loads a new robot (canonical or URDF).
    /// Preserves the connected state (if any), resets everything else.
    pub fn reconfigure(&mut self, dof: usize) {
        self.waypoints = RwLock::new(Vec::new());
        self.duration = RwLock::new(0.0);
        self.execution = RwLock::new(ExecutionSession::new("sim"));
        self.program = RwLock::new(None);
        self.clock_time = RwLock::new(Duration::ZERO);
        self.traj_time = RwLock::new(Duration::ZERO);
        self.delay_until = RwLock::new(None);
        self.event_cursor = RwLock::new(0);
        self.dispatched = RwLock::new(Vec::new());
        self.dof = dof;
        self.state.store(Arc::new(RobotState::default()));
    }

    // ── Runtime event dispatch state (PR 3) ──────────────────────────────

    /// The execution wall clock (always advances per tick).
    pub async fn clock_time(&self) -> Duration {
        *self.clock_time.read().await
    }

    /// The trajectory interpolation clock (frozen during active delays).
    pub async fn traj_time(&self) -> Duration {
        *self.traj_time.read().await
    }

    /// Events dispatched so far, in dispatch order.
    pub async fn dispatched_events(&self) -> Vec<RuntimeEvent> {
        self.dispatched.read().await.clone()
    }

    /// Reset the event-dispatch timeline for a fresh execution.
    async fn reset_event_timeline(&self) {
        *self.clock_time.write().await = Duration::ZERO;
        *self.traj_time.write().await = Duration::ZERO;
        *self.delay_until.write().await = None;
        *self.event_cursor.write().await = 0;
        *self.dispatched.write().await = Vec::new();
    }

    /// Advance the simulation by `dt` seconds, interpolating joint angles
    /// and updating the internal `RobotState`.
    ///
    /// Tick sequence (PR 3 — runtime event dispatch):
    ///
    /// 1. `clock_time += dt` — the wall clock ALWAYS advances.
    /// 2. If a `Delay` is active (`clock_time < delay_until`), the
    ///    trajectory is FROZEN: joints hold, `traj_time` does not advance.
    /// 3. Otherwise interpolate the trajectory (`traj_time += dt`) and
    ///    dispatch every event whose absolute `at_time <= clock_time`:
    ///    - `Delay(d)`: set `delay_until = clock_time + d` (hold starts now).
    ///    - `SetOutput`: record the output (simulation has no physical IO;
    ///      the record IS the observable dispatch).
    pub async fn advance_inner(&self, dt: f64) {
        let waypoints = self.waypoints.read().await;
        let duration = *self.duration.read().await;
        let mut execution = self.execution.write().await;
        let program = self.program.read().await;

        if execution.status != SessionStatus::Running || execution.status.is_terminal() {
            return;
        }
        if waypoints.is_empty() || duration <= 0.0 {
            return;
        }
        let total_steps = waypoints.len().saturating_sub(1);
        if total_steps == 0 {
            return;
        }

        // 1. The wall clock always advances.
        let mut clock = self.clock_time.write().await;
        *clock += Duration::from_secs_f64(dt);

        // 2. Active delay freezes the trajectory.
        let mut delay_until = self.delay_until.write().await;
        if let Some(until) = *delay_until {
            if *clock < until {
                // Frozen: hold the last published joints; do not advance
                // trajectory time and do not dispatch further events.
                drop(clock);
                drop(delay_until);
                drop(program);
                drop(execution);
                return;
            }
            // Delay elapsed — resume trajectory from the held joints.
            *delay_until = None;
        }
        drop(delay_until);

        // 3a. Advance trajectory interpolation.
        let progress = execution.advance(dt, duration);
        let mut traj = self.traj_time.write().await;
        *traj += Duration::from_secs_f64(dt);
        drop(traj);

        let frac = progress.clamp(0.0, 1.0);
        let idx_f = frac * total_steps as f64;
        let i = idx_f.floor() as usize;
        let j = (i + 1).min(waypoints.len() - 1);
        let local_frac = idx_f - i as f64;

        let joints: Vec<f64> = waypoints[i]
            .iter()
            .zip(&waypoints[j])
            .map(|(&a, &b)| a + (b - a) * local_frac)
            .collect();

        // 3b. Dispatch events whose absolute at_time has been reached.
        if let Some(program) = &*program {
            let mut cursor = self.event_cursor.write().await;
            let mut dispatched = self.dispatched.write().await;
            let mut delay_until = self.delay_until.write().await;
            while *cursor < program.events.len() {
                let event = &program.events[*cursor];
                if event.at_time > *clock {
                    break;
                }
                match &event.action {
                    RuntimeAction::Delay(d) => {
                        // Freeze the trajectory starting NOW, for `d`.
                        *delay_until = Some(*clock + *d);
                    }
                    RuntimeAction::SetOutput { .. } => {
                        dispatched.push(event.clone());
                    }
                }
                *cursor += 1;
            }
        }

        let new_revision = self.state.load().revision + 1;
        let new_state = RobotState {
            revision: new_revision,
            joints: JointState {
                positions: joints,
                velocities: vec![0.0; self.dof],
                torques: vec![0.0; self.dof],
            },
            execution: ExecutionState {
                current_program: None,
                current_segment: None,
                progress,
            },
            motion: MotionState {
                mode: if execution.status == SessionStatus::Completed {
                    MotionMode::Idle
                } else {
                    MotionMode::Moving
                },
                power_on: true,
                motion_enabled: true,
            },
            diagnostics: Diagnostics {
                timestamp: chrono::Utc::now(),
                ..Diagnostics::default()
            },
            ..RobotState::default()
        };

        self.state.store(Arc::new(new_state));
    }
}

#[async_trait]
impl RobotController for SimulationController {
    async fn connect(&mut self) -> Result<(), ControllerError> {
        if self.connected.swap(true, Ordering::SeqCst) {
            return Err(ControllerError::AlreadyConnected);
        }
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), ControllerError> {
        self.connected.store(false, Ordering::SeqCst);
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
        let waypoints: Vec<Vec<f64>> = plan
            .waypoints
            .iter()
            .map(|wp| wp.joints.clone())
            .collect();
        let duration = plan.duration;
        if waypoints.is_empty() || duration <= 0.0 {
            return Ok(());
        }

        let initial_positions = waypoints.first().cloned().unwrap_or_default();
        *self.waypoints.write().await = waypoints;
        *self.duration.write().await = duration;

        let mut exec = self.execution.write().await;
        exec.reset();
        exec.start();
        drop(exec);
        // t=0 = plan start: reset both clocks and the event dispatch state.
        self.reset_event_timeline().await;

        // Update the shared state to reflect active execution
        let new_revision = self.state.load().revision + 1;
        let new_state = RobotState {
            revision: new_revision,
            joints: JointState {
                positions: initial_positions,
                velocities: vec![0.0; self.dof],
                torques: vec![0.0; self.dof],
            },
            execution: ExecutionState {
                current_program: None,
                current_segment: None,
                progress: 0.0,
            },
            motion: MotionState {
                mode: MotionMode::Moving,
                power_on: true,
                motion_enabled: true,
            },
            diagnostics: Diagnostics {
                timestamp: chrono::Utc::now(),
                ..Diagnostics::default()
            },
            ..RobotState::default()
        };
        self.state.store(Arc::new(new_state));

        Ok(())
    }

    async fn stop(&mut self) -> Result<(), ControllerError> {
        self.execution.write().await.cancel();
        self.state.rcu(|prev| {
            let mut s = (**prev).clone();
            s.revision = prev.revision + 1;
            s.motion.mode = MotionMode::Idle;
            s.motion.motion_enabled = false;
            s.execution.progress = 1.0;
            Arc::new(s)
        });
        Ok(())
    }

    async fn pause(&mut self) -> Result<(), ControllerError> {
        self.execution.write().await.pause();
        self.state.rcu(|prev| {
            let mut s = (**prev).clone();
            s.revision = prev.revision + 1;
            s.motion.mode = MotionMode::Paused;
            Arc::new(s)
        });
        Ok(())
    }

    async fn resume(&mut self) -> Result<(), ControllerError> {
        self.execution.write().await.resume();
        self.state.rcu(|prev| {
            let mut s = (**prev).clone();
            s.revision = prev.revision + 1;
            s.motion.mode = MotionMode::Moving;
            Arc::new(s)
        });
        Ok(())
    }

    async fn advance(&self, dt: f64) -> Result<(), ControllerError> {
        self.advance_inner(dt).await;
        Ok(())
    }

    async fn load_runtime_program(
        &mut self,
        program: RuntimeProgram,
    ) -> Result<(), ControllerError> {
        let sorted = RuntimeProgram::new(program.events);
        *self.program.write().await = Some(sorted);
        *self.event_cursor.write().await = 0;
        *self.dispatched.write().await = Vec::new();
        Ok(())
    }

    async fn robot_state(&self) -> Arc<RobotState> {
        self.state.load_full()
    }

    fn capabilities(&self) -> BackendCapabilities {
        BackendCapabilities::full()
    }
}

// ═════════════════════════════════════════════════════════════════════
// TESTS — Runtime event dispatch (PR 3)
// ═════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use thalos_engine::core::{
        execution::{
            plan::{ExecutionSegment, ExecutionWaypoint, PlanInstruction},
            runtime::{RuntimeAction, RuntimeEvent, RuntimeProgram},
        },
        ids::OperationId,
        motion::target::{OutputChannel, OutputValue},
    };

    /// The 2-DOF trajectory `[0,0] → [1,1]` over `duration` seconds as an
    /// `ExecutionPlan` — the fixture every simulation test executes.
    fn test_plan() -> ExecutionPlan {
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
            program_id: None,
            program_revision: None,
            source_fingerprint: None,
            robot_id: None,
        }
    }

    /// Connect, load a program, and start the 2-DOF test trajectory.
    async fn controller_with_program(program: RuntimeProgram) -> SimulationController {
        let mut ctrl = SimulationController::new(2);
        // `connect`/`execute` disambiguate via the RobotController trait.
        <SimulationController as RobotController>::connect(&mut ctrl)
            .await
            .expect("connect");
        ctrl.load_runtime_program(program)
            .await
            .expect("load program");
        <SimulationController as RobotController>::execute(&mut ctrl, test_plan())
            .await
            .expect("execute");
        ctrl
    }

    fn set_output(op: &str, at_time: Duration) -> RuntimeEvent {
        RuntimeEvent {
            at_time,
            operation_id: OperationId(op.to_string()),
            action: RuntimeAction::SetOutput {
                channel: OutputChannel {
                    name: "gripper".into(),
                    channel_type: "digital".into(),
                },
                value: OutputValue::Bool(true),
            },
        }
    }

    fn delay(op: &str, at_time: Duration, duration: Duration) -> RuntimeEvent {
        RuntimeEvent {
            at_time,
            operation_id: OperationId(op.to_string()),
            action: RuntimeAction::Delay(duration),
        }
    }

    // ── SetOutput dispatches at accumulated clock time ───────────────────

    #[tokio::test]
    async fn set_output_dispatches_at_clock_time() {
        let ctrl = controller_with_program(RuntimeProgram::new(vec![set_output(
            "op-out",
            Duration::from_secs_f64(1.0),
        )]))
        .await;

        // clock 0.5s — before at_time: nothing dispatched
        ctrl.advance(0.5).await.expect("advance");
        assert!(
            ctrl.dispatched_events().await.is_empty(),
            "no events before at_time"
        );

        // clock 1.0s — at_time reached: SetOutput dispatched exactly at 1.0s
        ctrl.advance(0.5).await.expect("advance");
        let dispatched = ctrl.dispatched_events().await;
        assert_eq!(dispatched.len(), 1, "SetOutput dispatched at clock 1.0s");
        assert_eq!(
            dispatched[0].operation_id,
            OperationId("op-out".to_string())
        );
        assert!(matches!(
            dispatched[0].action,
            RuntimeAction::SetOutput { .. }
        ));
        assert_eq!(ctrl.clock_time().await, Duration::from_secs_f64(1.0));
    }

    // ── Delay freezes trajectory while clock advances ────────────────────

    #[tokio::test]
    async fn delay_freezes_trajectory_while_clock_advances() {
        let ctrl = controller_with_program(RuntimeProgram::new(vec![delay(
            "op-wait",
            Duration::from_secs_f64(1.0),
            Duration::from_millis(500),
        )]))
        .await;

        // Advance to the delay at_time (1.0s) in 0.25s ticks.
        // Trajectory is linear [0,0]→[1,1] over 2s: at traj 1.0 → joints 0.5.
        for _ in 0..4 {
            ctrl.advance(0.25).await.expect("advance");
        }
        let joints_at_delay = ctrl.robot_state().await.joints.positions.clone();
        assert_eq!(joints_at_delay, vec![0.5, 0.5]);
        assert_eq!(ctrl.clock_time().await, Duration::from_secs_f64(1.0));

        // clock 1.25s — inside the delay window: clock advances, joints hold.
        ctrl.advance(0.25).await.expect("advance");
        assert_eq!(ctrl.clock_time().await, Duration::from_secs_f64(1.25));
        assert_eq!(
            ctrl.robot_state().await.joints.positions,
            joints_at_delay,
            "trajectory must hold during delay while clock advances"
        );
        assert_eq!(
            ctrl.traj_time().await,
            Duration::from_secs_f64(1.0),
            "traj time must be frozen during delay"
        );

        // clock 1.5s — delay elapsed: trajectory resumes from held joints.
        ctrl.advance(0.25).await.expect("advance");
        assert_eq!(ctrl.clock_time().await, Duration::from_secs_f64(1.5));
        let resumed = ctrl.robot_state().await.joints.positions.clone();
        assert_eq!(
            resumed,
            vec![0.625, 0.625],
            "trajectory resumes from held joint state after delay"
        );
    }

    // ── Post-delay events fire at absolute at_time ───────────────────────

    #[tokio::test]
    async fn post_delay_event_dispatches_at_absolute_time() {
        // Delay at 1.0s (500ms) ends at 1.5s; SetOutput at 2.0s MUST fire at
        // exactly 2.0s from plan start — not 0.5s after the delay, and not
        // shifted by the delay duration.
        let ctrl = controller_with_program(RuntimeProgram::new(vec![
            delay(
                "op-wait",
                Duration::from_secs_f64(1.0),
                Duration::from_millis(500),
            ),
            set_output("op-out", Duration::from_secs_f64(2.0)),
        ]))
        .await;

        // Advance to clock 1.75s (0.25s after the delay ended).
        for _ in 0..7 {
            ctrl.advance(0.25).await.expect("advance");
        }
        assert_eq!(ctrl.clock_time().await, Duration::from_secs_f64(1.75));
        assert!(
            ctrl.dispatched_events().await.is_empty(),
            "SetOutput must NOT fire before its absolute at_time (2.0s)"
        );

        // clock 2.0s — the SetOutput fires exactly at its absolute at_time.
        ctrl.advance(0.25).await.expect("advance");
        let dispatched = ctrl.dispatched_events().await;
        assert_eq!(dispatched.len(), 1, "SetOutput fired at absolute 2.0s");
        assert_eq!(
            dispatched[0].operation_id,
            OperationId("op-out".to_string())
        );
        assert_eq!(ctrl.clock_time().await, Duration::from_secs_f64(2.0));
    }

    // ── Trajectory interpolation unchanged without events ────────────────

    #[tokio::test]
    async fn trajectory_unchanged_without_runtime_program() {
        // No program → dispatch is inert; linear interpolation behaves
        // exactly as before event dispatch existed (approval test).
        let mut ctrl = SimulationController::new(2);
        <SimulationController as RobotController>::connect(&mut ctrl)
            .await
            .expect("connect");
        <SimulationController as RobotController>::execute(&mut ctrl, test_plan())
            .await
            .expect("execute");

        ctrl.advance(0.5).await.expect("advance");
        assert_eq!(ctrl.robot_state().await.joints.positions, vec![0.25, 0.25]);

        ctrl.advance(0.5).await.expect("advance");
        assert_eq!(ctrl.robot_state().await.joints.positions, vec![0.5, 0.5]);

        ctrl.advance(1.0).await.expect("advance");
        assert_eq!(ctrl.robot_state().await.joints.positions, vec![1.0, 1.0]);
        assert_eq!(ctrl.traj_time().await, Duration::from_secs_f64(2.0));
    }
}
