use std::sync::Arc;

use tokio::sync::RwLock;

use thalos_engine::core::{
    execution::{
        plan::{ExecutionPlan, ExecutionSegment, ExecutionWaypoint, PlanInstruction},
        runtime::RuntimeProgram,
    },
    kinematics::{
        forward::{ForwardKinematics, result::FKResult},
        inverse::{DampedLeastSquaresSolver, IKConfig, IKGoal, IKSolver, result::IKResult},
    },
    models::{RobotModel, RobotRegistry},
    motion::segment::MotionSegment,
    robot::serial_chain::SerialChain,
    spatial::frame::FrameId,
};
use thalos_engine::planning::execution_plan_builder::ExecutionPlanBuilder;
use thalos_engine::planning::motion::program::{CompiledPlan, PlanningProgram};
use thalos_engine::planning::program_edit::ProgramEdit;

use thalos_engine::core::robot::adapter;
use thalos_importer::import_urdf;

use crate::backends::controller::RobotController;
use crate::backends::controller::simulation::SimulationController;
use crate::backends::manager::BackendManager;
use crate::commands::Command;
use crate::commands::handler::ExecutableCommand;
use crate::error::RuntimeError;
use crate::execution_boundary::velocity_retimer::VelocityRetimer;
use crate::execution_boundary::ExecutionSample as ProtocolSample;
use crate::motion_recorder::MotionRecorder;
use crate::plan::{ActiveMotionPlan, ExecutionMode, PlanState, SessionStatus};
use crate::services::command_history::{AppliedCommand, CommandMetrics};
use crate::session::{ExecutionSource, SessionManager};
use super::snapshot::{RuntimeSnapshot, TickDelta};
use crate::robot::{ActiveRobot, SceneRuntime};
use crate::telemetry::{
    ExecutionObserver, ExecutionRecorder, ExecutionSample as TelemetrySample, ExecutionTrace,
    TraceMetadata,
};

use std::time::Duration;

/// Estado de grabación de una ejecución en curso.
struct RecordingState {
    session_id: u64,
    recorder: MotionRecorder,
    execution_recorder: ExecutionRecorder,
    start_time: Duration,
    /// Execution mode of the running session (R1) — drives the repeat
    /// orchestration in the completion gate.
    mode: ExecutionMode,
    /// Current iteration, 1-based (R3). Incremented at each intermediate
    /// iteration completion; the terminal iteration is finalized as-is.
    iteration: u32,
    /// Total iterations from the mode (`None` for Once, R4).
    total_iterations: Option<u32>,
    /// Execution source captured at start (B). The synthetic upload-window
    /// delta needs it WITHOUT touching the controller — the background
    /// re-execute holds the controller write lock for the whole upload, and
    /// `BackendManager::active_source` reads it (would deadlock).
    source: ExecutionSource,
    /// v3: repeat is FIRMWARE-side (manifest repeat_count > 1, ESP32 loops
    /// back-to-back) — the host NEVER re-executes between passes. Iteration is
    /// derived from the overall progress on each tick; the completion gate
    /// fires only at the true end.
    firmware_repeat: bool,
    /// Repeat orchestration phase (B: async re-execute). `Uploading` while a
    /// background task re-executes the plan for the next iteration — ticks in
    /// that window return a synthetic delta instead of touching the controller
    /// or firing the completion gate again.
    repeat_phase: RepeatPhase,
    /// Async re-execute failure slot (B): set by the background task when the
    /// re-upload fails; the next tick drains it and fails the session with the
    /// real controller code (R5 parity with the old synchronous path).
    pending_reexecute_error: Option<PendingReexecuteError>,
}

/// Repeat orchestration phase (B). Gates completion detection so the stale
/// "Completed" state of the previous pass cannot re-trigger an upload while
/// the next iteration is already being uploaded by the background task.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RepeatPhase {
    /// No background re-execute in flight — the controller runs the current
    /// iteration (or the session is `Once`).
    Idle,
    /// A background task is uploading/starting the NEXT iteration. Completion
    /// detection is suppressed until the task resolves (`Idle` or failure).
    Uploading,
}

/// Async re-execute failure payload (B): the iteration to report on failure
/// (the COMPLETED iteration whose follow-up failed to start — parity with the
/// old synchronous path) plus the real `ControllerError`.
struct PendingReexecuteError {
    iteration: u32,
    source: crate::error::ControllerError,
}

/// Runtime IK solver configuration (spec `ik-config`): the runtime service
/// constructs its solver through the shared [`IKConfig`] type from these
/// preserved values (500/1e-6/0.1) — same set as plan analysis.
const IK_CONFIG: IKConfig = IKConfig {
    max_iterations: 500,
    tolerance: 1e-6,
    lambda: 0.1,
};

fn session_from_state(
    state: &Arc<crate::state::robot_state::RobotState>,
) -> Option<crate::plan::ExecutionSession> {
    use crate::state::robot_state::MotionMode;
    let progress = state.execution.progress;
    let status = match state.motion.mode {
        MotionMode::Idle if progress >= 1.0 => SessionStatus::Completed,
        MotionMode::Moving => SessionStatus::Running,
        MotionMode::Paused => SessionStatus::Paused,
        MotionMode::Stopping => SessionStatus::Cancelled,
        MotionMode::EStop => SessionStatus::Failed,
        _ => SessionStatus::Ready,
    };
    Some(crate::plan::ExecutionSession::derived(status, progress))
}

pub struct SceneService {
    runtime: RwLock<SceneRuntime>,
    manager: Arc<BackendManager>,
    sessions: Arc<SessionManager>,
    /// `Arc` so a background repeat re-execute (B) can update the phase/error
    /// slot without owning the whole service.
    recording: Arc<RwLock<Option<RecordingState>>>,
}

impl SceneService {
    pub fn new(manager: Arc<BackendManager>, model: RobotModel) -> Self {
        Self::with_session_manager(manager, model, Arc::new(SessionManager::new()))
    }

    pub fn with_session_manager(
        manager: Arc<BackendManager>,
        model: RobotModel,
        sessions: Arc<SessionManager>,
    ) -> Self {
        let chain = RobotRegistry::create_default(model);
        let dof = model.metadata().dof;
        let active_robot = ActiveRobot::new(Some(model), chain, vec![0.0; dof]);
        let robot_name = model.metadata().display_name.to_string();
        let runtime = SceneRuntime::new(active_robot, robot_name);

        Self {
            runtime: RwLock::new(runtime),
            manager,
            sessions,
            recording: Arc::new(RwLock::new(None)),
        }
    }

    fn compute_fk(chain: &SerialChain, joints: &[f64]) -> FKResult {
        let fk = ForwardKinematics::new(chain.clone());
        fk.evaluate(joints)
    }

    fn build_snapshot(runtime: &SceneRuntime, ik_result: Option<IKResult>) -> RuntimeSnapshot {
        let fk_result = Self::compute_fk(&runtime.active_robot.chain, &runtime.active_robot.joints);

        let scheduled_plan = runtime.scheduled_plan.as_ref().map(|sp| {
            ActiveMotionPlan::from_compiled_plan("preview", sp.clone())
        });

        RuntimeSnapshot {
            robot: runtime.active_robot.model,
            robot_source: runtime.robot_source.clone(),
            robot_name: runtime.robot_name.clone(),
            robot_id: runtime.robot_id.clone(),
            joints_meta: runtime.joints_meta.clone(),
            joints: runtime.active_robot.joints.clone(),
            chain: runtime.active_robot.chain.clone(),
            fk_result,
            ik_result,
            active_plan: runtime.active_plan.clone(),
            scheduled_plan,
            execution: None,
            active_tcp: runtime.active_tcp.clone(),
            generated_at: chrono::Utc::now(),
        }
    }

    /// Build a snapshot that includes execution state from the controller.
    ///
    /// Reads the controller's RobotState and derives ExecutionSession + joints.
    /// `repeat_meta` (mode + current iteration) is attached to the derived
    /// session when known — the controller state carries no repeat intent.
    async fn build_snapshot_with_execution(
        runtime: &tokio::sync::RwLock<SceneRuntime>,
        controller: &Arc<RwLock<dyn RobotController + Send + Sync>>,
        repeat_meta: Option<(ExecutionMode, u32)>,
    ) -> RuntimeSnapshot {
        let ctrl = controller.read().await;
        let state = ctrl.robot_state().await;
        let mut rt = runtime.write().await;
        rt.set_joints_from_state(&state.joints.positions);

        let fk_result = Self::compute_fk(&rt.active_robot.chain, &rt.active_robot.joints);
        // R4-001: the derived session carries the ACTIVE controller's source so
        // the badge reports Hardware/Esp32 when the ESP32 backend is connected.
        let source = ctrl.execution_source();
        let execution = session_from_state(&state).map(|exe| {
            let exe = exe.with_source(source);
            match repeat_meta {
                Some((mode, iteration)) => exe.with_repeat_state(mode, iteration),
                None => exe,
            }
        });

        // Sync the active_plan state with the execution
        if let Some(ref mut plan) = rt.active_plan {
            if let Some(ref exe) = execution {
                match exe.status {
                    SessionStatus::Running => plan.state = PlanState::Active,
                    SessionStatus::Paused => plan.state = PlanState::Paused,
                    SessionStatus::Completed => plan.state = PlanState::Completed,
                    SessionStatus::Cancelled => plan.state = PlanState::Cancelled,
                    SessionStatus::Failed => plan.state = PlanState::Failed,
                    SessionStatus::Ready => {}
                }
            }
        }

        let scheduled_plan = rt.scheduled_plan.as_ref().map(|sp| {
            ActiveMotionPlan::from_compiled_plan("preview", sp.clone())
        });

        RuntimeSnapshot {
            robot: rt.active_robot.model,
            robot_source: rt.robot_source.clone(),
            robot_name: rt.robot_name.clone(),
            robot_id: rt.robot_id.clone(),
            joints_meta: rt.joints_meta.clone(),
            joints: rt.active_robot.joints.clone(),
            chain: rt.active_robot.chain.clone(),
            fk_result,
            ik_result: None,
            active_plan: rt.active_plan.clone(),
            scheduled_plan,
            execution,
            active_tcp: rt.active_tcp.clone(),
            generated_at: chrono::Utc::now(),
        }
    }

    /// Read-only snapshot (no IK metadata).
    pub async fn snapshot(&self) -> Result<RuntimeSnapshot, RuntimeError> {
        let runtime = self.runtime.read().await;
        Ok(Self::build_snapshot(&runtime, None))
    }

    /// Execute a command (IK motion, FK set joints, etc.).
    pub async fn execute(&self, cmd: Command) -> Result<RuntimeSnapshot, RuntimeError> {
        let is_robot_change = matches!(cmd, Command::LoadRobot(_) | Command::LoadUrdfRobot { .. });

        let ik_result = {
            let mut runtime = self.runtime.write().await;
            cmd.execute(&mut *runtime)?
        };

        // If the robot changed, update the SimulationController with the new DOF
        if is_robot_change {
            let dof = {
                let rt = self.runtime.read().await;
                rt.active_robot.chain.dof_count()
            };
            let new_ctrl = Arc::new(RwLock::new(SimulationController::new(dof)))
                as Arc<RwLock<dyn RobotController + Send + Sync>>;
            // Silently replace — the manager handles disconnection
            let _ = self.manager.replace_controller(new_ctrl).await;
        }

        let runtime = self.runtime.read().await;
        Ok(Self::build_snapshot(&runtime, ik_result))
    }

    /// Parse URDF XML source, construct kinematics serial chain and joint metadata,
    /// build Command::LoadUrdfRobot and execute it on the runtime.
    pub async fn load_urdf_robot(&self, urdf_source: &str) -> Result<RuntimeSnapshot, RuntimeError> {
        use super::snapshot::JointMeta;

        let robot = import_urdf(urdf_source).map_err(|e| RuntimeError::InvalidUrdf {
            message: format!("Invalid URDF: {e}"),
        })?;

        let name = robot.name.clone();
        let chain = adapter::auto(&robot).map_err(|e| RuntimeError::UrdfChainError {
            message: format!("Cannot build chain: {e}"),
        })?;

        let joints_meta: Vec<JointMeta> = robot
            .bfs_joints()
            .unwrap_or_default()
            .iter()
            .filter(|j| !j.kind.is_fixed())
            .map(|j| JointMeta {
                name: j.name.clone(),
                kind: j.kind.to_string(),
                min: j.limits.map(|l| l.min),
                max: j.limits.map(|l| l.max),
            })
            .collect();

        let cmd = Command::LoadUrdfRobot {
            name,
            joints_meta,
            chain,
            robot,
            robot_id: urdf_robot_id(urdf_source),
        };

        self.execute(cmd).await
    }

    pub async fn solve_ik(
        &self,
        frame: FrameId,
        goal: IKGoal,
    ) -> Result<(Vec<f64>, IKResult), RuntimeError> {
        let runtime = self.runtime.read().await;
        let fk = ForwardKinematics::new(runtime.active_robot.chain.clone());
        let solver = DampedLeastSquaresSolver::from_config(fk, frame, IK_CONFIG);
        let q0 = runtime.active_robot.joints.clone();
        let result = solver.solve(&q0, goal)?;
        Ok((result.q.clone(), result))
    }

    // ── Program management ──

    /// Compile and store a motion program for preview.
    ///
    /// Accepts the `RuntimeProgram` (absolute `at_time` events) alongside the
    /// `CompiledPlan` (PR 3): the compiled trajectory is stored for preview
    /// and the event program is loaded into the controller so the tick loop
    /// dispatches `SetOutput`/`Delay` at their absolute times.
    pub async fn schedule_program(
        &self,
        compiled: CompiledPlan,
        runtime: RuntimeProgram,
    ) -> Result<RuntimeSnapshot, RuntimeError> {
        {
            let mut runtime = self.runtime.write().await;
            runtime.schedule_plan(compiled);
        }
        // Hand the event timeline to the controller (no-op for backends
        // that do not dispatch runtime events).
        if let Some(ctrl) = self.manager.get_controller().await {
            let mut c = ctrl.write().await;
            c.load_runtime_program(runtime).await?;
        }
        let runtime = self.runtime.read().await;
        Ok(Self::build_snapshot(&runtime, None))
    }

    pub async fn set_program_provenance(
        &self,
        program_id: impl Into<String>,
        revision: u64,
        fingerprint: impl Into<String>,
    ) {
        let mut runtime = self.runtime.write().await;
        runtime.set_program_provenance(program_id, revision, fingerprint);
    }

    /// Load a compiled plan for preview only (leaves active_plan = None).
    pub async fn preview_plan(
        &self,
        compiled: CompiledPlan,
    ) -> Result<RuntimeSnapshot, RuntimeError> {
        let mut runtime = self.runtime.write().await;
        runtime.preview_plan(compiled);
        Ok(Self::build_snapshot(&runtime, None))
    }

    /// Explicitly activate the previewed/scheduled plan for execution.
    pub async fn activate_plan(&self) -> Result<RuntimeSnapshot, RuntimeError> {
        let mut runtime = self.runtime.write().await;
        runtime.activate_plan()?;
        Ok(Self::build_snapshot(&runtime, None))
    }

    // ── Scene write-back (PR4 — design-first, D4/D5) ──

    /// Toggle the scene-writeback feature flag (design D5).
    ///
    /// OFF by default. Enabling it is the per-environment rollout step after
    /// integration tests pass. Flipping it back OFF restores the read-only
    /// behavior with zero code changes.
    pub async fn set_scene_writeback(&self, enabled: bool) {
        let mut runtime = self.runtime.write().await;
        runtime.set_scene_writeback(enabled);
    }

    /// Configure the command-history capacity (spec command-endpoints
    /// "History Cap"). Honors the optional `THALOS_HISTORY_CAP` env var read
    /// at the binary entry point; defaults to [`DEFAULT_HISTORY_CAP`].
    pub async fn set_history_cap(&self, cap: usize) {
        let mut runtime = self.runtime.write().await;
        runtime.with_history_cap(cap);
    }

    /// Apply a recompiled plan back to the runtime (design D4).
    ///
    /// Write-back path for `POST /plan/commands/apply`:
    /// 1. `SceneRuntime::replace_active_plan` — feature-flagged, snapshot +
    ///    atomic restore on failure.
    /// 2. On success, the applied command, its pre-computed inverse and the
    ///    plan metrics are recorded (D6) so PR5's `undo` can pop it in O(1)
    ///    and report the restored health without re-analysis.
    /// 3. `applied_program` links the entry to the program the apply wrote
    ///    back (R4-001) — undo refuses a stale inverse.
    ///
    /// `trajectory_to_waypoints` reads `scheduled_plan` first, so the new
    /// plan propagates to execution automatically.
    pub async fn apply_compiled_plan(
        &self,
        compiled: CompiledPlan,
        command: ProgramEdit,
        inverse: ProgramEdit,
        metrics: CommandMetrics,
        applied_program: Vec<thalos_engine::core::motion::segment::MotionSegment>,
    ) -> Result<RuntimeSnapshot, RuntimeError> {
        let mut runtime = self.runtime.write().await;
        runtime.replace_active_plan(compiled)?;
        runtime.record_applied_command(command, inverse, metrics, applied_program);
        Ok(Self::build_snapshot(&runtime, None))
    }

    /// Number of applied commands with stored inverses (undo history size).
    pub async fn history_len(&self) -> usize {
        let runtime = self.runtime.read().await;
        runtime.history_len()
    }

    /// Peek the last applied command together with the history version (PR2).
    ///
    /// The `(entry, version)` pair is read under a SINGLE read lock — the undo
    /// flow recompiles against `entry` and later commits with `version` as the
    /// expected value, closing the TOCTOU window between peek and commit.
    pub async fn last_applied_with_version(&self) -> (Option<AppliedCommand>, u64) {
        let runtime = self.runtime.read().await;
        let (entry, version) = runtime.last_applied_with_version();
        (entry.cloned(), version)
    }

    /// Undo the last applied command (design D6): pop (O(1)) + write back the
    /// recompiled inverse-applied plan WITHOUT recording a new entry.
    ///
    /// Atomic and feature-flagged via `SceneRuntime::undo_plan` (D4/D5). The
    /// R4-001 stale guard lives in the runtime: `current_program` is the
    /// program reconstructed from the active plan and must match the entry's
    /// `applied_program`, else `StaleUndo` — no mutation, history intact.
    /// PR2: `expected_version` is the history version read atomically with the
    /// last entry (`last_applied_with_version`); the runtime re-validates it
    /// under the write lock BEFORE any mutation (`UndoVersionMismatch`).
    /// The popped entry is returned so the API can report the restored
    /// metrics; the snapshot carries the restored active plan.
    pub async fn undo_compiled_plan(
        &self,
        current_program: &PlanningProgram,
        compiled: CompiledPlan,
        expected_version: u64,
    ) -> Result<(AppliedCommand, RuntimeSnapshot), RuntimeError> {
        let mut runtime = self.runtime.write().await;
        let popped = runtime.undo_plan(current_program, compiled, expected_version)?;
        Ok((popped, Self::build_snapshot(&runtime, None)))
    }

    /// Build the `ExecutionPlan` for the current runtime state — the REAL
    /// timestamp-carrying execution IR handed to `RobotController::execute`.
    ///
    /// Prefers `scheduled_plan` (multi-segment compiled programs): the pure
    /// [`ExecutionPlanBuilder`] maps every `TrajectoryPoint` → waypoint with
    /// its absolute timestamp and every `PlannedSegment` → `ExecutionSegment`
    /// 1:1 (MoveJ/MoveL/MoveLPosition per `PlannedSegment.source`). Falls
    /// back to `active_plan` (single-shot moves like PlanAndMoveJ, and the
    /// compiled-plan mirror) with an inline trajectory → waypoint mapping
    /// that ALSO preserves `tp.timestamp()`; segments map 1:1 when present,
    /// else a single MoveJ segment covers every waypoint.
    ///
    /// Returns `None` when no plan is loaded, the trajectory is empty, or
    /// the scheduled-plan builder fails — the caller's `has_wps` guard then
    /// skips the controller call (Once-without-plan behavior preserved).
    fn build_execution_plan(runtime: &SceneRuntime) -> Option<ExecutionPlan> {
        if runtime.active_plan.is_none() {
            return None;
        }
        let base_plan = if let Some(ref compiled) = runtime.scheduled_plan {
            let mut plan = ExecutionPlanBuilder::build(compiled).ok()?;
            if plan.segments.is_empty() && !plan.waypoints.is_empty() {
                let n = plan.waypoints.len();
                plan.segments.push(ExecutionSegment {
                    index: 0,
                    planned_segment_index: 0,
                    instruction: PlanInstruction::MoveJ,
                    waypoint_range: 0..n,
                });
            }
            plan
        } else if let Some(ref active) = runtime.active_plan {
            let traj = &active.trajectory;
            if traj.is_empty() {
                return None;
            }
            let waypoints: Vec<ExecutionWaypoint> = traj
                .waypoints()
                .iter()
                .map(|tp| ExecutionWaypoint {
                    joints: tp.joints().to_vec(),
                    timestamp: tp.timestamp(),
                })
                .collect();
            let n = waypoints.len();
            let segments: Vec<ExecutionSegment> = match &active.segments {
                Some(segments) if !segments.is_empty() => segments
                    .iter()
                    .enumerate()
                    .map(|(idx, seg)| ExecutionSegment {
                        index: idx,
                        planned_segment_index: idx,
                        instruction: match &seg.source {
                            MotionSegment::MoveJ { .. } => PlanInstruction::MoveJ,
                            MotionSegment::MoveL { .. } => PlanInstruction::MoveL,
                            MotionSegment::MoveLPosition { .. } => PlanInstruction::MoveL,
                        },
                        waypoint_range: seg.waypoint_range.clone(),
                    })
                    .collect(),
                _ => vec![ExecutionSegment {
                    index: 0,
                    planned_segment_index: 0,
                    instruction: PlanInstruction::MoveJ,
                    waypoint_range: 0..n,
                }],
            };
            ExecutionPlan {
                waypoints,
                segments,
                duration: traj.duration(),
                repeat_count: 1,
                program_id: None,
                program_revision: None,
                source_fingerprint: None,
                robot_id: None,
            }
        } else {
            return None;
        };

        let plan = if let Some(ref active) = runtime.active_plan {
            if active.program_revision.is_some() || active.source_fingerprint.is_some() {
                base_plan.with_provenance(
                    active.program_id.clone().unwrap_or_default(),
                    active.program_revision.unwrap_or(0),
                    active.source_fingerprint.clone().unwrap_or_default(),
                    Some(runtime.robot_id.clone()),
                )
            } else {
                base_plan
            }
        } else {
            base_plan
        };

        Some(VelocityRetimer::retime(&plan))
    }

    pub async fn start_execution(&self) -> Result<RuntimeSnapshot, RuntimeError> {
        self.start_execution_with_mode(ExecutionMode::Once).await
    }

    /// Start execution with an explicit mode (R1/R7).
    ///
    /// S8: `Repeat` requires a loaded plan — without one the request is
    /// refused with [`RuntimeError::NoActivePlan`] (4xx) BEFORE any controller
    /// or session work. `Once` preserves the legacy behavior: starting
    /// without a loaded plan still succeeds (existing tests depend on it).
    pub async fn start_execution_with_mode(
        &self,
        mode: ExecutionMode,
    ) -> Result<RuntimeSnapshot, RuntimeError> {
        {
            let runtime = self.runtime.read().await;
            if runtime.active_plan.is_none() && (runtime.scheduled_plan.is_some() || matches!(mode, ExecutionMode::Repeat { .. }) || runtime.active_program_revision.is_some()) {
                return Err(RuntimeError::NoActivePlan);
            }
            if let (Some(expected_rev), Some(expected_fp)) = (
                runtime.active_program_revision,
                runtime.active_source_fingerprint.as_ref(),
            ) {
                if runtime.active_plan.is_none() {
                    return Err(RuntimeError::NoActivePlan);
                }
                if let Some(plan) = Self::build_execution_plan(&runtime) {
                    if plan.is_stale_for(expected_rev, expected_fp) {
                        if plan.program_revision != Some(expected_rev) {
                            return Err(RuntimeError::StalePlanRevision {
                                expected: expected_rev,
                                actual: plan.program_revision.unwrap_or(0),
                            });
                        } else {
                            return Err(RuntimeError::StalePlanFingerprint {
                                expected: expected_fp.clone(),
                                actual: plan.source_fingerprint.clone().unwrap_or_default(),
                            });
                        }
                    }
                }
            }
        }

        // R3-001: with NO active controller (e.g. the hardware backend is
        // active but was never connected, or the device was disconnected while
        // active) start must fail EXPLICITLY with `not_connected` — a silent
        // 200 made the frontend report 'running' until the first tick dropped
        // it to 'idle' with no error and no CTA.
        let ctrl = self
            .manager
            .get_controller()
            .await
            .ok_or_else(|| RuntimeError::ControllerFailed {
                source: crate::error::ControllerError::NotConnected,
            })?;
        // v3 (firmware-side repeat): for backends that REPEAT INTERNALLY
        // (ESP32 — `repeat_count` in the manifest, loops back-to-back with NO
        // re-upload between passes), a `Repeat` mode is baked into the plan.
        // Simulation/Replay keep repeat_count=1 and repeat via the host
        // completion gate (B).
        let supports_firmware_repeat = {
            let c = ctrl.read().await;
            c.capabilities().firmware_repeat
        };
        let firmware_repeat = supports_firmware_repeat
            && matches!(mode, ExecutionMode::Repeat { count } if count > 1);
        {
            let mut plan = {
                let runtime = self.runtime.read().await;
                Self::build_execution_plan(&runtime)
            };
            if firmware_repeat {
                if let Some(ref mut p) = plan {
                    p.repeat_count = match mode {
                        ExecutionMode::Repeat { count } => count,
                        ExecutionMode::Once => 1,
                    };
                }
            }
            let (waypoints, duration) = match &plan {
                Some(p) => (
                    p.waypoints
                        .iter()
                        .map(|wp| wp.joints.clone())
                        .collect::<Vec<Vec<f64>>>(),
                    p.duration,
                ),
                None => (Vec::new(), 0.0),
            };

            // Execute on controller FIRST (before creating session).
            // If execution fails, no orphaned session is created.
            let has_wps = !waypoints.is_empty() && duration > 0.0;
            tracing::info!(
                mode = ?mode,
                waypoints = waypoints.len(),
                duration_s = duration,
                %has_wps,
                "start_execution — prepared"
            );
            if !has_wps {
                tracing::warn!(
                    waypoints = waypoints.len(),
                    duration_s = duration,
                    "start_execution — NO wire traffic: empty trajectory or zero duration"
                );
            }
            if has_wps {
                let plan = plan.expect("has_wps implies an ExecutionPlan");
                let mut c = ctrl.write().await;
                c.execute(plan).await?;
            }

            // Only now register the session — execution already started.
            let robot_name = {
                let runtime = self.runtime.read().await;
                runtime.robot_name.clone()
            };
            // R4-001: the source reflects the ACTIVE controller (Simulation vs
            // Hardware/Esp32), not a hardcoded value — the badge must be able to
            // say Hardware when the ESP32 backend is connected.
            let source = self.manager.active_source().await;
            let recording_source = source.clone();
            let wps_for_recorder = waypoints.clone();
            let joint_count = wps_for_recorder.first().map(|w| w.len()).unwrap_or(0);
            let robot_name_for_session = robot_name.clone();
            let session = self
                .sessions
                .register(
                    source.clone(),
                    "plan-exec".into(),
                    duration,
                    joint_count,
                    robot_name_for_session,
                    mode,
                )
                .await;

            let mut recorder = MotionRecorder::new();
            if !wps_for_recorder.is_empty() {
                recorder.set_target_waypoints(wps_for_recorder);
            }
            recorder.start(std::time::Duration::from_secs_f64(duration));

            let exec_metadata = TraceMetadata {
                session_id: session.id.to_string(),
                plan_id: session.plan_id.clone(),
                source: source,
                robot_name: robot_name.clone(),
                joint_count,
                duration: std::time::Duration::from_secs_f64(duration),
                sample_rate: 0.0,
            };
            let mut exec_recorder = ExecutionRecorder::new(exec_metadata);
            let ts = std::time::Duration::ZERO;
            ExecutionObserver::on_execution_started(&mut exec_recorder, ts);

            *self.recording.write().await = Some(RecordingState {
                session_id: session.id,
                recorder,
                execution_recorder: exec_recorder,
                start_time: std::time::Duration::ZERO,
                mode,
                iteration: 1,
                total_iterations: mode.total_iterations(),
                repeat_phase: RepeatPhase::Idle,
                pending_reexecute_error: None,
                source: recording_source,
                firmware_repeat,
            });
        }

        let repeat_meta = self.repeat_state().await;
        Ok(Self::build_snapshot_with_execution(&self.runtime, &ctrl, repeat_meta).await)
    }

    /// Current repeat state from the active recording — `(mode, iteration)`.
    ///
    /// `None` when no recording is active (Once sessions or post-finalize).
    async fn repeat_state(&self) -> Option<(ExecutionMode, u32)> {
        let recording = self.recording.read().await;
        recording.as_ref().map(|r| (r.mode, r.iteration))
    }

    /// Seek the active controller to a position (fraction 0.0–1.0).
    ///
    /// Only meaningful for replay/simulation backends.
    pub async fn seek_execution(&self, position: f64) -> Result<RuntimeSnapshot, RuntimeError> {
        if let Some(ctrl) = self.manager.get_controller().await {
            let ctrl_guard = ctrl.read().await;
            ctrl_guard
                .seek(position)
                .await
                .map_err(|e| RuntimeError::ControllerFailed { source: e })?;
            drop(ctrl_guard);
            let repeat_meta = self.repeat_state().await;
            return Ok(Self::build_snapshot_with_execution(&self.runtime, &ctrl, repeat_meta).await);
        }
        let runtime = self.runtime.read().await;
        Ok(Self::build_snapshot(&runtime, None))
    }

    pub async fn pause_execution(&self) -> Result<RuntimeSnapshot, RuntimeError> {
        if let Some(ctrl) = self.manager.get_controller().await {
            {
                let mut c = ctrl.write().await;
                c.pause().await?;
            } // write lock dropped
            let repeat_meta = self.repeat_state().await;
            return Ok(Self::build_snapshot_with_execution(&self.runtime, &ctrl, repeat_meta).await);
        }
        let runtime = self.runtime.read().await;
        Ok(Self::build_snapshot(&runtime, None))
    }

    pub async fn resume_execution(&self) -> Result<RuntimeSnapshot, RuntimeError> {
        if let Some(ctrl) = self.manager.get_controller().await {
            {
                let mut c = ctrl.write().await;
                c.resume().await?;
            } // write lock dropped
            let repeat_meta = self.repeat_state().await;
            return Ok(Self::build_snapshot_with_execution(&self.runtime, &ctrl, repeat_meta).await);
        }
        let runtime = self.runtime.read().await;
        Ok(Self::build_snapshot(&runtime, None))
    }

    pub async fn cancel_execution(&self) -> Result<RuntimeSnapshot, RuntimeError> {
        if let Some(ctrl) = self.manager.get_controller().await {
            // Capture the repeat state BEFORE finalizing — the recording is
            // consumed by finalize_recording and the DTO must still show the
            // iteration at cancel time (R12).
            let repeat_meta = self.repeat_state().await;
            {
                let mut c = ctrl.write().await;
                c.stop().await?;
            }
            // Finalize recording as Cancelled if active
            self.finalize_recording(Some(crate::plan::SessionStatus::Cancelled))
                .await;
            return Ok(Self::build_snapshot_with_execution(&self.runtime, &ctrl, repeat_meta).await);
        }
        let runtime = self.runtime.read().await;
        Ok(Self::build_snapshot(&runtime, None))
    }

    pub async fn reset_execution(&self) -> Result<RuntimeSnapshot, RuntimeError> {
        // Stop the active controller FIRST — a hardware run may still be in
        // progress on the device. Resetting must abort it, otherwise the next
        // Start finds the firmware mid-EXECUTING and has to go through the
        // NOT_IDLE STOP+retry recovery every time (mirrors cancel_execution).
        if let Some(ctrl) = self.manager.get_controller().await {
            let mut c = ctrl.write().await;
            c.stop().await?;
        }
        // Finalize any active recording as Cancelled first
        self.finalize_recording(Some(crate::plan::SessionStatus::Cancelled))
            .await;

        // Reset the plan state to Created (without starting execution)
        {
            let mut runtime = self.runtime.write().await;
            if let Some(ref mut plan) = runtime.active_plan {
                plan.state = crate::plan::PlanState::Created;
                plan.started_at = None;
                plan.completed_at = None;
            }
        }

        // Read-only snapshot (no controller execution)
        let runtime = self.runtime.read().await;
        Ok(Self::build_snapshot(&runtime, None))
    }

    /// Assemble a telemetry [`ExecutionTrace`] from raw protocol samples
    /// (`execution_boundary::ExecutionSample`), as required by the pinned
    /// trace-storage decision: telemetry samples carry `timestamp` from µs,
    /// empty velocities/accelerations, zeroed TCP, and
    /// `progress = seconds / plan_duration`.
    fn assemble_execution_trace(
        samples: &[ProtocolSample],
        plan_duration: f64,
        session_id: u64,
        plan_id: String,
        robot_name: String,
    ) -> ExecutionTrace {
        let joint_count = samples.first().map(|s| s.joints.len()).unwrap_or(0);
        let metadata = TraceMetadata {
            session_id: session_id.to_string(),
            plan_id,
            source: ExecutionSource::Hardware,
            robot_name,
            joint_count,
            duration: std::time::Duration::from_secs_f64(plan_duration),
            sample_rate: 0.0,
        };
        let mut trace = ExecutionTrace::new(metadata);
        let duration = plan_duration.max(1.0);
        for s in samples {
            let seconds = s.timestamp_us as f64 / 1_000_000.0;
            trace.push_sample(TelemetrySample {
                timestamp: std::time::Duration::from_micros(s.timestamp_us),
                joints: s.joints.clone(),
                velocities: vec![],
                accelerations: vec![],
                tcp_pose: [0.0; 7],
                tcp_velocity: [0.0; 6],
                tracking_error: None,
                progress: seconds / duration,
            });
        }
        trace
    }

    /// Finalizar la grabación activa (si existe) y guardar el trace.
    ///
    /// Si `terminal_status` es `Some`, usa ese estado en vez de `Completed`.
    /// Por defecto (`None`), usa `Completed`.
    async fn finalize_recording(&self, terminal_status: Option<crate::plan::SessionStatus>) {
        let mut recording = self.recording.write().await;
        if let Some(mut rec) = recording.take() {
            let trace = rec.recorder.stop();
            let ts = std::time::Duration::ZERO;
            rec.execution_recorder.on_execution_finished(ts);
            let exec_trace = rec.execution_recorder.trace();
            let status = terminal_status.unwrap_or(crate::plan::SessionStatus::Completed);
            let _ = self
                .sessions
                .complete_with_status(rec.session_id, trace, status)
                .await;
            if let Some(et) = exec_trace {
                self.sessions.save_execution_trace(rec.session_id, et).await;
            }
        }
    }

    // ── Tick ──

    /// Advance execution by `dt` seconds via the controller, then build
    /// a TickDelta from the resulting RobotState.
    ///
    /// Also records the state into the active MotionRecorder if recording
    /// is in progress, and finalizes the session when execution completes.
    pub async fn tick_execution_delta(&self, dt: f64) -> Result<TickDelta, RuntimeError> {
        // 0. Async re-execute (repeat) coordination (B) — runs BEFORE any
        // controller access: a background re-execute holds the controller
        // write lock for the WHOLE serial upload (10-17s on large plans).
        // Ticks in that window must NOT block on the controller read (would
        // blow the HTTP timeout), must NOT fire the completion gate again (the
        // stale Completed state of the previous pass would re-execute twice),
        // and must NOT record idle samples into the open iteration trace.
        let uploading_repeat: Option<(ExecutionMode, u32, ExecutionSource)> = {
            let mut recording = self.recording.write().await;
            if let Some(ref mut rec_state) = *recording {
                // An async re-execute failure surfaces on the NEXT tick with
                // the real controller code (R5 parity with the old synchronous
                // path — the failure is attributed to the COMPLETED iteration
                // whose follow-up failed to start).
                if let Some(err) = rec_state.pending_reexecute_error.take() {
                    let trace = rec_state.recorder.stop();
                    rec_state
                        .execution_recorder
                        .on_execution_finished(Duration::ZERO);
                    let exec_trace = rec_state.execution_recorder.trace();
                    self.sessions
                        .complete_with_status(
                            rec_state.session_id,
                            trace,
                            SessionStatus::Failed,
                        )
                        .await;
                    self.sessions
                        .set_iteration(rec_state.session_id, err.iteration)
                        .await;
                    if let Some(et) = exec_trace {
                        self.sessions
                            .save_execution_trace(rec_state.session_id, et)
                            .await;
                    }
                    *recording = None;
                    return Err(RuntimeError::ControllerFailed { source: err.source });
                }
                if matches!(rec_state.repeat_phase, RepeatPhase::Uploading) {
                    Some((
                        rec_state.mode,
                        rec_state.iteration,
                        rec_state.source.clone(),
                    ))
                } else {
                    None
                }
            } else {
                None
            }
        };
        if let Some((mode, iteration, source)) = uploading_repeat {
            return self.synthetic_uploading_delta(mode, iteration, source).await;
        }

        // 1. Advance simulation time via the controller trait.
        // R4-001: a real failure (e.g. `ConnectionLost`) from `advance` must
        // PROPAGATE as an execution failure — not be swallowed — so the code
        // reaches the frontend and the session can be marked failed. The only
        // ignorable case is `UnsupportedCapability`: real hardware backends
        // implement `advance` as the default `Err(UnsupportedCapability)` — time
        // is real, the tick reads state back below.
        if let Some(ctrl) = self.manager.get_controller().await {
            let ctrl_guard = ctrl.read().await;
            if let Err(e) = ctrl_guard.advance(dt).await {
                if !matches!(e, crate::error::ControllerError::UnsupportedCapability) {
                    return Err(RuntimeError::ControllerFailed { source: e });
                }
            }
        }

        // 2. Read state back & update runtime joints
        if let Some(ctrl) = self.manager.get_controller().await {
            let state = ctrl.read().await.robot_state().await;
            let mut runtime = self.runtime.write().await;
            runtime.set_joints_from_state(&state.joints.positions);

            let plan_duration = runtime
                .active_plan
                .as_ref()
                .map(|p| p.trajectory.duration())
                .unwrap_or(0.0);

            // Re-execution payload for intermediate repeat iterations — the
            // same ExecutionPlan the session started with. Captured HERE
            // while the runtime write guard is held: the tokio RwLock is
            // NOT reentrant, so reading the runtime again inside the
            // recording block below would deadlock this task.
            let re_execute_payload = Self::build_execution_plan(&runtime);

            // Active source determines progress UNITS (S3.6 / RISK-1):
            // hardware backends populate `execution.progress` in SECONDS
            // (esp32 map_firmware_state: fraction × plan_duration); simulation
            // keeps a 0..1 fraction.
            let active_source = self.manager.active_source().await;

            // Hoisted completion detection — evaluated on EVERY tick, outside
            // the recording gate, so the hardware execution trace is drained
            // and saved even when recording is not active (S3.6).
            //
            // RISK-1 / REL-01: for Hardware the gate compares SECONDS against
            // the active plan's duration (`>= plan_duration.max(1.0)`) — the
            // old fraction threshold (`>= 1.0`) finalized mid-run on any plan
            // > 1s and dropped the trace at true completion. Simulation keeps
            // the historical fraction/Idle gate.
            //
            // REL-03 / RES-06: EStop is a TERMINAL condition — it must
            // finalize the session (as Failed), never leave it Running.
            let estop = matches!(
                state.motion.mode,
                crate::state::robot_state::MotionMode::EStop
            );
            let completed = estop
                || match active_source {
                    ExecutionSource::Hardware => {
                        state.execution.progress >= plan_duration.max(1.0)
                    }
                    _ => {
                        state.execution.progress >= 1.0
                            || matches!(
                                state.motion.mode,
                                crate::state::robot_state::MotionMode::Idle
                            )
                    }
                };

            // Backend-conditional recording timestamp (S3.6).
            let progress_in_seconds = match active_source {
                ExecutionSource::Hardware => state.execution.progress,
                _ => state.execution.progress * plan_duration.max(1.0),
            };

            let mut completed_session_id: Option<u64> = None;
            // Repeat state to attach to the tick delta. Captured from the
            // recording at the START of the block; the intermediate branch
            // refreshes it AFTER incrementing so the delta for the tick that
            // finished iteration k reports the NEXT iteration (k+1). When the
            // recording is finalized this tick, this holds the FINAL value.
            let mut repeat_meta: Option<(ExecutionMode, u32)> = None;
            // Set ONLY when THIS tick finalized the final iteration as
            // Completed — gates the hardware execution-trace drain (S2, R6).
            let mut terminal_completion = false;
            // Set when THIS tick completed an INTERMEDIATE iteration and
            // re-executed the plan for k+1 — the delta built from the
            // pre-gate state must report Running(k+1), not the stale
            // Completed(k) that would stop the frontend tick loop (R8).
            let mut intermediate_restart = false;
            {
                // 3. Record the current state if recording
                let mut recording = self.recording.write().await;
                if let Some(ref mut rec_state) = *recording {
                    completed_session_id = Some(rec_state.session_id);
                    // v3 (firmware-side repeat): the ESP32 loops internally —
                    // derive the CURRENT pass from the OVERALL progress (so the
                    // badge advances and the gate sees iteration == total at the
                    // true end) and scale the recorder clock by the total so the
                    // trace stays monotonic across ALL passes.
                    if rec_state.firmware_repeat {
                        let total = rec_state.total_iterations.unwrap_or(1).max(1) as f64;
                        let plan_d = plan_duration.max(1.0);
                        let pass =
                            (progress_in_seconds / plan_d * total).floor() as u32 + 1;
                        rec_state.iteration = pass.min(total as u32).max(1);
                        self.sessions
                            .set_iteration(rec_state.session_id, rec_state.iteration)
                            .await;
                    }
                    repeat_meta = Some((rec_state.mode, rec_state.iteration));
                    let timestamp = {
                        let mut secs = progress_in_seconds;
                        if rec_state.firmware_repeat {
                            secs *= rec_state.total_iterations.unwrap_or(1).max(1) as f64;
                        }
                        let elapsed = rec_state.start_time
                            + std::time::Duration::from_secs_f64(secs);
                        elapsed
                    };
                    rec_state.recorder.record(timestamp, &state);
                    rec_state.execution_recorder.on_sample(timestamp, &state);

                    // Check if execution completed — finalize recording.
                    // REL-03 / RES-06: EStop finalizes as FAILED, not
                    // Completed — a stopped-by-error run must not report done.
                    //
                    // Repeat orchestration (R3/R4/R6, S1/S2):
                    // - EStop (any iteration) → Failed(iteration=k), never
                    //   re-executes (R5/R12), no execution trace (S2).
                    // - intermediate completion (iteration < total) →
                    //   keep BOTH recorders open (single accumulated trace,
                    //   NF3), increment the iteration, re-execute the plan.
                    //   A failed re-execute finalizes Failed(iteration) and
                    //   propagates the error (R5).
                    // - final completion (iteration == total) → Completed,
                    //   close recorders, persist the single trace, and drain
                    //   the hardware execution trace exactly once (R6/NF3).
                    if completed {
                        let iteration = rec_state.iteration;
                        let is_final = rec_state
                            .total_iterations
                            .map_or(true, |total| iteration >= total);

                        if estop {
                            let trace = rec_state.recorder.stop();
                            rec_state
                                .execution_recorder
                                .on_execution_finished(timestamp);
                            let exec_trace = rec_state.execution_recorder.trace();
                            self.sessions
                                .complete_with_status(
                                    rec_state.session_id,
                                    trace,
                                    SessionStatus::Failed,
                                )
                                .await;
                            self.sessions
                                .set_iteration(rec_state.session_id, iteration)
                                .await;
                            if let Some(et) = exec_trace {
                                self.sessions
                                    .save_execution_trace(rec_state.session_id, et)
                                    .await;
                            }
                            *recording = None;
                        } else if !is_final && !rec_state.firmware_repeat {
                            // Intermediate iteration: keep the recorders open,
                            // advance the base timestamp so samples stay
                            // monotonic, increment, then re-execute.
                            rec_state.iteration += 1;
                            repeat_meta = Some((rec_state.mode, rec_state.iteration));
                            rec_state.start_time +=
                                std::time::Duration::from_secs_f64(progress_in_seconds);
                            self.sessions
                                .set_iteration(rec_state.session_id, rec_state.iteration)
                                .await;

                            // B: the re-execute (full serial upload for the
                            // next iteration) runs in a BACKGROUND task — it
                            // must never block the tick request. While it is
                            // in flight the phase is `Uploading`: the stale
                            // Completed state of the previous pass cannot
                            // re-fire this gate, and ticks return a synthetic
                            // Running(k+1) delta without touching the
                            // controller (which the task holds).
                            let fail_iteration = iteration;
                            rec_state.repeat_phase = RepeatPhase::Uploading;
                            let re_execute = match (
                                self.manager.get_controller().await,
                                re_execute_payload,
                            ) {
                                (Some(ctrl), Some(plan)) => {
                                    let recording = self.recording.clone();
                                    tokio::spawn(async move {
                                        let result = {
                                            let mut c = ctrl.write().await;
                                            c.execute(plan).await
                                        };
                                        let mut rec = recording.write().await;
                                        if let Some(ref mut rs) = *rec {
                                            match result {
                                                Ok(()) => {
                                                    rs.repeat_phase = RepeatPhase::Idle
                                                }
                                                Err(source) => {
                                                    rs.pending_reexecute_error =
                                                        Some(PendingReexecuteError {
                                                            iteration: fail_iteration,
                                                            source,
                                                        })
                                                }
                                            }
                                        }
                                    });
                                    Ok(())
                                }
                                // Unreachable: Repeat start is gated on a
                                // loaded plan (S8). Fail loud rather than
                                // silently dropping the iteration.
                                _ => Err(crate::error::ControllerError::NotConnected),
                            };
                            if let Err(e) = re_execute {
                                // Re-execution failed synchronously (no
                                // controller / no plan) → the session fails at
                                // the current iteration; the error propagates
                                // so the frontend sees the real code (R5).
                                let trace = rec_state.recorder.stop();
                                rec_state
                                    .execution_recorder
                                    .on_execution_finished(timestamp);
                                self.sessions
                                    .complete_with_status(
                                        rec_state.session_id,
                                        trace,
                                        SessionStatus::Failed,
                                    )
                                    .await;
                                self.sessions
                                    .set_iteration(rec_state.session_id, iteration)
                                    .await;
                                *recording = None;
                                return Err(RuntimeError::ControllerFailed { source: e });
                            }
                            // The controller will be running iteration k+1
                            // once the upload lands.
                            intermediate_restart = true;
                        } else {
                            // Final iteration → Completed.
                            terminal_completion = true;
                            let trace = rec_state.recorder.stop();
                            rec_state
                                .execution_recorder
                                .on_execution_finished(timestamp);
                            let exec_trace = rec_state.execution_recorder.trace();
                            self.sessions
                                .complete_with_status(
                                    rec_state.session_id,
                                    trace,
                                    SessionStatus::Completed,
                                )
                                .await;
                            self.sessions
                                .set_iteration(rec_state.session_id, iteration)
                                .await;
                            if let Some(et) = exec_trace {
                                self.sessions
                                    .save_execution_trace(rec_state.session_id, et)
                                    .await;
                            }
                            *recording = None;
                        }
                    }
                }
            }

            // 3b. Drain the hardware execution trace (S3.6) — ONLY when THIS
            //     tick finalized the terminal Completed iteration (R6/NF3).
            //     Never on intermediate iterations (clear-on-take would
            //     consume the hardware samples before the final one) and
            //     never on EStop (S2: a failure emits no trace).
            if terminal_completion {
                if let Some(samples) = ctrl.read().await.take_execution_trace().await {
                    if !samples.is_empty() {
                        if let Some(session_id) = completed_session_id {
                            let robot_name = runtime.robot_name.clone();
                            let plan_id = runtime
                                .active_plan
                                .as_ref()
                                .map(|p| p.plan_id.clone())
                                .unwrap_or_default();
                            let trace = Self::assemble_execution_trace(
                                &samples,
                                plan_duration,
                                session_id,
                                plan_id,
                                robot_name,
                            );
                            self.sessions.save_execution_trace(session_id, trace).await;
                        }
                    }
                }
            }

            let fk_result =
                Self::compute_fk(&runtime.active_robot.chain, &runtime.active_robot.joints);

            let mut delta = TickDelta::from_robot_state(
                &state,
                runtime.active_robot.chain.clone(),
                fk_result,
                plan_duration,
                runtime.active_tcp.clone(),
            );
            // R4-001: tick deltas carry the ACTIVE controller's source so the
            // running badge keeps reflecting the real backend (Hardware/Esp32).
            if let Some(ref exe) = delta.execution {
                delta.execution = Some(exe.clone().with_source(active_source.clone()));
            }
            // Repeat state: the derived ExecutionSession knows nothing about
            // mode/iteration — attach the recording's live (or just-finalized)
            // values so the wire DTOs expose them (R8, EW3-EW6).
            if let Some((mode, iteration)) = repeat_meta {
                if let Some(ref exe) = delta.execution {
                    delta.execution = Some(exe.clone().with_repeat_state(mode, iteration));
                }
            }
            // Boundary-tick correction (R8): the delta was built from the
            // pre-gate `state`, which on an intermediate completion still says
            // Completed(k). The controller has ALREADY been restarted for
            // k+1 — report Running with a fresh progress so the frontend
            // keeps polling instead of treating the session as finished.
            if intermediate_restart {
                if let Some((mode, iteration)) = repeat_meta {
                    delta.execution = Some(
                        crate::plan::ExecutionSession::derived_with_source(
                            SessionStatus::Running,
                            0.0,
                            active_source.clone(),
                        )
                        .with_repeat_state(mode, iteration),
                    );
                }
            }
            // Normalize the execution-session time to SECONDS on the wire:
            // Hardware robot_state already reports seconds, but Simulation and
            // Replay report a 0..1 FRACTION. The DTO mapper divides
            // current_time by plan_duration for the progress bar (and exposes
            // it as elapsed_secs), so a raw fraction would cap the bar at
            // 1/plan_duration (e.g. ~10% for a 10s program) and show wrong
            // elapsed time.
            if !matches!(active_source, ExecutionSource::Hardware) {
                if let Some(ref mut exe) = delta.execution {
                    if plan_duration > 0.0 {
                        exe.current_time *= plan_duration;
                    }
                }
            }
            return Ok(delta);
        }

        // Fallback: no controller — read-only snapshot
        let runtime = self.runtime.read().await;
        let fk_result = Self::compute_fk(&runtime.active_robot.chain, &runtime.active_robot.joints);
        Ok(TickDelta {
            joints: runtime.active_robot.joints.clone(),
            chain: runtime.active_robot.chain.clone(),
            fk_result,
            execution: None,
            plan_duration: 0.0,
            active_tcp: runtime.active_tcp.clone(),
        })
    }

    /// Build a tick delta for the repeat upload window (B): the robot is
    /// stationary while the firmware receives the next manifest, so the delta
    /// replays the last known joints with a synthetic `Running(next_iteration)`
    /// session — the frontend keeps polling instead of treating the session as
    /// finished (R8). No wire traffic, no recording, no completion gate.
    async fn synthetic_uploading_delta(
        &self,
        mode: ExecutionMode,
        iteration: u32,
        source: ExecutionSource,
    ) -> Result<TickDelta, RuntimeError> {
        let runtime = self.runtime.read().await;
        let plan_duration = runtime
            .active_plan
            .as_ref()
            .map(|p| p.trajectory.duration())
            .unwrap_or(0.0);
        let fk_result = Self::compute_fk(&runtime.active_robot.chain, &runtime.active_robot.joints);
        let mut state = crate::state::robot_state::RobotState::default();
        state.joints.positions = runtime.active_robot.joints.clone();
        let state = Arc::new(state);
        let mut delta = TickDelta::from_robot_state(
            &state,
            runtime.active_robot.chain.clone(),
            fk_result,
            plan_duration,
            runtime.active_tcp.clone(),
        );
        delta.execution = Some(
            crate::plan::ExecutionSession::derived_with_source(
                SessionStatus::Running,
                0.0,
                source,
            )
            .with_repeat_state(mode, iteration),
        );
        Ok(delta)
    }

    /// Test-only (B): the current repeat orchestration phase — lets tests wait
    /// for the async re-execute to land before driving the next iteration.
    #[cfg(test)]
    pub(crate) async fn recording_repeat_phase(&self) -> Option<RepeatPhase> {
        self.recording.read().await.as_ref().map(|r| r.repeat_phase)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_engine::planning::motion::program::CompiledPlan;

    /// A VALID compiled plan: two waypoints, non-zero duration, target `[t, t]`.
    fn compiled_plan(t: f64) -> CompiledPlan {
        let points = vec![
            thalos_engine::core::trajectory::TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            thalos_engine::core::trajectory::TrajectoryPoint::new(vec![t, t], 1.0),
        ];
        CompiledPlan::new(thalos_engine::core::trajectory::Trajectory::new(points), vec![])
    }

    /// A MoveWaypoint edit — the shape the apply pipeline records.
    fn recorded_edit() -> (ProgramEdit, ProgramEdit) {
        let cmd = ProgramEdit::MoveWaypoint {
            segment_index: 0,
            new_target: vec![2.0, 2.0],
            old_target: Some(vec![1.0, 1.0]),
        };
        (cmd.clone(), cmd.inverse())
    }

    #[tokio::test]
    async fn reset_execution_preserves_command_history() {
        // Spec command-endpoints "Reset execution preserves history": resetting
        // execution must NOT clear the applied-command history — the program is
        // intact, so undo from a reset state stays valid.
        let manager = Arc::new(BackendManager::new());
        let service = SceneService::with_session_manager(
            manager,
            RobotModel::Planar2R,
            Arc::new(SessionManager::new()),
        );
        service.set_scene_writeback(true).await;

        // Seed the history with one applied command (feature-flagged apply).
        let (cmd, inverse) = recorded_edit();
        service
            .apply_compiled_plan(
                compiled_plan(1.0),
                cmd,
                inverse,
                CommandMetrics::new(0.4, 0.6),
                Vec::new(),
            )
            .await
            .expect("apply must succeed with the write-back flag on");
        assert_eq!(service.history_len().await, 1, "setup: one applied command");

        service
            .reset_execution()
            .await
            .expect("reset_execution must succeed");

        assert_eq!(
            service.history_len().await,
            1,
            "reset_execution must NOT clear command history (undo stays valid)"
        );
    }

    #[test]
    fn same_urdf_source_yields_same_id() {
        let source = r#"<robot name="a"><link name="base"/></robot>"#;
        let first = urdf_robot_id(source);
        let second = urdf_robot_id(source);
        assert_eq!(
            first, second,
            "identical URDF source must produce identical robot ids"
        );
        assert_ne!(first, "urdf", "id must not be the legacy literal 'urdf'");
    }

    #[test]
    fn different_urdf_source_yields_different_id() {
        let a = urdf_robot_id(r#"<robot name="a"><link name="base"/></robot>"#);
        let b = urdf_robot_id(r#"<robot name="b"><link name="base"/></robot>"#);
        assert_ne!(
            a, b,
            "URDF sources differing by one byte must yield different ids"
        );
    }

    #[test]
    fn id_matches_urdf_hash_format() {
        let id =
            urdf_robot_id(r#"<robot name="icebot"><link name="base"/><link name="tool"/></robot>"#);
        assert!(
            id.starts_with("urdf:"),
            "id must carry urdf: prefix, got {id}"
        );
        let hash = &id["urdf:".len()..];
        assert_eq!(hash.len(), 12, "id must carry 12 hex chars, got {id}");
        assert!(
            hash.chars().all(|c| c.is_ascii_hexdigit()),
            "id hash must be lowercase hex, got {id}"
        );
    }
}

/// Stable robot identity for URDF imports (spec robot-identity R1).
///
/// Deterministic `urdf:<hash>` id derived from the raw XML source: SHA-256 of
/// the raw bytes, truncated to the first 6 bytes (12 hex chars). Same file →
/// same id (R1.1); different bytes → different id. The raw source is used
/// (design D1) so the id never depends on parser behavior.
pub fn urdf_robot_id(source: &str) -> String {
    use sha2::{Digest, Sha256};

    let hash = Sha256::digest(source.as_bytes());
    format!("urdf:{}", hex::encode(&hash[..6]))
}
