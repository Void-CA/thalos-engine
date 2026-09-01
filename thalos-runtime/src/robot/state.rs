use thalos_engine::core::robot::tool_frame::ToolFrame;
use thalos_engine::core::spatial::frame::FrameId;
use thalos_engine::core::{
    kinematics::{
        forward::ForwardKinematics,
        inverse::{DampedLeastSquaresSolver, IKGoal, IKResult, IKSolver, IkError},
    },
    prelude::Trajectory,
};
use thalos_engine::models::Robot;
use thalos_engine::planning::motion::program::{CompiledPlan, PlanningProgram};
use thalos_engine::planning::program_edit::ProgramEdit;

use crate::error::RuntimeError;
use crate::services::command_history::{AppliedCommand, CommandHistory, CommandMetrics};
use crate::scene::JointMeta;
pub use thalos_engine::core::prelude::ActiveRobot;

use crate::plan::{ActiveMotionPlan, MotionType};

const IK_MAX_ITERS: usize = 500;
const IK_TOLERANCE: f64 = 1e-6;
const IK_LAMBDA: f64 = 0.1;

/// Feature gate name for the scene write-back surface (design D5).
///
/// `replace_active_plan` is the FIRST runtime-mutating surface introduced by
/// the analysis-advisor change. The flag is OFF by default — enable
/// per-environment only after integration tests pass. Rollback-safe: flipping
/// the flag off restores the previous read-only behavior with zero code
/// changes.
pub const SCENE_WRITEBACK_FLAG: &str = "scene-writeback";

/// Runtime state — plans, IK, and robot metadata.
///
/// Trajectory execution is delegated to the active `RobotController`
/// via `BackendManager`. This struct manages only plan metadata and
/// the kinematic model.
pub struct SceneRuntime {
    pub active_robot: ActiveRobot,
    pub robot_name: String,
    /// Canonical robot identity (spec robot-identity R1): catalog robots
    /// carry `metadata.id`; URDF imports carry `urdf:<sha256-trunc-12>`.
    /// Single source for every consumer — snapshots and the API DTO.
    pub robot_id: String,
    /// Original URDF model — `None` for built-in robots, `Some` for imports.
    pub robot_source: Option<Robot>,
    pub joints_meta: Vec<JointMeta>,

    /// Active Tool Center Point (TCP) frame.
    ///
    /// When `Some`, all analysis (workspace, singularity, manipulability)
    /// and IK default to this TCP instead of the flange (`chain.end_effector`).
    /// When `None`, the flange is used as the default working frame.
    pub active_tcp: Option<ToolFrame>,

    /// The compiled plan ready for visualisation and execution.
    /// Set by Preview — immutable, carries trajectory + segments.
    pub scheduled_plan: Option<CompiledPlan>,

    /// Active plan for snapshot backward compatibility.
    pub active_plan: Option<ActiveMotionPlan>,

    /// Feature gate for the scene write-back surface (design D5).
    ///
    /// OFF by default. `replace_active_plan` refuses to mutate the runtime
    /// while this flag is disabled — rollback = flip the flag off.
    scene_writeback_enabled: bool,

    /// Applied command history (design D6): pre-computed inverses, in apply
    /// order. PR5's `undo` pops the last entry in O(1) and applies its
    /// inverse. Stored in memory — no persistence in PR4/PR5.
    command_history: CommandHistory,

    next_plan_id: u64,
}

impl SceneRuntime {
    pub fn new(active_robot: ActiveRobot, robot_name: String) -> Self {
        // Initial identity derives from the catalog model when present
        // (design D4: explicit field, single writer via commands).
        let robot_id = active_robot
            .model
            .map(|m| m.metadata().id.to_string())
            .unwrap_or_default();
        Self {
            active_robot,
            robot_name,
            robot_id,
            robot_source: None,
            joints_meta: Vec::new(),
            active_tcp: None,
            scheduled_plan: None,
            active_plan: None,
            scene_writeback_enabled: false,
            command_history: CommandHistory::new(),
            next_plan_id: 0,
        }
    }

    /// Update `active_robot.joints` from a controller state (e.g. simulation tick).
    ///
    /// Silently drops the update when the controller state length differs from
    /// the chain DOF count — the chain is the source of truth and must not be
    /// resized to match a stale or mismatched controller. A mismatch usually
    /// indicates that a command (e.g. `MoveJ`) set joints without DOF
    /// validation; callers should fix the upstream validator instead of
    /// relaxing this guard.
    pub fn set_joints_from_state(&mut self, joints: &[f64]) {
        if joints.len() == self.active_robot.joints.len() {
            self.active_robot.joints.copy_from_slice(joints);
        } else {
            tracing::warn!(
                controller_len = joints.len(),
                chain_len = self.active_robot.joints.len(),
                "set_joints_from_state: controller joint count differs from chain DOF — update dropped"
            );
        }
    }

    pub fn solve_and_apply_ik(
        &mut self,
        frame: FrameId,
        goal: IKGoal,
    ) -> Result<IKResult, IkError> {
        let fk = ForwardKinematics::new(self.active_robot.chain.clone());
        let solver =
            DampedLeastSquaresSolver::new(fk, frame, IK_MAX_ITERS, IK_TOLERANCE, IK_LAMBDA);
        let q0 = self.active_robot.joints.clone();
        let result = solver.solve(&q0, goal)?;
        self.active_robot.joints = result.q.clone();
        Ok(result)
    }

    // ── Single-shot plan setters (MoveJ / MoveL) ──

    pub fn set_completed_plan(
        &mut self,
        trajectory: impl Into<Trajectory>,
        motion_type: MotionType,
    ) {
        let tid = self.next_plan_id();
        self.active_plan = Some(ActiveMotionPlan::completed(
            tid,
            trajectory.into(),
            motion_type,
        ));
    }

    pub fn set_created_plan(&mut self, trajectory: impl Into<Trajectory>, motion_type: MotionType) {
        let tid = self.next_plan_id();
        self.active_plan = Some(ActiveMotionPlan::created(
            tid,
            trajectory.into(),
            motion_type,
        ));
    }

    // ── Multi-segment program (Preview / Execution) ──

    /// Schedule a compiled multi-segment program for preview and optional execution.
    pub fn schedule_plan(&mut self, compiled: CompiledPlan) {
        let tid = self.next_plan_id();
        self.scheduled_plan = Some(compiled.clone());
        self.active_plan = Some(ActiveMotionPlan::from_compiled_plan(tid, compiled));
    }

    pub fn clear_plan(&mut self) {
        self.scheduled_plan = None;
        self.active_plan = None;
        // A cleared plan invalidates the applied-command history (spec
        // command-endpoints "Robot Change Cleanup"): stale inverses must not
        // survive — undo is only valid against the plan a command produced.
        self.command_history.clear();
    }

    /// Clear the applied-command history (spec command-endpoints "Robot Change
    /// Cleanup"). Robot changes discard the previous robot's undo stack.
    pub fn clear_command_history(&mut self) {
        self.command_history.clear();
    }

    /// Configure the command-history capacity (spec "History Cap").
    ///
    /// Honors the optional `THALOS_HISTORY_CAP` env var read at the binary
    /// entry point; defaults to [`DEFAULT_HISTORY_CAP`].
    pub fn with_history_cap(&mut self, cap: usize) {
        self.command_history.set_cap(cap);
    }

    // ── Scene write-back (PR4 — first runtime-mutating surface, D4/D5) ──

    /// Read the scene-writeback feature flag (design D5).
    pub fn scene_writeback_enabled(&self) -> bool {
        self.scene_writeback_enabled
    }

    /// Enable/disable the scene-writeback feature flag (design D5).
    ///
    /// Default is OFF. Enable per-environment after integration tests pass.
    pub fn set_scene_writeback(&mut self, enabled: bool) {
        self.scene_writeback_enabled = enabled;
    }

    /// Replace the active plan with a recompiled plan (design D4).
    ///
    /// This is the FIRST surface that mutates the runtime from outside the
    /// command pipeline, so it is deliberately conservative:
    /// 1. Feature gate (D5): while `scene-writeback` is disabled the method
    ///    errors and mutates NOTHING — rollback is a flag flip.
    /// 2. Snapshot (D4): the complete previous plan (scheduled_plan +
    ///    active_plan) is cloned BEFORE any mutation.
    /// 3. Validation: the replacement must be a real plan (non-empty,
    ///    non-zero duration) — an empty plan is rejected.
    /// 4. Restore: if any step fails, the snapshot is written back so the
    ///    runtime is byte-for-byte as before the call.
    ///
    /// Because `scheduled_plan` is the source for `trajectory_to_waypoints`
    /// (scene.rs), the write-back propagates to execution automatically.
    pub fn replace_active_plan(&mut self, compiled: CompiledPlan) -> Result<(), RuntimeError> {
        // 1. Feature gate (D5): flag OFF → error, NO mutation.
        if !self.scene_writeback_enabled {
            return Err(RuntimeError::FeatureDisabled {
                feature: SCENE_WRITEBACK_FLAG,
            });
        }

        // 2. Snapshot (D4): complete previous plan — scheduled + active.
        let snapshot = (self.scheduled_plan.clone(), self.active_plan.clone());

        // 3+4. Fallible steps run BEFORE the commit point; on ANY error the
        // snapshot is restored so the runtime is left exactly as before.
        let result = self.replace_active_plan_inner(compiled);
        if let Err(err) = result {
            self.scheduled_plan = snapshot.0;
            self.active_plan = snapshot.1;
            return Err(err);
        }
        Ok(())
    }

    /// Fallible core of the replacement. Only `Ok` commits; the caller
    /// restores the snapshot on `Err`.
    fn replace_active_plan_inner(
        &mut self,
        compiled: CompiledPlan,
    ) -> Result<(), RuntimeError> {
        if compiled.waypoint_count == 0 || compiled.duration <= 0.0 {
            return Err(RuntimeError::InvalidCompiledPlan {
                reason: format!(
                    "compiled plan carries {} waypoints and {:.3}s of motion",
                    compiled.waypoint_count, compiled.duration
                ),
            });
        }
        let tid = self.next_plan_id();
        self.scheduled_plan = Some(compiled.clone());
        self.active_plan = Some(ActiveMotionPlan::from_compiled_plan(tid, compiled));
        Ok(())
    }

    /// Record an applied command with its pre-computed inverse + metrics (D6).
    ///
    /// PR5's `undo` pops the last entry in O(1) and applies `inverse`; the
    /// metrics let the undo response report the restored health without
    /// re-running the analysis pipeline. `applied_program` links the entry to
    /// the program the apply produced (R4-001) — undo refuses when the active
    /// plan no longer matches it.
    pub fn record_applied_command(
        &mut self,
        command: ProgramEdit,
        inverse: ProgramEdit,
        metrics: CommandMetrics,
        applied_program: Vec<thalos_engine::core::motion::segment::MotionSegment>,
    ) {
        self.command_history.push(AppliedCommand {
            command,
            inverse,
            metrics,
            applied_program,
        });
    }

    /// Number of applied commands with stored inverses (undo history size).
    pub fn history_len(&self) -> usize {
        self.command_history.len()
    }

    /// Peek the last applied command (O(1)) — the undo endpoint resolves the
    /// stored inverse without mutating the history.
    pub fn last_applied_command(&self) -> Option<&AppliedCommand> {
        self.command_history.last()
    }

    /// Peek the last applied command together with the history version — the
    /// atomic `(entry, version)` pair the undo flow reads under a SINGLE lock
    /// before recompiling (PR2 TOCTOU). The version is re-validated at commit
    /// time by [`SceneRuntime::undo_plan`].
    pub fn last_applied_with_version(&self) -> (Option<&AppliedCommand>, u64) {
        self.command_history.last_with_version()
    }

    /// Pop the last applied command (O(1)) — commit step of the PR5 undo.
    pub fn pop_applied_command(&mut self) -> Option<AppliedCommand> {
        self.command_history.pop()
    }

    /// Undo the last applied command (design D6): pop (O(1)) + write back the
    /// inverse-applied plan. Atomic: on ANY failure the history entry is
    /// preserved and the runtime is restored to its previous state.
    ///
    /// R4-001 stale guard: the stored inverse is ONLY applied to the exact
    /// program the command produced. `current` is the program reconstructed
    /// from the active plan — if it no longer matches the entry's
    /// `applied_program`, undo returns `StaleUndo` WITHOUT mutation (a
    /// non-commanded path — e.g. a re-schedule — replaced the active plan).
    ///
    /// Order matters:
    /// 1. O(1) peek — an empty history errors BEFORE any mutation (spec
    ///    command-endpoints "Undo with empty history" → 409).
    /// 2. Stale guard — the current program must match the command's
    ///    pre-state; a mismatch errors BEFORE any mutation (R4-001 → 409).
    /// 3. Write-back — feature-flagged (D5) with snapshot + atomic restore
    ///    (D4); a failure here leaves both the plan AND the history intact.
    /// 4. Commit — only now drop the entry (O(1) pop).
    ///
    /// PR2 versioned undo (spec command-endpoints "Undo version mismatch"):
    /// `expected_version` is the history version read atomically with the last
    /// entry (`last_applied_with_version`). It is re-validated under the write
    /// lock BEFORE the stale guard and the commit — a concurrent apply/undo
    /// that mutated the history between the peek and the commit is rejected
    /// with `UndoVersionMismatch` and NOTHING is mutated or popped.
    pub fn undo_plan(
        &mut self,
        current: &PlanningProgram,
        compiled: CompiledPlan,
        expected_version: u64,
    ) -> Result<AppliedCommand, RuntimeError> {
        let entry = self
            .command_history
            .last()
            .cloned()
            .ok_or(RuntimeError::EmptyCommandHistory)?;
        // Version gate FIRST — closes the peek→recompile→commit TOCTOU window.
        if self.command_history.version() != expected_version {
            return Err(RuntimeError::UndoVersionMismatch {
                expected: expected_version,
                actual: self.command_history.version(),
            });
        }
        if !entry.matches_applied_program(current) {
            return Err(RuntimeError::StaleUndo);
        }
        self.replace_active_plan(compiled)?;
        self.command_history.pop();
        Ok(entry)
    }

    fn next_plan_id(&mut self) -> String {
        let id = self.next_plan_id;
        self.next_plan_id += 1;
        format!("plan-{}", id)
    }

    // ── TCP selection ──

    /// Select or clear the active Tool Center Point (TCP).
    ///
    /// If `tool_frame` is `Some`, validates that the frame exists in the robot chain.
    /// If `tool_frame` is `None`, clears the TCP (falls back to flange).
    ///
    /// Returns an error if the frame does not exist in the chain.
    pub fn select_tool_frame(&mut self, tool_frame: Option<ToolFrame>) -> Result<(), RuntimeError> {
        if let Some(tcp) = &tool_frame {
            // Validate that the frame exists in the chain
            if self
                .active_robot
                .chain
                .frames
                .get(&tcp.base_frame)
                .is_none()
            {
                return Err(RuntimeError::ToolFrameNotFound {
                    frame_id: match tcp.base_frame {
                        FrameId::Id(id) => id,
                        FrameId::World => 0,
                    },
                });
            }
        }
        self.active_tcp = tool_frame;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_engine::core::ids::OperationId;
    use thalos_engine::core::models::{RobotModel, RobotRegistry};
    use thalos_engine::core::trajectory::TrajectoryPoint;

    fn test_runtime() -> SceneRuntime {
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let active_robot = ActiveRobot::new(Some(RobotModel::Planar2R), chain, vec![0.0; 2]);
        SceneRuntime::new(active_robot, "test-bot".into())
    }

    /// A VALID compiled plan: two waypoints, non-zero duration, target `[t, t]`.
    fn compiled_plan(t: f64) -> CompiledPlan {
        let points = vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![t, t], 1.0),
        ];
        CompiledPlan::new(Trajectory::new(points), vec![])
    }

    /// An INVALID compiled plan: zero waypoints → fails replacement validation.
    fn empty_plan() -> CompiledPlan {
        CompiledPlan::new(Trajectory::new(vec![]), vec![])
    }

    /// Behavior-relevant signature of a CompiledPlan (Trajectory has no
    /// PartialEq — compare the actual trajectory data, not struct identity).
    fn compiled_signature(p: &CompiledPlan) -> (f64, usize, Vec<Vec<f64>>) {
        (
            p.duration,
            p.waypoint_count,
            p.merged_trajectory
                .waypoints()
                .iter()
                .map(|w| w.joints().to_vec())
                .collect(),
        )
    }

    /// Behavior-relevant signature of the active plan.
    fn active_signature(p: &ActiveMotionPlan) -> (String, Vec<Vec<f64>>) {
        (
            p.plan_id.clone(),
            p.trajectory.waypoints().iter().map(|w| w.joints().to_vec()).collect(),
        )
    }

    #[test]
    fn replace_active_plan_success_swaps_plan() {
        // Spec scene-writeback "Successful replacement": flag on + valid plan
        // → active_plan updated; snapshot of previous plan stored.
        let mut runtime = test_runtime();
        runtime.set_scene_writeback(true);
        runtime.schedule_plan(compiled_plan(1.0));
        let before = runtime.active_plan.clone();
        let before_id = before.as_ref().unwrap().plan_id.clone();

        runtime
            .replace_active_plan(compiled_plan(2.0))
            .expect("flag on + valid plan → replacement succeeds");

        let active = runtime.active_plan.clone().unwrap();
        assert_eq!(
            active.trajectory.waypoints().last().unwrap().joints(),
            &[2.0, 2.0],
            "active_plan must carry the NEW trajectory"
        );
        assert_ne!(
            active.plan_id, before_id,
            "replacement must allocate a NEW plan id"
        );
        assert_eq!(
            runtime
                .scheduled_plan
                .as_ref()
                .unwrap()
                .merged_trajectory
                .waypoints()
                .last()
                .unwrap()
                .joints(),
            &[2.0, 2.0],
            "scheduled_plan must carry the NEW compiled plan"
        );
    }

    #[test]
    fn replace_active_plan_failure_restores_previous_plan() {
        // Spec scene-writeback "Failure rollback" + "Snapshot integrity": a
        // compiled plan that fails validation must leave the runtime exactly
        // as before — the complete previous plan (trajectory + duration) is
        // restored from the snapshot.
        let mut runtime = test_runtime();
        runtime.set_scene_writeback(true);
        runtime.schedule_plan(compiled_plan(1.0));
        let before_scheduled = runtime.scheduled_plan.clone();
        let before_active = runtime.active_plan.clone();

        let err = runtime
            .replace_active_plan(empty_plan())
            .expect_err("empty plan must fail validation");
        assert!(
            matches!(err, RuntimeError::InvalidCompiledPlan { .. }),
            "empty plan must produce InvalidCompiledPlan, got {err:?}"
        );

        assert_eq!(
            compiled_signature(runtime.scheduled_plan.as_ref().unwrap()),
            compiled_signature(before_scheduled.as_ref().unwrap()),
            "scheduled_plan must be restored (snapshot integrity)"
        );
        assert_eq!(
            active_signature(runtime.active_plan.as_ref().unwrap()),
            active_signature(before_active.as_ref().unwrap()),
            "active_plan must be restored (snapshot integrity)"
        );
        assert_eq!(
            runtime.scheduled_plan.as_ref().unwrap().duration,
            before_scheduled.as_ref().unwrap().duration,
            "snapshot must preserve the previous plan duration"
        );
    }

    #[test]
    fn replace_active_plan_flag_off_errors_without_mutation() {
        // Spec scene-writeback "Flag disabled" (D5): default flag OFF → error,
        // active_plan unchanged, scheduled_plan unchanged.
        let mut runtime = test_runtime();
        assert!(
            !runtime.scene_writeback_enabled(),
            "scene-writeback flag must default to OFF (D5)"
        );
        runtime.schedule_plan(compiled_plan(1.0));
        let before_active = runtime.active_plan.clone();
        let before_scheduled = runtime.scheduled_plan.clone();

        let err = runtime
            .replace_active_plan(compiled_plan(2.0))
            .expect_err("flag off → replace must error");
        assert!(
            matches!(
                err,
                RuntimeError::FeatureDisabled {
                    feature: "scene-writeback"
                }
            ),
            "flag-off error must name the scene-writeback feature, got {err:?}"
        );

        assert_eq!(
            active_signature(runtime.active_plan.as_ref().unwrap()),
            active_signature(before_active.as_ref().unwrap()),
            "flag-off must NOT mutate the active plan"
        );
        assert_eq!(
            compiled_signature(runtime.scheduled_plan.as_ref().unwrap()),
            compiled_signature(before_scheduled.as_ref().unwrap()),
            "flag-off must NOT mutate the scheduled plan"
        );
    }

    #[test]
    fn replace_active_plan_flag_on_proceeds() {
        // Spec scene-writeback "Flag enabled": flag on + valid plan → replacement
        // proceeds normally.
        let mut runtime = test_runtime();
        runtime.set_scene_writeback(true);
        runtime.schedule_plan(compiled_plan(1.0));

        runtime
            .replace_active_plan(compiled_plan(2.0))
            .expect("flag on + valid plan → replacement proceeds");
    }

    #[test]
    fn replace_active_plan_failure_after_success_restores_latest_plan() {
        // Triangulation: the snapshot is taken PER CALL — a failure after an
        // earlier success must restore the LATEST committed plan, not the
        // original one.
        let mut runtime = test_runtime();
        runtime.set_scene_writeback(true);
        runtime.schedule_plan(compiled_plan(1.0));
        runtime.replace_active_plan(compiled_plan(2.0)).unwrap();
        let after_second = runtime.scheduled_plan.clone();

        let err = runtime
            .replace_active_plan(empty_plan())
            .expect_err("empty plan must fail validation");
        assert!(matches!(err, RuntimeError::InvalidCompiledPlan { .. }));
        assert_eq!(
            compiled_signature(runtime.scheduled_plan.as_ref().unwrap()),
            compiled_signature(after_second.as_ref().unwrap()),
            "restore must bring back the LATEST committed plan"
        );
    }

    // ── PR5 — undo O(1) via pre-computed inverse (design D6) ──

    /// A MoveWaypoint edit — the shape the apply pipeline records (PR4). The
    /// runtime test never applies it to a program (the API recompiles); it
    /// only needs to be recorded alongside its inverse + metrics.
    fn recorded_edit() -> (ProgramEdit, ProgramEdit) {
        let cmd = ProgramEdit::MoveWaypoint {
            segment_index: 0,
            new_target: vec![2.0, 2.0],
            old_target: Some(vec![1.0, 1.0]),
        };
        (cmd.clone(), cmd.inverse())
    }

    #[test]
    fn undo_restores_previous_plan_via_single_inverse() {
        // Spec command-endpoints "Undo restores previous plan": undo pops the
        // last applied command and writes the restored (inverse-applied) plan
        // back to the runtime — the previous trajectory comes back.
        let mut runtime = test_runtime();
        runtime.set_scene_writeback(true);
        runtime.schedule_plan(compiled_plan(1.0)); // the "previous" plan
        let (cmd, inverse) = recorded_edit();
        runtime.record_applied_command(
            cmd,
            inverse,
            CommandMetrics::new(0.4, 0.6),
            // The apply wrote a plan WITHOUT program segments (legacy compiled
            // plan) — the guard compares the reconstructed program (also empty).
            Vec::new(),
        );
        runtime.replace_active_plan(compiled_plan(2.0)).unwrap(); // the "applied" plan
        let applied_trajectory = runtime.active_plan.clone();
        assert_eq!(
            applied_trajectory
                .as_ref()
                .unwrap()
                .trajectory
                .waypoints()
                .last()
                .unwrap()
                .joints(),
            &[2.0, 2.0],
            "setup: the applied plan carries the NEW trajectory"
        );

        // undo: pop (O(1)) + write-back of the inverse-applied plan (recompiled
        // by the API layer — here represented by compiled_plan(1.0)).
        let current = PlanningProgram::new(Vec::new());
        let popped = runtime
            .undo_plan(&current, compiled_plan(1.0), runtime.command_history.version())
            .expect("non-empty history → undo succeeds");
        assert_eq!(
            popped.metrics,
            CommandMetrics::new(0.4, 0.6),
            "undo returns the POPPED entry so the API reports its stored metrics"
        );

        // The restored plan carries the PREVIOUS trajectory.
        let active = runtime.active_plan.as_ref().unwrap();
        assert_eq!(
            active.trajectory.waypoints().last().unwrap().joints(),
            &[1.0, 1.0],
            "undo must restore the previous plan trajectory"
        );
        assert_ne!(
            active.plan_id, applied_trajectory.as_ref().unwrap().plan_id,
            "restored plan gets a fresh id"
        );
        assert_eq!(
            runtime.history_len(),
            0,
            "undo pops the entry — the history is empty again"
        );
    }

    #[test]
    fn undo_with_empty_history_errors_without_mutation() {
        // Spec command-endpoints "Undo with empty history" (→ 409): no applied
        // commands → undo errors and mutates NOTHING.
        let mut runtime = test_runtime();
        runtime.set_scene_writeback(true);
        runtime.schedule_plan(compiled_plan(1.0));
        let before_active = runtime.active_plan.clone();
        let before_scheduled = runtime.scheduled_plan.clone();

        let err = runtime
            .undo_plan(
                &PlanningProgram::new(Vec::new()),
                compiled_plan(2.0),
                runtime.command_history.version(),
            )
            .expect_err("empty history → undo must error");
        assert!(
            matches!(err, RuntimeError::EmptyCommandHistory),
            "empty history must produce EmptyCommandHistory, got {err:?}"
        );
        assert_eq!(
            active_signature(runtime.active_plan.as_ref().unwrap()),
            active_signature(before_active.as_ref().unwrap()),
            "empty-history undo must NOT mutate the active plan"
        );
        assert_eq!(
            compiled_signature(runtime.scheduled_plan.as_ref().unwrap()),
            compiled_signature(before_scheduled.as_ref().unwrap()),
            "empty-history undo must NOT mutate the scheduled plan"
        );
    }

    #[test]
    fn undo_flag_off_errors_without_mutation_or_pop() {
        // Design D5 applies to undo too: with scene-writeback OFF the write-back
        // errors and the history entry is PRESERVED (atomicity — pop only
        // commits after a successful write-back).
        let mut runtime = test_runtime(); // flag OFF by default
        runtime.schedule_plan(compiled_plan(1.0));
        let (cmd, inverse) = recorded_edit();
        runtime.record_applied_command(cmd, inverse, CommandMetrics::new(0.4, 0.6), Vec::new());

        let err = runtime
            .undo_plan(
                &PlanningProgram::new(Vec::new()),
                compiled_plan(2.0),
                runtime.command_history.version(),
            )
            .expect_err("flag off → undo must error");
        assert!(
            matches!(err, RuntimeError::FeatureDisabled { .. }),
            "flag-off undo must produce FeatureDisabled, got {err:?}"
        );
        assert_eq!(
            runtime.history_len(),
            1,
            "flag-off undo must NOT pop the entry (atomicity)"
        );
        assert_eq!(
            runtime.active_plan.as_ref().unwrap().trajectory.waypoints().last().unwrap().joints(),
            &[1.0, 1.0],
            "flag-off undo must NOT mutate the active plan"
        );
    }

    #[test]
    fn undo_stale_inverse_errors_without_mutation_or_pop() {
        // R4-001: after an apply, replacing the active plan by a NON-commanded
        // path (e.g. schedule_plan) makes the stored inverse stale. undo_plan
        // must reject with StaleUndo, mutating NOTHING and preserving the
        // history entry.
        let mut runtime = test_runtime();
        runtime.set_scene_writeback(true);
        runtime.schedule_plan(compiled_plan(1.0));
        let (cmd, inverse) = recorded_edit();
        // The apply wrote a program carrying one MoveJ segment.
        let applied_program = vec![thalos_engine::core::motion::segment::MotionSegment::MoveJ {
            origin: OperationId("op-0".to_string()),
            target: vec![0.0, 0.0],
            max_velocity: Some(500.0),
            max_acceleration: Some(1000.0),
        }];
        runtime.record_applied_command(
            cmd,
            inverse,
            CommandMetrics::new(0.4, 0.6),
            applied_program,
        );
        runtime.replace_active_plan(compiled_plan(2.0)).unwrap(); // the "applied" plan

        // A DIFFERENT (re-scheduled) plan is now active — its program (empty)
        // no longer matches the recorded applied program.
        runtime.schedule_plan(compiled_plan(3.0));
        let rescheduled = runtime.active_plan.clone();
        let current = PlanningProgram::new(Vec::new());

        let err = runtime
            .undo_plan(&current, compiled_plan(1.0), runtime.command_history.version())
            .expect_err("stale inverse must be rejected");
        assert!(
            matches!(err, RuntimeError::StaleUndo),
            "stale undo must produce StaleUndo, got {err:?}"
        );
        assert_eq!(
            runtime.history_len(),
            1,
            "stale undo must NOT pop the entry"
        );
        assert_eq!(
            runtime.active_plan.as_ref().unwrap().plan_id,
            rescheduled.as_ref().unwrap().plan_id,
            "stale undo must NOT mutate the re-scheduled plan"
        );
    }

    // ── PR2 — versioned undo (spec command-endpoints "Undo version mismatch") ──

    #[test]
    fn undo_with_stale_expected_version_errors_without_mutation_or_pop() {
        // PR2: the undo flow reads (last entry, version) atomically, recompiles,
        // then commits with the expected version. A concurrent apply/undo that
        // bumped the version in between MUST be rejected — no pop, no plan
        // mutation (the entry and the plan stay exactly as they were).
        let mut runtime = test_runtime();
        runtime.set_scene_writeback(true);
        runtime.schedule_plan(compiled_plan(1.0));
        let (cmd, inverse) = recorded_edit();
        runtime.record_applied_command(cmd, inverse, CommandMetrics::new(0.4, 0.6), Vec::new());

        // The history holds one entry (version 1) — commit with a STALE
        // expected version, as if the history moved on between peek and commit.
        let err = runtime
            .undo_plan(&PlanningProgram::new(Vec::new()), compiled_plan(2.0), 999)
            .expect_err("stale expected version → undo must error");
        assert!(
            matches!(
                err,
                RuntimeError::UndoVersionMismatch {
                    expected: 999,
                    actual: 1
                }
            ),
            "stale version must produce UndoVersionMismatch, got {err:?}"
        );
        assert_eq!(
            runtime.history_len(),
            1,
            "version-mismatch undo must NOT pop the entry"
        );
        assert_eq!(
            runtime
                .active_plan
                .as_ref()
                .unwrap()
                .trajectory
                .waypoints()
                .last()
                .unwrap()
                .joints(),
            &[1.0, 1.0],
            "version-mismatch undo must NOT mutate the active plan"
        );
    }

    #[test]
    fn undo_with_current_expected_version_commits_and_pops() {
        // Triangulation of the version gate: the version read ATOMICALLY with
        // the last entry (last_applied_with_version) must commit normally —
        // the gate only rejects when the history actually moved on.
        let mut runtime = test_runtime();
        runtime.set_scene_writeback(true);
        runtime.schedule_plan(compiled_plan(1.0));
        let (cmd, inverse) = recorded_edit();
        runtime.record_applied_command(cmd, inverse, CommandMetrics::new(0.4, 0.6), Vec::new());

        let (last, version) = runtime.last_applied_with_version();
        assert!(last.is_some(), "one recorded command → last entry present");
        assert_eq!(version, 1, "one push → version 1");

        let popped = runtime
            .undo_plan(&PlanningProgram::new(Vec::new()), compiled_plan(1.0), version)
            .expect("current expected version → undo commits");
        assert_eq!(
            popped.metrics,
            CommandMetrics::new(0.4, 0.6),
            "the popped entry carries the stored metrics"
        );
        assert_eq!(runtime.history_len(), 0, "commit pops the entry");
    }

    // ── PR3 — history cap + robot-change cleanup (spec "History Cap" /
    //     "Robot Change Cleanup") ──

    #[test]
    fn with_history_cap_bounds_the_command_history() {
        // Spec "History Cap": the runtime honors a configured capacity —
        // pushes beyond it evict the oldest entries.
        let mut runtime = test_runtime();
        runtime.with_history_cap(3);
        for i in 1..=5 {
            let (cmd, inverse) = recorded_edit();
            runtime.record_applied_command(cmd, inverse, CommandMetrics::new(0.4, 0.6), Vec::new());
        }
        assert_eq!(
            runtime.history_len(),
            3,
            "history must be bounded by the configured cap"
        );
    }

    #[test]
    fn clear_plan_clears_the_command_history() {
        // Spec "Robot Change Cleanup": clearing the plan also discards the
        // applied-command history — stale inverses must not survive a plan
        // lifecycle reset.
        let mut runtime = test_runtime();
        runtime.schedule_plan(compiled_plan(1.0));
        let (cmd, inverse) = recorded_edit();
        runtime.record_applied_command(cmd, inverse, CommandMetrics::new(0.4, 0.6), Vec::new());
        assert_eq!(runtime.history_len(), 1, "setup: one applied command");

        runtime.clear_plan();

        assert!(runtime.scheduled_plan.is_none(), "clear_plan clears the plan");
        assert!(runtime.active_plan.is_none(), "clear_plan clears the active plan");
        assert_eq!(
            runtime.history_len(),
            0,
            "clear_plan must also clear the command history"
        );
    }

    #[test]
    fn clear_command_history_discards_entries_and_bumps_version() {
        // The dispatch cleanup path (LoadRobot/LoadUrdfRobot): clearing the
        // history is a mutation like any other — a concurrent undo commit
        // re-validated against the old version must be rejected (PR2 gate).
        let mut runtime = test_runtime();
        let (cmd, inverse) = recorded_edit();
        runtime.record_applied_command(cmd, inverse, CommandMetrics::new(0.4, 0.6), Vec::new());
        let version_before = runtime.last_applied_with_version().1;
        assert_eq!(version_before, 1, "setup: one push → version 1");

        runtime.clear_command_history();

        assert_eq!(runtime.history_len(), 0, "history must be empty");
        assert_eq!(
            runtime.last_applied_with_version().1,
            version_before + 1,
            "clearing the history is a mutation → version must bump"
        );
    }
}
