use chrono::{DateTime, Utc};

use super::session_status::SessionStatus;
use crate::session::execution_source::ExecutionSource;
use crate::plan::ExecutionMode;

/// Mutable execution state for a compiled plan.
///
/// Created when the user presses Start. Advances through the plan's
/// trajectory until completion or cancellation. The plan itself
/// (`CompiledPlan`) is immutable and shared.
#[derive(Debug, Clone)]
pub struct ExecutionSession {
    pub plan_id: String,
    pub status: SessionStatus,
    /// Current time position in the trajectory (seconds).
    pub current_time: f64,
    pub started_at: Option<DateTime<Utc>>,
    pub paused_at: Option<DateTime<Utc>>,
    pub completed_at: Option<DateTime<Utc>>,
    /// Origin of the execution ("Simulation" | "Hardware" | "Replay #N") —
    /// informational, exposed on the wire as `ExecutionDto.source` (PR4,
    /// item 9). Defaults to `Simulation`; controllers override when known.
    pub source: ExecutionSource,
    /// Execution mode of the session (R1). Defaults to `Once` — sessions
    /// derived from controller state carry no repeat intent.
    pub mode: ExecutionMode,
    /// Current iteration, 1-based (R3). Defaults to 1.
    pub iteration: u32,
    /// Total iterations from the mode — `None` for `Once` (R4, EW6).
    pub total_iterations: Option<u32>,
}

impl ExecutionSession {
    pub fn new(plan_id: impl Into<String>) -> Self {
        Self {
            plan_id: plan_id.into(),
            status: SessionStatus::Ready,
            current_time: 0.0,
            started_at: None,
            paused_at: None,
            completed_at: None,
            source: ExecutionSource::Simulation,
            mode: ExecutionMode::Once,
            iteration: 1,
            total_iterations: None,
        }
    }

    /// Start (or restart) execution — transitions to Running.
    pub fn start(&mut self) {
        self.status = SessionStatus::Running;
        self.started_at = Some(Utc::now());
        self.current_time = 0.0;
    }

    /// Advance the current time by `dt` seconds.
    ///
    /// Returns the new progress fraction 0.0–1.0.
    /// If the trajectory duration is reached, transitions to Completed.
    pub fn advance(&mut self, dt: f64, trajectory_duration: f64) -> f64 {
        if self.status != SessionStatus::Running {
            return self.progress(trajectory_duration);
        }

        self.current_time += dt;

        if trajectory_duration > 0.0 && self.current_time >= trajectory_duration {
            self.current_time = trajectory_duration;
            self.status = SessionStatus::Completed;
            self.completed_at = Some(Utc::now());
        }

        self.progress(trajectory_duration)
    }

    pub fn pause(&mut self) {
        if self.status == SessionStatus::Running {
            self.status = SessionStatus::Paused;
            self.paused_at = Some(Utc::now());
        }
    }

    pub fn resume(&mut self) {
        if self.status == SessionStatus::Paused {
            self.status = SessionStatus::Running;
        }
    }

    pub fn cancel(&mut self) {
        if !self.status.is_terminal() {
            self.status = SessionStatus::Cancelled;
            self.completed_at = Some(Utc::now());
        }
    }

    pub fn fail(&mut self) {
        if !self.status.is_terminal() {
            self.status = SessionStatus::Failed;
            self.completed_at = Some(Utc::now());
        }
    }

    /// Reset the session for re-execution — keeps the plan_id, resets state.
    pub fn reset(&mut self) {
        self.status = SessionStatus::Ready;
        self.current_time = 0.0;
        self.started_at = None;
        self.paused_at = None;
        self.completed_at = None;
    }

    /// Progress as fraction of trajectory duration (0.0–1.0).
    pub fn progress(&self, trajectory_duration: f64) -> f64 {
        if self.status.is_terminal() {
            return 1.0;
        }
        if trajectory_duration <= 0.0 {
            return 1.0;
        }
        (self.current_time / trajectory_duration).clamp(0.0, 1.0)
    }

    /// Create a derived session from external state (status + progress).
    /// Used by RuntimeSnapshot/TickDelta to represent controller state
    /// in the legacy execution session format.
    pub fn derived(status: SessionStatus, progress: f64) -> Self {
        Self::derived_with_source(status, progress, ExecutionSource::Simulation)
    }

    /// `derived` with an explicit origin (R4-001). Snapshot builders pass the
    /// ACTIVE controller's source so the badge reports Hardware/Esp32 instead
    /// of always Simulation. Informational only — execution flow is unchanged.
    pub fn derived_with_source(
        status: SessionStatus,
        progress: f64,
        source: ExecutionSource,
    ) -> Self {
        let current_time = if progress >= 1.0 && status.is_terminal() {
            1.0
        } else {
            progress
        };
        Self {
            plan_id: String::new(),
            status,
            current_time,
            started_at: Some(Utc::now()),
            paused_at: None,
            completed_at: if status.is_terminal() {
                Some(Utc::now())
            } else {
                None
            },
            source,
            mode: ExecutionMode::Once,
            iteration: 1,
            total_iterations: None,
        }
    }

    /// Attach repeat state to a session (R3/R4). Consumed by the scene
    /// service to expose `mode`/`iteration`/`total_iterations` on the wire
    /// DTOs — the session derived from controller state knows none of them.
    pub fn with_repeat_state(mut self, mode: ExecutionMode, iteration: u32) -> Self {
        self.mode = mode;
        self.iteration = iteration;
        self.total_iterations = mode.total_iterations();
        self
    }

    /// Override the informational source (R4-001). Consumed by snapshot
    /// builders that know the active controller but build the session from
    /// robot state. Never changes execution behavior.
    pub fn with_source(mut self, source: ExecutionSource) -> Self {
        self.source = source;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::ExecutionMode;

    /// R3: a fresh session starts at iteration 1; R1: the default mode is
    /// Once; total_iterations for Once is None (EW6 hides the badge).
    #[test]
    fn new_session_defaults_to_once_iteration_one() {
        let s = ExecutionSession::new("plan-1");
        assert_eq!(s.mode, ExecutionMode::Once);
        assert_eq!(s.iteration, 1);
        assert_eq!(s.total_iterations, None);
    }

    /// R3/R4 data model: a Repeat session starts at iteration 1 and the
    /// total derives from the mode.
    #[test]
    fn repeat_session_starts_at_iteration_one_with_total() {
        let s = ExecutionSession::new("plan-1")
            .with_repeat_state(ExecutionMode::Repeat { count: 5 }, 1);
        assert_eq!(s.mode, ExecutionMode::Repeat { count: 5 });
        assert_eq!(s.iteration, 1);
        assert_eq!(s.total_iterations, Some(5));
    }

    /// R4: the session carries the CURRENT iteration alongside the total —
    /// iteration 2 of Repeat { count: 3 } is a valid intermediate state.
    #[test]
    fn repeat_session_carries_intermediate_iteration() {
        let s = ExecutionSession::new("plan-1")
            .with_repeat_state(ExecutionMode::Repeat { count: 3 }, 2);
        assert_eq!(s.iteration, 2);
        assert_eq!(s.total_iterations, Some(3));
    }

    /// Derived sessions (built from controller RobotState, no repeat state
    /// known) keep the Once/1/None defaults — iteration UI stays hidden
    /// unless the scene service attaches repeat state (EW6/EW-S4).
    #[test]
    fn derived_sessions_carry_once_defaults() {
        let s = ExecutionSession::derived(SessionStatus::Completed, 1.0);
        assert_eq!(s.mode, ExecutionMode::Once);
        assert_eq!(s.iteration, 1);
        assert_eq!(s.total_iterations, None);
    }
}
