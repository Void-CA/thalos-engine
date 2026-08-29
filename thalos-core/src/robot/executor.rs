use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;
use serde::{Deserialize, Serialize};

use crate::robot::capability::ObservationRequirement;
use crate::robot::io::{CommandSink, DeviceIOError, ObservationSource};
use crate::robot::observation::{JointObservationAssessment, ObservationAssessment, ObservationQuality};
use crate::robot::policy::{ObservationResponsePolicy, PolicyDecision};
use crate::robot::state::{JointState, RobotState, StateDeviation};
use crate::trajectory::Trajectory;

/// Abstract clock source for execution cycle timing.
pub trait ExecutionClock: Send + Sync {
    fn now(&self) -> Duration;
}

/// Real-time system clock using instant elapsed time.
#[derive(Debug, Clone)]
pub struct SystemExecutionClock {
    start: std::time::Instant,
}

impl Default for SystemExecutionClock {
    fn default() -> Self {
        Self::new()
    }
}

impl SystemExecutionClock {
    pub fn new() -> Self {
        Self {
            start: std::time::Instant::now(),
        }
    }
}

impl ExecutionClock for SystemExecutionClock {
    fn now(&self) -> Duration {
        self.start.elapsed()
    }
}

/// Deterministic fake clock for reproducible testing of closed-loop execution timing.
#[derive(Debug, Clone)]
pub struct FakeExecutionClock {
    nanos: Arc<AtomicU64>,
}

impl Default for FakeExecutionClock {
    fn default() -> Self {
        Self::new()
    }
}

impl FakeExecutionClock {
    pub fn new() -> Self {
        Self {
            nanos: Arc::new(AtomicU64::new(0)),
        }
    }

    pub fn set(&self, time: Duration) {
        self.nanos.store(time.as_nanos() as u64, Ordering::SeqCst);
    }

    pub fn advance(&self, duration: Duration) {
        self.nanos
            .fetch_add(duration.as_nanos() as u64, Ordering::SeqCst);
    }
}

impl ExecutionClock for FakeExecutionClock {
    fn now(&self) -> Duration {
        Duration::from_nanos(self.nanos.load(Ordering::SeqCst))
    }
}

/// High-level operational state of the `ClosedLoopExecutor`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ExecutionStatus {
    /// Ready to start execution.
    Idle,
    /// Actively tracking and advancing along trajectory.
    Running,
    /// Temporarily holding current position due to observation quality degradation (e.g. stale telemetry).
    Holding,
    /// Temporarily paused due to excessive kinematic position/velocity deviation.
    Paused,
    /// Trajectory completed successfully.
    Completed,
    /// Terminated due to unrecoverable fault (e.g. invalid observation, NaN, I/O failure).
    Aborted,
}

/// Single discrete record of an execution feedback cycle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExecutionCycle {
    pub step_index: usize,
    pub timestamp: Duration,
    pub expected_state: RobotState,
    pub observed_state: RobotState,
    pub observation_assessment: ObservationAssessment,
    pub state_deviation: Option<StateDeviation>,
    pub policy_decision: PolicyDecision,
    pub status_after_cycle: ExecutionStatus,
}

impl ExecutionCycle {
    pub fn drift_report(&self) -> DriftReport {
        DriftReport::from_cycle(self)
    }
}

/// Detailed observation and mathematical drift report for a single joint during execution.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointDrift {
    pub joint_index: usize,
    pub expected_position: Option<f64>,
    pub observed_position: Option<f64>,
    pub position_drift: Option<f64>,
    pub expected_velocity: Option<f64>,
    pub observed_velocity: Option<f64>,
    pub velocity_drift: Option<f64>,
    pub expected_effort: Option<f64>,
    pub observed_effort: Option<f64>,
    pub effort_drift: Option<f64>,
    pub quality: JointObservationAssessment,
}

/// Consolidated, UI-ready execution telemetry and drift report for a single cycle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DriftReport {
    pub step_index: usize,
    pub timestamp_secs: f64,
    pub joint_drifts: Vec<JointDrift>,
    pub max_position_drift: Option<f64>,
    pub max_velocity_drift: Option<f64>,
    pub max_effort_drift: Option<f64>,
    pub overall_quality: ObservationQuality,
    pub policy_decision: PolicyDecision,
    pub execution_status: ExecutionStatus,
}

impl DriftReport {
    pub fn from_cycle(cycle: &ExecutionCycle) -> Self {
        let max_dof = cycle
            .expected_state
            .joints
            .len()
            .max(cycle.observed_state.joints.len());

        let mut joint_drifts = Vec::with_capacity(max_dof);

        for i in 0..max_dof {
            let exp_j = cycle.expected_state.joints.get(i);
            let obs_j = cycle.observed_state.joints.get(i);
            let quality = cycle
                .observation_assessment
                .joints
                .get(i)
                .copied()
                .unwrap_or(JointObservationAssessment {
                    position: ObservationQuality::Valid,
                    velocity: ObservationQuality::Valid,
                    effort: ObservationQuality::Valid,
                });

            let expected_position = exp_j.and_then(|j| j.position);
            let observed_position = obs_j.and_then(|j| j.position);
            let position_drift = match (observed_position, expected_position) {
                (Some(o), Some(e)) => Some(o - e),
                _ => None,
            };

            let expected_velocity = exp_j.and_then(|j| j.velocity);
            let observed_velocity = obs_j.and_then(|j| j.velocity);
            let velocity_drift = match (observed_velocity, expected_velocity) {
                (Some(o), Some(e)) => Some(o - e),
                _ => None,
            };

            let expected_effort = exp_j.and_then(|j| j.effort);
            let observed_effort = obs_j.and_then(|j| j.effort);
            let effort_drift = match (observed_effort, expected_effort) {
                (Some(o), Some(e)) => Some(o - e),
                _ => None,
            };

            joint_drifts.push(JointDrift {
                joint_index: i,
                expected_position,
                observed_position,
                position_drift,
                expected_velocity,
                observed_velocity,
                velocity_drift,
                expected_effort,
                observed_effort,
                effort_drift,
                quality,
            });
        }

        let max_position_drift = cycle
            .state_deviation
            .as_ref()
            .and_then(|dev| dev.max_position_error());
        let max_velocity_drift = cycle
            .state_deviation
            .as_ref()
            .and_then(|dev| dev.max_velocity_error());
        let max_effort_drift = cycle
            .state_deviation
            .as_ref()
            .and_then(|dev| dev.max_effort_error());

        let overall_quality = if cycle.observation_assessment.has_invalid() {
            ObservationQuality::Invalid
        } else if cycle.observation_assessment.has_missing() {
            ObservationQuality::Missing
        } else if cycle.observation_assessment.has_stale() {
            ObservationQuality::Stale
        } else {
            ObservationQuality::Valid
        };

        Self {
            step_index: cycle.step_index,
            timestamp_secs: cycle.timestamp.as_secs_f64(),
            joint_drifts,
            max_position_drift,
            max_velocity_drift,
            max_effort_drift,
            overall_quality,
            policy_decision: cycle.policy_decision,
            execution_status: cycle.status_after_cycle,
        }
    }
}

/// Errors occurring during closed-loop trajectory execution.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExecutionError {
    EmptyTrajectory,
    DeviceError(DeviceIOError),
    ExecutionAborted,
}

impl std::fmt::Display for ExecutionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExecutionError::EmptyTrajectory => write!(f, "cannot execute empty trajectory"),
            ExecutionError::DeviceError(e) => write!(f, "device I/O error: {e}"),
            ExecutionError::ExecutionAborted => write!(f, "execution aborted by policy"),
        }
    }
}

impl std::error::Error for ExecutionError {}

/// Closed-loop trajectory execution feedback engine.
///
/// Orchestrates the temporal loop:
/// `Expected State(t)` -> `CommandSink` -> `Robot/Adapter` -> `ObservationSource` ->
/// `ObservationAssessment` & `StateDeviation` -> `ObservationResponsePolicy` -> `ExecutionStatus`.
pub struct ClosedLoopExecutor<C, S, O> {
    clock: C,
    command_sink: S,
    observation_source: O,
    requirement: ObservationRequirement,
    policy: ObservationResponsePolicy,
    status: ExecutionStatus,
    current_step: usize,
}

impl<C: ExecutionClock, S: CommandSink, O: ObservationSource> ClosedLoopExecutor<C, S, O> {
    pub fn new(
        clock: C,
        command_sink: S,
        observation_source: O,
        requirement: ObservationRequirement,
        policy: ObservationResponsePolicy,
    ) -> Self {
        Self {
            clock,
            command_sink,
            observation_source,
            requirement,
            policy,
            status: ExecutionStatus::Idle,
            current_step: 0,
        }
    }

    pub fn status(&self) -> ExecutionStatus {
        self.status
    }

    pub fn current_step(&self) -> usize {
        self.current_step
    }

    /// Reset executor state for a new execution sequence.
    pub fn reset(&mut self) {
        self.status = ExecutionStatus::Idle;
        self.current_step = 0;
    }

    /// Execute a single discrete feedback step along the given trajectory.
    pub fn tick(&mut self, trajectory: &Trajectory) -> Result<Option<ExecutionCycle>, ExecutionError> {
        if trajectory.is_empty() {
            return Err(ExecutionError::EmptyTrajectory);
        }

        if self.status == ExecutionStatus::Completed || self.status == ExecutionStatus::Aborted {
            return Ok(None);
        }

        if self.status == ExecutionStatus::Idle {
            self.status = ExecutionStatus::Running;
            self.current_step = 0;
        }

        // Clamp step to valid waypoints range
        let step_idx = self.current_step.min(trajectory.len() - 1);
        let waypoint = &trajectory.waypoints()[step_idx];
        let now = self.clock.now();

        // Convert trajectory waypoint to expected RobotState
        let expected_joints: Vec<JointState> = waypoint
            .joints()
            .iter()
            .map(|&p| JointState::position(p))
            .collect();
        let expected_state = RobotState::new(waypoint.timestamp(), expected_joints);

        // 1. Dispatch command target
        if let Err(e) = self.command_sink.send_command(&expected_state) {
            self.status = ExecutionStatus::Aborted;
            return Err(ExecutionError::DeviceError(e));
        }

        // 2. Read observation snapshot
        let observed_state = match self.observation_source.read_observation() {
            Ok(state) => state,
            Err(e) => {
                self.status = ExecutionStatus::Aborted;
                return Err(ExecutionError::DeviceError(e));
            }
        };

        // 3. Evaluate observation quality & kinematic deviation
        let sample_age = if now.as_secs_f64() >= observed_state.timestamp {
            Duration::from_secs_f64(now.as_secs_f64() - observed_state.timestamp)
        } else {
            Duration::from_secs(0)
        };

        let assessment = ObservationAssessment::evaluate(&self.requirement, &observed_state, sample_age);
        let deviation = StateDeviation::compute(&expected_state, &observed_state);

        // 4. Evaluate operational response policy
        let decision = self.policy.evaluate(&assessment, deviation.as_ref());

        // 5. Update execution status & step progression
        match decision {
            PolicyDecision::Continue => {
                self.status = ExecutionStatus::Running;
                if self.current_step + 1 >= trajectory.len() {
                    self.status = ExecutionStatus::Completed;
                } else {
                    self.current_step += 1;
                }
            }
            PolicyDecision::Hold => {
                self.status = ExecutionStatus::Holding;
            }
            PolicyDecision::Pause => {
                self.status = ExecutionStatus::Paused;
            }
            PolicyDecision::Abort => {
                self.status = ExecutionStatus::Aborted;
            }
        }

        Ok(Some(ExecutionCycle {
            step_index: step_idx,
            timestamp: now,
            expected_state,
            observed_state,
            observation_assessment: assessment,
            state_deviation: deviation,
            policy_decision: decision,
            status_after_cycle: self.status,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::capability::{JointObservationRequirement, ObservationConstraint};
    use crate::robot::io::{FakeRobotAdapter, FakeRobotScenario};
    use crate::robot::policy::DeviationThresholds;
    use crate::trajectory::TrajectoryPoint;

    fn make_test_trajectory() -> Trajectory {
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![0.5], 0.1),
            TrajectoryPoint::new(vec![1.0], 0.2),
        ])
    }

    fn make_default_requirement() -> ObservationRequirement {
        ObservationRequirement::uniform(
            1,
            JointObservationRequirement {
                position: Some(ObservationConstraint::max_staleness(Duration::from_millis(100))),
                velocity: None,
                effort: None,
            },
        )
    }

    fn make_default_policy() -> ObservationResponsePolicy {
        ObservationResponsePolicy::new(DeviationThresholds::position(0.05))
    }

    #[test]
    fn nominal_trajectory_execution_completes() {
        let clock = FakeExecutionClock::new();
        let adapter = FakeRobotAdapter::new(1).with_scenario(FakeRobotScenario::Nominal);
        let mut executor = ClosedLoopExecutor::new(
            clock.clone(),
            adapter.clone(),
            adapter,
            make_default_requirement(),
            make_default_policy(),
        );

        let traj = make_test_trajectory();

        // Tick 0: step 0 -> 0.0 rad
        let c0 = executor.tick(&traj).unwrap().unwrap();
        assert_eq!(c0.step_index, 0);
        assert_eq!(c0.policy_decision, PolicyDecision::Continue);
        assert_eq!(c0.status_after_cycle, ExecutionStatus::Running);

        // Tick 1: step 1 -> 0.5 rad
        clock.advance(Duration::from_millis(100));
        let c1 = executor.tick(&traj).unwrap().unwrap();
        assert_eq!(c1.step_index, 1);
        assert_eq!(c1.policy_decision, PolicyDecision::Continue);
        assert_eq!(c1.status_after_cycle, ExecutionStatus::Running);

        // Tick 2: step 2 -> 1.0 rad (Last step)
        clock.advance(Duration::from_millis(100));
        let c2 = executor.tick(&traj).unwrap().unwrap();
        assert_eq!(c2.step_index, 2);
        assert_eq!(c2.policy_decision, PolicyDecision::Continue);
        assert_eq!(c2.status_after_cycle, ExecutionStatus::Completed);

        // Subsequent tick returns None (already completed)
        assert!(executor.tick(&traj).unwrap().is_none());
    }

    #[test]
    fn position_deviation_triggers_pause() {
        let clock = FakeExecutionClock::new();
        let adapter = FakeRobotAdapter::new(1).with_scenario(FakeRobotScenario::PositionDeviation(0.125));
        let mut executor = ClosedLoopExecutor::new(
            clock,
            adapter.clone(),
            adapter,
            make_default_requirement(),
            make_default_policy(),
        );

        let traj = make_test_trajectory();

        // Tick 0: Deviation 0.125 > 0.05 tolerance -> Pause
        let c0 = executor.tick(&traj).unwrap().unwrap();
        assert_eq!(c0.policy_decision, PolicyDecision::Pause);
        assert_eq!(c0.status_after_cycle, ExecutionStatus::Paused);
        assert_eq!(executor.current_step(), 0); // Did NOT advance step!
    }

    #[test]
    fn stale_observation_triggers_hold_and_recovers() {
        let clock = FakeExecutionClock::new();
        // Telemetry stale by 200ms (> 100ms max staleness requirement)
        let adapter = FakeRobotAdapter::new(1).with_scenario(FakeRobotScenario::StaleObservation(Duration::from_millis(200)));
        let mut executor = ClosedLoopExecutor::new(
            clock.clone(),
            adapter.clone(),
            adapter.clone(),
            make_default_requirement(),
            make_default_policy(),
        );

        let traj = make_test_trajectory();

        // Tick 0: Stale observation -> Hold
        let c0 = executor.tick(&traj).unwrap().unwrap();
        assert_eq!(c0.policy_decision, PolicyDecision::Hold);
        assert_eq!(c0.status_after_cycle, ExecutionStatus::Holding);
        assert_eq!(executor.current_step(), 0);

        // Recover telemetry quality on adapter to Nominal
        adapter.set_scenario(FakeRobotScenario::Nominal);

        let c1 = executor.tick(&traj).unwrap().unwrap();
        assert_eq!(c1.policy_decision, PolicyDecision::Continue);
        assert_eq!(c1.status_after_cycle, ExecutionStatus::Running);
    }

    #[test]
    fn invalid_observation_triggers_abort() {
        let clock = FakeExecutionClock::new();
        let adapter = FakeRobotAdapter::new(1).with_scenario(FakeRobotScenario::InvalidObservation);
        let mut executor = ClosedLoopExecutor::new(
            clock,
            adapter.clone(),
            adapter,
            make_default_requirement(),
            make_default_policy(),
        );

        let traj = make_test_trajectory();

        let c0 = executor.tick(&traj).unwrap().unwrap();
        assert_eq!(c0.policy_decision, PolicyDecision::Abort);
        assert_eq!(c0.status_after_cycle, ExecutionStatus::Aborted);
        assert_eq!(executor.status(), ExecutionStatus::Aborted);
    }

    #[test]
    fn drift_report_generation_from_execution_cycle() {
        let clock = FakeExecutionClock::new();
        let adapter = FakeRobotAdapter::new(1).with_scenario(FakeRobotScenario::PositionDeviation(0.04));
        let mut executor = ClosedLoopExecutor::new(
            clock,
            adapter.clone(),
            adapter,
            make_default_requirement(),
            make_default_policy(),
        );

        let traj = make_test_trajectory();
        let cycle = executor.tick(&traj).unwrap().unwrap();
        let report = cycle.drift_report();

        assert_eq!(report.step_index, 0);
        assert_eq!(report.joint_drifts.len(), 1);
        assert_eq!(report.joint_drifts[0].expected_position, Some(0.0));
        assert_eq!(report.joint_drifts[0].observed_position, Some(0.04));
        assert!((report.joint_drifts[0].position_drift.unwrap() - 0.04).abs() < 1e-6);
        assert_eq!(report.max_position_drift, Some(0.04));
        assert_eq!(report.overall_quality, ObservationQuality::Valid);
        assert_eq!(report.policy_decision, PolicyDecision::Continue);
    }
}
