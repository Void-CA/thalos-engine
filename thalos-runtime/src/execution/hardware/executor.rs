use std::time::{Duration, Instant};
use serde::{Deserialize, Serialize};
use thalos_engine::prelude::ExecutionSessionId;
use crate::execution::coordinator::ExecutionError;
use crate::execution::executor::{ExecutionExecutor, ExecutionSessionState};
use crate::execution::observation::{
    ExecutionSnapshot, ObservationSnapshot, RunSnapshot, SignalQuality,
};
use crate::execution::preflight::{ExecutionPreflight, PreflightCheck, PreflightCheckKind};
use crate::execution::hardware::command::RobotCommand;
use thalos_ports::robot::{RobotObservation, TransportError};
use crate::ports::robot::transport::{RobotTransport, TransportState};

/// TrackingState (ADR-014)
/// Operational state tracking physical convergence against plan waypoints.
/// This is INTERNAL to HardwareExecutor and does NOT compete with ExecutionSessionState.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrackingState {
    Idle,
    AwaitingObservation,
    Tracking,
    WaypointReached,
}

/// Physical execution error conditions detected during the closed loop.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum PhysicalExecutionError {
    /// Transport connection lost during execution.
    TransportLost,
    /// Command could not be delivered to transport.
    CommandFailed(String),
    /// No valid observation received within the deadline.
    ObservationTimeout,
    /// Observation received but too old to be used as evidence.
    ObservationStale { age: Duration, limit: Duration },
    /// Observation received but flagged as untrustworthy.
    ObservationInvalid { quality: SignalQuality },
    /// Robot received commands but did not converge within deadline.
    TargetNotReached { elapsed: Duration, timeout: Duration },
    /// Robot reported an internal fault.
    RobotFault(String),
}

impl std::fmt::Display for PhysicalExecutionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TransportLost => write!(f, "Transport connection lost"),
            Self::CommandFailed(e) => write!(f, "Command delivery failed: {e}"),
            Self::ObservationTimeout => write!(f, "Observation timeout: no valid evidence within deadline"),
            Self::ObservationStale { age, limit } => {
                write!(f, "Observation stale: age {:?} exceeds limit {:?}", age, limit)
            }
            Self::ObservationInvalid { quality } => {
                write!(f, "Observation invalid: quality {:?}", quality)
            }
            Self::TargetNotReached { elapsed, timeout } => {
                write!(f, "Target not reached: elapsed {:?}, timeout {:?}", elapsed, timeout)
            }
            Self::RobotFault(reason) => write!(f, "Robot fault: {reason}"),
        }
    }
}

impl std::error::Error for PhysicalExecutionError {}

/// Configuration for HardwareExecutor safety timeouts.
#[derive(Debug, Clone)]
pub struct HardwareSafetyConfig {
    /// Maximum time to wait for a valid observation after dispatching a command.
    pub observation_timeout: Duration,
    /// Maximum age of an observation to be considered fresh evidence.
    pub observation_staleness_limit: Duration,
    /// Maximum time to wait for the robot to converge to a waypoint.
    pub convergence_timeout: Duration,
}

impl Default for HardwareSafetyConfig {
    fn default() -> Self {
        Self {
            observation_timeout: Duration::from_millis(500),
            observation_staleness_limit: Duration::from_secs(1),
            convergence_timeout: Duration::from_secs(10),
        }
    }
}

/// HardwareExecutor (ADR-014)
/// Execution adapter dispatching RobotCommands over a RobotTransport
/// and advancing waypoints based solely on observed physical telemetry.
#[derive(Debug)]
pub struct HardwareExecutor<T: RobotTransport> {
    pub session_id: ExecutionSessionId,
    pub waypoints: Vec<Vec<f64>>,
    pub current_waypoint_idx: usize,
    pub state: ExecutionSessionState,
    pub tracking_state: TrackingState,
    pub transport: T,
    pub position_tolerance_rad: f64,
    pub elapsed_seconds: f64,

    // Safety configuration
    pub safety: HardwareSafetyConfig,

    // Observation tracking
    /// Last raw observation received from transport (may be stale/invalid).
    last_observation: Option<RobotObservation>,
    /// When the last observation was received (local monotonic clock).
    last_observation_at: Option<Instant>,
    /// Current valid evidence for convergence evaluation (validated this cycle).
    current_evidence: Option<RobotObservation>,
    /// When the current waypoint dispatch started.
    waypoint_started_at: Option<Instant>,
    /// Reason for failure, if any.
    pub fault_reason: Option<String>,
}

impl<T: RobotTransport> HardwareExecutor<T> {
    pub fn new(
        session_id: ExecutionSessionId,
        waypoints: Vec<Vec<f64>>,
        transport: T,
        position_tolerance_rad: f64,
    ) -> Self {
        Self {
            session_id,
            waypoints,
            current_waypoint_idx: 0,
            state: ExecutionSessionState::Dispatched,
            tracking_state: TrackingState::Idle,
            transport,
            position_tolerance_rad,
            elapsed_seconds: 0.0,
            safety: HardwareSafetyConfig::default(),
            last_observation: None,
            last_observation_at: None,
            current_evidence: None,
            waypoint_started_at: None,
            fault_reason: None,
        }
    }

    /// Builder-style constructor with custom safety configuration.
    pub fn with_safety(mut self, safety: HardwareSafetyConfig) -> Self {
        self.safety = safety;
        self
    }

    /// Fault the session with a reason. Transitions to Failed state.
    fn fail(&mut self, error: PhysicalExecutionError) -> f64 {
        self.state = ExecutionSessionState::Failed;
        self.tracking_state = TrackingState::Idle;
        self.fault_reason = Some(error.to_string());
        self.progress()
    }

    /// Check if the transport is still connected.
    fn check_transport(&self) -> Result<(), PhysicalExecutionError> {
        if self.transport.state() != TransportState::Connected {
            return Err(PhysicalExecutionError::TransportLost);
        }
        Ok(())
    }

    /// Validate whether an observation is usable as current evidence.
    fn validate_observation(
        &self,
        obs: &RobotObservation,
    ) -> Result<(), PhysicalExecutionError> {
        // Check staleness based on local reception time
        if let Some(last_at) = self.last_observation_at {
            let age = last_at.elapsed();
            if age > self.safety.observation_staleness_limit {
                return Err(PhysicalExecutionError::ObservationStale {
                    age,
                    limit: self.safety.observation_staleness_limit,
                });
            }
        }

        // Check signal quality
        if obs.signal_quality == SignalQuality::Invalid {
            return Err(PhysicalExecutionError::ObservationInvalid {
                quality: obs.signal_quality,
            });
        }

        Ok(())
    }

    /// Check if we've exceeded the observation timeout while awaiting.
    fn check_observation_timeout(&self) -> Result<(), PhysicalExecutionError> {
        if self.tracking_state == TrackingState::AwaitingObservation {
            if let Some(started) = self.waypoint_started_at {
                if started.elapsed() > self.safety.observation_timeout {
                    return Err(PhysicalExecutionError::ObservationTimeout);
                }
            }
        }
        Ok(())
    }

    /// Check if the robot has failed to converge within deadline.
    fn check_convergence_timeout(&self) -> Result<(), PhysicalExecutionError> {
        if self.tracking_state == TrackingState::Tracking {
            if let Some(started) = self.waypoint_started_at {
                if started.elapsed() > self.safety.convergence_timeout {
                    return Err(PhysicalExecutionError::TargetNotReached {
                        elapsed: started.elapsed(),
                        timeout: self.safety.convergence_timeout,
                    });
                }
            }
        }
        Ok(())
    }

    /// Compute maximum absolute joint error between target and observed positions.
    fn compute_max_error(target: &[f64], observed: &[f64]) -> f64 {
        target
            .iter()
            .zip(observed)
            .map(|(t, o)| (t - o).abs())
            .fold(0.0f64, f64::max)
    }

    /// Advance execution by `dt` seconds: poll transport observation and evaluate tracking.
    /// Returns Ok(progress) or Err(PhysicalExecutionError) if a safety condition is violated.
    pub fn tick(&mut self, dt: f64) -> Result<f64, PhysicalExecutionError> {
        if self.state != ExecutionSessionState::Running {
            return Ok(self.progress());
        }

        self.elapsed_seconds += dt;

        // 0. Transport connectivity check
        self.check_transport()?;

        // 1. Poll observation from transport
        if let Ok(Some(obs)) = self.transport.try_receive_observation() {
            self.last_observation = Some(obs);
            self.last_observation_at = Some(Instant::now());
        }

        // 2. Validate observation and extract current evidence
        self.current_evidence = None;
        if let Some(ref obs) = self.last_observation {
            match self.validate_observation(obs) {
                Ok(()) => {
                    self.current_evidence = Some(obs.clone());
                }
                Err(PhysicalExecutionError::ObservationStale { .. }) => {
                    // Stale observation: do not use as evidence
                }
                Err(PhysicalExecutionError::ObservationInvalid { .. }) => {
                    // Invalid observation: do not use as evidence
                }
                Err(_) => {}
            }
        }

        // 3. Check observation timeout (if awaiting)
        self.check_observation_timeout()?;

        // 4. Evaluate convergence against current waypoint using validated evidence
        if let Some(target_waypoint) = self.waypoints.get(self.current_waypoint_idx).cloned() {
            if let Some(ref evidence) = self.current_evidence {
                let max_error = Self::compute_max_error(&target_waypoint, &evidence.joint_positions_rad);

                if max_error <= self.position_tolerance_rad {
                    self.tracking_state = TrackingState::WaypointReached;
                    self.current_waypoint_idx += 1;

                    // Check if plan completed
                    if self.current_waypoint_idx >= self.waypoints.len() {
                        self.state = ExecutionSessionState::Completed;
                        self.tracking_state = TrackingState::Idle;
                        return Ok(self.progress());
                    } else {
                        // Dispatch next waypoint command
                        self.dispatch_current_waypoint()?;
                    }
                } else {
                    self.tracking_state = TrackingState::Tracking;
                    // 5. Check convergence timeout
                    self.check_convergence_timeout()?;
                }
            } else {
                self.tracking_state = TrackingState::AwaitingObservation;
            }
        }

        Ok(self.progress())
    }

    fn dispatch_current_waypoint(&mut self) -> Result<(), PhysicalExecutionError> {
        let waypoint = self.waypoints
            .get(self.current_waypoint_idx)
            .ok_or_else(|| PhysicalExecutionError::RobotFault(
                format!("No waypoint at index {}", self.current_waypoint_idx)
            ))?;

        let cmd = RobotCommand::MoveJoints {
            positions_rad: waypoint.clone(),
            velocities_rad_s: None,
        };

        self.transport.send(cmd).map_err(|e| match e {
            TransportError::Disconnected => PhysicalExecutionError::TransportLost,
            TransportError::NotReady => PhysicalExecutionError::TransportLost,
            TransportError::Timeout => PhysicalExecutionError::CommandFailed("Transport timeout".into()),
            TransportError::CommunicationFailure(msg) => PhysicalExecutionError::CommandFailed(msg),
        })?;

        self.tracking_state = TrackingState::AwaitingObservation;
        self.last_observation = None;
        self.last_observation_at = None;
        self.current_evidence = None;
        self.waypoint_started_at = Some(Instant::now());
        Ok(())
    }

    pub fn progress(&self) -> f64 {
        if self.state.is_terminal() {
            return 1.0;
        }
        if self.waypoints.is_empty() {
            return 1.0;
        }
        (self.current_waypoint_idx as f64 / self.waypoints.len() as f64).clamp(0.0, 1.0)
    }
}

impl<T: RobotTransport> ExecutionExecutor for HardwareExecutor<T> {
    fn start(&mut self) -> Result<(), ExecutionError> {
        if self.state != ExecutionSessionState::Dispatched && self.state != ExecutionSessionState::Reserved {
            return Err(ExecutionError::InvalidSessionState(self.state));
        }

        if self.transport.state() != TransportState::Connected {
            let preflight = ExecutionPreflight::new(vec![PreflightCheck::fail(
                PreflightCheckKind::Transport,
                "Transport is not connected",
            )]);
            return Err(ExecutionError::PreflightFailed(preflight));
        }

        self.state = ExecutionSessionState::Running;
        self.current_waypoint_idx = 0;
        self.elapsed_seconds = 0.0;
        self.fault_reason = None;

        self.dispatch_current_waypoint().map_err(|e| {
            ExecutionError::PreflightFailed(ExecutionPreflight::new(vec![PreflightCheck::fail(
                PreflightCheckKind::Transport,
                format!("Failed to dispatch first waypoint: {e}"),
            )]))
        })?;
        Ok(())
    }

    fn pause(&mut self) -> Result<(), ExecutionError> {
        if self.state == ExecutionSessionState::Running {
            self.state = ExecutionSessionState::Paused;
            let _ = self.transport.stop();
        }
        Ok(())
    }

    fn resume(&mut self) -> Result<(), ExecutionError> {
        if self.state == ExecutionSessionState::Paused {
            self.state = ExecutionSessionState::Running;
            if let Err(e) = self.dispatch_current_waypoint() {
                self.state = ExecutionSessionState::Failed;
                self.fault_reason = Some(e.to_string());
            }
        }
        Ok(())
    }

    fn cancel(&mut self) -> Result<(), ExecutionError> {
        if !self.state.is_terminal() {
            self.state = ExecutionSessionState::Cancelled;
            let _ = self.transport.stop();
        }
        Ok(())
    }

    fn state(&self) -> ExecutionSessionState {
        self.state
    }

    fn snapshot(&self) -> RunSnapshot {
        let exec_snap = ExecutionSnapshot {
            session_id: self.session_id.clone(),
            state: self.state,
            elapsed_seconds: self.elapsed_seconds,
            progress: self.progress(),
        };

        let (obs_event, quality) = if let Some(ref obs) = self.current_evidence {
            (
                crate::execution::observation::Observation {
                    session_id: Some(self.session_id.clone()),
                    sequence: obs.sequence,
                    sampled_at_ns: obs.sampled_at_ns,
                    received_at_ns: (self.elapsed_seconds * 1e9) as u64,
                    joint_positions: obs.joint_positions_rad.clone(),
                    joint_velocities: obs.joint_velocities_rad_s.clone(),
                    tcp_pose: obs.tcp_pose.unwrap_or([0.0; 7]),
                    signal_quality: obs.signal_quality,
                },
                obs.signal_quality,
            )
        } else {
            let now_ns = (self.elapsed_seconds * 1e9) as u64;
            (
                crate::execution::observation::Observation {
                    session_id: Some(self.session_id.clone()),
                    sequence: 0,
                    sampled_at_ns: now_ns,
                    received_at_ns: now_ns,
                    joint_positions: vec![0.0; 3],
                    joint_velocities: vec![0.0; 3],
                    tcp_pose: [0.0; 7],
                    signal_quality: SignalQuality::Nominal,
                },
                SignalQuality::Nominal,
            )
        };

        let obs_snap = ObservationSnapshot {
            latest: obs_event.clone(),
            signal_quality: quality,
            freshness_ns: 0,
        };

        let expected_joints = self
            .waypoints
            .get(self.current_waypoint_idx)
            .cloned()
            .unwrap_or_else(|| vec![0.0; obs_snap.latest.joint_positions.len()]);

        let dev = RunSnapshot::compute_deviation(&expected_joints, (self.elapsed_seconds * 1e9) as u64, &obs_event);

        RunSnapshot {
            execution: exec_snap,
            observation: obs_snap,
            deviation: dev,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::execution::hardware::fake::FakeRobotTransport;

    fn default_safety() -> HardwareSafetyConfig {
        HardwareSafetyConfig {
            observation_timeout: Duration::from_millis(500),
            observation_staleness_limit: Duration::from_secs(1),
            convergence_timeout: Duration::from_secs(5),
        }
    }

    #[test]
    fn test_hardware_executor_lifecycle_with_fake_transport() {
        let session_id = ExecutionSessionId("session-hw-01".into());
        let waypoints = vec![vec![0.5, 0.2, 0.0], vec![1.0, 0.5, -0.2]];
        let transport = FakeRobotTransport::new();
        let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.05)
            .with_safety(default_safety());

        assert_eq!(executor.state(), ExecutionSessionState::Dispatched);

        // Start dispatches waypoint 0 to transport
        executor.start().unwrap();
        assert_eq!(executor.state(), ExecutionSessionState::Running);
        assert_eq!(executor.transport.sent_commands.len(), 1);
        assert_eq!(executor.current_waypoint_idx, 0);

        // Tick without observation -> stays on waypoint 0, progress = 0.0
        let progress = executor.tick(0.1).unwrap();
        assert_eq!(progress, 0.0);
        assert_eq!(executor.tracking_state, TrackingState::AwaitingObservation);

        // Inject observation matching waypoint 0
        executor.transport.push_observation(RobotObservation {
            sampled_at_ns: 1000,
            sequence: 1,
            joint_positions_rad: vec![0.5, 0.2, 0.0],
            joint_velocities_rad_s: vec![0.0, 0.0, 0.0],
            tcp_pose: None,
            signal_quality: SignalQuality::Nominal,
        });

        // Tick processes observation -> advances to waypoint 1 and dispatches command
        executor.tick(0.1).unwrap();
        assert_eq!(executor.current_waypoint_idx, 1);
        assert_eq!(executor.transport.sent_commands.len(), 2);

        // Inject observation matching waypoint 1
        executor.transport.push_observation(RobotObservation {
            sampled_at_ns: 2000,
            sequence: 2,
            joint_positions_rad: vec![1.0, 0.5, -0.2],
            joint_velocities_rad_s: vec![0.0, 0.0, 0.0],
            tcp_pose: None,
            signal_quality: SignalQuality::Nominal,
        });

        // Tick completes plan
        executor.tick(0.1).unwrap();
        assert_eq!(executor.state(), ExecutionSessionState::Completed);
        assert_eq!(executor.progress(), 1.0);
    }

    #[test]
    fn test_hardware_executor_out_of_tolerance_does_not_advance() {
        let session_id = ExecutionSessionId("session-hw-02".into());
        let waypoints = vec![vec![1.0, 1.0, 1.0]];
        let transport = FakeRobotTransport::new();
        let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.05)
            .with_safety(default_safety());

        executor.start().unwrap();

        // Inject observation far out of tolerance (0.5 vs 1.0)
        executor.transport.push_observation(RobotObservation {
            sampled_at_ns: 1000,
            sequence: 1,
            joint_positions_rad: vec![0.5, 0.5, 0.5],
            joint_velocities_rad_s: vec![0.0, 0.0, 0.0],
            tcp_pose: None,
            signal_quality: SignalQuality::Nominal,
        });

        executor.tick(0.1).unwrap();
        assert_eq!(executor.state(), ExecutionSessionState::Running);
        assert_eq!(executor.current_waypoint_idx, 0);
        assert_eq!(executor.tracking_state, TrackingState::Tracking);
    }

    #[test]
    fn test_hardware_executor_cancel_stops_transport() {
        let session_id = ExecutionSessionId("session-hw-03".into());
        let waypoints = vec![vec![1.0, 1.0, 1.0]];
        let transport = FakeRobotTransport::new();
        let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.05);

        executor.start().unwrap();
        executor.cancel().unwrap();

        assert_eq!(executor.state(), ExecutionSessionState::Cancelled);
        assert_eq!(executor.transport.sent_commands.last(), Some(&RobotCommand::Stop));
    }

    #[test]
    fn test_transport_lost_faults_session() {
        let session_id = ExecutionSessionId("session-hw-04".into());
        let waypoints = vec![vec![0.5, 0.2, 0.0]];
        let mut transport = FakeRobotTransport::new();
        transport.state = TransportState::Connected;
        let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.05);

        executor.start().unwrap();

        // Simulate transport disconnection
        executor.transport.state = TransportState::Disconnected;

        let result = executor.tick(0.1);
        assert!(result.is_err());
        assert_eq!(executor.state(), ExecutionSessionState::Failed);
        assert!(matches!(
            result.unwrap_err(),
            PhysicalExecutionError::TransportLost
        ));
    }

    #[test]
    fn test_observation_timeout_faults_session() {
        let session_id = ExecutionSessionId("session-hw-05".into());
        let waypoints = vec![vec![0.5, 0.2, 0.0]];
        let transport = FakeRobotTransport::new();
        let safety = HardwareSafetyConfig {
            observation_timeout: Duration::from_millis(100),
            ..default_safety()
        };
        let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.05)
            .with_safety(safety);

        executor.start().unwrap();

        // Tick multiple times without observation to exceed timeout
        std::thread::sleep(Duration::from_millis(150));
        let result = executor.tick(0.1);

        assert!(result.is_err());
        assert_eq!(executor.state(), ExecutionSessionState::Failed);
        assert!(matches!(
            result.unwrap_err(),
            PhysicalExecutionError::ObservationTimeout
        ));
    }

    #[test]
    fn test_stale_observation_not_used_as_evidence() {
        let session_id = ExecutionSessionId("session-hw-06".into());
        let waypoints = vec![vec![0.5, 0.2, 0.0]];
        let transport = FakeRobotTransport::new();
        let safety = HardwareSafetyConfig {
            observation_staleness_limit: Duration::from_millis(50),
            ..default_safety()
        };
        let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.05)
            .with_safety(safety);

        executor.start().unwrap();

        // Inject observation
        executor.transport.push_observation(RobotObservation {
            sampled_at_ns: 1000,
            sequence: 1,
            joint_positions_rad: vec![0.5, 0.2, 0.0],
            joint_velocities_rad_s: vec![0.0, 0.0, 0.0],
            tcp_pose: None,
            signal_quality: SignalQuality::Nominal,
        });

        // Tick to consume observation
        executor.tick(0.01).unwrap();
        assert_eq!(executor.current_waypoint_idx, 1);

        // Now inject another observation for waypoint 1
        executor.transport.push_observation(RobotObservation {
            sampled_at_ns: 2000,
            sequence: 2,
            joint_positions_rad: vec![0.5, 0.2, 0.0], // Wrong position for waypoint 1
            joint_velocities_rad_s: vec![0.0, 0.0, 0.0],
            tcp_pose: None,
            signal_quality: SignalQuality::Nominal,
        });

        // Wait for staleness
        std::thread::sleep(Duration::from_millis(80));

        // Tick: observation should be stale, not used as evidence
        executor.tick(0.01).unwrap();
        // Should still be on waypoint 1 because stale evidence was discarded
        assert_eq!(executor.current_waypoint_idx, 1);
        assert_eq!(executor.tracking_state, TrackingState::AwaitingObservation);
    }

    #[test]
    fn test_invalid_observation_not_used_as_evidence() {
        let session_id = ExecutionSessionId("session-hw-07".into());
        let waypoints = vec![vec![0.5, 0.2, 0.0]];
        let transport = FakeRobotTransport::new();
        let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.05);

        executor.start().unwrap();

        // Inject invalid observation (matching position but invalid quality)
        executor.transport.push_observation(RobotObservation {
            sampled_at_ns: 1000,
            sequence: 1,
            joint_positions_rad: vec![0.5, 0.2, 0.0],
            joint_velocities_rad_s: vec![0.0, 0.0, 0.0],
            tcp_pose: None,
            signal_quality: SignalQuality::Invalid,
        });

        executor.tick(0.1).unwrap();
        // Should still be on waypoint 0 because invalid evidence was discarded
        assert_eq!(executor.current_waypoint_idx, 0);
        assert_eq!(executor.tracking_state, TrackingState::AwaitingObservation);
    }

    #[test]
    fn test_convergence_timeout_faults_session() {
        let session_id = ExecutionSessionId("session-hw-08".into());
        let waypoints = vec![vec![0.5, 0.2, 0.0]];
        let transport = FakeRobotTransport::new();
        let safety = HardwareSafetyConfig {
            convergence_timeout: Duration::from_millis(100),
            ..default_safety()
        };
        let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.05)
            .with_safety(safety);

        executor.start().unwrap();

        // Inject observation that never converges
        executor.transport.push_observation(RobotObservation {
            sampled_at_ns: 1000,
            sequence: 1,
            joint_positions_rad: vec![0.0, 0.0, 0.0], // Wrong position
            joint_velocities_rad_s: vec![0.0, 0.0, 0.0],
            tcp_pose: None,
            signal_quality: SignalQuality::Nominal,
        });

        // Wait for convergence timeout
        std::thread::sleep(Duration::from_millis(150));
        let result = executor.tick(0.1);

        assert!(result.is_err());
        assert_eq!(executor.state(), ExecutionSessionState::Failed);
        assert!(matches!(
            result.unwrap_err(),
            PhysicalExecutionError::TargetNotReached { .. }
        ));
    }

    #[test]
    fn test_command_send_failure_faults_session() {
        let session_id = ExecutionSessionId("session-hw-09".into());
        let waypoints = vec![vec![0.5, 0.2, 0.0], vec![1.0, 0.5, -0.2]];
        let mut transport = FakeRobotTransport::new();
        transport.should_fail_send = true;
        let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.05);

        let result = executor.start();
        assert!(result.is_err());
        assert_eq!(executor.state(), ExecutionSessionState::Failed);
    }
}
