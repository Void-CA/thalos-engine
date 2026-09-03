use serde::{Deserialize, Serialize};
use thalos_engine::prelude::ExecutionSessionId;
use crate::execution::coordinator::ExecutionError;
use crate::execution::executor::{ExecutionExecutor, ExecutionSessionState};
use crate::execution::observation::{
    ExecutionSnapshot, ObservationSnapshot, RunSnapshot, SignalQuality,
};
use crate::execution::preflight::{ExecutionPreflight, PreflightCheck, PreflightCheckKind};
use crate::execution::hardware::command::RobotCommand;
use thalos_ports::robot::RobotObservation;
use crate::ports::robot::transport::{RobotTransport, TransportState};

/// TrackingState (ADR-014)
/// Operational state tracking physical convergence against plan waypoints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrackingState {
    Idle,
    AwaitingObservation,
    Tracking,
    WaypointReached,
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
    pub last_observation: Option<RobotObservation>,
    pub position_tolerance_rad: f64,
    pub elapsed_seconds: f64,
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
            last_observation: None,
            position_tolerance_rad,
            elapsed_seconds: 0.0,
        }
    }

    /// Advance execution by `dt` seconds: poll transport observation and evaluate tracking.
    pub fn tick(&mut self, dt: f64) -> f64 {
        if self.state != ExecutionSessionState::Running {
            return self.progress();
        }

        self.elapsed_seconds += dt;

        // 1. Poll observation from transport
        if let Ok(Some(obs)) = self.transport.try_receive_observation() {
            self.last_observation = Some(obs);
        }

        // 2. Evaluate physical convergence against current target waypoint
        if let Some(target_waypoint) = self.waypoints.get(self.current_waypoint_idx).cloned() {
            if let Some(ref obs) = self.last_observation {
                let max_error = target_waypoint
                    .iter()
                    .zip(&obs.joint_positions_rad)
                    .map(|(exp, obs_val)| (*exp - *obs_val).abs())
                    .fold(0.0f64, f64::max);

                if max_error <= self.position_tolerance_rad {
                    self.tracking_state = TrackingState::WaypointReached;
                    self.current_waypoint_idx += 1;

                    // Check if plan completed
                    if self.current_waypoint_idx >= self.waypoints.len() {
                        self.state = ExecutionSessionState::Completed;
                        self.tracking_state = TrackingState::Idle;
                    } else {
                        // Dispatch next waypoint command
                        self.dispatch_current_waypoint();
                    }
                } else {
                    self.tracking_state = TrackingState::Tracking;
                }
            } else {
                self.tracking_state = TrackingState::AwaitingObservation;
            }
        }

        self.progress()
    }

    fn dispatch_current_waypoint(&mut self) {
        if let Some(waypoint) = self.waypoints.get(self.current_waypoint_idx) {
            let cmd = RobotCommand::MoveJoints {
                positions_rad: waypoint.clone(),
                velocities_rad_s: None,
            };
            if self.transport.send(cmd).is_ok() {
                self.tracking_state = TrackingState::AwaitingObservation;
            }
        }
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

        self.dispatch_current_waypoint();
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
            self.dispatch_current_waypoint();
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

        let (obs_event, quality) = if let Some(ref obs) = self.last_observation {
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

    #[test]
    fn test_hardware_executor_lifecycle_with_fake_transport() {
        let session_id = ExecutionSessionId("session-hw-01".into());
        let waypoints = vec![vec![0.5, 0.2, 0.0], vec![1.0, 0.5, -0.2]];
        let transport = FakeRobotTransport::new();
        let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.05);

        assert_eq!(executor.state(), ExecutionSessionState::Dispatched);

        // Start dispatches waypoint 0 to transport
        executor.start().unwrap();
        assert_eq!(executor.state(), ExecutionSessionState::Running);
        assert_eq!(executor.transport.sent_commands.len(), 1);
        assert_eq!(executor.current_waypoint_idx, 0);

        // Tick without observation -> stays on waypoint 0, progress = 0.0
        let progress = executor.tick(0.1);
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
        executor.tick(0.1);
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
        executor.tick(0.1);
        assert_eq!(executor.state(), ExecutionSessionState::Completed);
        assert_eq!(executor.progress(), 1.0);
    }

    #[test]
    fn test_hardware_executor_out_of_tolerance_does_not_advance() {
        let session_id = ExecutionSessionId("session-hw-02".into());
        let waypoints = vec![vec![1.0, 1.0, 1.0]];
        let transport = FakeRobotTransport::new();
        let mut executor = HardwareExecutor::new(session_id, waypoints, transport, 0.05);

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

        executor.tick(0.1);
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
}
