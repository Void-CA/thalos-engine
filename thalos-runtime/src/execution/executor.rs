use serde::{Deserialize, Serialize};
use thalos_engine::prelude::*;
use super::coordinator::ExecutionError;

/// ExecutionSessionState (ADR-014)
/// Formal operational state machine for active execution sessions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExecutionSessionState {
    Created,
    Reserved,
    Dispatched,
    Running,
    Paused,
    Completed,
    Cancelled,
    Failed,
}

impl ExecutionSessionState {
    pub fn is_terminal(&self) -> bool {
        matches!(self, Self::Completed | Self::Cancelled | Self::Failed)
    }

    pub fn is_active(&self) -> bool {
        matches!(self, Self::Running | Self::Paused)
    }
}

use super::observation::{
    ExecutionSnapshot, ObservationSnapshot, RunSnapshot, SignalQuality,
};

/// ExecutionExecutor (ADR-014)
/// Abstraction trait for executing motion plans on simulation or physical transport targets.
pub trait ExecutionExecutor: Send + Sync {
    fn start(&mut self) -> Result<(), ExecutionError>;
    fn pause(&mut self) -> Result<(), ExecutionError>;
    fn resume(&mut self) -> Result<(), ExecutionError>;
    fn cancel(&mut self) -> Result<(), ExecutionError>;
    fn state(&self) -> ExecutionSessionState;
    fn snapshot(&self) -> RunSnapshot;
}


/// SimulationExecutor (ADR-014)
/// Deterministic in-memory simulation executor for offline plan validation.
#[derive(Debug)]
pub struct SimulationExecutor {
    pub session_id: ExecutionSessionId,
    pub trajectory_duration: f64,
    pub current_time: f64,
    pub state: ExecutionSessionState,
}

impl SimulationExecutor {
    pub fn new(session_id: ExecutionSessionId, trajectory_duration: f64) -> Self {
        Self {
            session_id,
            trajectory_duration,
            current_time: 0.0,
            state: ExecutionSessionState::Dispatched,
        }
    }

    /// Advance simulated execution time by `dt` seconds.
    pub fn tick(&mut self, dt: f64) -> f64 {
        if self.state != ExecutionSessionState::Running {
            return self.progress();
        }

        self.current_time += dt;
        if self.trajectory_duration > 0.0 && self.current_time >= self.trajectory_duration {
            self.current_time = self.trajectory_duration;
            self.state = ExecutionSessionState::Completed;
        }

        self.progress()
    }

    /// Calculated progress fraction (0.0 to 1.0).
    pub fn progress(&self) -> f64 {
        if self.state.is_terminal() {
            return 1.0;
        }
        if self.trajectory_duration <= 0.0 {
            return 1.0;
        }
        (self.current_time / self.trajectory_duration).clamp(0.0, 1.0)
    }
}

impl ExecutionExecutor for SimulationExecutor {
    fn start(&mut self) -> Result<(), ExecutionError> {
        if self.state != ExecutionSessionState::Dispatched && self.state != ExecutionSessionState::Reserved {
            return Err(ExecutionError::InvalidSessionState(self.state));
        }
        self.state = ExecutionSessionState::Running;
        self.current_time = 0.0;
        Ok(())
    }

    fn pause(&mut self) -> Result<(), ExecutionError> {
        if self.state == ExecutionSessionState::Running {
            self.state = ExecutionSessionState::Paused;
        }
        Ok(())
    }

    fn resume(&mut self) -> Result<(), ExecutionError> {
        if self.state == ExecutionSessionState::Paused {
            self.state = ExecutionSessionState::Running;
        }
        Ok(())
    }

    fn cancel(&mut self) -> Result<(), ExecutionError> {
        if !self.state.is_terminal() {
            self.state = ExecutionSessionState::Cancelled;
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
            elapsed_seconds: self.current_time,
            progress: self.progress(),
        };

        // Simulated joint positions interpolating smoothly
        let frac = self.progress();
        let sim_joints = vec![frac * 1.57, frac * 0.78, frac * -0.5];
        let sim_velocities = if self.state == ExecutionSessionState::Running {
            vec![0.15, 0.08, -0.05]
        } else {
            vec![0.0, 0.0, 0.0]
        };

        let now_ns = (self.current_time * 1e9) as u64;
        let obs_event = super::observation::Observation {
            session_id: Some(self.session_id.clone()),
            sequence: (self.current_time * 100.0) as u64,
            sampled_at_ns: now_ns,
            received_at_ns: now_ns,
            joint_positions: sim_joints.clone(),
            joint_velocities: sim_velocities,
            tcp_pose: [frac * 200.0, 150.0, 300.0, 1.0, 0.0, 0.0, 0.0],
            signal_quality: SignalQuality::Nominal,
        };

        let obs_snap = ObservationSnapshot {
            latest: obs_event.clone(),
            signal_quality: SignalQuality::Nominal,
            freshness_ns: 0,
        };

        let dev = RunSnapshot::compute_deviation(&sim_joints, now_ns, &obs_event);

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

    #[test]
    fn test_simulation_executor_lifecycle() {
        let session_id = ExecutionSessionId("exec-01".into());
        let mut executor = SimulationExecutor::new(session_id, 10.0);

        assert_eq!(executor.state(), ExecutionSessionState::Dispatched);

        // Dispatched -> Running
        executor.start().unwrap();
        assert_eq!(executor.state(), ExecutionSessionState::Running);

        // Tick 5 seconds -> 50% progress
        let p1 = executor.tick(5.0);
        assert!((p1 - 0.5).abs() < 1e-6);
        assert_eq!(executor.state(), ExecutionSessionState::Running);

        // Tick remaining 5 seconds -> Completed
        let p2 = executor.tick(5.0);
        assert!((p2 - 1.0).abs() < 1e-6);
        assert_eq!(executor.state(), ExecutionSessionState::Completed);
    }
}
