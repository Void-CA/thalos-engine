//! MotionRecorder — graba un MotionTrace durante la ejecución.
//!
//! Se integra con el ciclo de tick: en cada tick, el recorder toma
//! una muestra del RobotState y la agrega a la traza.
//!
//! # Uso
//!
//! ```ignore
//! let recorder = MotionRecorder::new();
//! recorder.start(trajectory_duration);
//! // ... en cada tick:
//! recorder.record(timestamp, &state);
//! let trace = recorder.stop();
//! ```

use std::time::Duration;

use crate::motion_trace::{MotionSample, MotionTrace};
use crate::state::robot_state::RobotState;

/// Graba un MotionTrace durante la ejecución de una trayectoria.
pub struct MotionRecorder {
    trace: MotionTrace,
    start_time: Option<Duration>,
    end_time: Option<Duration>,
    /// Referencia a las posiciones objetivo de la trayectoria (opcional).
    target_waypoints: Option<Vec<Vec<f64>>>,
}

impl MotionRecorder {
    pub fn new() -> Self {
        Self {
            trace: MotionTrace::new(),
            start_time: None,
            end_time: None,
            target_waypoints: None,
        }
    }

    /// Iniciar la grabación para una trayectoria de duración conocida.
    pub fn start(&mut self, duration: Duration) {
        self.start_time = Some(Duration::ZERO);
        self.end_time = Some(duration);
        self.trace = MotionTrace::new();
    }

    /// Registrar una muestra en un instante dado.
    pub fn record(&mut self, timestamp: Duration, state: &RobotState) {
        let mut sample = MotionSample::from_state(timestamp, state);

        // Resolver posición objetivo por interpolación lineal si hay waypoints
        if let Some(ref waypoints) = self.target_waypoints {
            if waypoints.len() >= 2 {
                let progress = state.execution.progress.clamp(0.0, 1.0);
                let total_steps = waypoints.len().saturating_sub(1);
                let idx_f = progress * total_steps as f64;
                let i = idx_f.floor() as usize;
                let j = (i + 1).min(waypoints.len() - 1);
                let local_frac = idx_f - i as f64;
                let target: Vec<f64> = waypoints[i]
                    .iter()
                    .zip(&waypoints[j])
                    .map(|(&a, &b)| a + (b - a) * local_frac)
                    .collect();
                sample.target_joints = Some(target);
            }
        }

        self.trace.push(sample);
    }

    /// Finalizar la grabación y obtener la traza.
    pub fn stop(&mut self) -> MotionTrace {
        let trace = std::mem::take(&mut self.trace);
        self.start_time = None;
        self.end_time = None;
        trace
    }

    /// Establecer los waypoints de referencia para tracking error.
    pub fn set_target_waypoints(&mut self, waypoints: Vec<Vec<f64>>) {
        self.target_waypoints = Some(waypoints);
    }
}

impl Default for MotionRecorder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::robot_state::{ExecutionState, JointState};

    #[test]
    fn record_and_stop() {
        let mut recorder = MotionRecorder::new();
        recorder.start(Duration::from_secs_f64(2.0));

        let state = RobotState {
            joints: JointState {
                positions: vec![0.5, -0.3],
                velocities: vec![],
                torques: vec![],
            },
            execution: ExecutionState {
                current_program: None,
                current_segment: None,
                progress: 0.5,
            },
            ..RobotState::default()
        };
        recorder.record(Duration::from_secs_f64(1.0), &state);

        let trace = recorder.stop();
        assert_eq!(trace.len(), 1);
        assert!((trace.samples()[0].joints[0] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn recorder_with_target_waypoints() {
        let mut recorder = MotionRecorder::new();
        recorder.set_target_waypoints(vec![vec![0.0, 0.0], vec![1.0, 0.5]]);
        recorder.start(Duration::from_secs_f64(2.0));

        let state = RobotState {
            joints: JointState {
                positions: vec![0.5, 0.25],
                velocities: vec![],
                torques: vec![],
            },
            execution: ExecutionState {
                current_program: None,
                current_segment: None,
                progress: 0.5,
            },
            ..RobotState::default()
        };
        recorder.record(Duration::from_secs_f64(1.0), &state);

        let trace = recorder.stop();
        let sample = &trace.samples()[0];
        // Target should be interpolated at progress 0.5 → (0.5, 0.25)
        if let Some(ref target) = sample.target_joints {
            assert!((target[0] - 0.5).abs() < 1e-6);
            assert!((target[1] - 0.25).abs() < 1e-6);
        } else {
            panic!("expected target_joints");
        }
    }
}
