use std::time::Duration;

use crate::state::robot_state::RobotState;
use crate::telemetry::event::ExecutionEvent;
use crate::telemetry::trace::ExecutionTrace;

/// Observador de ejecución — recibe eventos y muestras sin interferir.
///
/// `ExecutionRecorder` es una implementación. Futuras implementaciones
/// pueden incluir WebSocket streaming, logging, exportación CSV, etc.
pub trait ExecutionObserver: Send + Sync {
    /// Notifica que una ejecución comenzó.
    fn on_execution_started(&mut self, _timestamp: Duration) {}

    /// Notifica una muestra del estado del robot.
    fn on_sample(&mut self, _timestamp: Duration, _state: &RobotState) {}

    /// Notifica un evento de ciclo de vida.
    fn on_event(&mut self, _event: ExecutionEvent) {}

    /// Notifica que la ejecución finalizó.
    fn on_execution_finished(&mut self, _timestamp: Duration) {}

    /// Obtener el trace acumulado (si corresponde).
    fn trace(&self) -> Option<ExecutionTrace> {
        None
    }
}

/// Recorder que construye un `ExecutionTrace` a partir de observaciones.
///
/// No ejecuta nada — solo observa y registra.
pub struct ExecutionRecorder {
    trace: ExecutionTrace,
    started: bool,
}

impl ExecutionRecorder {
    pub fn new(metadata: crate::telemetry::trace::TraceMetadata) -> Self {
        Self {
            trace: ExecutionTrace::new(metadata),
            started: false,
        }
    }
}

impl ExecutionObserver for ExecutionRecorder {
    fn on_execution_started(&mut self, timestamp: Duration) {
        if !self.started {
            self.started = true;
            self.trace.push_event(ExecutionEvent::Started { timestamp });
        }
    }

    fn on_sample(&mut self, timestamp: Duration, state: &RobotState) {
        let sample = crate::telemetry::trace::ExecutionSample {
            timestamp,
            joints: state.joints.positions.clone(),
            velocities: state.joints.velocities.clone(),
            accelerations: vec![], // derivadas numéricas se calculan post-hoc
            tcp_pose: state.cartesian.tcp_pose,
            tcp_velocity: state.cartesian.tcp_velocity,
            tracking_error: None, // se rellena si hay target_joints
            progress: state.execution.progress,
        };
        self.trace.push_sample(sample);
    }

    fn on_event(&mut self, event: ExecutionEvent) {
        self.trace.push_event(event);
    }

    fn on_execution_finished(&mut self, timestamp: Duration) {
        self.trace.push_event(
            if self
                .trace
                .events
                .iter()
                .any(|e| matches!(e, ExecutionEvent::Cancelled { .. }))
            {
                ExecutionEvent::Cancelled { timestamp }
            } else {
                ExecutionEvent::Completed { timestamp }
            },
        );
    }

    fn trace(&self) -> Option<ExecutionTrace> {
        Some(self.trace.clone())
    }
}
