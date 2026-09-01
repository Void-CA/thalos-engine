use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::time::Duration;

use crate::session::ExecutionSource;
use crate::telemetry::event::ExecutionEvent;

/// Serialize/deserialize Duration as seconds (f64).
mod duration_secs {
    use super::*;

    pub fn serialize<S>(dur: &Duration, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_f64(dur.as_secs_f64())
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Duration, D::Error>
    where
        D: Deserializer<'de>,
    {
        let secs = f64::deserialize(deserializer)?;
        Ok(Duration::from_secs_f64(secs))
    }
}

/// Metadatos de una ejecución.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TraceMetadata {
    /// ID de la sesión asociada.
    pub session_id: String,
    /// ID del plan ejecutado.
    pub plan_id: String,
    /// Backend utilizado.
    pub source: ExecutionSource,
    /// Nombre del robot.
    pub robot_name: String,
    /// Cantidad de articulaciones.
    pub joint_count: usize,
    /// Duración total de la trayectoria.
    #[serde(with = "duration_secs")]
    pub duration: Duration,
    /// Frecuencia de muestreo promedio (samples/segundo).
    pub sample_rate: f64,
}

/// Una muestra del estado del robot en un instante de tiempo.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutionSample {
    /// Tiempo desde el inicio de la ejecución.
    #[serde(with = "duration_secs")]
    pub timestamp: Duration,
    /// Posiciones articulares (rad).
    pub joints: Vec<f64>,
    /// Velocidades articulares (rad/s).
    pub velocities: Vec<f64>,
    /// Aceleraciones articulares (rad/s²).
    pub accelerations: Vec<f64>,
    /// Posición cartesiana del TCP: [x, y, z, qw, qx, qy, qz].
    pub tcp_pose: [f64; 7],
    /// Velocidad cartesiana del TCP: [vx, vy, vz, ωx, ωy, ωz].
    pub tcp_velocity: [f64; 6],
    /// Error de tracking (diferencia entre actual y objetivo).
    pub tracking_error: Option<f64>,
    /// Progreso de la trayectoria (0.0 a 1.0).
    pub progress: f64,
}

/// Traza completa de una ejecución.
///
/// Es el artifact central de observabilidad — captura todo lo que ocurrió
/// durante una ejecución: estado del robot en cada instante, eventos de
/// ciclo de vida, y metadatos de la sesión.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutionTrace {
    /// Metadatos de la ejecución.
    pub metadata: TraceMetadata,
    /// Muestras cronológicas ordenadas por timestamp.
    pub samples: Vec<ExecutionSample>,
    /// Eventos de ciclo de vida.
    pub events: Vec<ExecutionEvent>,
}

impl ExecutionTrace {
    pub fn new(metadata: TraceMetadata) -> Self {
        Self {
            metadata,
            samples: Vec::new(),
            events: Vec::new(),
        }
    }

    /// Duración total (timestamp del último sample).
    pub fn duration(&self) -> Duration {
        self.samples
            .last()
            .map(|s| s.timestamp)
            .unwrap_or(Duration::ZERO)
    }

    /// Cantidad de muestras.
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// Agregar una muestra.
    pub fn push_sample(&mut self, sample: ExecutionSample) {
        self.samples.push(sample);
    }

    /// Agregar un evento.
    pub fn push_event(&mut self, event: ExecutionEvent) {
        self.events.push(event);
    }
}
