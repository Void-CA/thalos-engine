//! MotionTrace — registro cronológico del estado del robot durante ejecución.
//!
//! Cada `MotionSample` captura el estado completo en un instante.
//! Un `MotionTrace` es una secuencia ordenada de samples.
//!
//! # Flujo
//!
//! ```text
//! Execution tick
//!      ↓
//! MotionRecorder::record(timestamp, state)
//!      ↓
//! MotionTrace
//!      ↓
//! Analysis / Visualization / Export
//! ```

use std::time::Duration;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::state::robot_state::RobotState;

/// Serialize Duration as seconds (f64).
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

/// Una muestra del estado del robot en un instante de tiempo.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionSample {
    /// Tiempo desde el inicio de la ejecución.
    #[serde(with = "duration_secs")]
    pub timestamp: Duration,
    /// Posiciones articulares actuales (rad).
    pub joints: Vec<f64>,
    /// Velocidades articulares (rad/s) — opcional según backend.
    pub velocities: Vec<f64>,
    /// Posiciones articulares objetivo (rad) — presente durante ejecución activa.
    pub target_joints: Option<Vec<f64>>,
    /// Progreso de la ejecución (0.0 a 1.0).
    pub progress: f64,
    /// Errores activos en este instante.
    pub errors: Vec<String>,
}

impl MotionSample {
    /// Crear una muestra desde un RobotState en un instante dado.
    pub fn from_state(timestamp: Duration, state: &RobotState) -> Self {
        Self {
            timestamp,
            joints: state.joints.positions.clone(),
            velocities: state.joints.velocities.clone(),
            target_joints: None, // se rellena externamente si hay referencia
            progress: state.execution.progress,
            errors: state.errors.iter().map(|e| e.to_string()).collect(),
        }
    }
}

/// Traza cronológica completa de una ejecución.
///
/// Almacena samples ordenados por timestamp.
/// Puede exportarse a CSV, visualizarse como gráficas, o compararse
/// con la trayectoria planificada.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotionTrace {
    samples: Vec<MotionSample>,
}

impl MotionTrace {
    pub fn new() -> Self {
        Self {
            samples: Vec::new(),
        }
    }

    /// Agregar una muestra. Asume que los timestamps son crecientes.
    pub fn push(&mut self, sample: MotionSample) {
        self.samples.push(sample);
    }

    /// Todas las muestras.
    pub fn samples(&self) -> &[MotionSample] {
        &self.samples
    }

    /// Duración total de la traza (timestamp del último sample).
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

    /// Está vacía.
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// Exportar a CSV.
    pub fn to_csv(&self) -> String {
        let mut csv = String::new();
        // Dynamic header based on actual joint count
        if let Some(first) = self.samples.first() {
            let n_joints = first.joints.len();
            let mut header = "timestamp_s".to_string();
            for i in 0..n_joints {
                header.push_str(&format!(",joint_{}", i));
            }
            for i in 0..first.velocities.len() {
                header.push_str(&format!(",velocity_{}", i));
            }
            header.push_str(",progress\n");
            csv.push_str(&header);

            for s in &self.samples {
                let t = s.timestamp.as_secs_f64();
                let j = s
                    .joints
                    .iter()
                    .map(|v| format!("{:.6}", v))
                    .collect::<Vec<_>>()
                    .join(",");
                let v = s
                    .velocities
                    .iter()
                    .map(|v| format!("{:.6}", v))
                    .collect::<Vec<_>>()
                    .join(",");
                if v.is_empty() {
                    csv.push_str(&format!("{},{},{:.4}\n", t, j, s.progress));
                } else {
                    csv.push_str(&format!("{},{},{},{:.4}\n", t, j, v, s.progress));
                }
            }
        }
        csv
    }

    /// Obtener las posiciones articulares como columnas (vector de series).
    pub fn joint_series(&self, joint_index: usize) -> Vec<(f64, f64)> {
        self.samples
            .iter()
            .filter_map(|s| {
                s.joints
                    .get(joint_index)
                    .map(|&pos| (s.timestamp.as_secs_f64(), pos))
            })
            .collect()
    }

    /// Velocidad cartesiana estimada del TCP (norma de la velocidad lineal).
    pub fn cartesian_speed_series(&self) -> Vec<(f64, f64)> {
        self.samples
            .windows(2)
            .map(|w| {
                let dt = (w[1].timestamp.as_secs_f64() - w[0].timestamp.as_secs_f64()).max(1e-6);
                let pos_diff: f64 = w[1]
                    .joints
                    .iter()
                    .zip(&w[0].joints)
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt();
                let speed = pos_diff / dt;
                (w[1].timestamp.as_secs_f64(), speed)
            })
            .collect()
    }

    /// Tracking error: diferencia RMS entre joints actual y target (si hay target).
    pub fn tracking_error_series(&self) -> Vec<(f64, f64)> {
        self.samples
            .iter()
            .filter_map(|s| {
                s.target_joints.as_ref().map(|target| {
                    let n = target.len().min(s.joints.len());
                    let rms = (0..n)
                        .map(|i| (s.joints[i] - target[i]).powi(2))
                        .sum::<f64>()
                        / n as f64;
                    (s.timestamp.as_secs_f64(), rms.sqrt())
                })
            })
            .collect()
    }
}

impl Default for MotionTrace {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::robot_state::{ExecutionState, JointState, RobotState};

    fn make_sample(t: f64, joints: Vec<f64>) -> MotionSample {
        let state = RobotState {
            joints: JointState {
                positions: joints,
                velocities: vec![0.0; 2],
                torques: vec![],
            },
            execution: ExecutionState {
                current_program: None,
                current_segment: None,
                progress: t / 2.0,
            },
            ..RobotState::default()
        };
        MotionSample::from_state(Duration::from_secs_f64(t), &state)
    }

    #[test]
    fn empty_trace() {
        let trace = MotionTrace::new();
        assert!(trace.is_empty());
        assert_eq!(trace.len(), 0);
        assert_eq!(trace.duration(), Duration::ZERO);
    }

    #[test]
    fn push_and_retrieve() {
        let mut trace = MotionTrace::new();
        trace.push(make_sample(0.0, vec![0.0, 0.0]));
        trace.push(make_sample(0.5, vec![0.5, 0.3]));
        trace.push(make_sample(1.0, vec![1.0, 0.5]));
        assert_eq!(trace.len(), 3);
        assert!((trace.duration().as_secs_f64() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn joint_series() {
        let mut trace = MotionTrace::new();
        trace.push(make_sample(0.0, vec![0.0, 0.0]));
        trace.push(make_sample(1.0, vec![1.0, 0.5]));
        let series = trace.joint_series(0);
        assert_eq!(series.len(), 2);
        assert!((series[1].1 - 1.0).abs() < 1e-6);
    }

    #[test]
    fn csv_export() {
        let mut trace = MotionTrace::new();
        trace.push(make_sample(0.0, vec![0.0, 0.0]));
        trace.push(make_sample(1.0, vec![1.0, 0.5]));
        let csv = trace.to_csv();
        assert!(csv.contains("timestamp_s"), "should have header");
        assert!(csv.contains("joint_0"), "should have joint_0 column");
        assert!(csv.lines().count() >= 3, "header + 2 data rows");
    }

    #[test]
    fn serde_roundtrip() {
        let mut trace = MotionTrace::new();
        trace.push(make_sample(0.0, vec![0.0, 0.0]));
        trace.push(make_sample(1.0, vec![1.0, 0.5]));
        let json = serde_json::to_string(&trace).expect("serialize");
        let restored: MotionTrace = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(restored.len(), trace.len());
        assert!((restored.duration().as_secs_f64() - 1.0).abs() < 1e-6);
        assert_eq!(restored.samples()[0].joints, vec![0.0, 0.0]);
        assert_eq!(restored.samples()[1].joints, vec![1.0, 0.5]);
    }

    #[test]
    fn tracking_error_without_target() {
        let mut trace = MotionTrace::new();
        trace.push(make_sample(0.0, vec![0.0, 0.0]));
        let err = trace.tracking_error_series();
        assert!(err.is_empty(), "no target_joints → no tracking error");
    }
}
