use serde::Serialize;

use crate::telemetry::trace::ExecutionTrace;

/// Estadísticas derivadas de un `ExecutionTrace`.
///
/// No se almacenan — se computan bajo demanda desde el trace.
#[derive(Debug, Clone, Serialize)]
pub struct ExecutionStatistics {
    /// Duración total de la ejecución (segundos).
    pub duration: f64,
    /// Cantidad de samples.
    pub sample_count: usize,
    /// Frecuencia de muestreo promedio (Hz).
    pub sample_rate: f64,
    /// Cantidad de articulaciones.
    pub joint_count: usize,
    /// Distancia total recorrida en espacio articular (rad).
    pub path_length: f64,
    /// Velocidad máxima por articulación (rad/s).
    pub max_joint_velocity: Vec<f64>,
    /// Velocidad media por articulación (rad/s).
    pub avg_joint_velocity: Vec<f64>,
    /// Máximo error de tracking (si hay target disponible).
    pub max_tracking_error: Option<f64>,
    /// Error de tracking promedio.
    pub avg_tracking_error: Option<f64>,
    /// Cantidad de eventos.
    pub event_count: usize,
    /// Cantidad de waypoints completados (según eventos).
    pub waypoints_completed: usize,
}

/// Analiza un `ExecutionTrace` y produce estadísticas.
pub struct TraceAnalyzer;

impl TraceAnalyzer {
    /// Computar estadísticas a partir de un trace.
    pub fn analyze(trace: &ExecutionTrace) -> ExecutionStatistics {
        let samples = &trace.samples;
        let n = samples.len();
        let joint_count = samples.first().map(|s| s.joints.len()).unwrap_or(0);

        // Duración
        let duration = if n >= 2 {
            samples.last().unwrap().timestamp.as_secs_f64()
                - samples.first().unwrap().timestamp.as_secs_f64()
        } else {
            0.0
        };

        // Sample rate
        let sample_rate = if duration > 0.0 {
            n as f64 / duration
        } else {
            0.0
        };

        // Path length (suma de distancias euclidianas entre configuraciones articulares)
        let path_length: f64 = samples
            .windows(2)
            .map(|w| {
                w[1].joints
                    .iter()
                    .zip(&w[0].joints)
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt()
            })
            .sum();

        // Velocidades por articulación
        let (max_vel, avg_vel) = if joint_count > 0 && n >= 2 {
            let mut maxes = vec![0.0f64; joint_count];
            let mut total = vec![0.0f64; joint_count];
            let mut count = 0usize;

            for w in samples.windows(2) {
                let dt = (w[1].timestamp.as_secs_f64() - w[0].timestamp.as_secs_f64()).max(1e-6);
                for j in 0..joint_count {
                    let v = (w[1].joints[j] - w[0].joints[j]).abs() / dt;
                    if v > maxes[j] {
                        maxes[j] = v;
                    }
                    total[j] += v;
                }
                count += 1;
            }

            let avg = if count > 0 {
                total.iter().map(|t| t / count as f64).collect()
            } else {
                vec![0.0; joint_count]
            };

            (maxes, avg)
        } else {
            (vec![], vec![])
        };

        // Tracking error (estimado de velocidades si no hay field directo)
        let errors: Vec<f64> = samples.iter().filter_map(|s| s.tracking_error).collect();

        let (max_tracking_err, avg_tracking_err) = if errors.is_empty() {
            (None, None)
        } else {
            let max = errors.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            let avg = errors.iter().sum::<f64>() / errors.len() as f64;
            (Some(max), Some(avg))
        };

        // Eventos
        let waypoints_completed = trace
            .events
            .iter()
            .filter(|e| {
                matches!(
                    e,
                    crate::telemetry::event::ExecutionEvent::WaypointReached { .. }
                )
            })
            .count();

        ExecutionStatistics {
            duration,
            sample_count: n,
            sample_rate,
            joint_count,
            path_length,
            max_joint_velocity: max_vel,
            avg_joint_velocity: avg_vel,
            max_tracking_error: max_tracking_err,
            avg_tracking_error: avg_tracking_err,
            event_count: trace.events.len(),
            waypoints_completed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::session::ExecutionSource;
    use crate::telemetry::trace::{ExecutionSample, TraceMetadata};
    use std::time::Duration;

    fn sample_trace() -> ExecutionTrace {
        let meta = TraceMetadata {
            session_id: "1".into(),
            plan_id: "p1".into(),
            source: ExecutionSource::Simulation,
            robot_name: "test".into(),
            joint_count: 2,
            duration: Duration::from_secs_f64(2.0),
            sample_rate: 0.0,
        };
        let mut trace = ExecutionTrace::new(meta);
        trace.push_sample(ExecutionSample {
            timestamp: Duration::from_secs_f64(0.0),
            joints: vec![0.0, 0.0],
            velocities: vec![],
            accelerations: vec![],
            tcp_pose: [0.0; 7],
            tcp_velocity: [0.0; 6],
            tracking_error: None,
            progress: 0.0,
        });
        trace.push_sample(ExecutionSample {
            timestamp: Duration::from_secs_f64(1.0),
            joints: vec![1.0, 0.5],
            velocities: vec![],
            accelerations: vec![],
            tcp_pose: [0.0; 7],
            tcp_velocity: [0.0; 6],
            tracking_error: Some(0.01),
            progress: 1.0,
        });
        trace
    }

    #[test]
    fn analyze_basic_statistics() {
        let trace = sample_trace();
        let stats = TraceAnalyzer::analyze(&trace);

        assert_eq!(stats.sample_count, 2);
        assert!((stats.duration - 1.0).abs() < 1e-6);
        assert!(stats.path_length > 0.0);
        assert_eq!(stats.joint_count, 2);
        assert!(stats.sample_rate > 0.0);
    }

    #[test]
    fn tracking_error_statistics() {
        let trace = sample_trace();
        let stats = TraceAnalyzer::analyze(&trace);
        assert!((stats.max_tracking_error.unwrap() - 0.01).abs() < 1e-6);
        assert!((stats.avg_tracking_error.unwrap() - 0.01).abs() < 1e-6);
    }

    #[test]
    fn empty_trace() {
        let meta = TraceMetadata {
            session_id: "0".into(),
            plan_id: "".into(),
            source: ExecutionSource::Simulation,
            robot_name: "".into(),
            joint_count: 0,
            duration: Duration::ZERO,
            sample_rate: 0.0,
        };
        let trace = ExecutionTrace::new(meta);
        let stats = TraceAnalyzer::analyze(&trace);
        assert_eq!(stats.sample_count, 0);
        assert_eq!(stats.duration, 0.0);
    }

    #[test]
    fn velocity_estimation() {
        let trace = sample_trace();
        let stats = TraceAnalyzer::analyze(&trace);
        assert_eq!(stats.max_joint_velocity.len(), 2);
        // Joint 0: 0→1 en 1s = 1 rad/s
        // Joint 1: 0→0.5 en 1s = 0.5 rad/s
        assert!((stats.max_joint_velocity[0] - 1.0).abs() < 1e-6);
        assert!((stats.max_joint_velocity[1] - 0.5).abs() < 1e-6);
    }
}
