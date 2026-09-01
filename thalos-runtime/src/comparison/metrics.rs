//! Métricas derivadas de la comparación plan vs ejecución.

use serde::Serialize;

use super::alignment::AlignedPair;

/// Métricas de error por articulación.
#[derive(Debug, Clone, Serialize)]
pub struct JointErrorMetrics {
    /// RMSE por articulación (rad).
    pub rmse: Vec<f64>,
    /// Error máximo absoluto por articulación (rad).
    pub max_error: Vec<f64>,
    /// Error promedio absoluto por articulación (rad).
    pub avg_error: Vec<f64>,
}

/// Métricas agregadas de la comparación.
#[derive(Debug, Clone, Serialize)]
pub struct ComparisonMetrics {
    /// RMSE global (promedio de RMSE de todas las articulaciones).
    pub global_rmse: f64,
    /// Error máximo absoluto global (rad).
    pub global_max_error: f64,
    /// Error promedio absoluto global (rad).
    pub global_avg_error: f64,
    /// Desglose por articulación.
    pub per_joint: JointErrorMetrics,
    /// Error de tracking máximo reportado (si la ejecución lo registró).
    pub max_tracking_error: Option<f64>,
    /// Error de tracking promedio.
    pub avg_tracking_error: Option<f64>,
    /// Máxima diferencia de velocidad por articulación (rad/s).
    pub max_velocity_deviation: Vec<f64>,
    /// Cantidad de pares alineados.
    pub aligned_count: usize,
}

/// Computar métricas de comparación a partir de pares alineados.
pub fn compute_metrics(pairs: &[AlignedPair]) -> ComparisonMetrics {
    if pairs.is_empty() {
        return ComparisonMetrics {
            global_rmse: 0.0,
            global_max_error: 0.0,
            global_avg_error: 0.0,
            per_joint: JointErrorMetrics {
                rmse: vec![],
                max_error: vec![],
                avg_error: vec![],
            },
            max_tracking_error: None,
            avg_tracking_error: None,
            max_velocity_deviation: vec![],
            aligned_count: 0,
        };
    }

    let dof = pairs[0].planned_joints.len();

    // Errores por articulación
    let mut sq_sum = vec![0.0f64; dof];
    let mut abs_sum = vec![0.0f64; dof];
    let mut max_err = vec![0.0f64; dof];

    let mut tracking_errors: Vec<f64> = Vec::new();

    for pair in pairs {
        for j in 0..dof {
            let err = (pair.actual_joints[j] - pair.planned_joints[j]).abs();
            sq_sum[j] += err * err;
            abs_sum[j] += err;
            if err > max_err[j] {
                max_err[j] = err;
            }
        }
        if let Some(te) = pair.tracking_error {
            tracking_errors.push(te);
        }
    }

    let n = pairs.len() as f64;
    let rmse: Vec<f64> = sq_sum.iter().map(|s| (s / n).sqrt()).collect();
    let avg_error: Vec<f64> = abs_sum.iter().map(|s| s / n).collect();

    let global_rmse = rmse.iter().sum::<f64>() / dof as f64;
    let global_max_error = max_err.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let global_avg_error = avg_error.iter().sum::<f64>() / dof as f64;

    // Tracking error
    let (max_te, avg_te) = if tracking_errors.is_empty() {
        (None, None)
    } else {
        let max = tracking_errors
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let avg = tracking_errors.iter().sum::<f64>() / tracking_errors.len() as f64;
        (Some(max), Some(avg))
    };

    // Desviación de velocidad
    let max_vel_dev: Vec<f64> = if !pairs[0].planned_velocities.is_empty() {
        (0..dof.min(
            pairs[0]
                .planned_velocities
                .len()
                .min(pairs[0].actual_velocities.len()),
        ))
            .map(|j| {
                pairs
                    .iter()
                    .map(|p| (p.actual_velocities[j] - p.planned_velocities[j]).abs())
                    .fold(f64::NEG_INFINITY, f64::max)
            })
            .collect()
    } else {
        vec![]
    };

    ComparisonMetrics {
        global_rmse,
        global_max_error,
        global_avg_error,
        per_joint: JointErrorMetrics {
            rmse,
            max_error: max_err,
            avg_error,
        },
        max_tracking_error: max_te,
        avg_tracking_error: avg_te,
        max_velocity_deviation: max_vel_dev,
        aligned_count: pairs.len(),
    }
}
