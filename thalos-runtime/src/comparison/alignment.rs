//! Alineación temporal entre plan y ejecución.
//!
//! Resuelve qué sample del `MotionTrace` (plan) corresponde a cada sample
//! del `ExecutionTrace` (ejecución), incluso cuando las frecuencias de
//! muestreo difieren.

use std::time::Duration;

use serde::{Serialize, Serializer};

use crate::motion_trace::MotionSample;
use crate::telemetry::ExecutionSample;

fn serialize_duration<S>(dur: &Duration, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    serializer.serialize_f64(dur.as_secs_f64())
}

/// Par alineado: un sample del plan y su correspondiente en la ejecución.
#[derive(Debug, Clone, Serialize)]
pub struct AlignedPair {
    /// Tiempo del par (se usa el del plan como referencia).
    #[serde(serialize_with = "serialize_duration")]
    pub timestamp: Duration,
    /// Posiciones articulares del plan (interpoladas).
    pub planned_joints: Vec<f64>,
    /// Posiciones articulares de la ejecución.
    pub actual_joints: Vec<f64>,
    /// Velocidades del plan (si están disponibles).
    pub planned_velocities: Vec<f64>,
    /// Velocidades de la ejecución.
    pub actual_velocities: Vec<f64>,
    /// Error de tracking reportado por la ejecución (si existe).
    pub tracking_error: Option<f64>,
}

/// Alineación completa entre plan y ejecución.
#[derive(Debug, Clone, Serialize)]
pub struct Alignment {
    /// Pares alineados ordenados por timestamp.
    pub pairs: Vec<AlignedPair>,
    /// Frecuencia de muestreo del plan (Hz).
    pub plan_sample_rate: f64,
    /// Frecuencia de muestreo de la ejecución (Hz).
    pub exec_sample_rate: f64,
}

/// Alinea un `MotionTrace` (plan) con un `ExecutionTrace` (ejecución).
///
/// Estrategia: para cada sample del plan, interpola linealmente la
/// ejecución en el mismo timestamp. Así ambos quedan con la misma
/// referencia temporal.
pub fn align(plan_samples: &[MotionSample], exec_samples: &[ExecutionSample]) -> Alignment {
    if plan_samples.is_empty() || exec_samples.is_empty() {
        return Alignment {
            pairs: vec![],
            plan_sample_rate: 0.0,
            exec_sample_rate: 0.0,
        };
    }

    let plan_duration = plan_samples.last().unwrap().timestamp.as_secs_f64()
        - plan_samples.first().unwrap().timestamp.as_secs_f64();
    let exec_duration = exec_samples.last().unwrap().timestamp.as_secs_f64()
        - exec_samples.first().unwrap().timestamp.as_secs_f64();

    let plan_sample_rate = if plan_duration > 0.0 && plan_samples.len() > 1 {
        plan_samples.len() as f64 / plan_duration
    } else {
        0.0
    };
    let exec_sample_rate = if exec_duration > 0.0 && exec_samples.len() > 1 {
        exec_samples.len() as f64 / exec_duration
    } else {
        0.0
    };

    let dof = plan_samples.first().map(|s| s.joints.len()).unwrap_or(0);

    let pairs: Vec<AlignedPair> = plan_samples
        .iter()
        .map(|plan_s| {
            let t = plan_s.timestamp;

            // Interpolar ejecución en el timestamp del plan
            let (actual_joints, actual_velocities, tracking_err) =
                interpolate_execution(exec_samples, t, dof);

            AlignedPair {
                timestamp: t,
                planned_joints: plan_s.joints.clone(),
                actual_joints,
                planned_velocities: plan_s.velocities.clone(),
                actual_velocities,
                tracking_error: tracking_err,
            }
        })
        .collect();

    Alignment {
        pairs,
        plan_sample_rate,
        exec_sample_rate,
    }
}

/// Interpolar muestras de ejecución en un timestamp arbitrario.
fn interpolate_execution(
    samples: &[ExecutionSample],
    t: Duration,
    dof: usize,
) -> (Vec<f64>, Vec<f64>, Option<f64>) {
    let t_secs = t.as_secs_f64();

    // Antes del primer sample → usar el primero
    if t_secs <= samples[0].timestamp.as_secs_f64() {
        return (
            samples[0].joints.clone(),
            samples[0].velocities.clone(),
            samples[0].tracking_error,
        );
    }

    // Después del último → usar el último
    let last = samples.len() - 1;
    if t_secs >= samples[last].timestamp.as_secs_f64() {
        return (
            samples[last].joints.clone(),
            samples[last].velocities.clone(),
            samples[last].tracking_error,
        );
    }

    // Buscar par que rodea a t
    let mut hi = 1;
    while hi < samples.len() && samples[hi].timestamp.as_secs_f64() < t_secs {
        hi += 1;
    }
    let lo = hi - 1;

    let t_lo = samples[lo].timestamp.as_secs_f64();
    let t_hi = samples[hi].timestamp.as_secs_f64();
    let frac = if (t_hi - t_lo).abs() < 1e-12 {
        0.0
    } else {
        ((t_secs - t_lo) / (t_hi - t_lo)).clamp(0.0, 1.0)
    };

    let joints: Vec<f64> = (0..dof.min(samples[lo].joints.len().min(samples[hi].joints.len())))
        .map(|i| samples[lo].joints[i] + (samples[hi].joints[i] - samples[lo].joints[i]) * frac)
        .collect();

    let velocities: Vec<f64> = if !samples[lo].velocities.is_empty()
        && samples[lo].velocities.len() == samples[hi].velocities.len()
    {
        (0..samples[lo].velocities.len())
            .map(|i| {
                samples[lo].velocities[i]
                    + (samples[hi].velocities[i] - samples[lo].velocities[i]) * frac
            })
            .collect()
    } else {
        vec![]
    };

    // Para tracking error: tomar el del sample más cercano
    let tracking_err = if t_secs - t_lo < t_hi - t_secs {
        samples[lo].tracking_error
    } else {
        samples[hi].tracking_error
    };

    (joints, velocities, tracking_err)
}
