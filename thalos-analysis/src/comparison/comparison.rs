//! Agregado principal de comparación plan vs ejecución (núcleo neutral).

use serde::Serialize;

use super::alignment::{Alignment, align};
use super::input::TracePoint;
use super::metrics::{ComparisonMetrics, compute_metrics};

/// Comparación completa entre un plan y su ejecución.
///
/// Es el artefacto central que responde:
/// > "¿La ejecución coincidió con el plan?"
#[derive(Debug, Clone, Serialize)]
pub struct PlanExecutionComparison {
    /// Alineación temporal entre plan y ejecución.
    pub alignment: Alignment,
    /// Métricas de error derivadas.
    pub metrics: ComparisonMetrics,
    /// ID del plan.
    pub plan_id: String,
    /// ID de la sesión de ejecución.
    pub execution_id: String,
    /// Nombre del robot.
    pub robot_name: String,
    /// Duración del plan (s).
    pub plan_duration: f64,
    /// Duración de la ejecución (s).
    pub execution_duration: f64,
}

/// Comparación neutral sobre puntos independientes del mecanismo de evidencia.
///
/// Es la autoridad del análisis: cualquier mecanismo puede producir
/// [`TracePoint`]s y compararlos aquí.
pub fn compare_points(
    plan: &[TracePoint],
    execution: &[TracePoint],
    plan_id: impl Into<String>,
    execution_id: impl Into<String>,
    robot_name: impl Into<String>,
) -> PlanExecutionComparison {
    let alignment = align(plan, execution);
    let metrics = compute_metrics(&alignment.pairs);

    PlanExecutionComparison {
        alignment,
        metrics,
        plan_id: plan_id.into(),
        execution_id: execution_id.into(),
        robot_name: robot_name.into(),
        plan_duration: span_seconds(plan),
        execution_duration: span_seconds(execution),
    }
}

fn span_seconds(points: &[TracePoint]) -> f64 {
    match (points.first(), points.last()) {
        (Some(first), Some(last)) => last.timestamp.saturating_sub(first.timestamp).as_secs_f64(),
        _ => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn neutral_points_compare_without_legacy_traces() {
        let plan = vec![
            TracePoint::new(Duration::ZERO, vec![0.0], vec![0.0], None),
            TracePoint::new(Duration::from_secs(1), vec![1.0], vec![1.0], None),
        ];
        let execution = vec![
            TracePoint::new(Duration::ZERO, vec![0.0], vec![0.0], None),
            TracePoint::new(Duration::from_secs(1), vec![1.0], vec![1.0], Some(0.0)),
        ];
        let result = compare_points(&plan, &execution, "p", "e", "robot");
        assert_eq!(result.metrics.aligned_count, 2);
        assert!(result.metrics.global_rmse < 1e-6);
    }
}
