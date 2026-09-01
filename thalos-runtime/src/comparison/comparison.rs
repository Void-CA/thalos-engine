//! Agregado principal de comparación plan vs ejecución.

use serde::Serialize;

use super::alignment::{Alignment, align};
use super::metrics::{ComparisonMetrics, compute_metrics};
use crate::motion_trace::MotionTrace;
use crate::telemetry::ExecutionTrace;

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
    /// ID del plan (MotionTrace).
    pub plan_id: String,
    /// ID de la sesión de ejecución (ExecutionTrace).
    pub execution_id: String,
    /// Nombre del robot.
    pub robot_name: String,
    /// Duración del plan (s).
    pub plan_duration: f64,
    /// Duración de la ejecución (s).
    pub execution_duration: f64,
}

/// Construye un `PlanExecutionComparison` a partir de plan y ejecución.
pub fn compare(
    plan: &MotionTrace,
    execution: &ExecutionTrace,
    plan_id: impl Into<String>,
    execution_id: impl Into<String>,
    robot_name: impl Into<String>,
) -> PlanExecutionComparison {
    let alignment = align(plan.samples(), &execution.samples);
    let metrics = compute_metrics(&alignment.pairs);

    let plan_duration = plan.duration().as_secs_f64();
    let execution_duration = execution.duration().as_secs_f64();

    PlanExecutionComparison {
        alignment,
        metrics,
        plan_id: plan_id.into(),
        execution_id: execution_id.into(),
        robot_name: robot_name.into(),
        plan_duration,
        execution_duration,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion_trace::MotionSample;
    use crate::session::ExecutionSource;
    use crate::telemetry::{ExecutionSample, TraceMetadata};
    use std::time::Duration;

    fn plan_samples() -> Vec<MotionSample> {
        vec![
            MotionSample {
                timestamp: Duration::from_secs_f64(0.0),
                joints: vec![0.0, 0.0],
                velocities: vec![0.0, 0.0],
                target_joints: None,
                progress: 0.0,
                errors: vec![],
            },
            MotionSample {
                timestamp: Duration::from_secs_f64(1.0),
                joints: vec![1.0, 0.5],
                velocities: vec![1.0, 0.5],
                target_joints: None,
                progress: 1.0,
                errors: vec![],
            },
        ]
    }

    fn exec_trace() -> ExecutionTrace {
        let meta = TraceMetadata {
            session_id: "1".into(),
            plan_id: "p1".into(),
            source: ExecutionSource::Simulation,
            robot_name: "test".into(),
            joint_count: 2,
            duration: Duration::from_secs_f64(1.0),
            sample_rate: 0.0,
        };
        let mut trace = ExecutionTrace::new(meta);
        trace.push_sample(ExecutionSample {
            timestamp: Duration::from_secs_f64(0.0),
            joints: vec![0.0, 0.0],
            velocities: vec![0.0, 0.0],
            accelerations: vec![],
            tcp_pose: [0.0; 7],
            tcp_velocity: [0.0; 6],
            tracking_error: None,
            progress: 0.0,
        });
        trace.push_sample(ExecutionSample {
            timestamp: Duration::from_secs_f64(1.0),
            joints: vec![1.0, 0.5],
            velocities: vec![1.0, 0.5],
            accelerations: vec![],
            tcp_pose: [0.0; 7],
            tcp_velocity: [0.0; 6],
            tracking_error: Some(0.01),
            progress: 1.0,
        });
        trace
    }

    #[test]
    fn compare_identical_plan_and_exec() {
        let mut plan = MotionTrace::new();
        for s in plan_samples() {
            plan.push(s);
        }
        let exec = exec_trace();

        let result = compare(&plan, &exec, "plan-1", "exec-1", "test-robot");
        assert_eq!(result.metrics.aligned_count, 2);
        assert!(result.metrics.global_rmse < 1e-6);
        assert!(result.metrics.max_tracking_error.unwrap() - 0.01 < 1e-6);
    }

    #[test]
    fn compare_with_deviation() {
        let mut plan = MotionTrace::new();
        for s in plan_samples() {
            plan.push(s);
        }

        let mut exec = exec_trace();
        // Modify second sample to create deviation
        if let Some(s) = exec.samples.last_mut() {
            s.joints = vec![1.1, 0.6]; // 0.1 rad deviation
        }

        let result = compare(&plan, &exec, "p1", "e1", "robot");
        assert!(result.metrics.global_rmse > 0.0);
        assert!((result.metrics.global_max_error - 0.1).abs() < 1e-6);
    }

    #[test]
    fn compare_empty_traces() {
        let plan = MotionTrace::new();
        let meta = TraceMetadata {
            session_id: "0".into(),
            plan_id: "".into(),
            source: ExecutionSource::Simulation,
            robot_name: "".into(),
            joint_count: 0,
            duration: Duration::ZERO,
            sample_rate: 0.0,
        };
        let exec = ExecutionTrace::new(meta);
        let result = compare(&plan, &exec, "", "", "");
        assert_eq!(result.metrics.aligned_count, 0);
    }
}
