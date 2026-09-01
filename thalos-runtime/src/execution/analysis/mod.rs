//! Análisis de ejecuciones comparadas contra el plan.
//!
//! El [`ExecutionAnalyzer`] toma un [`PlanExecutionComparison`] (plan vs
//! ejecución) y emite observaciones canónicas del modelo unificado
//! (spec planning-feedback-loop):
//!
//!   Comparison → ExecutionAnalyzer → Vec<Observation> → Aggregator → AnalysisReport
//!                                            → (PR 4b) IntentionOperator → Action
//!
//! La emisión es DIRECTA: la observación es el único vocabulario de análisis
//! (PR 7a eliminó el camino legacy `analyze_findings`).
//!
//! # Schema temporal (contrato C1)
//!
//! `compare()` alinea por TIMESTAMP: interpola la ejecución en el instante de
//! cada muestra del plan. El eje temporal real del análisis es el par alineado
//! (índice = muestra del plan, `timestamp` = instante). Cada observación
//! responde:
//! - **¿Cuándo?** → `location: Location::Timestamp(ms)` + `attributes["elapsed_ms"]`
//! - **¿En qué muestra?** → `attributes["sample"]` (índice 0-based del par alineado)
//! - **¿En qué instante?** → `attributes["elapsed_ms"]`
//! - **¿Durante qué segmento?** → NO DISPONIBLE hoy: el `MotionTrace`/alineación no
//!   modela segmentos (viven en el `PlanningProgram`, capa feedback); requiere
//!   mapeo plan→waypoint (trabajo futuro, PR 4b+).
//!
//! | Fenómeno (kind) | severity | location | attributes |
//! |---|---|---|---|
//! | `TrackingError` (RMSE global) | Warning | `Timestamp(0)` — agregado de sesión (abarca 0..`elapsed_ms`) | value, threshold, samples, elapsed_ms, plan_id, session_id |
//! | `TrackingSpike` (pico global) | Warning | `Timestamp(peak)` | value, threshold, joint, sample, elapsed_ms, plan_id, session_id |
//! | `JointDeviation` (por articulación) | Warning | `Timestamp(peak)` | joint, value, threshold, sample, elapsed_ms, plan_id, session_id |
//! | `VelocityDeviation` (por articulación) | Info | `Timestamp(peak)` | joint, value, threshold, sample, elapsed_ms, plan_id, session_id |
//!
//! Unidades: value/threshold en rad (posiciones) o rad/s (velocidades);
//! elapsed_ms en ms; sample/joint son enteros 0-based. `plan_id`/`session_id`
//! son los IDs del [`PlanExecutionComparison`] — el eslabón de trazabilidad con
//! el plan (contrato C3). `causes`/`related` se emiten vacíos: el productor no
//! conoce los `ObservationId` (los asigna el aggregator 1..=n); el enlace
//! causal ejecución→plan se formaliza en el feedback loop (PR 4b) y se
//! demuestra end-to-end en el test C4 de este módulo.

use std::collections::BTreeMap;

use crate::comparison::PlanExecutionComparison;
use thalos_engine::core::analysis::{
    attribute_value::AttributeValue,
    location::Location,
    observation::{ArtifactRef, Observation, ObservationId, ObservationKind, Severity},
};

/// Umbrales por defecto para detección de problemas de ejecución.
const GLOBAL_RMSE_WARNING: f64 = 0.05; // 0.05 rad ≈ 2.86°
const MAX_ERROR_SPIKE: f64 = 0.10; // 0.10 rad ≈ 5.73°
const JOINT_MAX_ERROR_WARNING: f64 = 0.10; // 0.10 rad por articulación
const VELOCITY_DEVIATION_WARNING: f64 = 1.0; // 1 rad/s

/// Analiza una comparación plan-vs-ejecución y produce hallazgos objetivos.
///
/// Sigue el mismo principio que `PlanAdvisor`:
/// - Nunca recalcula datos que ya están en el comparison
/// - Solo interpreta y clasifica
pub struct ExecutionAnalyzer;

impl ExecutionAnalyzer {
    /// Crear con umbrales por defecto.
    pub fn new() -> Self {
        Self
    }

    /// Analizar una comparación y producir observaciones canónicas.
    ///
    /// Emisión DIRECTA: la observación es el único vocabulario de análisis
    /// (PR 7a eliminó el camino legacy `analyze_findings`). El schema temporal
    /// está documentado en el module doc.
    ///
    /// `artifact` ancla cada observación (I3): `ArtifactRef::ExecutionSession`
    /// con el id de la sesión ejecutada (el caller lo conoce; el analizador no
    /// lo inventa — mismo principio que `TrajectoryAnalyzer::analyze`).
    pub fn analyze(
        &self,
        artifact: ArtifactRef,
        comparison: &PlanExecutionComparison,
    ) -> Vec<Observation> {
        let mut observations = Vec::new();
        let metrics = &comparison.metrics;

        // Sin datos alineados no hay nada que analizar.
        if metrics.aligned_count == 0 {
            return observations;
        }

        let plan_id = comparison.plan_id.clone();
        let session_id = comparison.execution_id.clone();
        let session_elapsed_ms = (comparison.execution_duration * 1000.0).max(0.0) as u64;

        let mut push = |kind: ObservationKind,
                        severity: Severity,
                        location: Location,
                        mut attributes: BTreeMap<String, AttributeValue>| {
            // C3: eslabón de trazabilidad con el plan vía sus ids (el enlace
            // por `causes` a observaciones del plan se formaliza en PR 4b).
            attributes.insert("plan_id".to_string(), AttributeValue::Text(plan_id.clone()));
            attributes.insert(
                "session_id".to_string(),
                AttributeValue::Text(session_id.clone()),
            );
            observations.push(Observation {
                id: ObservationId(0), // el aggregator reasigna 1..=n (decisión cerrada)
                kind,
                severity,
                artifact: artifact.clone(),
                location,
                attributes,
                causes: Vec::new(),
                related: Vec::new(),
            });
        };

        // Ubicación temporal de los picos (C1): primera muestra donde cada
        // articulación alcanza su máximo, y el pico global.
        let joint_peaks = joint_error_peaks(comparison);
        let velocity_peaks = velocity_deviation_peaks(comparison);
        let mut global_peak: Option<(usize, &Peak)> = None;
        for (j, peak) in joint_peaks.iter().enumerate() {
            match global_peak {
                None => global_peak = Some((j, peak)),
                Some((_, current)) if peak.value > current.value => global_peak = Some((j, peak)),
                _ => {}
            }
        }

        // 1. Error de tracking global (RMSE) — agregado de sesión completa.
        if metrics.global_rmse > GLOBAL_RMSE_WARNING {
            let mut attributes = BTreeMap::new();
            attributes.insert(
                "value".to_string(),
                AttributeValue::Number(metrics.global_rmse),
            );
            attributes.insert(
                "threshold".to_string(),
                AttributeValue::Number(GLOBAL_RMSE_WARNING),
            );
            attributes.insert(
                "samples".to_string(),
                AttributeValue::Integer(metrics.aligned_count as i64),
            );
            attributes.insert(
                "elapsed_ms".to_string(),
                AttributeValue::Number(session_elapsed_ms as f64),
            );
            push(
                ObservationKind::TrackingError,
                Severity::Warning,
                // Sesión-anclado: el RMSE abarca 0..elapsed_ms, no un instante.
                Location::Timestamp(0),
                attributes,
            );
        }

        // 2. Pico de error máximo — fenómeno distinto (C2), anclado al instante.
        if metrics.global_max_error > MAX_ERROR_SPIKE {
            if let Some((joint, peak)) = global_peak {
                let mut attributes = BTreeMap::new();
                attributes.insert(
                    "value".to_string(),
                    AttributeValue::Number(metrics.global_max_error),
                );
                attributes.insert(
                    "threshold".to_string(),
                    AttributeValue::Number(MAX_ERROR_SPIKE),
                );
                attributes.insert("joint".to_string(), AttributeValue::Integer(joint as i64));
                attributes.insert(
                    "sample".to_string(),
                    AttributeValue::Integer(peak.sample as i64),
                );
                attributes.insert(
                    "elapsed_ms".to_string(),
                    AttributeValue::Number(peak.elapsed_ms as f64),
                );
                push(
                    ObservationKind::TrackingSpike,
                    Severity::Warning,
                    Location::Timestamp(peak.elapsed_ms),
                    attributes,
                );
            }
        }

        // 3. Desviaciones por articulación — una observación por articulación.
        for (j, peak) in joint_peaks.iter().enumerate() {
            if metrics.per_joint.max_error[j] > JOINT_MAX_ERROR_WARNING {
                let mut attributes = BTreeMap::new();
                attributes.insert("joint".to_string(), AttributeValue::Integer(j as i64));
                attributes.insert(
                    "value".to_string(),
                    AttributeValue::Number(metrics.per_joint.max_error[j]),
                );
                attributes.insert(
                    "threshold".to_string(),
                    AttributeValue::Number(JOINT_MAX_ERROR_WARNING),
                );
                attributes.insert(
                    "sample".to_string(),
                    AttributeValue::Integer(peak.sample as i64),
                );
                attributes.insert(
                    "elapsed_ms".to_string(),
                    AttributeValue::Number(peak.elapsed_ms as f64),
                );
                push(
                    ObservationKind::JointDeviation,
                    Severity::Warning,
                    Location::Timestamp(peak.elapsed_ms),
                    attributes,
                );
            }
        }

        // 4. Desviaciones de velocidad — una observación por articulación.
        for (j, peak) in velocity_peaks.iter().enumerate() {
            if metrics.max_velocity_deviation[j] > VELOCITY_DEVIATION_WARNING {
                let mut attributes = BTreeMap::new();
                attributes.insert("joint".to_string(), AttributeValue::Integer(j as i64));
                attributes.insert(
                    "value".to_string(),
                    AttributeValue::Number(metrics.max_velocity_deviation[j]),
                );
                attributes.insert(
                    "threshold".to_string(),
                    AttributeValue::Number(VELOCITY_DEVIATION_WARNING),
                );
                attributes.insert(
                    "sample".to_string(),
                    AttributeValue::Integer(peak.sample as i64),
                );
                attributes.insert(
                    "elapsed_ms".to_string(),
                    AttributeValue::Number(peak.elapsed_ms as f64),
                );
                push(
                    ObservationKind::VelocityDeviation,
                    Severity::Info,
                    Location::Timestamp(peak.elapsed_ms),
                    attributes,
                );
            }
        }

        observations
    }
}

/// Pico de una serie por articulación: valor máximo y su PRIMERA ocurrencia.
#[derive(Debug, Clone, Copy)]
struct Peak {
    value: f64,
    /// Índice 0-based del par alineado (muestra del plan) donde ocurre el pico.
    sample: usize,
    /// Timestamp del pico, en milisegundos (C1).
    elapsed_ms: u64,
}

/// Ruido de coma flotante tolerable al comparar picos: diferencias por debajo
/// de este valor representan la MISMA desviación (p. ej. 0.15 rad calculado
/// con distinto redondeo en dos muestras adyacentes), no un pico mayor.
const PEAK_EPSILON: f64 = 1e-9;

impl Peak {
    fn zero() -> Self {
        Self {
            value: 0.0,
            sample: 0,
            elapsed_ms: 0,
        }
    }

    /// Registra el pico si supera al actual más allá del ruido de coma
    /// flotante — la PRIMERA muestra del máximo gana (semántica de onset y
    /// determinismo).
    fn record(&mut self, value: f64, sample: usize, elapsed_ms: u64) {
        if value > self.value + PEAK_EPSILON {
            *self = Peak {
                value,
                sample,
                elapsed_ms,
            };
        }
    }
}

/// Máximo error por articulación y su ubicación temporal, sobre los pares
/// alineados. Misma fórmula que `compute_metrics` (`|actual − planned|`), con
/// la ubicación que las métricas agregadas no retienen (C1).
fn joint_error_peaks(comparison: &PlanExecutionComparison) -> Vec<Peak> {
    let Some(first) = comparison.alignment.pairs.first() else {
        return Vec::new();
    };
    let dof = first.planned_joints.len();
    let mut peaks = vec![Peak::zero(); dof];
    for (i, pair) in comparison.alignment.pairs.iter().enumerate() {
        let t_ms = pair.timestamp.as_millis() as u64;
        for (j, peak) in peaks.iter_mut().enumerate() {
            if j < pair.actual_joints.len() && j < pair.planned_joints.len() {
                let err = (pair.actual_joints[j] - pair.planned_joints[j]).abs();
                peak.record(err, i, t_ms);
            }
        }
    }
    peaks
}

/// Máxima desviación de velocidad por articulación y su ubicación temporal.
/// Vacío si la alineación no transporta velocidades.
fn velocity_deviation_peaks(comparison: &PlanExecutionComparison) -> Vec<Peak> {
    let Some(first) = comparison.alignment.pairs.first() else {
        return Vec::new();
    };
    if first.planned_velocities.is_empty() || first.actual_velocities.is_empty() {
        return Vec::new();
    }
    let dof = first
        .planned_velocities
        .len()
        .min(first.actual_velocities.len());
    let mut peaks = vec![Peak::zero(); dof];
    for (i, pair) in comparison.alignment.pairs.iter().enumerate() {
        let t_ms = pair.timestamp.as_millis() as u64;
        for (j, peak) in peaks.iter_mut().enumerate() {
            if j < pair.planned_velocities.len() && j < pair.actual_velocities.len() {
                let dev = (pair.actual_velocities[j] - pair.planned_velocities[j]).abs();
                peak.record(dev, i, t_ms);
            }
        }
    }
    peaks
}

impl Default for ExecutionAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::comparison::compare;
    use crate::motion_trace::{MotionSample, MotionTrace};
    use crate::session::ExecutionSource;
    use crate::telemetry::{ExecutionSample, ExecutionTrace, TraceMetadata};
    use std::collections::BTreeMap;
    use std::time::Duration;
    use thalos_engine::core::analysis::attribute_value::AttributeValue;
    use thalos_engine::core::analysis::location::Location;
    use thalos_engine::core::analysis::observation::{
        ArtifactRef, Observation, ObservationId, ObservationKind, Severity,
    };
    use thalos_engine::core::ids::ExecutionSessionId;

    fn make_perfect_trace() -> (MotionTrace, ExecutionTrace) {
        let mut plan = MotionTrace::new();

        for i in 0..5 {
            let t = Duration::from_secs_f64(i as f64 * 0.1);
            let joints = vec![0.1 * i as f64, 0.05 * i as f64];
            plan.push(MotionSample {
                timestamp: t,
                joints: joints.clone(),
                velocities: vec![0.1, 0.05],
                target_joints: None,
                progress: i as f64 / 4.0,
                errors: vec![],
            });
        }

        let meta = TraceMetadata {
            session_id: "test".into(),
            plan_id: "p1".into(),
            source: ExecutionSource::Simulation,
            robot_name: "test".into(),
            joint_count: 2,
            duration: Duration::from_secs_f64(0.4),
            sample_rate: 10.0,
        };
        let mut trace = ExecutionTrace::new(meta);
        for i in 0..5 {
            let t = Duration::from_secs_f64(i as f64 * 0.1);
            trace.push_sample(ExecutionSample {
                timestamp: t,
                joints: vec![0.1 * i as f64, 0.05 * i as f64], // identical to plan
                velocities: vec![0.1, 0.05],
                accelerations: vec![],
                tcp_pose: [0.0; 7],
                tcp_velocity: [0.0; 6],
                tracking_error: None,
                progress: i as f64 / 4.0,
            });
        }
        (plan, trace)
    }

    fn make_deviated_trace() -> (MotionTrace, ExecutionTrace) {
        let mut plan = MotionTrace::new();
        let mut exec_samples = vec![];

        for i in 0..5 {
            let t = Duration::from_secs_f64(i as f64 * 0.1);
            let joints = vec![0.1 * i as f64, 0.05 * i as f64];
            plan.push(MotionSample {
                timestamp: t,
                joints: joints.clone(),
                velocities: vec![0.1, 0.05],
                target_joints: None,
                progress: i as f64 / 4.0,
                errors: vec![],
            });
            // Execution deviates at sample 2 and 3
            let exec_joints = if i >= 2 {
                vec![0.1 * i as f64 + 0.15, 0.05 * i as f64 - 0.12]
            } else {
                joints
            };
            exec_samples.push(ExecutionSample {
                timestamp: t,
                joints: exec_joints,
                velocities: vec![0.1, 0.05],
                accelerations: vec![],
                tcp_pose: [0.0; 7],
                tcp_velocity: [0.0; 6],
                tracking_error: Some(0.08),
                progress: i as f64 / 4.0,
            });
        }

        let meta = TraceMetadata {
            session_id: "test".into(),
            plan_id: "p1".into(),
            source: ExecutionSource::Simulation,
            robot_name: "test".into(),
            joint_count: 2,
            duration: Duration::from_secs_f64(0.4),
            sample_rate: 10.0,
        };
        let mut trace = ExecutionTrace::new(meta);
        for s in exec_samples {
            trace.push_sample(s);
        }
        (plan, trace)
    }

    fn make_velocity_deviated_trace() -> (MotionTrace, ExecutionTrace) {
        let (plan, mut exec) = make_deviated_trace();
        // Velocity spike at sample 3 (t = 0.3 s): joint 0 deviates 1.5 rad/s
        // beyond the default 1.0 rad/s threshold.
        for (i, sample) in exec.samples.iter_mut().enumerate() {
            if i == 3 {
                sample.velocities = vec![0.1 + 1.5, 0.05];
            }
        }
        (plan, exec)
    }

    /// Artifact ancla de sesión para los tests (I3).
    fn session_artifact() -> ArtifactRef {
        ArtifactRef::ExecutionSession(ExecutionSessionId("e1".to_string()))
    }

    #[test]
    fn no_observations_for_perfect_execution() {
        let (plan, exec) = make_perfect_trace();
        let comparison = compare(&plan, &exec, "p1", "e1", "test");
        let analyzer = ExecutionAnalyzer::new();
        let observations = analyzer.analyze(session_artifact(), &comparison);

        assert!(
            observations.is_empty(),
            "Perfect execution should produce no observations, got: {:?}",
            observations.iter().map(|o| o.kind).collect::<Vec<_>>()
        );
    }

    #[test]
    fn detects_tracking_error_from_deviation() {
        let (plan, exec) = make_deviated_trace();
        let comparison = compare(&plan, &exec, "p1", "e1", "test");
        let analyzer = ExecutionAnalyzer::new();
        let observations = analyzer.analyze(session_artifact(), &comparison);

        // Should detect global tracking error
        let has_tracking_error = observations
            .iter()
            .any(|o| matches!(o.kind, ObservationKind::TrackingError));
        assert!(
            has_tracking_error,
            "Deviated execution should produce TrackingError"
        );

        // Should detect joint-level deviations
        let has_joint_deviation = observations
            .iter()
            .any(|o| matches!(o.kind, ObservationKind::JointDeviation));
        assert!(
            has_joint_deviation,
            "Deviated execution should produce JointDeviation"
        );

        // Should produce at least 2 observations
        assert!(
            observations.len() >= 2,
            "Expected >= 2 observations, got {}",
            observations.len()
        );
    }

    #[test]
    fn empty_comparison_produces_no_observations() {
        let plan = MotionTrace::new();
        let meta = TraceMetadata {
            session_id: "e".into(),
            plan_id: "p".into(),
            source: ExecutionSource::Simulation,
            robot_name: "".into(),
            joint_count: 0,
            duration: Duration::ZERO,
            sample_rate: 0.0,
        };
        let exec = crate::telemetry::ExecutionTrace::new(meta);
        let comparison = compare(&plan, &exec, "", "", "");
        let analyzer = ExecutionAnalyzer::new();
        let observations = analyzer.analyze(session_artifact(), &comparison);

        assert!(observations.is_empty());
    }

    #[test]
    fn observations_use_correct_kinds() {
        let (plan, exec) = make_deviated_trace();
        let comparison = compare(&plan, &exec, "p1", "e1", "test");
        let analyzer = ExecutionAnalyzer::new();
        let observations = analyzer.analyze(session_artifact(), &comparison);

        for o in &observations {
            match o.kind {
                ObservationKind::TrackingError
                | ObservationKind::TrackingSpike
                | ObservationKind::JointDeviation
                | ObservationKind::VelocityDeviation => {
                    // These are the expected kinds from ExecutionAnalyzer
                }
                other => {
                    panic!(
                        "Unexpected observation kind from ExecutionAnalyzer: {:?}",
                        other
                    );
                }
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PR 4a RED — ExecutionAnalyzer emits canonical Observations (tasks 4.1-4.2)
    // ═══════════════════════════════════════════════════════════════════════
    //
    // Spec planning-feedback-loop: execution findings are replaced by
    // observations. These tests were written FIRST and fail against the
    // pre-migration API.

    fn number(attrs: &BTreeMap<String, AttributeValue>, key: &str) -> f64 {
        match attrs.get(key) {
            Some(AttributeValue::Number(n)) => *n,
            other => panic!("attribute `{key}` must be a Number, got {other:?}"),
        }
    }

    fn integer(attrs: &BTreeMap<String, AttributeValue>, key: &str) -> i64 {
        match attrs.get(key) {
            Some(AttributeValue::Integer(n)) => *n,
            other => panic!("attribute `{key}` must be an Integer, got {other:?}"),
        }
    }

    fn text<'a>(attrs: &'a BTreeMap<String, AttributeValue>, key: &str) -> &'a str {
        match attrs.get(key) {
            Some(AttributeValue::Text(t)) => t.as_str(),
            other => panic!("attribute `{key}` must be a Text, got {other:?}"),
        }
    }

    /// Igualdad tolerante a ruido de coma flotante (valores derivados de
    /// restas/interpolaciones).
    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    // ── 4.1 RED: TrackingError as a canonical Observation ───────────────────
    #[test]
    fn analysis_emits_tracking_error_observation() {
        let (plan, exec) = make_deviated_trace();
        let comparison = compare(&plan, &exec, "p1", "e1", "test");
        let artifact = ArtifactRef::ExecutionSession(ExecutionSessionId("e1".to_string()));

        let observations = ExecutionAnalyzer::new().analyze(artifact.clone(), &comparison);

        let tracking = observations
            .iter()
            .find(|o| o.kind == ObservationKind::TrackingError)
            .expect("deviated execution must emit a TrackingError observation");

        // I2/I3: machine-identifiable — kind, artifact, location, typed
        // attributes; the legacy `message` field is gone (I1).
        assert_eq!(tracking.severity, Severity::Warning);
        assert_eq!(tracking.artifact, artifact);
        assert!(matches!(tracking.location, Location::Timestamp(_)));

        // value/threshold survive as typed data (spec planning-feedback-loop).
        assert_eq!(
            number(&tracking.attributes, "value"),
            comparison.metrics.global_rmse
        );
        assert_eq!(
            number(&tracking.attributes, "threshold"),
            0.05 // ExecutionThresholds::default().global_rmse_warning
        );
    }

    // ── 4.1 RED: clean trace → no observations ──────────────────────────────
    #[test]
    fn perfect_execution_emits_no_observations() {
        let (plan, exec) = make_perfect_trace();
        let comparison = compare(&plan, &exec, "p1", "e1", "test");
        let artifact = ArtifactRef::ExecutionSession(ExecutionSessionId("e1".to_string()));

        let observations = ExecutionAnalyzer::new().analyze(artifact, &comparison);
        assert!(
            observations.is_empty(),
            "clean execution must emit no observations, got {observations:?}"
        );
    }

    // ── 4.1 RED triangulation: empty comparison → no observations ───────────
    #[test]
    fn empty_comparison_emits_no_observations() {
        let plan = MotionTrace::new();
        let meta = TraceMetadata {
            session_id: "e".into(),
            plan_id: "p".into(),
            source: ExecutionSource::Simulation,
            robot_name: "".into(),
            joint_count: 0,
            duration: Duration::ZERO,
            sample_rate: 0.0,
        };
        let exec = crate::telemetry::ExecutionTrace::new(meta);
        let comparison = compare(&plan, &exec, "", "", "");
        let artifact = ArtifactRef::ExecutionSession(ExecutionSessionId("e".to_string()));

        let observations = ExecutionAnalyzer::new().analyze(artifact, &comparison);
        assert!(observations.is_empty());
    }

    // ── 4.1 RED + C2: distinct runtime phenomena → distinct kinds ───────────
    #[test]
    fn runtime_phenomena_have_distinct_observation_kinds() {
        let (plan, exec) = make_deviated_trace();
        let comparison = compare(&plan, &exec, "p1", "e1", "test");
        let artifact = ArtifactRef::ExecutionSession(ExecutionSessionId("e1".to_string()));

        let observations = ExecutionAnalyzer::new().analyze(artifact, &comparison);
        let kinds: Vec<ObservationKind> = observations.iter().map(|o| o.kind).collect();

        // The deviated trace triggers three DIFFERENT phenomena; each must be
        // its own ObservationKind so the expert system can react to each
        // specifically (user contract C2 — RuntimeDeviation is NOT a catch-all).
        assert!(
            kinds.contains(&ObservationKind::TrackingError),
            "sustained RMSE must be its own kind, got {kinds:?}"
        );
        assert!(
            kinds.contains(&ObservationKind::TrackingSpike),
            "transient peak must be its own kind, got {kinds:?}"
        );
        assert!(
            kinds.contains(&ObservationKind::JointDeviation),
            "per-joint deviation must be its own kind, got {kinds:?}"
        );
    }

    // ── C2: velocity deviation is its own phenomenon ────────────────────────
    #[test]
    fn velocity_deviation_is_a_distinct_phenomenon() {
        let (plan, exec) = make_velocity_deviated_trace();
        let comparison = compare(&plan, &exec, "p1", "e1", "test");
        let artifact = ArtifactRef::ExecutionSession(ExecutionSessionId("e1".to_string()));

        let observations = ExecutionAnalyzer::new().analyze(artifact, &comparison);
        let velocity = observations
            .iter()
            .find(|o| o.kind == ObservationKind::VelocityDeviation)
            .expect("velocity-deviated execution must emit a VelocityDeviation observation");

        assert_eq!(velocity.severity, Severity::Info);
        assert!(approx(number(&velocity.attributes, "value"), 1.5));
        assert_eq!(number(&velocity.attributes, "threshold"), 1.0);
        assert_eq!(integer(&velocity.attributes, "joint"), 0);
    }

    // ── 4.1 RED + C1: temporal preservation (user contract C1) ──────────────
    #[test]
    fn observations_carry_temporal_attributes() {
        let (plan, exec) = make_deviated_trace();
        let comparison = compare(&plan, &exec, "p1", "e1", "test");
        let artifact = ArtifactRef::ExecutionSession(ExecutionSessionId("e1".to_string()));

        let observations = ExecutionAnalyzer::new().analyze(artifact, &comparison);

        // Spike: max |error| = 0.15 rad happens at aligned pair 2 → t = 200 ms.
        let spike = observations
            .iter()
            .find(|o| o.kind == ObservationKind::TrackingSpike)
            .expect("deviated trace must emit a spike");
        assert!(matches!(spike.location, Location::Timestamp(200)));
        assert_eq!(integer(&spike.attributes, "sample"), 2);
        assert_eq!(number(&spike.attributes, "elapsed_ms"), 200.0);
        assert!(approx(number(&spike.attributes, "value"), 0.15));
        assert_eq!(integer(&spike.attributes, "joint"), 0);

        // Joint 1 peaks at the same pair (0.12 rad).
        let joint_dev = observations
            .iter()
            .find(|o| {
                o.kind == ObservationKind::JointDeviation && integer(&o.attributes, "joint") == 1
            })
            .expect("joint 1 must deviate");
        assert_eq!(integer(&joint_dev.attributes, "sample"), 2);
        assert_eq!(number(&joint_dev.attributes, "elapsed_ms"), 200.0);
        assert!(approx(number(&joint_dev.attributes, "value"), 0.12));

        // The session-level aggregate names the whole aligned span.
        let tracking = observations
            .iter()
            .find(|o| o.kind == ObservationKind::TrackingError)
            .expect("deviated trace must emit a tracking error");
        assert_eq!(integer(&tracking.attributes, "samples"), 5);
        assert_eq!(number(&tracking.attributes, "elapsed_ms"), 400.0);

        // C3: every observation names its plan and session (the alignment link).
        for o in &observations {
            assert_eq!(text(&o.attributes, "plan_id"), "p1");
            assert_eq!(text(&o.attributes, "session_id"), "e1");
        }
    }

    // ── C4 (mandatory): root-cause chain — execution deviation → causes →
    //    plan singularity → Action ───────────────────────────────────────────
    #[test]
    fn root_cause_chain_execution_deviation_to_plan_to_action() {
        use thalos_engine::core::analysis::action::{ActionId, ActionKind};
        use thalos_engine::core::analysis::aggregator::{Aggregator, DefaultAggregator};
        use thalos_engine::core::analysis::scoring::DefaultScoringPolicy;
        use thalos_engine::core::models::{RobotModel, RobotRegistry};
        use thalos_engine::core::trajectory::{Trajectory, TrajectoryPoint};
        use thalos_engine::planning::{advisor::PlanAdvisor, analysis::TrajectoryAnalyzer};

        // 1. PLAN layer: a singular configuration → a Singularity/NearSingularity
        //    observation anchored to the MotionPlan (TrajectoryAnalyzer, PR 3).
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0), // arm fully extended → singular
            TrajectoryPoint::new(vec![0.5, 1.57], 1.0), // good configuration
        ]);
        let plan_artifact =
            ArtifactRef::MotionPlan(thalos_engine::core::ids::MotionPlanId("mp-c4".to_string()));
        let mut plan_observations =
            TrajectoryAnalyzer::new(&chain, None).analyze(plan_artifact.clone(), &traj);
        let singular_idx = plan_observations
            .iter()
            .position(|o| {
                matches!(
                    o.kind,
                    ObservationKind::Singularity | ObservationKind::NearSingularity
                )
            })
            .expect("fully-extended arm must yield a singular observation");

        // 2. EXECUTION layer: execution deviating from that plan → execution
        //    observations anchored to the ExecutionSession (this PR).
        let (plan_trace, exec_trace) = make_deviated_trace();
        let comparison = compare(&plan_trace, &exec_trace, "mp-c4", "es-c4", "test");
        let exec_artifact = ArtifactRef::ExecutionSession(ExecutionSessionId("es-c4".to_string()));
        let mut exec_observations =
            ExecutionAnalyzer::new().analyze(exec_artifact.clone(), &comparison);
        let exec_dev_idx = exec_observations
            .iter()
            .position(|o| {
                matches!(
                    o.kind,
                    ObservationKind::TrackingError | ObservationKind::TrackingSpike
                )
            })
            .expect("deviated execution must yield execution observations");

        // 3. LINK (C3): producers cannot know aggregator-assigned ids (the
        //    aggregator reassigns 1..=n and remaps references), so the linking
        //    step — numbering + wiring `causes` — lives in the test today. The
        //    PR 4b feedback loop formalizes it (documented pending work).
        let plan_count = plan_observations.len();
        let mut all: Vec<Observation> = Vec::with_capacity(plan_count + exec_observations.len());
        all.append(&mut plan_observations);
        all.append(&mut exec_observations);
        for (i, obs) in all.iter_mut().enumerate() {
            obs.id = ObservationId((i + 1) as u32);
        }
        let singular_id = all[singular_idx].id;
        let exec_dev_id = all[plan_count + exec_dev_idx].id;

        // Feedback → plan direction only (planning-feedback-loop spec I4).
        assert_ne!(exec_dev_id, singular_id);
        all.iter_mut()
            .find(|o| o.id == exec_dev_id)
            .expect("execution observation must exist")
            .causes
            .push(singular_id);

        // 4. AGGREGATE: the combined feedback→plan graph → report; validate()
        //    must accept the acyclic chain.
        let mut report = DefaultAggregator::new(DefaultScoringPolicy).aggregate(exec_artifact, all);
        assert_eq!(report.validate(), Ok(()));

        // 5. ADVISOR: the plan singularity is remediated; the runtime deviation
        //    has no plan-level rule — the advisor never invents remediation (C2).
        let mut actions = PlanAdvisor.advise(&report.observations);
        assert!(
            actions.iter().any(|a| a.target_observation == singular_id),
            "the singular plan observation must be remediated"
        );
        assert!(
            actions.iter().any(|a| a.kind == ActionKind::Singularity),
            "singularity remediation must be the Singularity action"
        );
        assert!(
            !actions.iter().any(|a| a.target_observation == exec_dev_id),
            "runtime phenomena have no plan-level remediation rule (C2)"
        );
        // The user-contract C4 example names the IK remediation
        // (ActionKind::IkSolution); that action exists in the vocabulary but no
        // finding produces it today — the advisor maps singularity → the
        // Singularity action, its real 1:1 rule (advisor::remediation).
        for (i, action) in actions.iter_mut().enumerate() {
            action.id = ActionId((i + 1) as u32); // action ids are assigned downstream too
        }
        report.actions = actions;
        assert_eq!(report.validate(), Ok(()));
        assert!(report.summary.quality_index < 1.0);

        // NAVIGATION: walk the chain end-to-end.
        let exec_obs = report
            .observations
            .iter()
            .find(|o| o.id == exec_dev_id)
            .expect("execution observation present in report");
        assert_eq!(
            exec_obs.causes,
            vec![singular_id],
            "execution deviation is caused by the plan singularity"
        );
        let cause = report
            .observations
            .iter()
            .find(|o| o.id == singular_id)
            .expect("plan singularity present in report");
        assert_eq!(cause.artifact, plan_artifact);
        assert!(
            report
                .actions
                .iter()
                .any(|a| a.target_observation == singular_id),
            "the chain ends in a remediation Action targeting the plan observation"
        );
    }
}
