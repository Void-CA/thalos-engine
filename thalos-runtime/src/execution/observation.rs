use serde::{Deserialize, Serialize};
use thalos_engine::prelude::*;
use super::executor::ExecutionSessionState;

pub use thalos_ports::SignalQuality;

/// Hecho de telemetría individual emitido por el proceso/controlador físico.
/// Preserva el tiempo de origen ($t_{src}$) y el tiempo de recepción local ($t_{rcv}$).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Observation {
    pub session_id: Option<ExecutionSessionId>,
    pub sequence: u64,
    /// Estampa de tiempo en el origen ($t_{src}$) en nanosegundos (reloj del robot/controlador).
    pub sampled_at_ns: u64,
    /// Estampa de tiempo de recepción ($t_{rcv}$) en nanosegundos (reloj de la estación Thalos).
    pub received_at_ns: u64,
    pub joint_positions: Vec<f64>,
    pub joint_velocities: Vec<f64>,
    pub tcp_pose: [f64; 7],
    pub signal_quality: SignalQuality,
}

impl Observation {
    /// Latencia de transporte en nanosegundos ($\Delta t_{comm} = t_{rcv} - t_{src}$).
    pub fn comm_latency_ns(&self) -> u64 {
        self.received_at_ns.saturating_sub(self.sampled_at_ns)
    }
}

/// Estado operativo derivado actual de la observabilidad.
/// Distingue la evidencia individual (`Observation`) del estado filtrado en vivo.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ObservationSnapshot {
    pub latest: Observation,
    pub signal_quality: SignalQuality,
    pub freshness_ns: u64,
}

/// ExecutionSnapshot (ADR-014)
/// Snapshot ligero del estado operacional del ciclo de vida de una sesión activa.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ExecutionSnapshot {
    pub session_id: ExecutionSessionId,
    pub state: ExecutionSessionState,
    pub elapsed_seconds: f64,
    pub progress: f64,
}

/// Caracterización diferencial entre la trayectoria esperada y la observada.
/// Anchored con las estampas de tiempo de la expectativa y la observación para auditoría explicable.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ExecutionDeviation {
    pub expected_at_ns: u64,
    pub observed_sampled_at_ns: u64,
    pub max_joint_error_rad: f64,
    pub joint_errors: Vec<f64>,
    pub tcp_error_mm: f64,
    pub tracking_error: f64,
}

/// Resultado normativo graduado de la evaluación de política sobre desviaciones y contexto.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum DecisionOutcome {
    Ignore,
    Record,
    Notify,
    ModifyExecution { speed_scale_percent: u32 },
    Stop,
}

/// Decisión explicable producida por el motor de evaluación de política.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExplicableDecision {
    pub outcome: DecisionOutcome,
    pub reason: String,
    pub deviation: Option<ExecutionDeviation>,
    pub signal_quality: SignalQuality,
    pub persistence_count: u32,
    pub evaluated_at_ns: u64,
}

/// Acción de respuesta emitida hacia la planta con soporte de verificación posterior.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VerifiableAction {
    pub action_id: u64,
    pub issued_at_ns: u64,
    pub action_type: String,
    pub target: String,
    pub verified_at_ns: Option<u64>,
}

/// Estado de réplica diferida (UC-09), desacoplado del hecho físico de la observabilidad.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ReplicationState {
    Local,
    Pending,
    Replicated { acknowledged_at_ns: u64 },
    Failed { reason: String, retries: u32 },
}

/// Registro del estado de infraestructura para la evidencia de una observación.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObservationReplicationStatus {
    pub observation_sequence: u64,
    pub session_id: ExecutionSessionId,
    pub state: ReplicationState,
    pub updated_at_ns: u64,
}

/// RunSnapshot (ADR-014)
/// Consolidado de observabilidad en vivo para la interfaz (RoboticsRunSurface).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RunSnapshot {
    pub execution: ExecutionSnapshot,
    pub observation: ObservationSnapshot,
    pub deviation: Option<ExecutionDeviation>,
}

impl RunSnapshot {
    pub fn compute_deviation(
        expected_joints: &[f64],
        expected_time_ns: u64,
        observed: &Observation,
    ) -> Option<ExecutionDeviation> {
        if expected_joints.len() != observed.joint_positions.len() {
            return None;
        }

        let joint_errors: Vec<f64> = expected_joints
            .iter()
            .zip(&observed.joint_positions)
            .map(|(exp, obs)| (exp - obs).abs())
            .collect();

        let max_joint_error_rad = joint_errors
            .iter()
            .copied()
            .fold(0.0f64, f64::max);

        let tracking_error = max_joint_error_rad; // Proximal joint error
        let tcp_error_mm = tracking_error * 100.0; // Simulated mm error scaling

        Some(ExecutionDeviation {
            expected_at_ns: expected_time_ns,
            observed_sampled_at_ns: observed.sampled_at_ns,
            max_joint_error_rad,
            joint_errors,
            tcp_error_mm,
            tracking_error,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dual_timestamp_observation_latency() {
        let obs = Observation {
            session_id: Some(ExecutionSessionId("exec-test".into())),
            sequence: 101,
            sampled_at_ns: 1_000_000_000,
            received_at_ns: 1_005_000_000,
            joint_positions: vec![0.0, 1.0, 0.5],
            joint_velocities: vec![0.0, 0.0, 0.0],
            tcp_pose: [0.0; 7],
            signal_quality: SignalQuality::Nominal,
        };

        assert_eq!(obs.comm_latency_ns(), 5_000_000); // 5 ms transport latency
    }

    #[test]
    fn test_compute_explicable_deviation() {
        let expected = vec![0.0, 1.0, 0.5];
        let observed = Observation {
            session_id: Some(ExecutionSessionId("exec-test".into())),
            sequence: 1,
            sampled_at_ns: 1000,
            received_at_ns: 1005,
            joint_positions: vec![0.01, 0.99, 0.52],
            joint_velocities: vec![0.0, 0.0, 0.0],
            tcp_pose: [0.0; 7],
            signal_quality: SignalQuality::Nominal,
        };

        let dev = RunSnapshot::compute_deviation(&expected, 990, &observed).unwrap();
        assert_eq!(dev.expected_at_ns, 990);
        assert_eq!(dev.observed_sampled_at_ns, 1000);
        assert!((dev.max_joint_error_rad - 0.02).abs() < 1e-6);
        assert_eq!(dev.joint_errors.len(), 3);
    }

    #[test]
    fn test_replication_status_decoupled_from_observation() {
        let session_id = ExecutionSessionId("exec-100".into());
        let status = ObservationReplicationStatus {
            observation_sequence: 42,
            session_id: session_id.clone(),
            state: ReplicationState::Pending,
            updated_at_ns: 2000,
        };

        assert_eq!(status.state, ReplicationState::Pending);
    }
}

