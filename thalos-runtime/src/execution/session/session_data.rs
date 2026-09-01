use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

use crate::motion_trace::MotionTrace;
use crate::plan::session_status::SessionStatus;
use crate::plan::ExecutionMode;

use super::execution_source::ExecutionSource;

/// Datos persistentes de una sesión de ejecución.
///
/// Es una entidad de negocio, NO derivada de RobotState.
/// Describe qué ocurrió, cuándo, con qué origen y resultado.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SessionData {
    /// ID numérico secuencial (Execution #15).
    pub id: u64,
    /// ID del plan que se ejecutó.
    pub plan_id: String,
    /// Origen de la ejecución.
    pub source: ExecutionSource,
    /// Estado final de la sesión.
    pub status: SessionStatus,
    /// Cuándo comenzó la ejecución.
    pub started_at: Option<DateTime<Utc>>,
    /// Cuándo se pausó (última pausa).
    pub paused_at: Option<DateTime<Utc>>,
    /// Cuándo terminó (completed, cancelled, failed).
    pub completed_at: Option<DateTime<Utc>>,
    /// Duración total de la trayectoria en segundos.
    pub duration: f64,
    /// Cantidad de articulaciones.
    pub joint_count: usize,
    /// Nombre del robot (para display).
    pub robot_name: String,
    /// Execution mode (SM1). `#[serde(default)]` — persisted JSON from
    /// previous versions (or a missing field) loads as `Once` (S5).
    #[serde(default)]
    pub mode: ExecutionMode,
    /// Current iteration, 1-based (SM2). `#[serde(default = "default_iteration")]`
    /// — old JSON without the field loads as iteration 1.
    #[serde(default = "default_iteration")]
    pub iteration: u32,
    /// Total iterations from the mode; `None` for `Once` (SM3). Omitted from
    /// the wire when None so old session files stay shape-stable.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub total_iterations: Option<u32>,
}

/// Serde default for `SessionData::iteration` (SM2): a session starts at
/// iteration 1 — never 0.
fn default_iteration() -> u32 {
    1
}

/// Una sesión completa con su trace asociado.
#[derive(Debug, Clone)]
pub struct SessionWithTrace {
    pub session: SessionData,
    pub trace: Option<MotionTrace>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use crate::plan::ExecutionMode;

    /// JSON persisted by the PREVIOUS version — no mode/iteration/
    /// total_iterations fields (S5, SM-S1).
    fn old_session_json() -> serde_json::Value {
        json!({
            "id": 1,
            "plan_id": "plan-1",
            "source": "Simulation",
            "status": "Completed",
            "started_at": null,
            "paused_at": null,
            "completed_at": null,
            "duration": 2.0,
            "joint_count": 2,
            "robot_name": "test_robot"
        })
    }

    /// S5 / SM-S1: old JSON loads with defaults — Once, iteration 1,
    /// total_iterations None; no error.
    #[test]
    fn old_session_json_loads_with_defaults() {
        let session: SessionData =
            serde_json::from_value(old_session_json()).expect("old JSON must deserialize");
        assert_eq!(session.mode, ExecutionMode::Once);
        assert_eq!(session.iteration, 1);
        assert_eq!(session.total_iterations, None);
    }

    /// S6 / SM-S2: a Repeat { count: 3 } session at iteration 2 round-trips
    /// with all fields equal.
    #[test]
    fn new_session_json_round_trips() {
        let mut session: SessionData =
            serde_json::from_value(old_session_json()).expect("old JSON must deserialize");
        session.mode = ExecutionMode::Repeat { count: 3 };
        session.iteration = 2;
        session.total_iterations = Some(3);

        let value = serde_json::to_value(&session).expect("serialize");
        // Wire format (decision #1): externally-tagged, lowercase — a Repeat
        // mode serializes as `{"repeat":{"count":N}}`.
        assert_eq!(value["mode"], json!({ "repeat": { "count": 3 } }));
        assert_eq!(value["iteration"], 2);
        assert_eq!(value["total_iterations"], 3);

        let back: SessionData = serde_json::from_value(value).expect("deserialize round-trip");
        assert_eq!(back.mode, ExecutionMode::Repeat { count: 3 });
        assert_eq!(back.iteration, 2);
        assert_eq!(back.total_iterations, Some(3));
    }

    /// SM3: an Once session omits `total_iterations` on the wire
    /// (skip_serializing_if) and carries `mode: "once"`.
    #[test]
    fn once_session_omits_total_iterations_on_wire() {
        let session: SessionData =
            serde_json::from_value(old_session_json()).expect("old JSON must deserialize");
        let value = serde_json::to_value(&session).expect("serialize");
        assert_eq!(value["mode"], json!("once"));
        assert_eq!(value["iteration"], 1);
        assert!(value.get("total_iterations").is_none());
    }
}
