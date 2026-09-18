use serde::{Deserialize, Serialize};

use crate::execution::session::{ExecutionSessionState, TickResult};
use crate::ids::ExecutionSessionId;

/// Invariante temporal con distinción explícita de relojes de medición, recepción y evaluación.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TemporalInvariants {
    /// Cuándo ocurrió la adquisición en el origen físico/simulado (microsegundos UNIX).
    pub sampled_at_us: u64,
    /// Cuándo fue recibida la muestra por la capa de transporte/adquisición de Thalos (microsegundos UNIX).
    pub received_at_us: u64,
    /// Cuándo se evaluó el tick de control en ExecutionSession (microsegundos UNIX).
    pub evaluated_at_us: u64,
}

impl TemporalInvariants {
    pub fn new(sampled_at_us: u64, received_at_us: u64, evaluated_at_us: u64) -> Self {
        Self {
            sampled_at_us,
            received_at_us,
            evaluated_at_us,
        }
    }

}

/// Contrato semántico de eventos de dominio emitidos por el runtime.
///
/// Principio rector: "El estado de dominio es autoritativo; los eventos son observaciones de sus transiciones".
// `TickEvaluated` carries the full `TickResult` by value on purpose: this is a
// public event-vocabulary type and boxing it would change its shape for every
// consumer (runtime bus, persistence, DTO mapping). The size difference is accepted.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ExecutionEvent {
    /// Instanciación de una nueva sesión de ejecución.
    SessionCreated {
        session_id: ExecutionSessionId,
        program_id: String,
        timestamp_us: u64,
    },
    /// Transición autoritativa en la máquina de estados de ciclo de vida.
    LifecycleChanged {
        session_id: ExecutionSessionId,
        previous: ExecutionSessionState,
        current: ExecutionSessionState,
        timestamp_us: u64,
    },
    /// Agregado completo de observabilidad emitido tras la evaluación y ejecución (post-act) del tick k.
    TickEvaluated {
        session_id: ExecutionSessionId,
        result: TickResult,
        temporal: TemporalInvariants,
    },
    /// Fallo crítico durante el ciclo de vida o la ejecución de una acción.
    SessionFaulted {
        session_id: ExecutionSessionId,
        reason: String,
        timestamp_us: u64,
    },
}
