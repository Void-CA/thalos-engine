use std::sync::{Arc, Mutex};
use serde::{Deserialize, Serialize};
use super::domain::{ExecutionSessionId, LifecycleState, TickResult};

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

    pub fn current(sampled_at_us: u64) -> Self {
        let now_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64;

        Self {
            sampled_at_us,
            received_at_us: now_us,
            evaluated_at_us: now_us,
        }
    }
}

/// Contrato semántico de eventos de dominio emitidos por el runtime.
///
/// Principio rector: "El estado de dominio es autoritativo; los eventos son observaciones de sus transiciones".
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
        previous: LifecycleState,
        current: LifecycleState,
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

/// Trait para suscriptores a eventos de ejecución del dominio.
pub trait EventSubscriber: Send + Sync {
    fn on_event(&self, event: &ExecutionEvent);
}

/// Bus de eventos thread-safe en memoria para publicar observaciones del dominio.
#[derive(Clone, Default)]
pub struct ExecutionEventBus {
    subscribers: Arc<Mutex<Vec<Arc<dyn EventSubscriber>>>>,
}

impl ExecutionEventBus {
    pub fn new() -> Self {
        Self {
            subscribers: Arc::new(Mutex::new(Vec::new())),
        }
    }

    pub fn subscribe(&self, subscriber: Arc<dyn EventSubscriber>) {
        self.subscribers.lock().unwrap().push(subscriber);
    }

    pub fn publish(&self, event: ExecutionEvent) {
        let subscribers = self.subscribers.lock().unwrap().clone();
        for sub in subscribers {
            sub.on_event(&event);
        }
    }
}

impl std::fmt::Debug for ExecutionEventBus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "ExecutionEventBus {{ subscribers: {} }}",
            self.subscribers.lock().unwrap().len()
        )
    }
}
