use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use serde::{Deserialize, Serialize};

use super::domain::{ExecutionConfiguration, ExecutionSessionId, LifecycleState, TickOutcome, TickResult};
use super::events::{EventSubscriber, ExecutionEvent, TemporalInvariants};

/// Registro inmutable de un tick evaluado dentro de una historia de ejecución.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HistoricalTickRecord {
    /// Resultado inmutable del tick k (observación + decisión + acción + outcome post-act).
    pub result: TickResult,
    /// Invariante temporal con relojes de adquisición, ingesta y evaluación.
    pub temporal: TemporalInvariants,
}

/// Registro inmutable de una transición de ciclo de vida.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HistoricalLifecycleTransition {
    pub previous: LifecycleState,
    pub current: LifecycleState,
    pub timestamp_us: u64,
}

/// Registro de fallos ocurridos durante la sesión.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HistoricalFaultRecord {
    pub tick_index: u64,
    pub reason: String,
    pub timestamp_us: u64,
}

/// Historia completa e inmutable de una `ExecutionSession`.
///
/// Responsabilidad: Reconstrucción determinista y auditoría post-hoc de una sesión.
/// Límite de Dominio: Acotado al ciclo de vida de UNA sesión individual (Session-Scoped).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExecutionHistory {
    pub session_id: ExecutionSessionId,
    pub program_id: String,
    pub configuration: ExecutionConfiguration,
    pub created_at_us: u64,
    pub completed_at_us: Option<u64>,
    pub final_lifecycle: LifecycleState,
    pub lifecycle_transitions: Vec<HistoricalLifecycleTransition>,
    pub ticks: Vec<HistoricalTickRecord>,
    pub faults: Vec<HistoricalFaultRecord>,
}

impl ExecutionHistory {
    pub fn new(
        session_id: ExecutionSessionId,
        program_id: String,
        configuration: ExecutionConfiguration,
        created_at_us: u64,
    ) -> Self {
        Self {
            session_id,
            program_id,
            configuration,
            created_at_us,
            completed_at_us: None,
            final_lifecycle: LifecycleState::Created,
            lifecycle_transitions: Vec::new(),
            ticks: Vec::new(),
            faults: Vec::new(),
        }
    }

    pub fn record_lifecycle(&mut self, previous: LifecycleState, current: LifecycleState, timestamp_us: u64) {
        self.final_lifecycle = current.clone();
        if current.is_terminal() {
            self.completed_at_us = Some(timestamp_us);
        }
        self.lifecycle_transitions.push(HistoricalLifecycleTransition {
            previous,
            current,
            timestamp_us,
        });
    }

    pub fn record_tick(&mut self, result: TickResult, temporal: TemporalInvariants) {
        if let TickOutcome::Faulted(ref reason) = result.outcome {
            self.faults.push(HistoricalFaultRecord {
                tick_index: result.tick.index,
                reason: reason.clone(),
                timestamp_us: temporal.evaluated_at_us,
            });
        }
        self.ticks.push(HistoricalTickRecord { result, temporal });
    }

    pub fn record_fault(&mut self, reason: String, timestamp_us: u64) {
        let tick_index = self.ticks.last().map(|t| t.result.tick.index).unwrap_or(0);
        self.faults.push(HistoricalFaultRecord {
            tick_index,
            reason,
            timestamp_us,
        });
    }
}

/// Almacén en memoria de historias de ejecuciones (`ExecutionHistory`),
/// que actúa como `EventSubscriber` consumiendo eventos del `ExecutionEventBus`.
///
/// Demuestra la separación:
/// - `ExecutionEventBus` (distribución efímera en tiempo real)
/// - `ExecutionHistoryStore` (reconstrucción estructurada por sesión)
#[derive(Clone, Default)]
pub struct ExecutionHistoryStore {
    histories: Arc<Mutex<HashMap<ExecutionSessionId, ExecutionHistory>>>,
}

impl ExecutionHistoryStore {
    pub fn new() -> Self {
        Self {
            histories: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn get_history(&self, session_id: &ExecutionSessionId) -> Option<ExecutionHistory> {
        self.histories.lock().unwrap().get(session_id).cloned()
    }

    pub fn list_histories(&self) -> Vec<ExecutionHistory> {
        self.histories.lock().unwrap().values().cloned().collect()
    }
}

impl EventSubscriber for ExecutionHistoryStore {
    fn on_event(&self, event: &ExecutionEvent) {
        let mut map = self.histories.lock().unwrap();

        match event {
            ExecutionEvent::SessionCreated {
                session_id,
                program_id,
                timestamp_us,
            } => {
                map.entry(session_id.clone()).or_insert_with(|| {
                    ExecutionHistory::new(
                        session_id.clone(),
                        program_id.clone(),
                        ExecutionConfiguration::default(),
                        *timestamp_us,
                    )
                });
            }
            ExecutionEvent::LifecycleChanged {
                session_id,
                previous,
                current,
                timestamp_us,
            } => {
                if let Some(history) = map.get_mut(session_id) {
                    history.record_lifecycle(previous.clone(), current.clone(), *timestamp_us);
                }
            }
            ExecutionEvent::TickEvaluated {
                session_id,
                result,
                temporal,
            } => {
                if let Some(history) = map.get_mut(session_id) {
                    history.record_tick(result.clone(), temporal.clone());
                }
            }
            ExecutionEvent::SessionFaulted {
                session_id,
                reason,
                timestamp_us,
            } => {
                if let Some(history) = map.get_mut(session_id) {
                    history.record_fault(reason.clone(), *timestamp_us);
                }
            }
        }
    }
}
