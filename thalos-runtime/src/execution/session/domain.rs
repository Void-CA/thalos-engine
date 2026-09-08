use std::collections::HashMap;
use std::sync::Mutex;
use std::time::Duration;
use serde::{Deserialize, Serialize};
use thalos_core::device::ChannelObservation;

/// Identificador único para una sesión de ejecución.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ExecutionSessionId(pub String);

impl ExecutionSessionId {
    pub fn new(id: impl Into<String>) -> Self {
        Self(id.into())
    }

    pub fn generate() -> Self {
        Self(format!("exec-{}", uuid::Uuid::new_v4()))
    }
}

impl std::fmt::Display for ExecutionSessionId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// Entorno donde se evalúa y ejecuta la sesión.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Environment {
    VirtualSimulation,
    Physical,
    /// Sensor-informed simulation.
    Hybrid,
}

/// Cardinalidad o repetición del patrón del programa dentro de la sesión.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Cardinality {
    Once,
    Counted(u32),
    Continuous,
}

/// Política de reactividad del runtime ante observaciones.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Reactivity {
    Reactive,
    NonReactive,
}

/// Criterio de terminación formal de la sesión de ejecución.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TerminationPolicy {
    NaturalCompletion,
    Condition(String),
    UserStop,
    SafetyFault,
    Timeout(Duration),
}

/// Configuración inmutable de las cuatro dimensiones ortogonales de la sesión.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionConfiguration {
    pub environment: Environment,
    pub cardinality: Cardinality,
    pub reactivity: Reactivity,
    pub termination: TerminationPolicy,
}

impl Default for ExecutionConfiguration {
    fn default() -> Self {
        Self {
            environment: Environment::VirtualSimulation,
            cardinality: Cardinality::Once,
            reactivity: Reactivity::NonReactive,
            termination: TerminationPolicy::NaturalCompletion,
        }
    }
}

/// Estados del ciclo de vida de la entidad ExecutionSession.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum LifecycleState {
    Created,
    Initializing,
    Running,
    Paused,
    Completed,
    Stopped,
    Faulted(String),
}

impl LifecycleState {
    pub fn is_terminal(&self) -> bool {
        matches!(
            self,
            Self::Completed | Self::Stopped | Self::Faulted(_)
        )
    }
}

/// Estado de las variables y puntero de programa DSL.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ProgramState {
    pub program_counter: usize,
    pub local_vars: HashMap<String, String>,
}

/// Estado físico o simulado actual del robot.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RobotState {
    pub joints: Vec<f64>,
    pub velocities: Vec<f64>,
}

/// Estado estimado ideal/modelo digital del robot.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ExpectedState {
    pub simulated_joints: Vec<f64>,
}

/// A collection of observations associated with a single execution sampling point.
///
/// `captured_at_us` marks when the bundle was assembled — it does NOT guarantee
/// that all contained observations share the same physical timestamp.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ObservationBundle {
    pub captured_at_us: u64,
    pub observations: HashMap<String, ChannelObservation>,
}

/// Contexto de observación agrupado para alimentar la evaluación del tick k.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TickContext {
    pub observations: ObservationBundle,
    pub robot: RobotState,
    pub expected: ExpectedState,
}

impl TickContext {
    pub fn new(observations: ObservationBundle, robot: RobotState, expected: ExpectedState) -> Self {
        Self {
            observations,
            robot,
            expected,
        }
    }
}

/// Estado de ciclos de control e iteraciones.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CycleState {
    pub tick_count: u64,
    pub current_cycle: u32,
}

/// Contexto inmutable de observaciones capturadas para el tick de control k.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlTick {
    pub index: u64,
    pub timestamp_ns: u64,
    pub observations: ObservationBundle,
    pub robot: RobotState,
    pub expected: ExpectedState,
}

/// Decisión semántica derivada de la evaluación del programa en el tick k.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Decision {
    Continue,
    BranchTaken { branch_name: String },
    MotionAction { motion_type: String, target_name: String },
    WaitAction { duration_secs: f64 },
    TerminateSession { reason: String },
    NoOp,
}

/// Acción de control disparada por la decisión hacia los actuadores/simulador.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Action {
    DispatchMotion { kind: String, target: String },
    SetOutput { name: String, value: bool },
    HoldPosition,
    None,
}

/// Resultado de la ejecución de una acción en el tick k.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TickOutcome {
    Success,
    Preempted,
    Faulted(String),
    SessionCompleted,
}

/// Resultado atómico y trazable devuelto por la evaluación de un ControlTick.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TickResult {
    pub tick: ControlTick,
    pub decision: Decision,
    pub action: Action,
    pub outcome: TickOutcome,
}

/// Estado completo de runtime agrupado para la sesión.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SessionState {
    pub program: ProgramState,
    pub robot: RobotState,
    pub expected: ExpectedState,
    pub observations: ObservationBundle,
    pub cycle: CycleState,
}

/// Error retornado al intentar transiciones de ciclo de vida inválidas.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("Transición de ciclo de vida inválida desde {from:?} hasta {to_action}")]
pub struct InvalidLifecycleTransition {
    pub from: LifecycleState,
    pub to_action: &'static str,
}

/// Error retornado por operaciones de orquestación de dominio.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ExecutionDomainError {
    #[error("Sesión no encontrada: {0}")]
    SessionNotFound(ExecutionSessionId),

    #[error("Transición de ciclo de vida inválida: {0}")]
    InvalidLifecycle(#[from] InvalidLifecycleTransition),

    #[error("La sesión no está en estado Running (estado actual: {0:?})")]
    NotRunning(LifecycleState),
}

/// Entidad de dominio ExecutionSession.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutionSession {
    pub id: ExecutionSessionId,
    pub program_id: String,
    pub station_id: Option<String>,
    pub robotics_module_id: Option<String>,
    pub configuration: ExecutionConfiguration,
    pub lifecycle: LifecycleState,
    pub state: SessionState,
    pub history: Vec<LifecycleState>,
}

impl ExecutionSession {
    pub fn new(program_id: impl Into<String>, configuration: ExecutionConfiguration) -> Self {
        let initial_state = LifecycleState::Created;
        Self {
            id: ExecutionSessionId::generate(),
            program_id: program_id.into(),
            station_id: None,
            robotics_module_id: None,
            configuration,
            lifecycle: initial_state.clone(),
            state: SessionState::default(),
            history: vec![initial_state],
        }
    }

    fn record_transition(&mut self, next: LifecycleState) {
        self.lifecycle = next.clone();
        self.history.push(next);
    }

    /// Transición: Created -> Initializing
    pub fn initialize(&mut self) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            LifecycleState::Created => {
                self.record_transition(LifecycleState::Initializing);
                Ok(())
            }
            _ => Err(InvalidLifecycleTransition {
                from: self.lifecycle.clone(),
                to_action: "initialize",
            }),
        }
    }

    /// Transición: Initializing / Paused -> Running
    pub fn start(&mut self) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            LifecycleState::Initializing | LifecycleState::Paused => {
                self.record_transition(LifecycleState::Running);
                Ok(())
            }
            _ => Err(InvalidLifecycleTransition {
                from: self.lifecycle.clone(),
                to_action: "start",
            }),
        }
    }

    /// Transición: Running -> Paused
    pub fn pause(&mut self) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            LifecycleState::Running => {
                self.record_transition(LifecycleState::Paused);
                Ok(())
            }
            _ => Err(InvalidLifecycleTransition {
                from: self.lifecycle.clone(),
                to_action: "pause",
            }),
        }
    }

    /// Transición: Running / Paused -> Stopped
    pub fn stop(&mut self) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            LifecycleState::Running | LifecycleState::Paused => {
                self.record_transition(LifecycleState::Stopped);
                Ok(())
            }
            _ => Err(InvalidLifecycleTransition {
                from: self.lifecycle.clone(),
                to_action: "stop",
            }),
        }
    }

    /// Transición: Cualquier estado activo -> Faulted
    pub fn fault(&mut self, reason: impl Into<String>) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            LifecycleState::Completed | LifecycleState::Stopped => Err(InvalidLifecycleTransition {
                from: self.lifecycle.clone(),
                to_action: "fault",
            }),
            _ => {
                self.record_transition(LifecycleState::Faulted(reason.into()));
                Ok(())
            }
        }
    }

    /// Transición: Running -> Completed
    pub fn complete(&mut self) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            LifecycleState::Running => {
                self.record_transition(LifecycleState::Completed);
                Ok(())
            }
            _ => Err(InvalidLifecycleTransition {
                from: self.lifecycle.clone(),
                to_action: "complete",
            }),
        }
    }

    /// Manejo de la semántica de ciclo del programa (ProgramCycleCompletion).
    /// Incrementa el contador de ciclos y determina si la sesión debe continuar o finalizar según la Cardinalidad.
    pub fn complete_program_cycle(&mut self) -> bool {
        self.state.cycle.current_cycle += 1;
        match self.configuration.cardinality {
            Cardinality::Once => false,
            Cardinality::Counted(n) => self.state.cycle.current_cycle < n,
            Cardinality::Continuous => true,
        }
    }

    /// Avanza el contador de ticks de control k de forma estrictamente monotónica.
    pub fn advance_tick(&mut self) {
        self.state.cycle.tick_count += 1;
    }

    /// Ejecuta la evaluación atómica de un ControlTick k contra un TickContext inmutable.
    /// Respeta la secuencia del RFC: Acquire -> Observe -> Evaluate Termination -> Evaluate Program -> Decide -> Act.
    pub fn evaluate_tick(
        &mut self,
        context: TickContext,
        eval_fn: impl FnOnce(&ObservationBundle, &RobotState) -> (Decision, Action),
    ) -> Result<TickResult, InvalidLifecycleTransition> {
        if self.lifecycle != LifecycleState::Running {
            return Err(InvalidLifecycleTransition {
                from: self.lifecycle.clone(),
                to_action: "evaluate_tick",
            });
        }

        self.advance_tick();
        let tick_index = self.state.cycle.tick_count;

        let tick = ControlTick {
            index: tick_index,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            observations: context.observations.clone(),
            robot: context.robot.clone(),
            expected: context.expected.clone(),
        };

        // 1. Actualizar estado latched en la sesión
        self.state.observations = context.observations;
        self.state.robot = context.robot;
        self.state.expected = context.expected;

        // 2. Evaluación de condición de terminación previa a la acción
        let condition_met = match self.configuration.termination {
            TerminationPolicy::Condition(ref cond_channel) => {
                if let Some(obs) = tick.observations.observations.get(cond_channel) {
                    let val = match obs.value {
                        thalos_core::device::ChannelValue::Scalar(v) => v,
                        thalos_core::device::ChannelValue::Integer(v) => v as f64,
                        thalos_core::device::ChannelValue::Boolean(v) => {
                            if v { 1.0 } else { 0.0 }
                        }
                    };
                    if val > 0.0 {
                        Some(cond_channel.clone())
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
            _ => None,
        };

        if let Some(cond_channel) = condition_met {
            self.complete()?;
            return Ok(TickResult {
                tick,
                decision: Decision::TerminateSession {
                    reason: format!("Termination condition '{}' satisfied", cond_channel),
                },
                action: Action::None,
                outcome: TickOutcome::SessionCompleted,
            });
        }

        // 3. Evaluación de programa y selección de decisión/acción
        let (decision, action) = eval_fn(&tick.observations, &tick.robot);

        Ok(TickResult {
            tick,
            decision,
            action,
            outcome: TickOutcome::Success,
        })
    }
}

/// Registro thread-safe en memoria para entidades ExecutionSession.
#[derive(Debug, Default)]
pub struct SessionRegistry {
    sessions: Mutex<HashMap<ExecutionSessionId, ExecutionSession>>,
}

impl SessionRegistry {
    pub fn new() -> Self {
        Self {
            sessions: Mutex::new(HashMap::new()),
        }
    }

    pub fn register(&self, session: ExecutionSession) -> ExecutionSessionId {
        let id = session.id.clone();
        self.sessions.lock().unwrap().insert(id.clone(), session);
        id
    }

    pub fn get(&self, id: &ExecutionSessionId) -> Option<ExecutionSession> {
        self.sessions.lock().unwrap().get(id).cloned()
    }

    pub fn list_sessions(&self) -> Vec<ExecutionSessionId> {
        self.sessions.lock().unwrap().keys().cloned().collect()
    }

    pub fn with_session_mut<F, R>(&self, id: &ExecutionSessionId, f: F) -> Result<R, ExecutionDomainError>
    where
        F: FnOnce(&mut ExecutionSession) -> Result<R, ExecutionDomainError>,
    {
        let mut guard = self.sessions.lock().unwrap();
        let session = guard
            .get_mut(id)
            .ok_or_else(|| ExecutionDomainError::SessionNotFound(id.clone()))?;
        f(session)
    }
}

/// Orquestador puro de dominio para gestionar el ciclo de vida y despacho de ticks sobre ExecutionSession.
#[derive(Debug, Default)]
pub struct DomainExecutionCoordinator {
    pub registry: SessionRegistry,
    pub event_bus: super::events::ExecutionEventBus,
}

impl DomainExecutionCoordinator {
    pub fn new() -> Self {
        Self {
            registry: SessionRegistry::new(),
            event_bus: super::events::ExecutionEventBus::new(),
        }
    }

    pub fn with_event_bus(event_bus: super::events::ExecutionEventBus) -> Self {
        Self {
            registry: SessionRegistry::new(),
            event_bus,
        }
    }

    pub fn create_session(
        &self,
        program_id: impl Into<String>,
        config: ExecutionConfiguration,
    ) -> ExecutionSessionId {
        let prog = program_id.into();
        let session = ExecutionSession::new(prog.clone(), config);
        let id = self.registry.register(session);

        let now_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64;

        self.event_bus.publish(super::events::ExecutionEvent::SessionCreated {
            session_id: id.clone(),
            program_id: prog,
            timestamp_us: now_us,
        });

        id
    }

    pub fn create_session_with_target(
        &self,
        station_id: impl Into<String>,
        robotics_module_id: impl Into<String>,
        program_id: impl Into<String>,
        config: ExecutionConfiguration,
    ) -> ExecutionSessionId {
        let st_id = station_id.into();
        let rob_id = robotics_module_id.into();
        let prog = program_id.into();
        let mut session = ExecutionSession::new(prog.clone(), config);
        session.station_id = Some(st_id);
        session.robotics_module_id = Some(rob_id);
        let id = self.registry.register(session);

        let now_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64;

        self.event_bus.publish(super::events::ExecutionEvent::SessionCreated {
            session_id: id.clone(),
            program_id: prog,
            timestamp_us: now_us,
        });

        id
    }

    pub fn initialize(&self, id: &ExecutionSessionId) -> Result<(), ExecutionDomainError> {
        self.registry.with_session_mut(id, |session| {
            let prev = session.lifecycle.clone();
            session.initialize()?;
            let curr = session.lifecycle.clone();

            let now_us = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64;

            self.event_bus.publish(super::events::ExecutionEvent::LifecycleChanged {
                session_id: id.clone(),
                previous: prev,
                current: curr,
                timestamp_us: now_us,
            });

            Ok(())
        })
    }

    pub fn start(&self, id: &ExecutionSessionId) -> Result<(), ExecutionDomainError> {
        self.registry.with_session_mut(id, |session| {
            let prev = session.lifecycle.clone();
            session.start()?;
            let curr = session.lifecycle.clone();

            let now_us = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64;

            self.event_bus.publish(super::events::ExecutionEvent::LifecycleChanged {
                session_id: id.clone(),
                previous: prev,
                current: curr,
                timestamp_us: now_us,
            });

            Ok(())
        })
    }

    pub fn pause(&self, id: &ExecutionSessionId) -> Result<(), ExecutionDomainError> {
        self.registry.with_session_mut(id, |session| {
            let prev = session.lifecycle.clone();
            session.pause()?;
            let curr = session.lifecycle.clone();

            let now_us = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64;

            self.event_bus.publish(super::events::ExecutionEvent::LifecycleChanged {
                session_id: id.clone(),
                previous: prev,
                current: curr,
                timestamp_us: now_us,
            });

            Ok(())
        })
    }

    pub fn stop(&self, id: &ExecutionSessionId) -> Result<(), ExecutionDomainError> {
        self.registry.with_session_mut(id, |session| {
            let prev = session.lifecycle.clone();
            session.stop()?;
            let curr = session.lifecycle.clone();

            let now_us = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64;

            self.event_bus.publish(super::events::ExecutionEvent::LifecycleChanged {
                session_id: id.clone(),
                previous: prev,
                current: curr,
                timestamp_us: now_us,
            });

            Ok(())
        })
    }

    /// Despacha un tick k sobre la sesión especificada.
    /// Valida explícitamente que la sesión esté en estado Running.
    pub fn tick(
        &self,
        id: &ExecutionSessionId,
        context: TickContext,
        eval_fn: impl FnOnce(&ObservationBundle, &RobotState) -> (Decision, Action),
    ) -> Result<TickResult, ExecutionDomainError> {
        let sampled_at_us = context.observations.captured_at_us;
        let res = self.registry.with_session_mut(id, |session| {
            if session.lifecycle != LifecycleState::Running {
                return Err(ExecutionDomainError::NotRunning(session.lifecycle.clone()));
            }
            let res = session.evaluate_tick(context, eval_fn)?;
            Ok(res)
        })?;

        let temporal = super::events::TemporalInvariants::current(sampled_at_us);
        self.event_bus.publish(super::events::ExecutionEvent::TickEvaluated {
            session_id: id.clone(),
            result: res.clone(),
            temporal,
        });

        Ok(res)
    }

    /// Despacha un tick k interactuando con un ExecutionRunner para la adquisición de estado y ejecución de la acción.
    pub fn tick_with_runner(
        &self,
        id: &ExecutionSessionId,
        runner: &mut impl super::runner::ExecutionRunner,
        eval_fn: impl FnOnce(&ObservationBundle, &RobotState) -> (Decision, Action),
    ) -> Result<TickResult, ExecutionDomainError> {
        let context = runner.acquire();
        let sampled_at_us = context.observations.captured_at_us;

        let mut result = self.registry.with_session_mut(id, |session| {
            if session.lifecycle != LifecycleState::Running {
                return Err(ExecutionDomainError::NotRunning(session.lifecycle.clone()));
            }
            let res = session.evaluate_tick(context, eval_fn)?;
            Ok(res)
        })?;

        if result.outcome != TickOutcome::SessionCompleted {
            let outcome = runner.act(&result.action);
            result.outcome = outcome;
        }

        let temporal = super::events::TemporalInvariants::current(sampled_at_us);
        self.event_bus.publish(super::events::ExecutionEvent::TickEvaluated {
            session_id: id.clone(),
            result: result.clone(),
            temporal,
        });

        Ok(result)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Channel Access Evaluation (6.5)
// ═══════════════════════════════════════════════════════════════════════════

/// Extract a scalar value from a ChannelObservation.
pub fn extract_scalar(obs: &ChannelObservation) -> f64 {
    match obs.value {
        thalos_core::device::ChannelValue::Scalar(v) => v,
        thalos_core::device::ChannelValue::Integer(v) => v as f64,
        thalos_core::device::ChannelValue::Boolean(v) => if v { 1.0 } else { 0.0 },
    }
}

/// Evaluate a channel access against an ObservationBundle.
///
/// This bridges the DSL compiler's `SemanticExpr::ChannelAccess { module, channel }`
/// with the runtime observation data. The module parameter is currently unused
/// (channel IDs are globally unique in the observation bundle) but preserved
/// for future namespace scoping.
pub fn eval_channel_access(
    _module: &str,
    channel: &str,
    bundle: &ObservationBundle,
) -> Result<f64, ChannelAccessError> {
    bundle.observations
        .get(channel)
        .map(|obs| extract_scalar(obs))
        .ok_or_else(|| ChannelAccessError::ChannelNotFound(channel.to_string()))
}

/// Error type for channel access evaluation.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ChannelAccessError {
    #[error("Channel not found: {0}")]
    ChannelNotFound(String),

    #[error("Channel value not available: {0}")]
    ValueUnavailable(String),
}

// ═══════════════════════════════════════════════════════════════════════════
// Derived Signal Evaluation (6.6)
// ═══════════════════════════════════════════════════════════════════════════

/// Resolve a signal reference to a scalar value within an ObservationBundle.
///
/// If the signal is found in the bundle, its value is extracted.
/// If not found, returns 0.0 (allows derived signals to operate on
/// partially-available data during startup or degraded states).
fn resolve_signal(signal_id: &str, bundle: &ObservationBundle) -> f64 {
    bundle.observations
        .get(signal_id)
        .map(|obs| extract_scalar(obs))
        .unwrap_or(0.0)
}

/// Evaluate a derived signal expression against an ObservationBundle.
///
/// Pure derived signals can be evaluated by Interconnection because they
/// only transform signal data without requiring domain semantics.
pub fn eval_derived_signal(
    derived: &thalos_core::device::DerivedSignal,
    bundle: &ObservationBundle,
) -> Result<f64, ChannelAccessError> {
    match &derived.expression {
        thalos_core::device::SignalExpression::Channel(signal_id) => {
            Ok(resolve_signal(signal_id, bundle))
        }
        thalos_core::device::SignalExpression::Add(a, b) => {
            Ok(resolve_signal(a, bundle) + resolve_signal(b, bundle))
        }
        thalos_core::device::SignalExpression::Subtract(a, b) => {
            Ok(resolve_signal(a, bundle) - resolve_signal(b, bundle))
        }
        thalos_core::device::SignalExpression::Multiply(a, b) => {
            Ok(resolve_signal(a, bundle) * resolve_signal(b, bundle))
        }
        thalos_core::device::SignalExpression::Divide(a, b) => {
            let divisor = resolve_signal(b, bundle);
            if divisor == 0.0 {
                Err(ChannelAccessError::ValueUnavailable(
                    format!("Division by zero in derived signal {}", derived.signal_id)
                ))
            } else {
                Ok(resolve_signal(a, bundle) / divisor)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lifecycle_transitions() {
        let mut session = ExecutionSession::new("weld_main", ExecutionConfiguration::default());
        assert_eq!(session.lifecycle, LifecycleState::Created);

        assert!(session.pause().is_err());
        assert!(session.initialize().is_ok());
        assert_eq!(session.lifecycle, LifecycleState::Initializing);

        assert!(session.start().is_ok());
        assert_eq!(session.lifecycle, LifecycleState::Running);

        assert!(session.pause().is_ok());
        assert_eq!(session.lifecycle, LifecycleState::Paused);

        assert!(session.start().is_ok());
        assert_eq!(session.lifecycle, LifecycleState::Running);

        assert!(session.complete().is_ok());
        assert_eq!(session.lifecycle, LifecycleState::Completed);
    }

    #[test]
    fn test_cycle_completion_cardinality() {
        let mut session_once = ExecutionSession::new("test", ExecutionConfiguration {
            cardinality: Cardinality::Once,
            ..Default::default()
        });
        assert!(!session_once.complete_program_cycle());

        let mut session_counted = ExecutionSession::new("test", ExecutionConfiguration {
            cardinality: Cardinality::Counted(3),
            ..Default::default()
        });
        assert!(session_counted.complete_program_cycle());
        assert!(session_counted.complete_program_cycle());
        assert!(!session_counted.complete_program_cycle());
    }

    #[test]
    fn test_deterministic_tick_decision_branching() {
        let mut session = ExecutionSession::new("reactive_program", ExecutionConfiguration {
            reactivity: Reactivity::Reactive,
            ..Default::default()
        });
        session.initialize().unwrap();
        session.start().unwrap();

        let eval_logic = |obs: &ObservationBundle, _rob: &RobotState| {
            let target_x = obs.observations.get("camera.target_x")
                .map(|o| match o.value {
                    thalos_core::device::ChannelValue::Scalar(v) => v,
                    thalos_core::device::ChannelValue::Integer(v) => v as f64,
                    thalos_core::device::ChannelValue::Boolean(v) => if v { 1.0 } else { 0.0 },
                })
                .unwrap_or(0.0);
            if target_x > 80.0 {
                (
                    Decision::MotionAction {
                        motion_type: "movej".to_string(),
                        target_name: "target_high".to_string(),
                    },
                    Action::DispatchMotion {
                        kind: "movej".to_string(),
                        target: "target_high".to_string(),
                    },
                )
            } else {
                (
                    Decision::MotionAction {
                        motion_type: "movej".to_string(),
                        target_name: "target_low".to_string(),
                    },
                    Action::DispatchMotion {
                        kind: "movej".to_string(),
                        target: "target_low".to_string(),
                    },
                )
            }
        };

        let mut obs1 = ObservationBundle::default();
        obs1.observations.insert("camera.target_x".to_string(), ChannelObservation {
            channel_id: "camera.target_x".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(100.0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        let ctx1 = TickContext::new(obs1, RobotState::default(), ExpectedState::default());
        let res1 = session.evaluate_tick(ctx1, eval_logic).unwrap();
        assert_eq!(res1.tick.index, 1);
        assert_eq!(
            res1.decision,
            Decision::MotionAction {
                motion_type: "movej".to_string(),
                target_name: "target_high".to_string()
            }
        );

        let mut obs2 = ObservationBundle::default();
        obs2.observations.insert("camera.target_x".to_string(), ChannelObservation {
            channel_id: "camera.target_x".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(50.0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        let ctx2 = TickContext::new(obs2, RobotState::default(), ExpectedState::default());
        let res2 = session.evaluate_tick(ctx2, eval_logic).unwrap();
        assert_eq!(res2.tick.index, 2);
        assert_eq!(
            res2.decision,
            Decision::MotionAction {
                motion_type: "movej".to_string(),
                target_name: "target_low".to_string()
            }
        );
    }

    #[test]
    fn test_atomic_single_snapshot_multi_channel_latching() {
        let mut session = ExecutionSession::new("multi_channel_test", ExecutionConfiguration::default());
        session.initialize().unwrap();
        session.start().unwrap();

        let mut obs = ObservationBundle::default();
        obs.observations.insert("camera.target_x".to_string(), ChannelObservation {
            channel_id: "camera.target_x".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(100.0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        obs.observations.insert("camera.target_y".to_string(), ChannelObservation {
            channel_id: "camera.target_y".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(50.0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        let ctx = TickContext::new(obs, RobotState::default(), ExpectedState::default());

        let res = session.evaluate_tick(ctx, |obs, _rob| {
            let x = obs.observations.get("camera.target_x")
                .map(|o| match o.value {
                    thalos_core::device::ChannelValue::Scalar(v) => v,
                    _ => 0.0,
                })
                .unwrap_or(0.0);
            let y = obs.observations.get("camera.target_y")
                .map(|o| match o.value {
                    thalos_core::device::ChannelValue::Scalar(v) => v,
                    _ => 0.0,
                })
                .unwrap_or(0.0);
            assert_eq!(x, 100.0);
            assert_eq!(y, 50.0);
            (Decision::Continue, Action::None)
        }).unwrap();

        let x = res.tick.observations.observations.get("camera.target_x")
            .map(|o| match o.value {
                thalos_core::device::ChannelValue::Scalar(v) => v,
                _ => 0.0,
            })
            .unwrap_or(0.0);
        let y = res.tick.observations.observations.get("camera.target_y")
            .map(|o| match o.value {
                thalos_core::device::ChannelValue::Scalar(v) => v,
                _ => 0.0,
            })
            .unwrap_or(0.0);
        assert_eq!(x, 100.0);
        assert_eq!(y, 50.0);
    }

    #[test]
    fn test_termination_condition_with_scalar_channel() {
        let mut session = ExecutionSession::new("term_test", ExecutionConfiguration {
            termination: TerminationPolicy::Condition("safety_stop".to_string()),
            ..Default::default()
        });
        session.initialize().unwrap();
        session.start().unwrap();

        // safety_stop = 0.0 → should NOT terminate
        let mut obs = ObservationBundle::default();
        obs.observations.insert("safety_stop".to_string(), ChannelObservation {
            channel_id: "safety_stop".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(0.0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        let ctx = TickContext::new(obs, RobotState::default(), ExpectedState::default());
        let res = session.evaluate_tick(ctx, |_obs, _rob| (Decision::Continue, Action::None)).unwrap();
        assert_eq!(res.outcome, TickOutcome::Success);
        assert_eq!(res.decision, Decision::Continue);

        // safety_stop = 1.0 → should terminate
        let mut obs2 = ObservationBundle::default();
        obs2.observations.insert("safety_stop".to_string(), ChannelObservation {
            channel_id: "safety_stop".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(1.0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        let ctx2 = TickContext::new(obs2, RobotState::default(), ExpectedState::default());
        let res2 = session.evaluate_tick(ctx2, |_obs, _rob| (Decision::Continue, Action::None)).unwrap();
        assert_eq!(res2.outcome, TickOutcome::SessionCompleted);
        assert!(matches!(res2.decision, Decision::TerminateSession { .. }));
    }

    #[test]
    fn test_termination_condition_with_boolean_channel() {
        let mut session = ExecutionSession::new("bool_term", ExecutionConfiguration {
            termination: TerminationPolicy::Condition("e_stop".to_string()),
            ..Default::default()
        });
        session.initialize().unwrap();
        session.start().unwrap();

        // Boolean false → should NOT terminate
        let mut obs = ObservationBundle::default();
        obs.observations.insert("e_stop".to_string(), ChannelObservation {
            channel_id: "e_stop".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Boolean(false),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        let ctx = TickContext::new(obs, RobotState::default(), ExpectedState::default());
        let res = session.evaluate_tick(ctx, |_obs, _rob| (Decision::Continue, Action::None)).unwrap();
        assert_eq!(res.outcome, TickOutcome::Success);

        // Boolean true → should terminate
        let mut obs2 = ObservationBundle::default();
        obs2.observations.insert("e_stop".to_string(), ChannelObservation {
            channel_id: "e_stop".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Boolean(true),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        let ctx2 = TickContext::new(obs2, RobotState::default(), ExpectedState::default());
        let res2 = session.evaluate_tick(ctx2, |_obs, _rob| (Decision::Continue, Action::None)).unwrap();
        assert_eq!(res2.outcome, TickOutcome::SessionCompleted);
    }

    #[test]
    fn test_termination_condition_with_integer_channel() {
        let mut session = ExecutionSession::new("int_term", ExecutionConfiguration {
            termination: TerminationPolicy::Condition("error_code".to_string()),
            ..Default::default()
        });
        session.initialize().unwrap();
        session.start().unwrap();

        // error_code = 0 → should NOT terminate
        let mut obs = ObservationBundle::default();
        obs.observations.insert("error_code".to_string(), ChannelObservation {
            channel_id: "error_code".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Integer(0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        let ctx = TickContext::new(obs, RobotState::default(), ExpectedState::default());
        let res = session.evaluate_tick(ctx, |_obs, _rob| (Decision::Continue, Action::None)).unwrap();
        assert_eq!(res.outcome, TickOutcome::Success);

        // error_code = 5 → should terminate
        let mut obs2 = ObservationBundle::default();
        obs2.observations.insert("error_code".to_string(), ChannelObservation {
            channel_id: "error_code".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Integer(5),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        let ctx2 = TickContext::new(obs2, RobotState::default(), ExpectedState::default());
        let res2 = session.evaluate_tick(ctx2, |_obs, _rob| (Decision::Continue, Action::None)).unwrap();
        assert_eq!(res2.outcome, TickOutcome::SessionCompleted);
    }

    #[test]
    fn test_termination_condition_missing_channel() {
        let mut session = ExecutionSession::new("missing_term", ExecutionConfiguration {
            termination: TerminationPolicy::Condition("nonexistent_channel".to_string()),
            ..Default::default()
        });
        session.initialize().unwrap();
        session.start().unwrap();

        // Channel doesn't exist in observation bundle → should NOT terminate
        let obs = ObservationBundle::default(); // empty
        let ctx = TickContext::new(obs, RobotState::default(), ExpectedState::default());
        let res = session.evaluate_tick(ctx, |_obs, _rob| (Decision::Continue, Action::None)).unwrap();
        assert_eq!(res.outcome, TickOutcome::Success);
        assert_eq!(res.decision, Decision::Continue);
    }

    #[test]
    fn test_termination_condition_with_degraded_quality() {
        let mut session = ExecutionSession::new("degraded_term", ExecutionConfiguration {
            termination: TerminationPolicy::Condition("sensor".to_string()),
            ..Default::default()
        });
        session.initialize().unwrap();
        session.start().unwrap();

        // Degraded quality but value > 0 → should still terminate
        // (quality is metadata; the value interpretation is the consumer's responsibility)
        let mut obs = ObservationBundle::default();
        obs.observations.insert("sensor".to_string(), ChannelObservation {
            channel_id: "sensor".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(1.0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Degraded,
        });
        let ctx = TickContext::new(obs, RobotState::default(), ExpectedState::default());
        let res = session.evaluate_tick(ctx, |_obs, _rob| (Decision::Continue, Action::None)).unwrap();
        assert_eq!(res.outcome, TickOutcome::SessionCompleted);
    }

    #[test]
    fn test_eval_fn_receives_observations_not_acquisition() {
        let mut session = ExecutionSession::new("obs_test", ExecutionConfiguration::default());
        session.initialize().unwrap();
        session.start().unwrap();

        let mut obs = ObservationBundle::default();
        obs.observations.insert("joint_1.position".to_string(), ChannelObservation {
            channel_id: "joint_1.position".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(1.57),
            unit: Some("rad".to_string()),
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        let ctx = TickContext::new(obs, RobotState::default(), ExpectedState::default());

        let res = session.evaluate_tick(ctx, |bundle, _rob| {
            let val = bundle.observations.get("joint_1.position")
                .map(|o| match o.value {
                    thalos_core::device::ChannelValue::Scalar(v) => v,
                    _ => 0.0,
                })
                .unwrap_or(0.0);
            assert!((val - 1.57).abs() < 0.001);
            (Decision::Continue, Action::None)
        }).unwrap();
        assert_eq!(res.outcome, TickOutcome::Success);
    }

    #[test]
    fn test_eval_channel_access() {
        let mut bundle = ObservationBundle::default();
        bundle.observations.insert("temperature".to_string(), ChannelObservation {
            channel_id: "temperature".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(72.5),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });

        let val = eval_channel_access("device", "temperature", &bundle).unwrap();
        assert!((val - 72.5).abs() < 0.001);

        // Missing channel returns error
        assert!(eval_channel_access("device", "missing", &bundle).is_err());
    }

    #[test]
    fn test_eval_channel_access_integer() {
        let mut bundle = ObservationBundle::default();
        bundle.observations.insert("error_code".to_string(), ChannelObservation {
            channel_id: "error_code".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Integer(42),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });

        let val = eval_channel_access("device", "error_code", &bundle).unwrap();
        assert!((val - 42.0).abs() < 0.001);
    }

    #[test]
    fn test_eval_channel_access_boolean() {
        let mut bundle = ObservationBundle::default();
        bundle.observations.insert("safety_stop".to_string(), ChannelObservation {
            channel_id: "safety_stop".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Boolean(true),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });

        let val = eval_channel_access("device", "safety_stop", &bundle).unwrap();
        assert!((val - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_eval_derived_signal_channel() {
        let mut bundle = ObservationBundle::default();
        bundle.observations.insert("temperature".to_string(), ChannelObservation {
            channel_id: "temperature".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(72.5),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });

        let derived = thalos_core::device::DerivedSignal::new(
            "temp_copy",
            thalos_core::device::SignalExpression::Channel("temperature".to_string()),
        );

        let val = eval_derived_signal(&derived, &bundle).unwrap();
        assert!((val - 72.5).abs() < 0.001);
    }

    #[test]
    fn test_eval_derived_signal_add() {
        let mut bundle = ObservationBundle::default();
        bundle.observations.insert("a".to_string(), ChannelObservation {
            channel_id: "a".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(10.0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        bundle.observations.insert("b".to_string(), ChannelObservation {
            channel_id: "b".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(20.0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });

        let derived = thalos_core::device::DerivedSignal::new(
            "sum",
            thalos_core::device::SignalExpression::Add("a".to_string(), "b".to_string()),
        );

        let val = eval_derived_signal(&derived, &bundle).unwrap();
        assert!((val - 30.0).abs() < 0.001);
    }

    #[test]
    fn test_eval_derived_signal_subtract() {
        let mut bundle = ObservationBundle::default();
        bundle.observations.insert("target".to_string(), ChannelObservation {
            channel_id: "target".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(100.0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });
        bundle.observations.insert("actual".to_string(), ChannelObservation {
            channel_id: "actual".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: thalos_core::device::ChannelValue::Scalar(95.0),
            unit: None,
            quality: thalos_core::device::SignalQuality::Nominal,
        });

        let derived = thalos_core::device::DerivedSignal::new(
            "error",
            thalos_core::device::SignalExpression::Subtract("target".to_string(), "actual".to_string()),
        );

        let val = eval_derived_signal(&derived, &bundle).unwrap();
        assert!((val - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_eval_derived_signal_divide_by_zero() {
        let bundle = ObservationBundle::default();

        let derived = thalos_core::device::DerivedSignal::new(
            "ratio",
            thalos_core::device::SignalExpression::Divide("a".to_string(), "b".to_string()),
        );

        let result = eval_derived_signal(&derived, &bundle);
        assert!(result.is_err());
    }

    #[test]
    fn test_eval_derived_signal_missing_inputs_use_zero() {
        let bundle = ObservationBundle::default();

        let derived = thalos_core::device::DerivedSignal::new(
            "missing_sum",
            thalos_core::device::SignalExpression::Add("x".to_string(), "y".to_string()),
        );

        let val = eval_derived_signal(&derived, &bundle).unwrap();
        assert!((val - 0.0).abs() < 0.001);
    }
}
