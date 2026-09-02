use std::collections::HashMap;
use std::sync::Mutex;
use std::time::Duration;
use serde::{Deserialize, Serialize};

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

/// Snapshot congelado de telemetría de canales en un tick k.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AcquisitionSnapshot {
    pub timestamp_us: u64,
    pub channels: HashMap<String, f64>,
}

/// Contexto de observación agrupado para alimentar la evaluación del tick k.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TickContext {
    pub acquisition: AcquisitionSnapshot,
    pub robot: RobotState,
    pub expected: ExpectedState,
}

impl TickContext {
    pub fn new(acquisition: AcquisitionSnapshot, robot: RobotState, expected: ExpectedState) -> Self {
        Self {
            acquisition,
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
    pub acquisition: AcquisitionSnapshot,
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
    pub acquisition: AcquisitionSnapshot,
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
        eval_fn: impl FnOnce(&AcquisitionSnapshot, &RobotState) -> (Decision, Action),
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
            acquisition: context.acquisition.clone(),
            robot: context.robot.clone(),
            expected: context.expected.clone(),
        };

        // 1. Actualizar estado latched en la sesión
        self.state.acquisition = context.acquisition;
        self.state.robot = context.robot;
        self.state.expected = context.expected;

        // 2. Evaluación de condición de terminación previa a la acción
        let condition_met = match self.configuration.termination {
            TerminationPolicy::Condition(ref cond_channel) => {
                if let Some(&val) = tick.acquisition.channels.get(cond_channel) {
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
        let (decision, action) = eval_fn(&tick.acquisition, &tick.robot);

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
        eval_fn: impl FnOnce(&AcquisitionSnapshot, &RobotState) -> (Decision, Action),
    ) -> Result<TickResult, ExecutionDomainError> {
        let sampled_at_us = context.acquisition.timestamp_us;
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
        eval_fn: impl FnOnce(&AcquisitionSnapshot, &RobotState) -> (Decision, Action),
    ) -> Result<TickResult, ExecutionDomainError> {
        let context = runner.acquire();
        let sampled_at_us = context.acquisition.timestamp_us;

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

        let eval_logic = |acq: &AcquisitionSnapshot, _rob: &RobotState| {
            let target_x = acq.channels.get("camera.target_x").copied().unwrap_or(0.0);
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

        let mut acq1 = AcquisitionSnapshot::default();
        acq1.channels.insert("camera.target_x".to_string(), 100.0);
        let ctx1 = TickContext::new(acq1, RobotState::default(), ExpectedState::default());
        let res1 = session.evaluate_tick(ctx1, eval_logic).unwrap();
        assert_eq!(res1.tick.index, 1);
        assert_eq!(
            res1.decision,
            Decision::MotionAction {
                motion_type: "movej".to_string(),
                target_name: "target_high".to_string()
            }
        );

        let mut acq2 = AcquisitionSnapshot::default();
        acq2.channels.insert("camera.target_x".to_string(), 50.0);
        let ctx2 = TickContext::new(acq2, RobotState::default(), ExpectedState::default());
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

        let mut acq = AcquisitionSnapshot::default();
        acq.channels.insert("camera.target_x".to_string(), 100.0);
        acq.channels.insert("camera.target_y".to_string(), 50.0);
        let ctx = TickContext::new(acq, RobotState::default(), ExpectedState::default());

        let res = session.evaluate_tick(ctx, |acq, _rob| {
            let x = acq.channels.get("camera.target_x").copied().unwrap_or(0.0);
            let y = acq.channels.get("camera.target_y").copied().unwrap_or(0.0);
            assert_eq!(x, 100.0);
            assert_eq!(y, 50.0);
            (Decision::Continue, Action::None)
        }).unwrap();

        assert_eq!(res.tick.acquisition.channels.get("camera.target_x"), Some(&100.0));
        assert_eq!(res.tick.acquisition.channels.get("camera.target_y"), Some(&50.0));
    }
}
