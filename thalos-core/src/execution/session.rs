use std::collections::HashMap;
use std::time::Duration;

use serde::{Deserialize, Serialize};

use crate::device::{ChannelObservation, ChannelValue, DerivedSignal, SignalExpression};
use crate::ids::ExecutionSessionId;
use crate::kinematics::{SpatialState, TcpPose};

/// Formal operational state machine for active execution sessions.
///
/// Lifecycle authority of the [`ExecutionSession`] aggregate. This is the
/// authoritative session state; `SessionStatus` is only its UI projection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExecutionSessionState {
    Created,
    Reserved,
    Dispatched,
    Running,
    Paused,
    Completed,
    Cancelled,
    Failed,
}

impl ExecutionSessionState {
    pub fn is_terminal(&self) -> bool {
        matches!(self, Self::Completed | Self::Cancelled | Self::Failed)
    }

    pub fn is_active(&self) -> bool {
        matches!(self, Self::Running | Self::Paused)
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

/// Estado de las variables y puntero de programa DSL.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ProgramState {
    pub program_counter: usize,
    pub local_vars: HashMap<String, String>,
}

/// Estado físico o simulado actual del robot.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RobotSample {
    pub joints: Vec<f64>,
    pub velocities: Vec<f64>,
}

/// Estado estimado ideal/modelo digital del robot.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ExpectedState {
    pub simulated_joints: Vec<f64>,
}

/// Estado runtime producido por un runner para el tick k.
///
/// `robot` es el estado observado/actual, `expected` el modelo digital, y `tcp`
/// la pose TCP derivada por la autoridad cinemática compartida (opcional: no
/// toda ejecución tiene un modelo cinemático resoluble).
///
/// `spatial` es el estado espacial NEUTRAL del modelo cinemático, derivado de
/// la MISMA evaluación FK que `tcp`. No contiene conceptos de rendering
/// (ids visuales, links, escala, meshes); la proyección visual es
/// responsabilidad de la capa visual. Ver
/// `docs/system/architecture/spatial-state-contract.md`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RuntimeState {
    pub robot: RobotSample,
    pub expected: ExpectedState,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tcp: Option<TcpPose>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spatial: Option<SpatialState>,
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
    pub robot: RobotSample,
    pub expected: ExpectedState,
    /// Evaluation timestamp (nanoseconds) supplied by the caller. The engine
    /// does **not** read a clock (ADR-020): the runtime stamps this before
    /// dispatching the tick. `0` means "unstamped".
    #[serde(default)]
    pub timestamp_ns: u64,
}

impl TickContext {
    pub fn new(observations: ObservationBundle, robot: RobotSample, expected: ExpectedState) -> Self {
        Self {
            observations,
            robot,
            expected,
            timestamp_ns: 0,
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
    pub robot: RobotSample,
    pub expected: ExpectedState,
    /// World TCP pose produced by the runtime's kinematic authority, when
    /// available. Part of the tick's observable state — not computed by the UI.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tcp: Option<TcpPose>,
    /// Neutral spatial state derived from the SAME FK evaluation that produces
    /// `tcp`. Transient tick observation — not latched into `SessionState`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spatial: Option<SpatialState>,
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
    /// Command a motion to a resolved joint configuration.
    ///
    /// `joints` is the COMMANDED configuration the runner must produce — it is
    /// the runtime state, not a reference to the plan. `kind`/`target` remain
    /// the plan reference (e.g. `movej` / `wp3`) for traceability.
    DispatchMotion {
        kind: String,
        target: String,
        joints: Vec<f64>,
    },
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
    pub robot: RobotSample,
    pub expected: ExpectedState,
    pub observations: ObservationBundle,
    pub cycle: CycleState,
}

/// Error retornado al intentar transiciones de ciclo de vida inválidas.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("Transición de ciclo de vida inválida desde {from:?} hasta {to_action}")]
pub struct InvalidLifecycleTransition {
    pub from: ExecutionSessionState,
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
    NotRunning(ExecutionSessionState),
}

/// Entidad de dominio ExecutionSession.
///
/// Lifecycle authority: `ExecutionSessionState`.
/// `TrackingState` in HardwareExecutor is orthogonal and does NOT compete.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutionSession {
    pub id: ExecutionSessionId,
    pub program_id: String,
    pub station_id: Option<String>,
    pub robotics_module_id: Option<String>,
    /// Program revision this session was prepared against (see ProgramRecord).
    pub program_revision: Option<u64>,
    /// SHA-256 fingerprint of the source that revision had at preparation time.
    pub source_fingerprint: Option<String>,
    pub configuration: ExecutionConfiguration,
    pub lifecycle: ExecutionSessionState,
    pub state: SessionState,
    pub history: Vec<ExecutionSessionState>,
}
impl ExecutionSession {
    /// Create a session with a caller-provided id — so a durable record can be
    /// created for the session BEFORE any event is published. Identity minting
    /// (the `exec-<uuid>` convention) is an application concern.
    pub fn new_with_id(
        id: ExecutionSessionId,
        program_id: impl Into<String>,
        configuration: ExecutionConfiguration,
    ) -> Self {
        let initial_state = ExecutionSessionState::Created;
        Self {
            id,
            program_id: program_id.into(),
            station_id: None,
            robotics_module_id: None,
            program_revision: None,
            source_fingerprint: None,
            configuration,
            lifecycle: initial_state,
            state: SessionState::default(),
            history: vec![initial_state],
        }
    }

    /// True when the captured program snapshot (revision + fingerprint) no
    /// longer matches the current persisted program — e.g. the program was
    /// edited and saved after this session was prepared.
    pub fn is_stale_for(&self, current_revision: u64, current_fingerprint: &str) -> bool {
        program_snapshot_is_stale(
            self.program_revision,
            self.source_fingerprint.as_deref(),
            current_revision,
            current_fingerprint,
        )
    }

    fn record_transition(&mut self, next: ExecutionSessionState) {
        self.lifecycle = next;
        self.history.push(next);
    }

    /// Transición: Created -> Reserved
    pub fn initialize(&mut self) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            ExecutionSessionState::Created => {
                self.record_transition(ExecutionSessionState::Reserved);
                Ok(())
            }
            _ => Err(InvalidLifecycleTransition {
                from: self.lifecycle,
                to_action: "initialize",
            }),
        }
    }

    /// Transición: Reserved/Dispatched/Paused -> Running
    pub fn start(&mut self) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            ExecutionSessionState::Reserved
            | ExecutionSessionState::Dispatched
            | ExecutionSessionState::Paused => {
                self.record_transition(ExecutionSessionState::Running);
                Ok(())
            }
            _ => Err(InvalidLifecycleTransition {
                from: self.lifecycle,
                to_action: "start",
            }),
        }
    }

    /// Transición: Running -> Paused
    pub fn pause(&mut self) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            ExecutionSessionState::Running => {
                self.record_transition(ExecutionSessionState::Paused);
                Ok(())
            }
            _ => Err(InvalidLifecycleTransition {
                from: self.lifecycle,
                to_action: "pause",
            }),
        }
    }

    /// Transición: Running/Paused -> Cancelled
    pub fn stop(&mut self) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            ExecutionSessionState::Running | ExecutionSessionState::Paused => {
                self.record_transition(ExecutionSessionState::Cancelled);
                Ok(())
            }
            _ => Err(InvalidLifecycleTransition {
                from: self.lifecycle,
                to_action: "stop",
            }),
        }
    }

    /// Transición: Cualquier estado activo -> Failed
    pub fn fault(&mut self, _reason: impl Into<String>) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            ExecutionSessionState::Completed | ExecutionSessionState::Cancelled => {
                Err(InvalidLifecycleTransition {
                    from: self.lifecycle,
                    to_action: "fault",
                })
            }
            _ => {
                // Store fault reason in session state for observability
                self.record_transition(ExecutionSessionState::Failed);
                Ok(())
            }
        }
    }

    /// Transición: Running -> Completed
    pub fn complete(&mut self) -> Result<(), InvalidLifecycleTransition> {
        match self.lifecycle {
            ExecutionSessionState::Running => {
                self.record_transition(ExecutionSessionState::Completed);
                Ok(())
            }
            _ => Err(InvalidLifecycleTransition {
                from: self.lifecycle,
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
        eval_fn: impl FnOnce(&ObservationBundle, &RobotSample) -> (Decision, Action),
    ) -> Result<TickResult, InvalidLifecycleTransition> {
        if self.lifecycle != ExecutionSessionState::Running {
            return Err(InvalidLifecycleTransition {
                from: self.lifecycle,
                to_action: "evaluate_tick",
            });
        }

        self.advance_tick();
        let tick_index = self.state.cycle.tick_count;

        // The clock is a runtime mechanism (ADR-020): the caller stamps the
        // context; the engine only reads it.
        let timestamp_ns = context.timestamp_ns;

        let tick = ControlTick {
            index: tick_index,
            timestamp_ns,
            observations: context.observations.clone(),
            robot: context.robot.clone(),
            expected: context.expected.clone(),
            // TCP and spatial are produced AFTER `act` (see `tick_with_runner`).
            tcp: None,
            spatial: None,
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
                        ChannelValue::Scalar(v) => v,
                        ChannelValue::Integer(v) => v as f64,
                        ChannelValue::Boolean(v) => {
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
// ═══════════════════════════════════════════════════════════════════════════
// Channel Access Evaluation (6.5)
// ═══════════════════════════════════════════════════════════════════════════

/// Extract a scalar value from a ChannelObservation.
pub fn extract_scalar(obs: &ChannelObservation) -> f64 {
    match obs.value {
        ChannelValue::Scalar(v) => v,
        ChannelValue::Integer(v) => v as f64,
        ChannelValue::Boolean(v) => if v { 1.0 } else { 0.0 },
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
        .map(extract_scalar)
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
        .map(extract_scalar)
        .unwrap_or(0.0)
}

/// Evaluate a derived signal expression against an ObservationBundle.
///
/// Pure derived signals can be evaluated by Interconnection because they
/// only transform signal data without requiring domain semantics.
pub fn eval_derived_signal(
    derived: &DerivedSignal,
    bundle: &ObservationBundle,
) -> Result<f64, ChannelAccessError> {
    match &derived.expression {
        SignalExpression::Channel(signal_id) => {
            Ok(resolve_signal(signal_id, bundle))
        }
        SignalExpression::Add(a, b) => {
            Ok(resolve_signal(a, bundle) + resolve_signal(b, bundle))
        }
        SignalExpression::Subtract(a, b) => {
            Ok(resolve_signal(a, bundle) - resolve_signal(b, bundle))
        }
        SignalExpression::Multiply(a, b) => {
            Ok(resolve_signal(a, bundle) * resolve_signal(b, bundle))
        }
        SignalExpression::Divide(a, b) => {
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
/// Canonical SHA-256 fingerprint (hex) of a program source.
///
/// Domain policy (single source of truth for "does this source correspond to
/// this program revision?"), not persistence. Moved here from the ports layer
/// so runtime models do not depend on ports.
pub fn source_fingerprint(source: &str) -> String {
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    hasher.update(source.as_bytes());
    format!("{:x}", hasher.finalize())
}

/// True when an execution snapshot (revision + optional fingerprint) no longer
/// matches the current persisted program, i.e. the program changed since the
/// execution was prepared. A missing fingerprint is "unknown" and does not by
/// itself mark the snapshot stale.
pub fn program_snapshot_is_stale(
    snapshot_revision: Option<u64>,
    snapshot_fingerprint: Option<&str>,
    current_revision: u64,
    current_fingerprint: &str,
) -> bool {
    if let Some(rev) = snapshot_revision
        && rev != current_revision
    {
        return true;
    }
    if let Some(fp) = snapshot_fingerprint
        && fp != current_fingerprint
    {
        return true;
    }
    false
}

#[cfg(test)]
mod staleness_tests {
    use super::*;

    #[test]
    fn fingerprint_is_deterministic_and_source_sensitive() {
        let a = source_fingerprint("fn main() { movej(PARK) }");
        assert_eq!(a, source_fingerprint("fn main() { movej(PARK) }"));
        assert_ne!(a, source_fingerprint("fn main() { movel(PARK) }"));
    }

    #[test]
    fn same_revision_and_fingerprint_is_not_stale() {
        let fp = source_fingerprint("src");
        assert!(!program_snapshot_is_stale(Some(1), Some(&fp), 1, &fp));
    }

    #[test]
    fn revision_change_is_stale() {
        let fp = source_fingerprint("src");
        assert!(program_snapshot_is_stale(Some(1), Some(&fp), 2, &fp));
    }

    #[test]
    fn fingerprint_mismatch_at_same_revision_is_stale() {
        let fp1 = source_fingerprint("src-1");
        let fp2 = source_fingerprint("src-2");
        assert!(program_snapshot_is_stale(Some(1), Some(&fp1), 1, &fp2));
    }

    #[test]
    fn unknown_snapshot_fields_are_not_stale_by_themselves() {
        let fp = source_fingerprint("src");
        assert!(!program_snapshot_is_stale(None, None, 7, &fp));
    }
}
