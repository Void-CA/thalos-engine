use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use thalos_core::device::{ChannelObservation, SignalQuality};
use super::domain::{
    Action, ExpectedState, ObservationBundle, RobotState, TickContext, TickOutcome,
};

/// Abstracción del entorno de ejecución (Simulación, Hardware Físico, etc.).
///
/// El runner es un proveedor de estado (`acquire`) y un actuador de acciones (`act`).
/// No toma decisiones ni gestiona el estado de la sesión: esa autoridad es de `ExecutionSession`.
pub trait ExecutionRunner: Send + Sync {
    /// Adquiere el contexto de observación inmutable para el tick k.
    fn acquire(&mut self) -> TickContext;

    /// Ejecuta la acción determinada por la decisión del tick en el entorno correspondiente.
    fn act(&mut self, action: &Action) -> TickOutcome;
}

/// Provides observation data to the execution domain.
///
/// This trait is the contractual boundary between Interconnection (provider)
/// and Execution (consumer). Execution defines WHAT it needs; Interconnection
/// implements HOW to provide it.
pub trait ObservationProvider: Send + Sync {
    fn snapshot(&self) -> ObservationBundle;
}

/// Issues commands through the interconnection layer.
///
/// Execution defines WHAT domain commands to issue; Interconnection implements
/// HOW to transport them to the physical equipment.
pub trait CommandProvider: Send + Sync {
    fn dispatch(
        &mut self,
        command: &RobotCommand,
    ) -> Result<(), CommandError>;
}

/// Provides observation of the robot state (joint positions, velocities).
pub trait RobotObservationProvider: Send + Sync {
    fn observe(&self) -> RobotState;
}

/// A domain-level command issued by Execution toward physical equipment.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RobotCommand {
    pub kind: String,
    pub target: String,
    pub parameters: HashMap<String, String>,
}

/// Error produced when a command cannot be delivered.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum CommandError {
    #[error("Command delivery failed: {0}")]
    DeliveryFailed(String),

    #[error("Hardware not connected")]
    NotConnected,

    #[error("Command rejected by equipment: {0}")]
    Rejected(String),
}

/// In-memory observation provider for testing and development.
#[derive(Debug, Clone, Default)]
pub struct InMemoryObservationProvider {
    observations: Arc<Mutex<HashMap<String, ChannelObservation>>>,
}

impl InMemoryObservationProvider {
    pub fn new() -> Self {
        Self {
            observations: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn set_channel(&self, name: impl Into<String>, value: f64) {
        let name = name.into();
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let obs = ChannelObservation {
            channel_id: name.clone(),
            sampled_at_ns: now_ns,
            received_at_ns: now_ns,
            value: thalos_core::device::ChannelValue::Scalar(value),
            unit: None,
            quality: SignalQuality::Nominal,
        };
        self.observations.lock().unwrap().insert(name, obs);
    }

    pub fn set_observation(&self, name: impl Into<String>, obs: ChannelObservation) {
        self.observations.lock().unwrap().insert(name.into(), obs);
    }
}

impl ObservationProvider for InMemoryObservationProvider {
    fn snapshot(&self) -> ObservationBundle {
        let observations = self.observations.lock().unwrap().clone();
        let captured_at_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64;
        ObservationBundle {
            captured_at_us,
            observations,
        }
    }
}

/// Proveedor mutable de estado del robot que implementa `RobotObservationProvider`.
#[derive(Debug, Clone, Default)]
pub struct SharedRobotObservation {
    state: Arc<Mutex<RobotState>>,
}

impl SharedRobotObservation {
    pub fn new(initial: RobotState) -> Self {
        Self {
            state: Arc::new(Mutex::new(initial)),
        }
    }

    pub fn update(&self, joints: Vec<f64>, velocities: Vec<f64>) {
        let mut guard = self.state.lock().unwrap();
        guard.joints = joints;
        guard.velocities = velocities;
    }
}

impl RobotObservationProvider for SharedRobotObservation {
    fn observe(&self) -> RobotState {
        self.state.lock().unwrap().clone()
    }
}

/// Runner modular que combina un `ObservationProvider` y un `RobotObservationProvider`.
#[derive(Debug)]
pub struct TelemetryExecutionRunner<A, R>
where
    A: ObservationProvider,
    R: RobotObservationProvider,
{
    pub observation_provider: A,
    pub robot_provider: R,
    pub expected_state: ExpectedState,
    pub is_connected: bool,
}

impl<A, R> TelemetryExecutionRunner<A, R>
where
    A: ObservationProvider,
    R: RobotObservationProvider,
{
    pub fn new(observation_provider: A, robot_provider: R, expected_state: ExpectedState) -> Self {
        Self {
            observation_provider,
            robot_provider,
            expected_state,
            is_connected: true,
        }
    }

    pub fn with_connection_status(mut self, is_connected: bool) -> Self {
        self.is_connected = is_connected;
        self
    }
}

impl<A, R> ExecutionRunner for TelemetryExecutionRunner<A, R>
where
    A: ObservationProvider,
    R: RobotObservationProvider,
{
    fn acquire(&mut self) -> TickContext {
        TickContext {
            observations: self.observation_provider.snapshot(),
            robot: self.robot_provider.observe(),
            expected: self.expected_state.clone(),
        }
    }

    fn act(&mut self, action: &Action) -> TickOutcome {
        if !self.is_connected {
            return TickOutcome::Faulted("Hardware disconnected during action dispatch".to_string());
        }

        match action {
            Action::DispatchMotion { .. } => TickOutcome::Success,
            Action::SetOutput { .. } => TickOutcome::Success,
            Action::HoldPosition => TickOutcome::Success,
            Action::None => TickOutcome::Success,
        }
    }
}

/// Runner de simulación virtual para pruebas y modelado.
#[derive(Debug, Default)]
pub struct SimulationRunner {
    pub current_context: TickContext,
}

impl SimulationRunner {
    pub fn new(initial_context: TickContext) -> Self {
        Self {
            current_context: initial_context,
        }
    }

    pub fn set_context(&mut self, context: TickContext) {
        self.current_context = context;
    }
}

impl ExecutionRunner for SimulationRunner {
    fn acquire(&mut self) -> TickContext {
        self.current_context.clone()
    }

    fn act(&mut self, action: &Action) -> TickOutcome {
        match action {
            Action::DispatchMotion { .. } => TickOutcome::Success,
            Action::SetOutput { .. } => TickOutcome::Success,
            Action::HoldPosition => TickOutcome::Success,
            Action::None => TickOutcome::Success,
        }
    }
}

/// Runner de hardware físico (o mock de hardware).
#[derive(Debug, Default)]
pub struct PhysicalRunner {
    pub current_context: TickContext,
    pub is_connected: bool,
}

impl PhysicalRunner {
    pub fn new(initial_context: TickContext, is_connected: bool) -> Self {
        Self {
            current_context: initial_context,
            is_connected,
        }
    }

    pub fn set_context(&mut self, context: TickContext) {
        self.current_context = context;
    }
}

impl ExecutionRunner for PhysicalRunner {
    fn acquire(&mut self) -> TickContext {
        self.current_context.clone()
    }

    fn act(&mut self, action: &Action) -> TickOutcome {
        if !self.is_connected {
            return TickOutcome::Faulted("Physical hardware disconnected".to_string());
        }

        match action {
            Action::DispatchMotion { .. } => TickOutcome::Success,
            Action::SetOutput { .. } => TickOutcome::Success,
            Action::HoldPosition => TickOutcome::Success,
            Action::None => TickOutcome::Success,
        }
    }
}
