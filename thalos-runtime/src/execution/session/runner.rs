use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use super::domain::{
    Action, AcquisitionSnapshot, ExpectedState, RobotState, TickContext, TickOutcome,
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

/// Trait para proveedores de adquisición de señales de sensores/canales.
pub trait AcquisitionProvider: Send + Sync {
    fn snapshot(&self) -> AcquisitionSnapshot;
}

/// Trait para proveedores de observación del estado del robot (articulaciones, velocidades).
pub trait RobotObservationProvider: Send + Sync {
    fn observe(&self) -> RobotState;
}

/// Registro en memoria de canales que implementa `AcquisitionProvider`.
#[derive(Debug, Clone, Default)]
pub struct InMemoryAcquisitionRegistry {
    channels: Arc<Mutex<HashMap<String, f64>>>,
}

impl InMemoryAcquisitionRegistry {
    pub fn new() -> Self {
        Self {
            channels: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn set_channel(&self, name: impl Into<String>, value: f64) {
        self.channels.lock().unwrap().insert(name.into(), value);
    }
}

impl AcquisitionProvider for InMemoryAcquisitionRegistry {
    fn snapshot(&self) -> AcquisitionSnapshot {
        let channels = self.channels.lock().unwrap().clone();
        let timestamp_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64;

        AcquisitionSnapshot {
            timestamp_us,
            channels,
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

/// Runner modular que combina un `AcquisitionProvider` y un `RobotObservationProvider`.
pub struct TelemetryExecutionRunner<A, R>
where
    A: AcquisitionProvider,
    R: RobotObservationProvider,
{
    pub acquisition_provider: A,
    pub robot_provider: R,
    pub expected_state: ExpectedState,
    pub is_connected: bool,
}

impl<A, R> TelemetryExecutionRunner<A, R>
where
    A: AcquisitionProvider,
    R: RobotObservationProvider,
{
    pub fn new(acquisition_provider: A, robot_provider: R, expected_state: ExpectedState) -> Self {
        Self {
            acquisition_provider,
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
    A: AcquisitionProvider,
    R: RobotObservationProvider,
{
    fn acquire(&mut self) -> TickContext {
        TickContext {
            acquisition: self.acquisition_provider.snapshot(),
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
