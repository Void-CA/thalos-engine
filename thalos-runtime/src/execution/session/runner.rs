use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use thalos_core::device::{ChannelObservation, SignalQuality};
use thalos_core::robot::RobotCommand;
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

/// No-op command provider for testing and backward compatibility.
#[derive(Debug, Clone, Default)]
pub struct NoopCommandProvider;

impl CommandProvider for NoopCommandProvider {
    fn dispatch(&mut self, _command: &RobotCommand) -> Result<(), CommandError> {
        Ok(())
    }
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

/// Adapter that bridges CommandProvider to RobotTransport.
///
/// This is the integration point between the execution domain and physical hardware.
/// It translates domain-level RobotCommands into transport-level send operations.
///
/// Invariant: This adapter translates contracts; it does not introduce execution semantics.
/// Waypoint progression, convergence checks, and safety logic belong to HardwareExecutor.
pub struct TransportCommandProvider<T: thalos_ports::robot::RobotTransport> {
    transport: std::sync::Arc<std::sync::Mutex<T>>,
}

impl<T: thalos_ports::robot::RobotTransport> TransportCommandProvider<T> {
    pub fn new(transport: std::sync::Arc<std::sync::Mutex<T>>) -> Self {
        Self { transport }
    }
}

impl<T: thalos_ports::robot::RobotTransport> CommandProvider for TransportCommandProvider<T> {
    fn dispatch(&mut self, command: &RobotCommand) -> Result<(), CommandError> {
        let mut transport = self.transport.lock().map_err(|e| {
            CommandError::DeliveryFailed(format!("Transport lock poisoned: {e}"))
        })?;

        // Check transport state before sending
        match transport.state() {
            thalos_ports::robot::TransportState::Connected => {}
            state => {
                return Err(CommandError::NotConnected);
            }
        }

        // Send command through transport
        transport.send(command.clone()).map_err(|e| {
            CommandError::DeliveryFailed(format!("Transport send failed: {e}"))
        })?;

        Ok(())
    }
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

/// Runner modular que combina un `ObservationProvider`, un `RobotObservationProvider`,
/// y un `CommandProvider` para dispatch de comandos al hardware.
#[derive(Debug)]
pub struct TelemetryExecutionRunner<A, R, C>
where
    A: ObservationProvider,
    R: RobotObservationProvider,
    C: CommandProvider,
{
    pub observation_provider: A,
    pub robot_provider: R,
    pub command_provider: C,
    pub expected_state: ExpectedState,
    pub is_connected: bool,
}

impl<A, R, C> TelemetryExecutionRunner<A, R, C>
where
    A: ObservationProvider,
    R: RobotObservationProvider,
    C: CommandProvider,
{
    pub fn new(
        observation_provider: A,
        robot_provider: R,
        command_provider: C,
        expected_state: ExpectedState,
    ) -> Self {
        Self {
            observation_provider,
            robot_provider,
            command_provider,
            expected_state,
            is_connected: true,
        }
    }

    pub fn with_connection_status(mut self, is_connected: bool) -> Self {
        self.is_connected = is_connected;
        self
    }
}

impl<A, R, C> ExecutionRunner for TelemetryExecutionRunner<A, R, C>
where
    A: ObservationProvider,
    R: RobotObservationProvider,
    C: CommandProvider,
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
            Action::DispatchMotion { kind, target } => {
                let cmd = match kind.as_str() {
                    "movej" => RobotCommand::MoveJoints {
                        positions_rad: vec![], // TODO: resolve target to joint positions
                        velocities_rad_s: None,
                    },
                    _ => RobotCommand::Stop,
                };
                match self.command_provider.dispatch(&cmd) {
                    Ok(()) => TickOutcome::Success,
                    Err(e) => TickOutcome::Faulted(format!("Command dispatch failed: {e}")),
                }
            }
            Action::SetOutput { .. } => TickOutcome::Success,
            Action::HoldPosition => {
                match self.command_provider.dispatch(&RobotCommand::Stop) {
                    Ok(()) => TickOutcome::Success,
                    Err(e) => TickOutcome::Faulted(format!("Hold position failed: {e}")),
                }
            }
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

/// Test double that captures dispatched RobotCommands for assertion.
#[derive(Debug, Clone, Default)]
pub struct CapturingCommandProvider {
    commands: Arc<Mutex<Vec<RobotCommand>>>,
}

impl CapturingCommandProvider {
    pub fn new() -> Self {
        Self {
            commands: Arc::new(Mutex::new(Vec::new())),
        }
    }

    pub fn captured(&self) -> Vec<RobotCommand> {
        self.commands.lock().unwrap().clone()
    }

    pub fn clear(&self) {
        self.commands.lock().unwrap().clear();
    }
}

impl CommandProvider for CapturingCommandProvider {
    fn dispatch(&mut self, command: &RobotCommand) -> Result<(), CommandError> {
        self.commands.lock().unwrap().push(command.clone());
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::device::ChannelValue;

    #[test]
    fn in_memory_observation_provider_scalar_values() {
        let provider = InMemoryObservationProvider::new();
        provider.set_channel("temperature", 72.5);
        provider.set_channel("pressure", 1013.25);

        let bundle = provider.snapshot();
        let temp = bundle.observations.get("temperature").unwrap();
        assert_eq!(temp.value, ChannelValue::Scalar(72.5));
        assert_eq!(temp.quality, SignalQuality::Nominal);

        let pres = bundle.observations.get("pressure").unwrap();
        assert_eq!(pres.value, ChannelValue::Scalar(1013.25));
    }

    #[test]
    fn in_memory_observation_provider_timestamps() {
        let before_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_micros() as u64;

        let provider = InMemoryObservationProvider::new();
        provider.set_channel("ch1", 1.0);

        let bundle = provider.snapshot();
        let after_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_micros() as u64;

        assert!(bundle.captured_at_us >= before_us);
        assert!(bundle.captured_at_us <= after_us);

        let obs = bundle.observations.get("ch1").unwrap();
        assert!(obs.sampled_at_ns > 0);
        assert!(obs.received_at_ns > 0);
    }

    #[test]
    fn in_memory_observation_provider_updates_between_ticks() {
        let provider = InMemoryObservationProvider::new();
        provider.set_channel("sensor", 10.0);

        let bundle1 = provider.snapshot();
        assert_eq!(
            bundle1.observations.get("sensor").unwrap().value,
            ChannelValue::Scalar(10.0)
        );

        provider.set_channel("sensor", 25.0);
        let bundle2 = provider.snapshot();
        assert_eq!(
            bundle2.observations.get("sensor").unwrap().value,
            ChannelValue::Scalar(25.0)
        );
    }

    #[test]
    fn in_memory_observation_provider_explicit_observation() {
        let provider = InMemoryObservationProvider::new();
        let obs = ChannelObservation {
            channel_id: "vibration".to_string(),
            sampled_at_ns: 1000,
            received_at_ns: 1100,
            value: ChannelValue::Scalar(0.42),
            unit: Some("g".to_string()),
            quality: SignalQuality::Degraded,
        };
        provider.set_observation("vibration", obs);

        let bundle = provider.snapshot();
        let vib = bundle.observations.get("vibration").unwrap();
        assert_eq!(vib.quality, SignalQuality::Degraded);
        assert_eq!(vib.unit.as_deref(), Some("g"));
        assert_eq!(vib.sampled_at_ns, 1000);
    }

    #[test]
    fn capturing_command_provider_records_commands() {
        let mut provider = CapturingCommandProvider::new();
        let cmd = RobotCommand::MoveJoints {
            positions_rad: vec![0.1, 0.2, 0.3],
            velocities_rad_s: None,
        };
        provider.dispatch(&cmd).unwrap();
        provider.dispatch(&RobotCommand::Stop).unwrap();

        let captured = provider.captured();
        assert_eq!(captured.len(), 2);
        assert_eq!(captured[0], RobotCommand::MoveJoints {
            positions_rad: vec![0.1, 0.2, 0.3],
            velocities_rad_s: None,
        });
        assert_eq!(captured[1], RobotCommand::Stop);
    }

    #[test]
    fn capturing_command_provider_clear() {
        let mut provider = CapturingCommandProvider::new();
        provider.dispatch(&RobotCommand::Stop).unwrap();
        assert_eq!(provider.captured().len(), 1);

        provider.clear();
        assert_eq!(provider.captured().len(), 0);
    }

    #[test]
    fn observation_bundle_with_channel_value_types() {
        let provider = InMemoryObservationProvider::new();

        // Boolean observation
        provider.set_observation("safety_stop", ChannelObservation {
            channel_id: "safety_stop".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: ChannelValue::Boolean(true),
            unit: None,
            quality: SignalQuality::Nominal,
        });

        // Integer observation
        provider.set_observation("error_code", ChannelObservation {
            channel_id: "error_code".to_string(),
            sampled_at_ns: 0,
            received_at_ns: 0,
            value: ChannelValue::Integer(42),
            unit: None,
            quality: SignalQuality::Nominal,
        });

        let bundle = provider.snapshot();
        assert_eq!(
            bundle.observations.get("safety_stop").unwrap().value,
            ChannelValue::Boolean(true)
        );
        assert_eq!(
            bundle.observations.get("error_code").unwrap().value,
            ChannelValue::Integer(42)
        );
    }
}
