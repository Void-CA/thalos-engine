use crate::execution::session::{
    Action, ObservationBundle, RobotSample, RuntimeState, TickContext, TickOutcome,
};
use crate::robot::RobotCommand;

/// Abstracción del entorno de ejecución (Simulación, Hardware Físico, etc.).
///
/// El runner es un proveedor de estado (`acquire`) y un actuador de acciones (`act`).
/// No toma decisiones ni gestiona el estado de la sesión: esa autoridad es de `ExecutionSession`.
///
/// Contract only: implementations live in [`super::runners`]. This module must
/// not reference any concrete runner.
pub trait ExecutionRunner: Send + Sync {
    /// Adquiere el contexto de observación inmutable para el tick k.
    fn acquire(&mut self) -> TickContext;

    /// Ejecuta la acción determinada por la decisión del tick en el entorno correspondiente.
    fn act(&mut self, action: &Action) -> TickOutcome;

    /// Estado runtime producido por la última acción.
    ///
    /// Se consulta DESPUÉS de `act` para que la observación del tick refleje el
    /// estado que el runtime realmente produjo (no el contexto previo a la
    /// acción). Incluye la pose TCP cuando la autoridad cinemática compartida
    /// puede resolverla.
    fn runtime_state(&self) -> RuntimeState {
        RuntimeState::default()
    }
}

/// A boxed runner is itself a runner.
///
/// This lets a consumer hold an execution capability chosen at runtime
/// (`Box<dyn ExecutionRunner + Send + Sync>`) without naming any concrete
/// runner — the mechanism of *how* the capability was built stays outside the
/// consumer. Generic over `?Sized` so it applies to trait objects.
impl<T: ExecutionRunner + ?Sized> ExecutionRunner for Box<T> {
    fn acquire(&mut self) -> TickContext {
        (**self).acquire()
    }

    fn act(&mut self, action: &Action) -> TickOutcome {
        (**self).act(action)
    }

    fn runtime_state(&self) -> RuntimeState {
        (**self).runtime_state()
    }
}

/// Provides observation data to the execution domain.
///
/// This trait is the contractual boundary between Interconnection (provider)
/// and Execution (consumer). Execution defines WHAT it needs; Interconnection
/// implements HOW to provide it.
///
/// `snapshot` takes `&mut self` because a real source is STATEFUL (it buffers
/// transport readings and consumes them over successive ticks). A stateful
/// boundary must not be forced behind interior mutability just to satisfy an
/// immutable receiver.
pub trait ObservationProvider: Send + Sync {
    fn snapshot(&mut self) -> ObservationBundle;
}

/// Issues commands through the interconnection layer.
///
/// Execution defines WHAT domain commands to issue; Interconnection implements
/// HOW to transport them to the physical equipment.
pub trait CommandProvider: Send + Sync {
    fn dispatch(&mut self, command: &RobotCommand) -> Result<(), CommandError>;
}

/// Provides observation of the robot state (joint positions, velocities).
pub trait RobotObservationProvider: Send + Sync {
    fn observe(&self) -> RobotSample;
}

#[cfg(test)]
mod tests {
    use super::*;

    struct DummyRunner;

    impl ExecutionRunner for DummyRunner {
        fn acquire(&mut self) -> TickContext {
            TickContext::default()
        }

        fn act(&mut self, _action: &Action) -> TickOutcome {
            TickOutcome::Success
        }
    }

    #[test]
    fn boxed_runner_is_a_runner_and_delegates() {
        let mut runner: Box<dyn ExecutionRunner + Send + Sync> = Box::new(DummyRunner);
        let _ = runner.acquire();
        assert_eq!(runner.act(&Action::None), TickOutcome::Success);
        let _ = runner.runtime_state();
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
