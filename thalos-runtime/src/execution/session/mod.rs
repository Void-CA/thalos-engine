pub mod domain;
pub mod events;
pub mod execution_source;
pub mod history;
pub mod manager;
pub mod runner;
pub mod session_data;

pub use domain::{
    Action, Cardinality, ChannelAccessError, ControlTick, CycleState, Decision,
    DomainExecutionCoordinator, Environment, ExecutionConfiguration, ExecutionDomainError,
    ExecutionSession as DomainExecutionSession, ExecutionSessionId, ExpectedState,
    InvalidLifecycleTransition, LifecycleState, ObservationBundle, ProgramState, Reactivity,
    RobotState, SessionRegistry, SessionState, TerminationPolicy, TickContext, TickOutcome,
    TickResult, eval_channel_access, eval_derived_signal, extract_scalar,
};
pub use events::{EventSubscriber, ExecutionEvent, ExecutionEventBus, TemporalInvariants};
pub use execution_source::ExecutionSource;
pub use history::{
    ExecutionHistory, ExecutionHistoryStore, HistoricalFaultRecord, HistoricalLifecycleTransition,
    HistoricalTickRecord,
};
pub use manager::SessionManager;
pub use runner::{
    CapturingCommandProvider, CommandError, CommandProvider, ExecutionRunner,
    InMemoryObservationProvider, NoopCommandProvider, ObservationProvider, PhysicalRunner,
    RobotObservationProvider, SharedRobotObservation, SimulationRunner, TelemetryExecutionRunner,
};
pub use session_data::{SessionData, SessionWithTrace};
