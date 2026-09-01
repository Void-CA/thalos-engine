use thiserror::Error;
use thalos_engine::prelude::*;
use crate::resources::ResourceRegistry;
use super::state::{OperationalSession, StationRuntimeState};

/// StationRuntimeError (ADR-014)
#[derive(Debug, Error, PartialEq, Eq)]
pub enum StationRuntimeError {
    #[error("Invalid station state transition from {from:?} to {to:?}")]
    InvalidStateTransition {
        from: StationRuntimeState,
        to: StationRuntimeState,
    },

    #[error("Station is not operational (current state: {0:?})")]
    NotOperational(StationRuntimeState),

    #[error("No active session exists to stop")]
    NoActiveSession,
}

/// StationRuntime (ADR-014)
/// Operational supervisor managing station lifecycle, resource context, and active sessions.
#[derive(Debug)]
pub struct StationRuntime {
    station: Station,
    registry: ResourceRegistry,
    state: StationRuntimeState,
    active_session: Option<OperationalSession>,
}

impl StationRuntime {
    /// Instantiate a new StationRuntime supervisor in `Created` state.
    pub fn new(station: Station, registry: ResourceRegistry) -> Self {
        Self {
            station,
            registry,
            state: StationRuntimeState::Created,
            active_session: None,
        }
    }

    /// Access the underlying Station definition.
    pub fn station(&self) -> &Station {
        &self.station
    }

    /// Access the global ResourceRegistry.
    pub fn registry(&self) -> &ResourceRegistry {
        &self.registry
    }

    /// Current operational lifecycle state.
    pub fn state(&self) -> StationRuntimeState {
        self.state
    }

    /// Active operational session, if any.
    pub fn active_session(&self) -> Option<&OperationalSession> {
        self.active_session.as_ref()
    }

    /// Transition supervisor from `Created` -> `Starting` -> `Ready`.
    pub fn start(&mut self) -> Result<(), StationRuntimeError> {
        match self.state {
            StationRuntimeState::Created | StationRuntimeState::Stopped => {
                self.state = StationRuntimeState::Starting;
                // Initialization / resource discovery hook place
                self.state = StationRuntimeState::Ready;
                Ok(())
            }
            other => Err(StationRuntimeError::InvalidStateTransition {
                from: other,
                to: StationRuntimeState::Starting,
            }),
        }
    }

    /// Transition supervisor from `Ready` / `Active` -> `Stopping` -> `Stopped`.
    pub fn stop(&mut self) -> Result<(), StationRuntimeError> {
        match self.state {
            StationRuntimeState::Ready | StationRuntimeState::Active => {
                self.state = StationRuntimeState::Stopping;
                // Graceful cancellation of active sessions / tasks
                self.active_session = None;
                self.state = StationRuntimeState::Stopped;
                Ok(())
            }
            other => Err(StationRuntimeError::InvalidStateTransition {
                from: other,
                to: StationRuntimeState::Stopping,
            }),
        }
    }

    /// Start an operational session: transitions `Ready` -> `Active`.
    pub fn start_session(&mut self) -> Result<OperationalSession, StationRuntimeError> {
        if self.state != StationRuntimeState::Ready && self.state != StationRuntimeState::Active {
            return Err(StationRuntimeError::NotOperational(self.state));
        }

        let session_id_str = format!("ops-session-{}", uuid::Uuid::new_v4());
        let session = OperationalSession::new(session_id_str, self.station.id.clone());

        self.active_session = Some(session.clone());
        self.state = StationRuntimeState::Active;

        Ok(session)
    }

    /// Stop the active operational session: if no active sessions remain, returns `Ready`.
    pub fn stop_session(&mut self) -> Result<(), StationRuntimeError> {
        if self.active_session.is_none() {
            return Err(StationRuntimeError::NoActiveSession);
        }

        self.active_session = None;
        if self.state == StationRuntimeState::Active {
            self.state = StationRuntimeState::Ready;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_station_runtime_lifecycle_flow() {
        let robot_ref = ResourceRef::new("scara-01", ResourceKind::Robot);
        let station = Station::new("cell-01", "Cell 01", vec![robot_ref]);
        let registry = ResourceRegistry::new();

        let mut runtime = StationRuntime::new(station, registry);
        assert_eq!(runtime.state(), StationRuntimeState::Created);

        // Created -> Ready
        runtime.start().unwrap();
        assert_eq!(runtime.state(), StationRuntimeState::Ready);

        // Ready -> Active (start session)
        let session = runtime.start_session().unwrap();
        assert_eq!(runtime.state(), StationRuntimeState::Active);
        assert_eq!(runtime.active_session().unwrap().id, session.id);

        // Active -> Ready (stop session)
        runtime.stop_session().unwrap();
        assert_eq!(runtime.state(), StationRuntimeState::Ready);
        assert!(runtime.active_session().is_none());

        // Ready -> Stopped
        runtime.stop().unwrap();
        assert_eq!(runtime.state(), StationRuntimeState::Stopped);
    }

    #[test]
    fn test_invalid_transition_from_created_to_active() {
        let station = Station::new("cell-01", "Cell 01", vec![]);
        let registry = ResourceRegistry::new();

        let mut runtime = StationRuntime::new(station, registry);
        let res = runtime.start_session();

        assert_eq!(
            res,
            Err(StationRuntimeError::NotOperational(
                StationRuntimeState::Created
            ))
        );
    }
}
