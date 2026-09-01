use std::fmt;
pub use thalos_core::robot::{RobotAction, RobotCommand, RobotObservation};

/// Physical/simulated transport connection state (L1 Port).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportState {
    Disconnected,
    Connecting,
    Connected,
    Faulted,
}

/// Generic, hardware-agnostic transport boundary errors (L1 Port).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TransportError {
    Disconnected,
    NotReady,
    Timeout,
    CommunicationFailure(String),
}

impl fmt::Display for TransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Disconnected => write!(f, "transport is disconnected"),
            Self::NotReady => write!(f, "transport is not ready"),
            Self::Timeout => write!(f, "transport operation timed out"),
            Self::CommunicationFailure(e) => write!(f, "communication failure: {}", e),
        }
    }
}

impl std::error::Error for TransportError {}

/// Abstract RobotTransport Domain Port (ADR-014 / L1 Port)
pub trait RobotTransport: Send + Sync {
    fn state(&self) -> TransportState;
    fn send(&mut self, command: RobotCommand) -> Result<(), TransportError>;
    fn stop(&mut self) -> Result<(), TransportError>;
    fn try_receive_observation(&mut self) -> Result<Option<RobotObservation>, TransportError>;
}
