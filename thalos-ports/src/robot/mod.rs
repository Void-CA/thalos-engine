#[cfg(feature = "test-support")]
pub mod fake;

pub mod transport;

pub use thalos_core::robot::{RobotAction, RobotCommand, RobotObservation};
pub use transport::{RobotTransport, TransportError, TransportState};

#[cfg(feature = "test-support")]
pub use fake::FakeRobotTransport;

