use serde::{Deserialize, Serialize};

/// Origin of an execution — a domain concept, not a technical implementation.
///
/// The user does not select a "HardwareBackend"; they select "Hardware". The
/// API resolves the enum into the concrete backend.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum ExecutionSource {
    /// Execution in kinematic simulation.
    Simulation,
    /// Execution on physical hardware.
    Hardware,
}

impl std::fmt::Display for ExecutionSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExecutionSource::Simulation => write!(f, "Simulation"),
            ExecutionSource::Hardware => write!(f, "Hardware"),
        }
    }
}
