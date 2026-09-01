use serde::{Deserialize, Serialize};

/// Origen de una ejecución — concepto de dominio, no implementación técnica.
///
/// El usuario no selecciona "HardwareBackend". Selecciona "Hardware".
/// La API resuelve el enum al backend concreto.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum ExecutionSource {
    /// Ejecución en simulación cinemática.
    Simulation,
    /// Ejecución en robot físico.
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
