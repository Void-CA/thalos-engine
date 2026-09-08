use serde::{Deserialize, Serialize};

/// Direction of signal flow relative to Thalos.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalDirection {
    /// Information flowing from physical world into Thalos.
    Input,
    /// Information flowing from Thalos toward physical world.
    Output,
}

/// Whether a signal originates externally or is computed within Thalos.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalKind {
    /// Signal originated from an external device/sensor.
    Source,
    /// Signal computed from other signals within Thalos.
    Derived,
}

/// A unit of information exchanged or computed within the system.
///
/// Signals are the fundamental information carriers in the Interconnection model.
/// Source signals have physical origins and require Bindings. Derived signals are
/// computed from other signals and carry provenance metadata.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Signal {
    pub id: String,
    pub direction: SignalDirection,
    pub kind: SignalKind,
    pub unit: Option<String>,
}

impl Signal {
    pub fn source(id: impl Into<String>, direction: SignalDirection) -> Self {
        Self {
            id: id.into(),
            direction,
            kind: SignalKind::Source,
            unit: None,
        }
    }

    pub fn derived(id: impl Into<String>, direction: SignalDirection) -> Self {
        Self {
            id: id.into(),
            direction,
            kind: SignalKind::Derived,
            unit: None,
        }
    }

    pub fn with_unit(mut self, unit: impl Into<String>) -> Self {
        self.unit = Some(unit.into());
        self
    }
}

/// Mathematical expression for computing a derived signal.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignalExpression {
    /// Reference to another signal by ID.
    Channel(String),
    /// Addition of two signals.
    Add(String, String),
    /// Subtraction of two signals.
    Subtract(String, String),
    /// Multiplication of two signals.
    Multiply(String, String),
    /// Division of two signals.
    Divide(String, String),
}

/// A signal computed from other signals, carrying provenance metadata.
///
/// DerivedSignal enables traceability: given any derived signal value,
/// the system can trace back to its source signals and computation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DerivedSignal {
    pub signal_id: String,
    pub inputs: Vec<String>,
    pub expression: SignalExpression,
}

impl DerivedSignal {
    pub fn new(signal_id: impl Into<String>, expression: SignalExpression) -> Self {
        let inputs = match &expression {
            SignalExpression::Channel(a) => vec![a.clone()],
            SignalExpression::Add(a, b)
            | SignalExpression::Subtract(a, b)
            | SignalExpression::Multiply(a, b)
            | SignalExpression::Divide(a, b) => vec![a.clone(), b.clone()],
        };
        Self {
            signal_id: signal_id.into(),
            inputs,
            expression,
        }
    }
}

/// Mapping from a domain signal to a physical channel endpoint.
///
/// Bindings are only necessary for Source signals — Derived signals
/// have no physical origin to bind to.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SignalBinding {
    pub signal_id: String,
    pub channel_id: String,
    pub endpoint_id: String,
}

/// Concrete communication participant.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Endpoint {
    pub id: String,
    pub kind: EndpointKind,
    pub connection_state: ConnectionState,
}

/// Types of physical endpoints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EndpointKind {
    Esp32,
    Plc,
    RobotController,
    Simulator,
    Virtual,
}

/// Connection state of an endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Degraded,
    Faulted,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signal_source_creation() {
        let s = Signal::source("robot.joint_1.position", SignalDirection::Input)
            .with_unit("rad");
        assert_eq!(s.kind, SignalKind::Source);
        assert_eq!(s.direction, SignalDirection::Input);
        assert_eq!(s.unit.as_deref(), Some("rad"));
    }

    #[test]
    fn signal_derived_creation() {
        let s = Signal::derived("robot.joint_1.error", SignalDirection::Input);
        assert_eq!(s.kind, SignalKind::Derived);
    }

    #[test]
    fn derived_signal_provenance() {
        let ds = DerivedSignal::new(
            "robot.joint_1.error",
            SignalExpression::Subtract(
                "robot.target.joint_1.position".to_string(),
                "robot.joint_1.position".to_string(),
            ),
        );
        assert_eq!(ds.inputs.len(), 2);
        assert!(ds.inputs.contains(&"robot.target.joint_1.position".to_string()));
        assert!(ds.inputs.contains(&"robot.joint_1.position".to_string()));
    }

    #[test]
    fn signal_binding_source_only() {
        let binding = SignalBinding {
            signal_id: "robot.joint_1.position".to_string(),
            channel_id: "esp32.joint.1.pos".to_string(),
            endpoint_id: "esp32-cell01".to_string(),
        };
        assert_eq!(binding.signal_id, "robot.joint_1.position");
    }

    #[test]
    fn endpoint_connection_states() {
        let ep = Endpoint {
            id: "esp32-cell01".to_string(),
            kind: EndpointKind::Esp32,
            connection_state: ConnectionState::Connected,
        };
        assert_eq!(ep.connection_state, ConnectionState::Connected);
    }
}
