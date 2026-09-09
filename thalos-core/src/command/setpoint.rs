//! Setpoint command — requests a controller to maintain a desired operating value.
//!
//! The resource assumes closed-loop regulation (PID, etc.).
//! Thalos observes the process variable through resource-provided observations.
//!
//! ```text
//! Thalos → Setpoint(70°C) → PLC → PID → Heater
//!                                            ↓
//!                                    temperature = 70°C
//!                                            ↓
//!                                    Observation → Thalos
//! ```
//!
//! ## Unit/type caveat (ADR-019 §4)
//!
//! `SetpointValue` currently carries raw numeric values without physical units.
//! This is intentional for MVP — it validates the architecture without solving
//! the units problem. Future phases should introduce unit-aware values
//! (e.g. `Temperature(70.0, Celsius)`).

use serde::{Deserialize, Serialize};

use crate::resource::ResourceRef;

/// A command requesting a controller to maintain a desired operating value.
///
/// The target resource assumes full responsibility for:
/// - Closed-loop regulation (PID, on/off, etc.)
/// - Monitoring the process variable
/// - Reporting state/fault to Thalos
///
/// Thalos evaluates setpoint achievement by observing the resource's
/// process variable (temperature, pressure, speed, etc.).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SetpointCommand {
    pub target: ResourceRef,
    /// The process variable name (e.g. "temperature", "pressure", "speed_rpm").
    pub variable: String,
    /// The desired value. Unit/type metadata is deferred to a future phase.
    pub value: SetpointValue,
}

/// A typed value for setpoint commands.
///
/// Carries raw numeric values without physical units for MVP.
/// Future phases should introduce unit-aware variants.
///
/// **Variant order matters for `#[serde(untagged)]`**: `Integer` before `Float`
/// ensures `1200` deserializes as `Integer(1200)`, not `Float(1200.0)`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum SetpointValue {
    Integer(i32),
    Float(f64),
    Bool(bool),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resource::ResourceKind;

    #[test]
    fn temperature_setpoint() {
        let cmd = SetpointCommand {
            target: ResourceRef::new("heater-01", ResourceKind::Device),
            variable: "temperature".into(),
            value: SetpointValue::Float(70.0),
        };
        assert_eq!(cmd.target.id.as_str(), "heater-01");
        assert_eq!(cmd.variable, "temperature");
        assert_eq!(cmd.value, SetpointValue::Float(70.0));
    }

    #[test]
    fn pressure_setpoint_integer() {
        let cmd = SetpointCommand {
            target: ResourceRef::new("pump-01", ResourceKind::Device),
            variable: "pressure_bar".into(),
            value: SetpointValue::Integer(5),
        };
        assert_eq!(cmd.value, SetpointValue::Integer(5));
    }

    #[test]
    fn bool_setpoint() {
        let cmd = SetpointCommand {
            target: ResourceRef::new("valve-01", ResourceKind::Device),
            variable: "enabled".into(),
            value: SetpointValue::Bool(true),
        };
        assert_eq!(cmd.value, SetpointValue::Bool(true));
    }

    #[test]
    fn serde_round_trip() {
        let cases = vec![
            SetpointCommand {
                target: ResourceRef::new("h1", ResourceKind::Device),
                variable: "temp".into(),
                value: SetpointValue::Float(70.0),
            },
            SetpointCommand {
                target: ResourceRef::new("p1", ResourceKind::Device),
                variable: "speed".into(),
                value: SetpointValue::Integer(1200),
            },
            SetpointCommand {
                target: ResourceRef::new("v1", ResourceKind::Device),
                variable: "active".into(),
                value: SetpointValue::Bool(true),
            },
        ];
        for cmd in cases {
            let json = serde_json::to_string(&cmd).expect("serialize");
            let back: SetpointCommand = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(cmd, back);
        }
    }
}
