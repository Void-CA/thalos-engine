//! Typed, machine-readable attribute values carried by observations (D5).
//!
//! Attributes are the free-form typed data of an [`Observation`](crate::analysis::observation::Observation).
//! They complement the structured [`kind`](crate::analysis::observation::ObservationKind) /
//! [`location`](crate::analysis::location::Location) vocabulary with phenomenon-specific
//! measurements (thresholds, values, identifiers).
//!
//! # Invariants
//!
//! - **Machine-readable** (spec I2): a value is always typed data (`Number`, `Integer`, `Bool`,
//!   `Text`), never a presentation string. Renderers own presentation.
//! - **Deterministic** (design D5): serialization is stable. The enum uses serde's default
//!   external tagging, and attribute maps use `BTreeMap`, so equal values always serialize to
//!   identical bytes — no hash-order nondeterminism.
//! - **No cycles**: values are plain data with no references into the planner, runtime, or UI.
//!
//! # Extensibility
//!
//! The enum is `#[non_exhaustive]`: new variants can be added without breaking downstream
//! exhaustive matches. Consumers must always match with a wildcard arm.

use serde::{Deserialize, Serialize};

/// A single typed attribute value attached to an [`Observation`](crate::analysis::observation::Observation).
///
/// The variant set is intentionally closed: `Number` (floating point) and `Integer` are
/// distinct so that wire consumers can preserve integrality without float round-trips.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum AttributeValue {
    /// A floating-point measurement (e.g. `value`, `threshold` in a tracking observation).
    Number(f64),
    /// A UTF-8 string datum (e.g. an identifier, never a localized message — spec I1).
    Text(String),
    /// A boolean flag.
    Bool(bool),
    /// A signed integral measurement.
    Integer(i64),
}

#[cfg(test)]
mod tests {
    use super::AttributeValue;
    use serde_json::json;

    #[test]
    fn number_round_trip() {
        let value = AttributeValue::Number(0.5);
        let json = serde_json::to_string(&value).expect("serialize");
        let back: AttributeValue = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(back, value);
    }

    #[test]
    fn text_round_trip() {
        let value = AttributeValue::Text("max_tracking_error".to_string());
        let json = serde_json::to_string(&value).expect("serialize");
        let back: AttributeValue = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(back, value);
    }

    #[test]
    fn bool_round_trip() {
        let value = AttributeValue::Bool(true);
        let json = serde_json::to_string(&value).expect("serialize");
        let back: AttributeValue = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(back, value);
    }

    #[test]
    fn integer_round_trip() {
        let value = AttributeValue::Integer(42);
        let json = serde_json::to_string(&value).expect("serialize");
        let back: AttributeValue = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(back, value);
    }

    #[test]
    fn serialization_is_deterministic() {
        let value = AttributeValue::Text("tracking".to_string());
        let first = serde_json::to_string(&value).expect("serialize");
        let second = serde_json::to_string(&value).expect("serialize");
        assert_eq!(first, second);
    }

    #[test]
    fn number_is_machine_readable_json_number_not_text() {
        // D5: values are typed data, never presentation strings.
        let value = AttributeValue::Number(2.5);
        let json = serde_json::to_value(&value).expect("to_value");
        assert_eq!(json, json!({"Number": 2.5}));
    }
}
