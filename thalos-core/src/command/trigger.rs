//! Trigger command — discrete event request with no persistent control target.
//!
//! A trigger represents an event-like interaction: camera capture, gripper
//! actuation, valve toggle, conveyor stop. No persistent control loop is
//! involved — the resource executes the event and may acknowledge completion.
//!
//! ```text
//! Thalos → Trigger(camera, capture) → Camera → capture() → ACK
//! ```
//!
//! ## Semantics (ADR-019 §3)
//!
//! `Trigger` is **not** a setpoint. The value represents an event payload,
//! not a persistent operating value:
//!
//! - `camera.capture = true` means "emit the capture event", not
//!   "hold capture at value true".
//! - `gripper.close = true` means "execute close action", not
//!   "maintain gripper at closed state".

use serde::{Deserialize, Serialize};

use crate::resource::ResourceRef;

/// A command requesting a discrete event execution by a resource.
///
/// The target resource assumes responsibility for:
/// - Executing the event
/// - Acknowledging completion (future)
///
/// No persistent control loop is involved.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TriggerCommand {
    pub target: ResourceRef,
    /// The channel or event name (e.g. "capture", "close", "stop").
    pub channel: String,
    /// Event payload. Represents the event parameter, not a persistent value.
    pub value: TriggerValue,
}

/// A typed value for trigger commands.
///
/// Represents event parameters, not operating values. `Bool` is the
/// primary variant for MVP; `Integer` supports sequenced triggers or
/// parameterized events.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum TriggerValue {
    Bool(bool),
    Integer(i32),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resource::ResourceKind;

    #[test]
    fn camera_capture_trigger() {
        let cmd = TriggerCommand {
            target: ResourceRef::new("camera-01", ResourceKind::Device),
            channel: "capture".into(),
            value: TriggerValue::Bool(true),
        };
        assert_eq!(cmd.target.id.as_str(), "camera-01");
        assert_eq!(cmd.channel, "capture");
        assert_eq!(cmd.value, TriggerValue::Bool(true));
    }

    #[test]
    fn gripper_close_trigger() {
        let cmd = TriggerCommand {
            target: ResourceRef::new("gripper-01", ResourceKind::Device),
            channel: "close".into(),
            value: TriggerValue::Bool(true),
        };
        assert_eq!(cmd.value, TriggerValue::Bool(true));
    }

    #[test]
    fn sequenced_trigger_integer() {
        let cmd = TriggerCommand {
            target: ResourceRef::new("conveyor-01", ResourceKind::Device),
            channel: "pulse".into(),
            value: TriggerValue::Integer(3),
        };
        assert_eq!(cmd.value, TriggerValue::Integer(3));
    }

    #[test]
    fn serde_round_trip() {
        let cases = vec![
            TriggerCommand {
                target: ResourceRef::new("c1", ResourceKind::Device),
                channel: "capture".into(),
                value: TriggerValue::Bool(true),
            },
            TriggerCommand {
                target: ResourceRef::new("c1", ResourceKind::Device),
                channel: "pulse".into(),
                value: TriggerValue::Integer(5),
            },
        ];
        for cmd in cases {
            let json = serde_json::to_string(&cmd).expect("serialize");
            let back: TriggerCommand = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(cmd, back);
        }
    }
}
