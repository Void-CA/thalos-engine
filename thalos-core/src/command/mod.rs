//! Physical Command Semantics (ADR-019).
//!
//! `Command` is the semantic contract between Thalos's supervisory layer and
//! the physical resources it coordinates. Each command carries explicit semantics
//! about *what* is being requested and *who* assumes control responsibility.
//!
//! ```text
//! Intent (planning layer)
//!   ↓
//! Command (this module)
//!   ↓
//! Local Controller (PLC / MCU / robot controller)
//!   ↓
//! Actuation (physical process)
//! ```
//!
//! ## Design rules
//!
//! - `Command` is an **enum**, not a struct with generic payload. This prevents
//!   structurally invalid states (e.g. `semantics: Motion` + `payload: Bool`).
//! - Each variant owns its typed contract. No `Box<dyn Any>` or type erasure.
//! - `ResourceRef` is reused from `crate::resource`. No duplication.
//! - `MotionPose` is reused from `crate::motion::target`. No second version.
//! - `DirectActuation` is explicitly excluded from the default vocabulary (ADR-019 §11).
//! - `Intent` is a planning-layer concern, not a `Command` variant.

pub mod motion;
pub mod setpoint;
pub mod trigger;

pub use motion::{MotionCommand, MotionKind};
pub use setpoint::{SetpointCommand, SetpointValue};
pub use trigger::{TriggerCommand, TriggerValue};

use serde::{Deserialize, Serialize};

use crate::resource::ResourceRef;

// ---------------------------------------------------------------------------
// CommandSemantics — classification of physical interaction type
// ---------------------------------------------------------------------------

/// The semantic class of a physical command.
///
/// Determines what control responsibility the target resource assumes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandSemantics {
    /// Resource assumes trajectory generation and servo control.
    Motion,
    /// Resource assumes closed-loop regulation (PID, etc.).
    Setpoint,
    /// Discrete event; no persistent control loop involved.
    Trigger,
}

// ---------------------------------------------------------------------------
// Command — the unified physical command type
// ---------------------------------------------------------------------------

/// A semantically explicit command directed at a physical resource.
///
/// `Command` is the output of Thalos's supervisory decision layer and the input
/// to the Interconnection transport layer. It carries *what* is being requested,
/// not *how* the request is delivered.
///
/// # Three cases (ADR-019 §15)
///
/// | Case | Variant | Resource assumes |
/// |------|---------|------------------|
/// | Robot motion | `Motion` | Trajectory + servo |
/// | Temperature setpoint | `Setpoint` | PID regulation |
/// | Camera trigger | `Trigger` | Event execution |
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Command {
    Motion(MotionCommand),
    Setpoint(SetpointCommand),
    Trigger(TriggerCommand),
}

impl Command {
    /// The semantic class of this command.
    pub fn semantics(&self) -> CommandSemantics {
        match self {
            Command::Motion(_) => CommandSemantics::Motion,
            Command::Setpoint(_) => CommandSemantics::Setpoint,
            Command::Trigger(_) => CommandSemantics::Trigger,
        }
    }

    /// The target resource this command is directed at.
    pub fn target(&self) -> &ResourceRef {
        match self {
            Command::Motion(c) => &c.target,
            Command::Setpoint(c) => &c.target,
            Command::Trigger(c) => &c.target,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resource::ResourceKind;

    fn robot_ref(id: &str) -> ResourceRef {
        ResourceRef::new(id, ResourceKind::Robot)
    }

    fn device_ref(id: &str) -> ResourceRef {
        ResourceRef::new(id, ResourceKind::Device)
    }

    // ── Case A: Motion ────────────────────────────────────────────────

    #[test]
    fn motion_command_semantics() {
        let cmd = Command::Motion(MotionCommand {
            target: robot_ref("robot-01"),
            kind: MotionKind::Joint {
                positions_rad: vec![0.5, 0.2, -0.3],
                velocities_rad_s: None,
            },
            profile: None,
        });
        assert_eq!(cmd.semantics(), CommandSemantics::Motion);
        assert_eq!(cmd.target().id.as_str(), "robot-01");
    }

    #[test]
    fn motion_stop_semantics() {
        let cmd = Command::Motion(MotionCommand {
            target: robot_ref("robot-01"),
            kind: MotionKind::Stop,
            profile: None,
        });
        assert_eq!(cmd.semantics(), CommandSemantics::Motion);
    }

    // ── Case B: Setpoint ─────────────────────────────────────────────

    #[test]
    fn setpoint_command_semantics() {
        let cmd = Command::Setpoint(SetpointCommand {
            target: device_ref("heater-01"),
            variable: "temperature".into(),
            value: SetpointValue::Float(70.0),
        });
        assert_eq!(cmd.semantics(), CommandSemantics::Setpoint);
        assert_eq!(cmd.target().id.as_str(), "heater-01");
    }

    #[test]
    fn setpoint_integer_value() {
        let cmd = Command::Setpoint(SetpointCommand {
            target: device_ref("pump-01"),
            variable: "speed_rpm".into(),
            value: SetpointValue::Integer(1200),
        });
        assert_eq!(cmd.semantics(), CommandSemantics::Setpoint);
    }

    // ── Case C: Trigger ──────────────────────────────────────────────

    #[test]
    fn trigger_command_semantics() {
        let cmd = Command::Trigger(TriggerCommand {
            target: device_ref("camera-01"),
            channel: "capture".into(),
            value: TriggerValue::Bool(true),
        });
        assert_eq!(cmd.semantics(), CommandSemantics::Trigger);
        assert_eq!(cmd.target().id.as_str(), "camera-01");
    }

    // ── Serde round-trip ─────────────────────────────────────────────

    #[test]
    fn serde_round_trip_motion() {
        let cmd = Command::Motion(MotionCommand {
            target: robot_ref("robot-01"),
            kind: MotionKind::Joint {
                positions_rad: vec![1.0, 0.5, -0.2],
                velocities_rad_s: Some(vec![0.1, 0.1, 0.1]),
            },
            profile: None,
        });
        let json = serde_json::to_string(&cmd).expect("serialize");
        assert!(json.contains(r#""type":"motion""#));
        let back: Command = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(cmd, back);
    }

    #[test]
    fn serde_round_trip_setpoint() {
        let cmd = Command::Setpoint(SetpointCommand {
            target: device_ref("heater-01"),
            variable: "temperature".into(),
            value: SetpointValue::Float(70.0),
        });
        let json = serde_json::to_string(&cmd).expect("serialize");
        assert!(json.contains(r#""type":"setpoint""#));
        let back: Command = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(cmd, back);
    }

    #[test]
    fn serde_round_trip_trigger() {
        let cmd = Command::Trigger(TriggerCommand {
            target: device_ref("camera-01"),
            channel: "capture".into(),
            value: TriggerValue::Bool(true),
        });
        let json = serde_json::to_string(&cmd).expect("serialize");
        assert!(json.contains(r#""type":"trigger""#));
        let back: Command = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(cmd, back);
    }

    // ── Structural safety ────────────────────────────────────────────

    #[test]
    fn enum_prevents_invalid_semantics_payload_combination() {
        // With an enum, this state is structurally impossible:
        // Command { semantics: Motion, payload: Bool(true) }
        // Each variant enforces its own contract.
        let motion = Command::Motion(MotionCommand {
            target: robot_ref("r1"),
            kind: MotionKind::Stop,
            profile: None,
        });
        let setpoint = Command::Setpoint(SetpointCommand {
            target: device_ref("d1"),
            variable: "temp".into(),
            value: SetpointValue::Float(70.0),
        });
        assert_ne!(motion, setpoint);
        assert_eq!(motion.semantics(), CommandSemantics::Motion);
        assert_eq!(setpoint.semantics(), CommandSemantics::Setpoint);
    }
}
