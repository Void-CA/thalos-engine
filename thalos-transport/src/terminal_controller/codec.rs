//! Terminal Controller wire encoding (Fase 4.0 conformance).
//!
//! Pure encode/decode: no I/O, no Thalos domain types beyond the declared
//! capability/evidence vocabulary. See `docs/plans/13.0-external-provider-conformance.md`.

/// A negotiated capability descriptor as advertised by the endpoint.
///
/// This mirrors the `CAPABILITIES` frame **exactly**: what the resource *can
/// do*. It MUST NOT carry availability or mechanism (capability contract).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoteCapabilities {
    pub accepted: Vec<String>,
    pub observations: Vec<String>,
    pub max_evidence: String,
    pub target_resolution: String,
    pub joint_names: Vec<String>,
}

/// One decoded inbound frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TerminalFrame {
    Capabilities(RemoteCapabilities),
    Accepted { command_id: String },
    Rejected { command_id: String, reason: String },
    Started { command_id: String },
    Completed { command_id: String },
    Failed { command_id: String, reason: String },
    Aborted { command_id: String, reason: String },
    State { mode: String, progress: Option<String> },
}

/// A frame the endpoint could not parse.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FrameError(pub String);

impl std::fmt::Display for FrameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for FrameError {}

/// Encode the provider handshake.
pub fn encode_hello(contract_version: &str) -> String {
    format!("HELLO {contract_version}\n")
}

/// Encode a command frame: `COMMAND <id> <capability> <payload>`.
pub fn encode_command(command_id: &str, capability: &str, payload: &str) -> String {
    if payload.is_empty() {
        format!("COMMAND {command_id} {capability}\n")
    } else {
        format!("COMMAND {command_id} {capability} {payload}\n")
    }
}

/// Encode the provider stop frame.
pub fn encode_stop() -> String {
    "STOP\n".to_string()
}

/// Decode one inbound line (without the trailing newline).
pub fn decode(line: &str) -> Result<TerminalFrame, FrameError> {
    let line = line.trim();
    if line.is_empty() {
        return Err(FrameError("empty frame".into()));
    }

    let mut parts = line.split(' ');
    let verb = parts.next().unwrap_or_default();

    match verb {
        "CAPABILITIES" => decode_capabilities(parts.next().unwrap_or_default()),
        "ACCEPTED" => Ok(TerminalFrame::Accepted {
            command_id: required(parts.next(), "ACCEPTED command_id")?,
        }),
        "STARTED" => Ok(TerminalFrame::Started {
            command_id: required(parts.next(), "STARTED command_id")?,
        }),
        "COMPLETED" => Ok(TerminalFrame::Completed {
            command_id: required(parts.next(), "COMPLETED command_id")?,
        }),
        "REJECTED" => {
            let command_id = required(parts.next(), "REJECTED command_id")?;
            let reason = parts.collect::<Vec<_>>().join(" ");
            Ok(TerminalFrame::Rejected { command_id, reason })
        }
        "FAILED" => {
            let command_id = required(parts.next(), "FAILED command_id")?;
            let reason = parts.collect::<Vec<_>>().join(" ");
            Ok(TerminalFrame::Failed { command_id, reason })
        }
        "ABORTED" => {
            let command_id = required(parts.next(), "ABORTED command_id")?;
            let reason = parts.collect::<Vec<_>>().join(" ");
            Ok(TerminalFrame::Aborted { command_id, reason })
        }
        "STATE" => decode_state(parts.next().unwrap_or_default()),
        other => Err(FrameError(format!("unknown frame verb: {other}"))),
    }
}

fn required(value: Option<&str>, what: &str) -> Result<String, FrameError> {
    value
        .filter(|v| !v.is_empty())
        .map(str::to_string)
        .ok_or_else(|| FrameError(format!("missing {what}")))
}

/// Parse `accepted=..;observations=..;max_evidence=..;target_resolution=..[;joint_names=..]`.
fn decode_capabilities(body: &str) -> Result<TerminalFrame, FrameError> {
    let mut accepted = Vec::new();
    let mut observations = Vec::new();
    let mut max_evidence = String::new();
    let mut target_resolution = String::new();
    let mut joint_names = Vec::new();

    for field in body.split(';').filter(|f| !f.is_empty()) {
        let (key, value) = field
            .split_once('=')
            .ok_or_else(|| FrameError(format!("malformed capability field: {field}")))?;
        match key {
            "accepted" => accepted = split_csv(value),
            "observations" => observations = split_csv(value),
            "max_evidence" => max_evidence = value.to_string(),
            "target_resolution" => target_resolution = value.to_string(),
            "joint_names" => joint_names = split_csv(value),
            other => {
                // Unknown fields are tolerated (additive minor changes).
                let _ = other;
            }
        }
    }

    if max_evidence.is_empty() {
        return Err(FrameError("CAPABILITIES missing max_evidence".into()));
    }

    Ok(TerminalFrame::Capabilities(RemoteCapabilities {
        accepted,
        observations,
        max_evidence,
        target_resolution: if target_resolution.is_empty() {
            "host".to_string()
        } else {
            target_resolution
        },
        joint_names,
    }))
}

/// Parse `mode=..[;progress=..]`.
fn decode_state(body: &str) -> Result<TerminalFrame, FrameError> {
    let mut mode = String::new();
    let mut progress = None;
    for field in body.split(';').filter(|f| !f.is_empty()) {
        let (key, value) = field
            .split_once('=')
            .ok_or_else(|| FrameError(format!("malformed state field: {field}")))?;
        match key {
            "mode" => mode = value.to_string(),
            "progress" => progress = Some(value.to_string()),
            _ => {}
        }
    }
    if mode.is_empty() {
        return Err(FrameError("STATE missing mode".into()));
    }
    Ok(TerminalFrame::State { mode, progress })
}

fn split_csv(value: &str) -> Vec<String> {
    value
        .split(',')
        .filter(|v| !v.is_empty())
        .map(str::to_string)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_profile_b_capabilities() {
        let frame = decode(
            "CAPABILITIES accepted=move_j;observations=joint_position;max_evidence=E1;target_resolution=host",
        )
        .unwrap();
        match frame {
            TerminalFrame::Capabilities(c) => {
                assert_eq!(c.accepted, vec!["move_j"]);
                assert_eq!(c.observations, vec!["joint_position"]);
                assert_eq!(c.max_evidence, "E1");
                assert_eq!(c.target_resolution, "host");
            }
            other => panic!("expected CAPABILITIES, got {other:?}"),
        }
    }

    #[test]
    fn decodes_profile_a_capabilities() {
        let frame = decode(
            "CAPABILITIES accepted=move_j,move_l;observations=joint_position;max_evidence=E2;target_resolution=host",
        )
        .unwrap();
        match frame {
            TerminalFrame::Capabilities(c) => {
                assert_eq!(c.accepted, vec!["move_j", "move_l"]);
                assert_eq!(c.max_evidence, "E2");
            }
            other => panic!("expected CAPABILITIES, got {other:?}"),
        }
    }

    #[test]
    fn tolerates_unknown_additive_fields() {
        let frame = decode(
            "CAPABILITIES accepted=move_j;observations=;max_evidence=E1;future_field=whatever",
        )
        .unwrap();
        assert!(matches!(frame, TerminalFrame::Capabilities(_)));
    }

    #[test]
    fn decodes_lifecycle_frames() {
        assert_eq!(
            decode("ACCEPTED c1").unwrap(),
            TerminalFrame::Accepted { command_id: "c1".into() }
        );
        assert_eq!(
            decode("STARTED c1").unwrap(),
            TerminalFrame::Started { command_id: "c1".into() }
        );
        assert_eq!(
            decode("COMPLETED c1").unwrap(),
            TerminalFrame::Completed { command_id: "c1".into() }
        );
        assert_eq!(
            decode("REJECTED c1 UNSUPPORTED_CAPABILITY").unwrap(),
            TerminalFrame::Rejected { command_id: "c1".into(), reason: "UNSUPPORTED_CAPABILITY".into() }
        );
    }

    #[test]
    fn does_not_carry_availability_or_mechanism() {
        // The decoded descriptor exposes only what the endpoint can do.
        let frame = decode(
            "CAPABILITIES accepted=move_j;observations=joint_position;max_evidence=E1;target_resolution=host",
        )
        .unwrap();
        let TerminalFrame::Capabilities(c) = frame else { panic!() };
        // No availability / mechanism field exists in the type at all.
        assert_eq!(c.accepted, vec!["move_j"]);
    }

    #[test]
    fn encodes_hello_and_command() {
        assert_eq!(encode_hello("1.0"), "HELLO 1.0\n");
        assert_eq!(
            encode_command("c1", "move_j", "joints=0.1,0.2"),
            "COMMAND c1 move_j joints=0.1,0.2\n"
        );
        assert_eq!(encode_command("c2", "move_j", ""), "COMMAND c2 move_j\n");
        assert_eq!(encode_stop(), "STOP\n");
    }

    #[test]
    fn rejects_unknown_verb() {
        assert!(decode("NOPE x").is_err());
    }
}
