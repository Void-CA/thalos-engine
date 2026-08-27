use std::time::Duration;

use serde::{Deserialize, Serialize};

use crate::ids::OperationId;
use crate::motion::target::{OutputChannel, OutputValue};

// ---------------------------------------------------------------------------
// RuntimeAction — what the runtime should do (non-planifiable actions)
// ---------------------------------------------------------------------------

/// A runtime action that cannot be planned geometrically.
///
/// Two variants exist in v1:
/// - `Delay`: wait for a specified duration
/// - `SetOutput`: set a digital/analog output channel to a value
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RuntimeAction {
    Delay(Duration),
    SetOutput {
        channel: OutputChannel,
        value: OutputValue,
    },
}

// ---------------------------------------------------------------------------
// RuntimeEvent — an event in a RuntimeProgram with operation-level tracing
// ---------------------------------------------------------------------------

/// A single event in a `RuntimeProgram`, linked back to the originating
/// operation via `operation_id`.
///
/// `at_time` is the absolute time from plan start (`t=0` = plan start), the
/// same timeline origin as `CompiledPlan`. It is assigned by the
/// `TimelineScheduler` (logical → temporal post-pass), never by the resolver.
/// `Duration` serializes with serde's default `{secs,nanos}` shape — the same
/// convention as `ProgramInstruction` (Q1).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RuntimeEvent {
    /// Absolute time from plan start at which this event fires.
    pub at_time: Duration,
    pub operation_id: OperationId,
    pub action: RuntimeAction,
}

// ---------------------------------------------------------------------------
// RuntimeProgram — the complete set of runtime events for one execution
// ---------------------------------------------------------------------------

/// The complete set of runtime events for one execution.
///
/// Contains a linear `Vec<RuntimeEvent>` sorted by `at_time` (invariant:
/// events fire in absolute time order). The runtime interprets this
/// alongside the `CompiledPlan` to produce the final execution timeline.
/// `RuntimeProgram` and `CompiledPlan` share the absolute timeline origin
/// (`t=0` = plan start) but remain separate artifacts (I5) — linkage is by
/// temporal query, never stored references.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RuntimeProgram {
    pub events: Vec<RuntimeEvent>,
}

impl RuntimeProgram {
    /// Construct a program, sorting events by absolute `at_time`.
    ///
    /// Events SHALL be sorted by `at_time` (spec: RuntimeProgram Structure).
    pub fn new(mut events: Vec<RuntimeEvent>) -> Self {
        events.sort_by_key(|e| e.at_time);
        Self { events }
    }
}

impl Default for RuntimeProgram {
    fn default() -> Self {
        Self { events: Vec::new() }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::target::*;
    use serde_json;

    fn sample_set_output_event() -> RuntimeEvent {
        RuntimeEvent {
            at_time: Duration::from_millis(1500),
            operation_id: OperationId("op-3".to_string()),
            action: RuntimeAction::SetOutput {
                channel: OutputChannel {
                    name: "gripper".into(),
                    channel_type: "digital".into(),
                },
                value: OutputValue::Bool(true),
            },
        }
    }

    // ── RuntimeEvent serde round-trip (rt) ───────────────────────────────

    #[test]
    fn runtime_event_serde_round_trip() {
        let event = sample_set_output_event();

        let json = serde_json::to_string(&event).expect("serialize");
        let decoded: RuntimeEvent = serde_json::from_str(&json).expect("deserialize");

        assert_eq!(event, decoded, "round-trip must be lossless");
    }

    #[test]
    fn runtime_event_serde_round_trip_delay_variant() {
        let event = RuntimeEvent {
            at_time: Duration::from_secs(2),
            operation_id: OperationId("op-wait".to_string()),
            action: RuntimeAction::Delay(Duration::from_millis(750)),
        };

        let json = serde_json::to_string(&event).expect("serialize");
        let decoded: RuntimeEvent = serde_json::from_str(&json).expect("deserialize");

        assert_eq!(event, decoded, "Delay variant must round-trip losslessly");
        match decoded.action {
            RuntimeAction::Delay(d) => assert_eq!(d, Duration::from_millis(750)),
            _ => panic!("expected Delay"),
        }
    }

    #[test]
    fn runtime_event_serde_uses_default_duration_shape() {
        // D1: Duration serializes with serde's default `{secs,nanos}` shape —
        // consistent with ProgramInstruction (Q1). No humantime_serde.
        let event = sample_set_output_event();
        let json = serde_json::to_string(&event).expect("serialize");
        assert!(
            json.contains("\"at_time\":{\"secs\":1,\"nanos\":500000000}"),
            "at_time must serialize as {{secs,nanos}}: {json}"
        );
    }

    #[test]
    fn runtime_action_serde_round_trip() {
        let delay = RuntimeAction::Delay(Duration::from_millis(500));
        let json = serde_json::to_string(&delay).expect("serialize");
        let decoded: RuntimeAction = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(delay, decoded);

        let output = sample_set_output_event().action;
        let json = serde_json::to_string(&output).expect("serialize");
        let decoded: RuntimeAction = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(output, decoded);
    }

    #[test]
    fn runtime_program_serde_round_trip() {
        let program = RuntimeProgram::new(vec![
            sample_set_output_event(),
            RuntimeEvent {
                at_time: Duration::from_secs(1),
                operation_id: OperationId("op-1".to_string()),
                action: RuntimeAction::Delay(Duration::from_millis(250)),
            },
        ]);

        let json = serde_json::to_string(&program).expect("serialize");
        let decoded: RuntimeProgram = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(
            program, decoded,
            "RuntimeProgram must round-trip losslessly"
        );
    }

    // ── at_time absolute semantics (rt) ──────────────────────────────────

    #[test]
    fn at_time_is_absolute_from_plan_start() {
        // Spec: a SetOutput scheduled 2.5s after plan start MUST carry
        // at_time == Duration::from_millis(2500).
        let event = RuntimeEvent {
            at_time: Duration::from_millis(2500),
            operation_id: OperationId("op-x".to_string()),
            action: RuntimeAction::SetOutput {
                channel: OutputChannel {
                    name: "vacuum".into(),
                    channel_type: "digital".into(),
                },
                value: OutputValue::Bool(true),
            },
        };
        assert_eq!(event.at_time, Duration::from_millis(2500));
    }

    #[test]
    fn runtime_program_sorts_events_by_at_time() {
        // Spec: events at 3.0s, 1.0s, 2.0s SHALL be ordered 1.0s, 2.0s, 3.0s
        // when constructed via `RuntimeProgram::new`.
        let mk = |secs: u64, op: &str| RuntimeEvent {
            at_time: Duration::from_secs(secs),
            operation_id: OperationId(op.to_string()),
            action: RuntimeAction::Delay(Duration::ZERO),
        };
        let program = RuntimeProgram::new(vec![mk(3, "c"), mk(1, "a"), mk(2, "b")]);

        let times: Vec<Duration> = program.events.iter().map(|e| e.at_time).collect();
        assert_eq!(
            times,
            vec![
                Duration::from_secs(1),
                Duration::from_secs(2),
                Duration::from_secs(3)
            ]
        );
        assert_eq!(program.events[0].operation_id, OperationId("a".to_string()));
        assert_eq!(program.events[2].operation_id, OperationId("c".to_string()));
    }

    // ── RuntimeAction construction ───────────────────────────────────────

    #[test]
    fn delay_action_constructs_with_duration() {
        let action = RuntimeAction::Delay(Duration::from_secs(5));
        match &action {
            RuntimeAction::Delay(d) => assert_eq!(*d, Duration::from_secs(5)),
            _ => panic!("Expected Delay variant"),
        }
    }

    #[test]
    fn delay_action_milliseconds() {
        let action = RuntimeAction::Delay(Duration::from_millis(1500));
        match &action {
            RuntimeAction::Delay(d) => assert_eq!(*d, Duration::from_millis(1500)),
            _ => panic!("Expected Delay variant"),
        }
    }

    #[test]
    fn set_output_action_constructs_with_channel_and_value() {
        let channel = OutputChannel {
            name: "gripper".into(),
            channel_type: "digital".into(),
        };
        let value = OutputValue::Bool(true);

        let action = RuntimeAction::SetOutput {
            channel: channel.clone(),
            value: value.clone(),
        };

        match &action {
            RuntimeAction::SetOutput {
                channel: c,
                value: v,
            } => {
                assert_eq!(*c, channel);
                assert_eq!(*v, value);
            }
            _ => panic!("Expected SetOutput variant"),
        }
    }

    #[test]
    fn set_output_analog_value() {
        let channel = OutputChannel {
            name: "vacuum".into(),
            channel_type: "analog".into(),
        };
        let value = OutputValue::Integer(75);

        let action = RuntimeAction::SetOutput { channel, value };
        match &action {
            RuntimeAction::SetOutput {
                channel: c,
                value: v,
            } => {
                assert_eq!(c.name, "vacuum");
                assert_eq!(*v, OutputValue::Integer(75));
            }
            _ => panic!("Expected SetOutput variant"),
        }
    }

    // ── RuntimeAction equality ───────────────────────────────────────────

    #[test]
    fn delay_actions_equal() {
        let a = RuntimeAction::Delay(Duration::from_secs(3));
        let b = RuntimeAction::Delay(Duration::from_secs(3));
        assert_eq!(a, b);
    }

    #[test]
    fn delay_actions_inequal() {
        let a = RuntimeAction::Delay(Duration::from_secs(3));
        let b = RuntimeAction::Delay(Duration::from_secs(5));
        assert_ne!(a, b);
    }

    #[test]
    fn set_output_actions_equal() {
        let a = RuntimeAction::SetOutput {
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        };
        let b = RuntimeAction::SetOutput {
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        };
        assert_eq!(a, b);
    }

    #[test]
    fn set_output_actions_inequal_channel() {
        let a = RuntimeAction::SetOutput {
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        };
        let b = RuntimeAction::SetOutput {
            channel: OutputChannel {
                name: "vacuum".into(),
                channel_type: "analog".into(),
            },
            value: OutputValue::Bool(true),
        };
        assert_ne!(a, b);
    }

    #[test]
    fn different_variants_not_equal() {
        let delay = RuntimeAction::Delay(Duration::from_secs(1));
        let output = RuntimeAction::SetOutput {
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        };
        assert_ne!(delay, output);
    }

    // ── RuntimeEvent construction ────────────────────────────────────────

    #[test]
    fn runtime_event_holds_operation_id_and_action() {
        let op_id = OperationId("op-42".to_string());
        let action = RuntimeAction::Delay(Duration::from_secs(2));

        let event = RuntimeEvent {
            at_time: Duration::ZERO,
            operation_id: op_id.clone(),
            action,
        };

        assert_eq!(event.operation_id, OperationId("op-42".to_string()));
        match &event.action {
            RuntimeAction::Delay(d) => assert_eq!(*d, Duration::from_secs(2)),
            _ => panic!("Expected Delay"),
        }
    }

    #[test]
    fn runtime_event_set_output_with_origin() {
        let op_id = OperationId("set-1".to_string());
        let event = RuntimeEvent {
            at_time: Duration::ZERO,
            operation_id: op_id.clone(),
            action: RuntimeAction::SetOutput {
                channel: OutputChannel {
                    name: "gripper".into(),
                    channel_type: "digital".into(),
                },
                value: OutputValue::Bool(false),
            },
        };

        assert_eq!(event.operation_id, OperationId("set-1".to_string()));
        match &event.action {
            RuntimeAction::SetOutput { value, .. } => {
                assert_eq!(*value, OutputValue::Bool(false));
            }
            _ => panic!("Expected SetOutput"),
        }
    }

    #[test]
    fn runtime_event_equality() {
        let a = RuntimeEvent {
            at_time: Duration::ZERO,
            operation_id: OperationId("evt-1".to_string()),
            action: RuntimeAction::Delay(Duration::from_secs(3)),
        };
        let b = RuntimeEvent {
            at_time: Duration::ZERO,
            operation_id: OperationId("evt-1".to_string()),
            action: RuntimeAction::Delay(Duration::from_secs(3)),
        };
        assert_eq!(a, b);
    }

    #[test]
    fn runtime_event_inequality_operation_id() {
        let a = RuntimeEvent {
            at_time: Duration::ZERO,
            operation_id: OperationId("evt-1".to_string()),
            action: RuntimeAction::Delay(Duration::from_secs(3)),
        };
        let b = RuntimeEvent {
            at_time: Duration::ZERO,
            operation_id: OperationId("evt-2".to_string()),
            action: RuntimeAction::Delay(Duration::from_secs(3)),
        };
        assert_ne!(a, b);
    }

    // ── RuntimeProgram construction ──────────────────────────────────────

    #[test]
    fn runtime_program_empty_valid() {
        let program = RuntimeProgram { events: vec![] };
        assert_eq!(program.events.len(), 0);
    }

    #[test]
    fn runtime_program_empty_iterable() {
        let program = RuntimeProgram { events: vec![] };
        assert_eq!(program.events.iter().count(), 0);
    }

    #[test]
    fn runtime_program_with_multiple_events() {
        let program = RuntimeProgram {
            events: vec![
                RuntimeEvent {
                    at_time: Duration::ZERO,
                    operation_id: OperationId("op-1".to_string()),
                    action: RuntimeAction::Delay(Duration::from_secs(2)),
                },
                RuntimeEvent {
                    at_time: Duration::ZERO,
                    operation_id: OperationId("op-2".to_string()),
                    action: RuntimeAction::SetOutput {
                        channel: OutputChannel {
                            name: "gripper".into(),
                            channel_type: "digital".into(),
                        },
                        value: OutputValue::Bool(true),
                    },
                },
            ],
        };

        assert_eq!(program.events.len(), 2);
        assert!(
            matches!(program.events[0].action, RuntimeAction::Delay(_)),
            "First event should be Delay"
        );
        assert!(
            matches!(program.events[1].action, RuntimeAction::SetOutput { .. }),
            "Second event should be SetOutput"
        );
    }

    #[test]
    fn runtime_program_order_preserved() {
        let program = RuntimeProgram {
            events: vec![
                RuntimeEvent {
                    at_time: Duration::ZERO,
                    operation_id: OperationId("first".to_string()),
                    action: RuntimeAction::Delay(Duration::from_secs(1)),
                },
                RuntimeEvent {
                    at_time: Duration::ZERO,
                    operation_id: OperationId("second".to_string()),
                    action: RuntimeAction::Delay(Duration::from_secs(2)),
                },
                RuntimeEvent {
                    at_time: Duration::ZERO,
                    operation_id: OperationId("third".to_string()),
                    action: RuntimeAction::Delay(Duration::from_secs(3)),
                },
            ],
        };

        let ids: Vec<&str> = program
            .events
            .iter()
            .map(|e| e.operation_id.as_str())
            .collect();
        assert_eq!(ids, vec!["first", "second", "third"]);
    }

    #[test]
    fn runtime_program_clone_and_eq() {
        let program = RuntimeProgram {
            events: vec![RuntimeEvent {
                at_time: Duration::ZERO,
                operation_id: OperationId("op-1".to_string()),
                action: RuntimeAction::Delay(Duration::from_secs(5)),
            }],
        };

        let cloned = program.clone();
        assert_eq!(program, cloned);
    }
}
