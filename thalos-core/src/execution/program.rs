use serde::{Deserialize, Serialize};
use std::time::Duration;

use crate::ids::OperationId;
use crate::motion::target::{MotionProfile, MotionTarget, OutputChannel, OutputValue};

// ---------------------------------------------------------------------------
// ExecutionInstruction — 4 variants forming the canonical IR-1 instruction set
// ---------------------------------------------------------------------------

/// A single instruction in an `ExecutionProgram`.
///
/// Exactly four variants exist in v1:
/// - `MoveJ`: joint-space movement to a target
/// - `MoveL`: linear (Cartesian) movement to a target
/// - `Delay`: wait for a duration
/// - `SetOutput`: set a digital/analog output channel
///
/// All variants carry an `origin: OperationId` linking back to the source IR
/// operation for traceability across the compiler → execution pipeline.
///
/// Serialized as an internally-tagged enum (`"type": "move_j"`, etc.) for
/// consistent JSON interchange across all consumers.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ExecutionInstruction {
    MoveJ {
        origin: OperationId,
        target: MotionTarget,
        profile: MotionProfile,
    },
    MoveL {
        origin: OperationId,
        target: MotionTarget,
        profile: MotionProfile,
    },
    Delay {
        origin: OperationId,
        duration: Duration,
    },
    SetOutput {
        origin: OperationId,
        channel: OutputChannel,
        value: OutputValue,
    },
}

// ---------------------------------------------------------------------------
// ExecutionMetadata — provenance metadata
// ---------------------------------------------------------------------------

/// Provenance metadata attached to every `ExecutionProgram`.
///
/// Exactly two fields: `schema_version` for format evolution detection, and
/// `source_project` for pipeline traceability. Timestamps and compiler build
/// metadata belong in a wrapping `CompilationRecord`, not here — keeping the
/// program deterministic for snapshot testing and caching.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExecutionMetadata {
    pub schema_version: u32,
    pub source_project: String,
}

// ---------------------------------------------------------------------------
// ExecutionProgram — the core execution program
// ---------------------------------------------------------------------------

/// A complete execution program — the bytecode of the platform.
///
/// Contains a linear `Vec<ExecutionInstruction>` and `ExecutionMetadata` for
/// provenance. Instructions are self-contained (no implicit state from prior
/// instructions). Order is preserved.
///
/// `ExecutionProgram` is the contract between lowering (compiler) and execution
/// (backends). Any backend can consume it without depending on the compiler.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExecutionProgram {
    pub instructions: Vec<ExecutionInstruction>,
    pub metadata: ExecutionMetadata,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::target::*;
    use std::time::Duration;

    /// Build a canonical 4-instruction sequence for order tests.
    fn sample_instructions() -> Vec<ExecutionInstruction> {
        vec![
            ExecutionInstruction::MoveJ {
                origin: OperationId("1".to_string()),
                target: MotionTarget::Pose(MotionPose {
                    position: [0.0, 0.0, 0.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    frame: "world".into(),
                }),
                profile: MotionProfile {
                    max_velocity: 500.0,
                    max_acceleration: 1000.0,
                    max_jerk: None,
                },
            },
            ExecutionInstruction::MoveL {
                origin: OperationId("2".to_string()),
                target: MotionTarget::Pose(MotionPose {
                    position: [1.0, 0.0, 0.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    frame: "world".into(),
                }),
                profile: MotionProfile {
                    max_velocity: 250.0,
                    max_acceleration: 500.0,
                    max_jerk: None,
                },
            },
            ExecutionInstruction::Delay {
                origin: OperationId("3".to_string()),
                duration: Duration::from_secs(2),
            },
            ExecutionInstruction::SetOutput {
                origin: OperationId("4".to_string()),
                channel: OutputChannel {
                    name: "gripper".into(),
                    channel_type: "digital".into(),
                },
                value: OutputValue::Bool(true),
            },
        ]
    }

    // ── Construction tests ───────────────────────────────────────────────

    #[test]
    fn empty_program_valid() {
        let program = ExecutionProgram {
            instructions: vec![],
            metadata: ExecutionMetadata {
                schema_version: 1,
                source_project: "test".into(),
            },
        };
        assert_eq!(program.instructions.len(), 0);
        assert_eq!(program.metadata.schema_version, 1);
        assert_eq!(program.metadata.source_project, "test");
    }

    #[test]
    fn empty_program_iterable() {
        let program = ExecutionProgram {
            instructions: vec![],
            metadata: ExecutionMetadata {
                schema_version: 1,
                source_project: "test".into(),
            },
        };
        let count = program.instructions.iter().count();
        assert_eq!(count, 0, "Empty program should yield zero items");
    }

    #[test]
    fn mixed_instructions_preserve_order() {
        let instructions = sample_instructions();
        let program = ExecutionProgram {
            instructions,
            metadata: ExecutionMetadata {
                schema_version: 1,
                source_project: "test".into(),
            },
        };

        assert_eq!(program.instructions.len(), 4);
        assert!(
            matches!(program.instructions[0], ExecutionInstruction::MoveJ { .. }),
            "First instruction should be MoveJ"
        );
        assert!(
            matches!(program.instructions[1], ExecutionInstruction::MoveL { .. }),
            "Second instruction should be MoveL"
        );
        assert!(
            matches!(program.instructions[2], ExecutionInstruction::Delay { .. }),
            "Third instruction should be Delay"
        );
        assert!(
            matches!(
                program.instructions[3],
                ExecutionInstruction::SetOutput { .. }
            ),
            "Fourth instruction should be SetOutput"
        );
    }

    #[test]
    fn metadata_construction() {
        let metadata = ExecutionMetadata {
            schema_version: 2,
            source_project: "thalos-demo".into(),
        };
        assert_eq!(metadata.schema_version, 2);
        assert_eq!(metadata.source_project, "thalos-demo");
    }

    // ── Variant construction ─────────────────────────────────────────────

    #[test]
    fn move_j_constructs_with_origin_and_fields() {
        let origin = OperationId("1".to_string());
        let target = MotionTarget::Pose(MotionPose {
            position: [1.0, 2.0, 3.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "world".into(),
        });
        let profile = MotionProfile {
            max_velocity: 500.0,
            max_acceleration: 1000.0,
            max_jerk: None,
        };

        let instr = ExecutionInstruction::MoveJ {
            origin,
            target: target.clone(),
            profile: profile.clone(),
        };

        match &instr {
            ExecutionInstruction::MoveJ {
                origin: o,
                target: t,
                profile: p,
            } => {
                assert_eq!(o, &OperationId("1".to_string()));
                assert_eq!(*t, target);
                assert_eq!(*p, profile);
            }
            _ => panic!("Expected MoveJ variant"),
        }
    }

    #[test]
    fn move_l_constructs_with_origin_and_fields() {
        let origin = OperationId("2".to_string());
        let target = MotionTarget::Pose(MotionPose {
            position: [4.0, 5.0, 6.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "tool0".into(),
        });
        let profile = MotionProfile {
            max_velocity: 250.0,
            max_acceleration: 500.0,
            max_jerk: Some(750.0),
        };

        let instr = ExecutionInstruction::MoveL {
            origin,
            target: target.clone(),
            profile: profile.clone(),
        };

        match &instr {
            ExecutionInstruction::MoveL {
                origin: o,
                target: t,
                profile: p,
            } => {
                assert_eq!(o, &OperationId("2".to_string()));
                assert_eq!(*t, target);
                assert_eq!(*p, profile);
            }
            _ => panic!("Expected MoveL variant"),
        }
    }

    #[test]
    fn delay_constructs_with_origin_and_duration() {
        let origin = OperationId("3".to_string());
        let duration = Duration::from_millis(1500);

        let instr = ExecutionInstruction::Delay { origin, duration };

        match &instr {
            ExecutionInstruction::Delay {
                origin: o,
                duration: d,
            } => {
                assert_eq!(o, &OperationId("3".to_string()));
                assert_eq!(*d, Duration::from_millis(1500));
            }
            _ => panic!("Expected Delay variant"),
        }
    }

    #[test]
    fn set_output_constructs_with_origin_channel_value() {
        let origin = OperationId("4".to_string());
        let channel = OutputChannel {
            name: "gripper".into(),
            channel_type: "digital".into(),
        };
        let value = OutputValue::Bool(true);

        let instr = ExecutionInstruction::SetOutput {
            origin,
            channel: channel.clone(),
            value: value.clone(),
        };

        match &instr {
            ExecutionInstruction::SetOutput {
                origin: o,
                channel: c,
                value: v,
            } => {
                assert_eq!(o, &OperationId("4".to_string()));
                assert_eq!(*c, channel);
                assert_eq!(*v, value);
            }
            _ => panic!("Expected SetOutput variant"),
        }
    }

    // ── Serde round-trip ─────────────────────────────────────────────────

    #[test]
    fn instruction_serde_round_trip_all_variants() {
        let instructions = vec![
            ExecutionInstruction::MoveJ {
                origin: OperationId("1".to_string()),
                target: MotionTarget::Pose(MotionPose {
                    position: [1.0, 0.0, 0.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    frame: "world".into(),
                }),
                profile: MotionProfile {
                    max_velocity: 100.0,
                    max_acceleration: 200.0,
                    max_jerk: None,
                },
            },
            ExecutionInstruction::MoveL {
                origin: OperationId("2".to_string()),
                target: MotionTarget::Pose(MotionPose {
                    position: [2.0, 0.0, 0.0],
                    orientation: [0.0, 0.0, 0.0, 1.0],
                    frame: "base".into(),
                }),
                profile: MotionProfile {
                    max_velocity: 300.0,
                    max_acceleration: 600.0,
                    max_jerk: Some(900.0),
                },
            },
            ExecutionInstruction::Delay {
                origin: OperationId("3".to_string()),
                duration: Duration::from_secs(5),
            },
            ExecutionInstruction::SetOutput {
                origin: OperationId("4".to_string()),
                channel: OutputChannel {
                    name: "vacuum".into(),
                    channel_type: "analog".into(),
                },
                value: OutputValue::Integer(42),
            },
        ];

        for instr in &instructions {
            let json = serde_json::to_string(instr).expect("serialize");
            let decoded: ExecutionInstruction = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(*instr, decoded, "round-trip failed for {instr:?}");
        }
    }

    #[test]
    fn program_serde_round_trip() {
        let program = ExecutionProgram {
            instructions: sample_instructions(),
            metadata: ExecutionMetadata {
                schema_version: 1,
                source_project: "test".into(),
            },
        };

        let json = serde_json::to_string(&program).expect("serialize");
        let decoded: ExecutionProgram = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(program, decoded);
    }

    #[test]
    fn serde_internally_tagged_type_tags() {
        let move_j = ExecutionInstruction::MoveJ {
            origin: OperationId("1".to_string()),
            target: MotionTarget::Pose(MotionPose {
                position: [0.0, 0.0, 0.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
                frame: "world".into(),
            }),
            profile: MotionProfile {
                max_velocity: 100.0,
                max_acceleration: 200.0,
                max_jerk: None,
            },
        };
        let json = serde_json::to_string(&move_j).expect("serialize");
        assert!(
            json.contains(r#""type":"move_j""#),
            "Expected type tag 'move_j', got: {json}"
        );

        let delay = ExecutionInstruction::Delay {
            origin: OperationId("2".to_string()),
            duration: Duration::from_secs(3),
        };
        let json = serde_json::to_string(&delay).expect("serialize");
        assert!(
            json.contains(r#""type":"delay""#),
            "Expected type tag 'delay', got: {json}"
        );

        let set_output = ExecutionInstruction::SetOutput {
            origin: OperationId("3".to_string()),
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        };
        let json = serde_json::to_string(&set_output).expect("serialize");
        assert!(
            json.contains(r#""type":"set_output""#),
            "Expected type tag 'set_output', got: {json}"
        );
    }

    #[test]
    fn serde_forward_compat_unknown_field() {
        let json = r#"{
            "type":"move_j",
            "origin":"1",
            "target":{"type":"pose","position":[0.0,0.0,0.0],"orientation":[0.0,0.0,0.0,1.0],"frame":"world"},
            "profile":{"max_velocity":100.0,"max_acceleration":200.0,"max_jerk":null},
            "unknown_field":"should_be_ignored"
        }"#;
        let result: Result<ExecutionInstruction, _> = serde_json::from_str(json);
        assert!(
            result.is_ok(),
            "Should tolerate unknown fields for forward compatibility"
        );
    }

    #[test]
    fn clone_and_eq_after_round_trip() {
        let original = ExecutionInstruction::MoveJ {
            origin: OperationId("42".to_string()),
            target: MotionTarget::Pose(MotionPose {
                position: [1.0, 2.0, 3.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
                frame: "world".into(),
            }),
            profile: MotionProfile {
                max_velocity: 100.0,
                max_acceleration: 200.0,
                max_jerk: None,
            },
        };

        // Clone
        let cloned = original.clone();
        assert_eq!(original, cloned, "Clone should produce equal value");

        // Round-trip
        let json = serde_json::to_string(&original).expect("serialize");
        let decoded: ExecutionInstruction = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(original, decoded, "Round-trip should preserve equality");

        // Clone after round-trip
        let decoded_clone = decoded.clone();
        assert_eq!(
            decoded, decoded_clone,
            "Clone after round-trip should preserve equality"
        );
    }

    #[test]
    fn metadata_serde_round_trip() {
        let metadata = ExecutionMetadata {
            schema_version: 3,
            source_project: "ci-test".into(),
        };
        let json = serde_json::to_string(&metadata).expect("serialize");
        let decoded: ExecutionMetadata = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(metadata, decoded);
    }
}
