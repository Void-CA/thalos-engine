use serde::{Deserialize, Serialize};
use std::time::Duration;
use thalos_core::ids::OperationId;

use crate::resource::{LocationId, ObjectId, ToolId};

/// Parameters for a Pick operation: grasp an object, optionally with a specific tool.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PickOp {
    pub origin: OperationId,
    pub object: ObjectId,
    pub tool: Option<ToolId>,
}

/// Parameters for a Place operation: release a held object at a destination location.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlaceOp {
    pub origin: OperationId,
    pub object: ObjectId,
    pub destination: LocationId,
    pub tool: Option<ToolId>,
}

/// Parameters for a MoveTo operation: navigate to a location, optionally with a tool.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MoveToOp {
    pub origin: OperationId,
    pub destination: LocationId,
    pub tool: Option<ToolId>,
}

/// Parameters for a Wait operation: pause execution for a duration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WaitOp {
    pub origin: OperationId,
    pub duration: Duration,
}

/// Parameters for a Home operation: return to home position.
///
/// No parameters beyond origin — Home is always a parameterless return to
/// the configured home pose.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HomeOp {
    pub origin: OperationId,
}

/// A semantic operation in a task-level program.
///
/// Exactly five variants exist in v1:
/// - `Pick`: grasp an object
/// - `Place`: release an object at a location
/// - `MoveTo`: navigate to a location
/// - `Wait`: pause for a duration
/// - `Home`: return to home position
///
/// All variants carry an `origin: OperationId` for traceability.
/// Serialized as an internally-tagged enum for consistent JSON interchange.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum SemanticOperation {
    Pick(PickOp),
    Place(PlaceOp),
    MoveTo(MoveToOp),
    Wait(WaitOp),
    Home(HomeOp),
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_origin() -> OperationId {
        OperationId("op-7".to_string())
    }

    fn sample_object() -> ObjectId {
        ObjectId("bolt-1".to_string())
    }

    fn sample_location() -> LocationId {
        LocationId("shelf-a".to_string())
    }

    fn sample_tool() -> ToolId {
        ToolId("gripper-1".to_string())
    }

    // ── PickOp ────────────────────────────────────────────────────────────

    #[test]
    fn pick_op_constructs_with_origin_and_object() {
        let op = PickOp {
            origin: sample_origin(),
            object: sample_object(),
            tool: None,
        };
        assert_eq!(op.origin, sample_origin());
        assert_eq!(op.object, sample_object());
        assert!(op.tool.is_none());
    }

    #[test]
    fn pick_op_with_optional_tool() {
        let op = PickOp {
            origin: sample_origin(),
            object: sample_object(),
            tool: Some(sample_tool()),
        };
        assert_eq!(op.tool, Some(sample_tool()));
    }

    #[test]
    fn pick_op_origin_propagates() {
        let origin = OperationId("pick-42".to_string());
        let op = PickOp {
            origin: origin.clone(),
            object: sample_object(),
            tool: None,
        };
        let PickOp { origin: o, .. } = &op;
        assert_eq!(*o, origin);
    }

    #[test]
    fn pick_op_no_extra_fields() {
        let op = PickOp {
            origin: sample_origin(),
            object: sample_object(),
            tool: None,
        };
        // Only `origin`, `object`, `tool` — no pose, position, or geometry
        let PickOp {
            origin: _,
            object: _,
            tool: _,
        } = op;
    }

    // ── PlaceOp ───────────────────────────────────────────────────────────

    #[test]
    fn place_op_constructs_with_all_fields() {
        let op = PlaceOp {
            origin: sample_origin(),
            object: sample_object(),
            destination: sample_location(),
            tool: None,
        };
        assert_eq!(op.origin, sample_origin());
        assert_eq!(op.object, sample_object());
        assert_eq!(op.destination, sample_location());
        assert!(op.tool.is_none());
    }

    #[test]
    fn place_op_with_tool() {
        let op = PlaceOp {
            origin: sample_origin(),
            object: sample_object(),
            destination: sample_location(),
            tool: Some(sample_tool()),
        };
        assert_eq!(op.tool, Some(sample_tool()));
    }

    // ── MoveToOp ──────────────────────────────────────────────────────────

    #[test]
    fn move_to_op_constructs_with_origin_and_location() {
        let op = MoveToOp {
            origin: sample_origin(),
            destination: sample_location(),
            tool: None,
        };
        assert_eq!(op.origin, sample_origin());
        assert_eq!(op.destination, sample_location());
        assert!(op.tool.is_none());
    }

    #[test]
    fn move_to_op_with_tool() {
        let op = MoveToOp {
            origin: sample_origin(),
            destination: sample_location(),
            tool: Some(sample_tool()),
        };
        assert_eq!(op.tool, Some(sample_tool()));
    }

    #[test]
    fn move_to_op_no_object_field() {
        let op = MoveToOp {
            origin: sample_origin(),
            destination: sample_location(),
            tool: None,
        };
        // MoveToOp has `origin`, `destination`, `tool` — no `object` field
        let MoveToOp {
            origin: _,
            destination: _,
            tool: _,
        } = op;
    }

    // ── WaitOp ────────────────────────────────────────────────────────────

    #[test]
    fn wait_op_constructs_with_duration() {
        let op = WaitOp {
            origin: sample_origin(),
            duration: Duration::from_secs(5),
        };
        assert_eq!(op.origin, sample_origin());
        assert_eq!(op.duration, Duration::from_secs(5));
    }

    #[test]
    fn wait_op_zero_duration() {
        let op = WaitOp {
            origin: sample_origin(),
            duration: Duration::ZERO,
        };
        assert_eq!(op.duration, Duration::ZERO);
    }

    #[test]
    fn wait_op_origin_propagates() {
        let origin = OperationId("wait-99".to_string());
        let op = WaitOp {
            origin: origin.clone(),
            duration: Duration::from_secs(2),
        };
        let WaitOp { origin: o, .. } = &op;
        assert_eq!(*o, origin);
    }

    #[test]
    fn wait_op_no_optional_fields() {
        // WaitOp only has `origin` and `duration` — no tool, no location
        let op = WaitOp {
            origin: sample_origin(),
            duration: Duration::from_secs(1),
        };
        let WaitOp {
            origin: _,
            duration: _,
        } = op;
    }

    // ── HomeOp ────────────────────────────────────────────────────────────

    #[test]
    fn home_op_has_only_origin() {
        let op = HomeOp {
            origin: sample_origin(),
        };
        assert_eq!(op.origin, sample_origin());
    }

    #[test]
    fn home_op_no_optional_fields() {
        // HomeOp has only `origin` — no object, location, tool, or duration
        let op = HomeOp {
            origin: sample_origin(),
        };
        let HomeOp { origin: o } = &op;
        assert_eq!(o, &sample_origin());
    }

    // ── SemanticOperation enum ────────────────────────────────────────────

    #[test]
    fn operation_pick_variant() {
        let op = SemanticOperation::Pick(PickOp {
            origin: sample_origin(),
            object: sample_object(),
            tool: None,
        });
        match &op {
            SemanticOperation::Pick(p) => {
                assert_eq!(p.object, sample_object());
            }
            _ => panic!("Expected Pick variant"),
        }
    }

    #[test]
    fn operation_place_variant() {
        let op = SemanticOperation::Place(PlaceOp {
            origin: sample_origin(),
            object: sample_object(),
            destination: sample_location(),
            tool: None,
        });
        match &op {
            SemanticOperation::Place(p) => {
                assert_eq!(p.destination, sample_location());
            }
            _ => panic!("Expected Place variant"),
        }
    }

    #[test]
    fn operation_move_to_variant() {
        let op = SemanticOperation::MoveTo(MoveToOp {
            origin: sample_origin(),
            destination: sample_location(),
            tool: None,
        });
        match &op {
            SemanticOperation::MoveTo(m) => {
                assert_eq!(m.destination, sample_location());
            }
            _ => panic!("Expected MoveTo variant"),
        }
    }

    #[test]
    fn operation_wait_variant() {
        let op = SemanticOperation::Wait(WaitOp {
            origin: sample_origin(),
            duration: Duration::from_secs(3),
        });
        match &op {
            SemanticOperation::Wait(w) => {
                assert_eq!(w.duration, Duration::from_secs(3));
            }
            _ => panic!("Expected Wait variant"),
        }
    }

    #[test]
    fn operation_home_variant() {
        let op = SemanticOperation::Home(HomeOp {
            origin: sample_origin(),
        });
        match &op {
            SemanticOperation::Home(h) => {
                assert_eq!(h.origin, sample_origin());
            }
            _ => panic!("Expected Home variant"),
        }
    }

    // ── Serde round-trip per variant ──────────────────────────────────────

    #[test]
    fn pick_op_serde_round_trip() {
        let op = SemanticOperation::Pick(PickOp {
            origin: sample_origin(),
            object: sample_object(),
            tool: Some(sample_tool()),
        });
        let json = serde_json::to_string(&op).expect("serialize");
        let decoded: SemanticOperation = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(op, decoded);
    }

    #[test]
    fn place_op_serde_round_trip() {
        let op = SemanticOperation::Place(PlaceOp {
            origin: sample_origin(),
            object: sample_object(),
            destination: sample_location(),
            tool: None,
        });
        let json = serde_json::to_string(&op).expect("serialize");
        let decoded: SemanticOperation = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(op, decoded);
    }

    #[test]
    fn move_to_op_serde_round_trip() {
        let op = SemanticOperation::MoveTo(MoveToOp {
            origin: sample_origin(),
            destination: sample_location(),
            tool: None,
        });
        let json = serde_json::to_string(&op).expect("serialize");
        let decoded: SemanticOperation = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(op, decoded);
    }

    #[test]
    fn wait_op_serde_round_trip() {
        let op = SemanticOperation::Wait(WaitOp {
            origin: sample_origin(),
            duration: Duration::from_secs(10),
        });
        let json = serde_json::to_string(&op).expect("serialize");
        let decoded: SemanticOperation = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(op, decoded);
    }

    #[test]
    fn home_op_serde_round_trip() {
        let op = SemanticOperation::Home(HomeOp {
            origin: sample_origin(),
        });
        let json = serde_json::to_string(&op).expect("serialize");
        let decoded: SemanticOperation = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(op, decoded);
    }

    #[test]
    fn serde_type_tags() {
        let pick = SemanticOperation::Pick(PickOp {
            origin: sample_origin(),
            object: sample_object(),
            tool: None,
        });
        let json = serde_json::to_string(&pick).expect("serialize");
        assert!(
            json.contains(r#""type":"pick""#),
            "Expected 'pick' type tag, got: {json}"
        );

        let home = SemanticOperation::Home(HomeOp {
            origin: sample_origin(),
        });
        let json = serde_json::to_string(&home).expect("serialize");
        assert!(
            json.contains(r#""type":"home""#),
            "Expected 'home' type tag, got: {json}"
        );
    }

    #[test]
    fn serde_forward_compat_unknown_field() {
        let json = r#"{
            "type": "pick",
            "origin": "op-1",
            "object": "bolt-1",
            "tool": null,
            "unknown_extra": "should_be_ignored"
        }"#;
        let result: Result<SemanticOperation, _> = serde_json::from_str(json);
        assert!(
            result.is_ok(),
            "Should tolerate unknown fields for forward compatibility"
        );
    }

    #[test]
    fn clone_and_eq_after_round_trip() {
        let original = SemanticOperation::Wait(WaitOp {
            origin: sample_origin(),
            duration: Duration::from_secs(42),
        });
        let cloned = original.clone();
        assert_eq!(original, cloned, "Clone should produce equal value");

        let json = serde_json::to_string(&original).expect("serialize");
        let decoded: SemanticOperation = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(original, decoded, "Round-trip should preserve equality");

        let decoded_clone = decoded.clone();
        assert_eq!(
            decoded, decoded_clone,
            "Clone after round-trip should preserve equality"
        );
    }
}
