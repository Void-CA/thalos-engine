use serde::{Deserialize, Serialize};

use crate::operation::SemanticOperation;

/// A semantic program — a linear sequence of task-level operations.
///
/// Represents *what* the robot should achieve using logical resource IDs,
/// independent of geometry or motion planning. Insertion order is preserved.
///
/// `SemanticProgram` is the input to the lowering pipeline, which resolves
/// logical IDs into concrete `ExecutionProgram` instructions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SemanticProgram {
    pub operations: Vec<SemanticOperation>,
}

impl SemanticProgram {
    /// Construct a new semantic program from an ordered list of operations.
    pub fn new(operations: Vec<SemanticOperation>) -> Self {
        Self { operations }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use thalos_core::ids::OperationId;

    use crate::operation::*;
    use crate::resource::*;

    fn sample_origin() -> OperationId {
        OperationId("op-1".to_string())
    }

    fn mixed_operations() -> Vec<SemanticOperation> {
        vec![
            SemanticOperation::Pick(PickOp {
                origin: sample_origin(),
                object: ObjectId("bolt-1".to_string()),
                tool: None,
            }),
            SemanticOperation::Place(PlaceOp {
                origin: OperationId("op-2".to_string()),
                object: ObjectId("bolt-1".to_string()),
                destination: LocationId("tray-1".to_string()),
                tool: None,
            }),
            SemanticOperation::MoveTo(MoveToOp {
                origin: OperationId("op-3".to_string()),
                destination: LocationId("shelf-a".to_string()),
                tool: None,
            }),
            SemanticOperation::Wait(WaitOp {
                origin: OperationId("op-4".to_string()),
                duration: Duration::from_secs(2),
            }),
            SemanticOperation::Home(HomeOp {
                origin: OperationId("op-5".to_string()),
            }),
        ]
    }

    // ── Construction ────────────────────────────────────────────────────

    #[test]
    fn program_constructs_with_operations() {
        let ops = mixed_operations();
        let program = SemanticProgram::new(ops);
        assert_eq!(program.operations.len(), 5);
    }

    #[test]
    fn empty_program_valid() {
        let program = SemanticProgram::new(vec![]);
        assert_eq!(program.operations.len(), 0);
    }

    // ── Empty program serde ─────────────────────────────────────────────

    #[test]
    fn empty_program_serializes_to_valid_json() {
        let program = SemanticProgram::new(vec![]);
        let json = serde_json::to_string(&program).expect("serialize");
        assert!(json.contains(r#""operations":[]"#));
    }

    #[test]
    fn empty_program_round_trip() {
        let program = SemanticProgram::new(vec![]);
        let json = serde_json::to_string(&program).expect("serialize");
        let decoded: SemanticProgram = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(program, decoded);
        assert_eq!(decoded.operations.len(), 0);
    }

    // ── Mixed-variant order preservation ────────────────────────────────

    #[test]
    fn mixed_variants_preserve_order() {
        let ops = mixed_operations();
        let program = SemanticProgram::new(ops);

        assert_eq!(program.operations.len(), 5);
        assert!(
            matches!(program.operations[0], SemanticOperation::Pick(_)),
            "First operation should be Pick"
        );
        assert!(
            matches!(program.operations[1], SemanticOperation::Place(_)),
            "Second operation should be Place"
        );
        assert!(
            matches!(program.operations[2], SemanticOperation::MoveTo(_)),
            "Third operation should be MoveTo"
        );
        assert!(
            matches!(program.operations[3], SemanticOperation::Wait(_)),
            "Fourth operation should be Wait"
        );
        assert!(
            matches!(program.operations[4], SemanticOperation::Home(_)),
            "Fifth operation should be Home"
        );
    }

    #[test]
    fn mixed_variant_order_serde_round_trip() {
        let ops = mixed_operations();
        let program = SemanticProgram::new(ops);

        let json = serde_json::to_string(&program).expect("serialize");
        let decoded: SemanticProgram = serde_json::from_str(&json).expect("deserialize");

        assert_eq!(program, decoded);

        // Verify specific variant ordering after deserialization
        assert!(matches!(decoded.operations[0], SemanticOperation::Pick(_)));
        assert!(matches!(decoded.operations[1], SemanticOperation::Place(_)));
        assert!(matches!(
            decoded.operations[2],
            SemanticOperation::MoveTo(_)
        ));
        assert!(matches!(decoded.operations[3], SemanticOperation::Wait(_)));
        assert!(matches!(decoded.operations[4], SemanticOperation::Home(_)));
    }

    // ── Lossless round-trip ─────────────────────────────────────────────

    #[test]
    fn serde_round_trip_lossless() {
        let ops = mixed_operations();
        let program = SemanticProgram::new(ops);
        let json = serde_json::to_string(&program).expect("serialize");
        let decoded: SemanticProgram = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(program, decoded);
    }

    // ── Forward compatibility ───────────────────────────────────────────

    #[test]
    fn serde_forward_compat_unknown_fields() {
        let json = r#"{
            "operations": [],
            "unknown_field": "should_be_ignored"
        }"#;
        let result: Result<SemanticProgram, _> = serde_json::from_str(json);
        assert!(
            result.is_ok(),
            "Should tolerate unknown fields for forward compatibility"
        );
    }

    // ── Program iterable ────────────────────────────────────────────────

    #[test]
    fn program_iterable_empty() {
        let program = SemanticProgram::new(vec![]);
        let count = program.operations.iter().count();
        assert_eq!(count, 0, "Empty program should yield zero items");
    }

    #[test]
    fn program_iterable_with_ops() {
        let program = SemanticProgram::new(mixed_operations());
        let count = program.operations.iter().count();
        assert_eq!(count, 5);
    }
}
