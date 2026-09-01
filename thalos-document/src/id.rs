// ---------------------------------------------------------------------------
// ID newtypes — String-backed, serde-compatible, type-safe identifiers
// ---------------------------------------------------------------------------

/// Re-export the unified `OperationId` from `thalos_engine::core`.
///
/// Single source of truth — all crates use the same `OperationId(String)` type,
/// eliminating conversion at crate boundaries.
pub use thalos_engine::core::ids::OperationId;

// ---------------------------------------------------------------------------
// Unified semantic resource identifiers — re-exported from thalos_engine::core
// ---------------------------------------------------------------------------

/// Single source of truth — all crates use the same type, eliminating
/// conversion at crate boundaries.
pub use thalos_engine::core::ids::{ObjectId, LocationId, ToolId, TaskDocumentId};

/// Local alias so our `ProgramDocument` uses an ergonomic `ProgramDocumentId`
/// while mapping to the external `thalos_engine` `TaskDocumentId` type.
pub type ProgramDocumentId = TaskDocumentId;

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json;

    // --- Construction and equality ---

    #[test]
    fn operation_id_construction_and_equality() {
        let a = OperationId("op_1".to_string());
        let b = OperationId("op_1".to_string());
        let c = OperationId("op_2".to_string());
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    // --- Semantic resource IDs ---

    #[test]
    fn object_id_construction_and_equality() {
        let a = ObjectId("obj_01".to_string());
        let b = ObjectId("obj_01".to_string());
        let c = ObjectId("obj_02".to_string());
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn location_id_construction_and_equality() {
        let a = LocationId("station_a".to_string());
        let b = LocationId("station_a".to_string());
        assert_eq!(a, b);
    }

    #[test]
    fn tool_id_construction_and_equality() {
        let a = ToolId("gripper".to_string());
        let b = ToolId("gripper".to_string());
        assert_eq!(a, b);
    }

    #[test]
    fn semantic_ids_serde_round_trip() {
        // Test ObjectId
        let o = ObjectId("bolt".to_string());
        let json = serde_json::to_string(&o).unwrap();
        let back: ObjectId = serde_json::from_str(&json).unwrap();
        assert_eq!(o, back);
        // Test LocationId
        let l = LocationId("tray".to_string());
        let json = serde_json::to_string(&l).unwrap();
        let back: LocationId = serde_json::from_str(&json).unwrap();
        assert_eq!(l, back);
        // Test ToolId
        let t = ToolId("vacuum".to_string());
        let json = serde_json::to_string(&t).unwrap();
        let back: ToolId = serde_json::from_str(&json).unwrap();
        assert_eq!(t, back);
    }

    // --- Cross-crate type identity ---
    //
    // These compile only if the re-exported type IS the same type as
    // the upstream definition — assignment without conversion is the proof.

    #[test]
    fn object_id_is_same_across_crates() {
        // thalos_document::id::ObjectId re-exports thalos_engine::core::ids::ObjectId,
        // so assignment between them requires no conversion.
        let _: thalos_engine::core::ids::ObjectId = {
            let id = ObjectId("test".to_string());
            id
        };
    }

    #[test]
    fn location_id_is_same_across_crates() {
        let _: thalos_engine::core::ids::LocationId = {
            let id = LocationId("test".to_string());
            id
        };
    }

    #[test]
    fn tool_id_is_same_across_crates() {
        let _: thalos_engine::core::ids::ToolId = {
            let id = ToolId("test".to_string());
            id
        };
    }

    #[test]
    fn program_document_id_is_same_across_crates() {
        let _: ProgramDocumentId = {
            let id = TaskDocumentId("test".to_string());
            id
        };
    }
}
