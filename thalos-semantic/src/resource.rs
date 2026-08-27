// ---------------------------------------------------------------------------
// Unified semantic resource identifiers — re-exported from thalos_core
//
// Single source of truth — all crates use the exact same id types across
// crate boundaries, eliminating conversion at every boundary.
// ---------------------------------------------------------------------------
pub use thalos_core::ids::{ObjectId, LocationId, ToolId, TaskDocumentId};

#[cfg(test)]
mod tests {
    use super::*;

    // ── Construction ────────────────────────────────────────────────────

    #[test]
    fn object_id_constructs_from_string() {
        let id = ObjectId("part-42".to_string());
        assert_eq!(id.0, "part-42");
    }

    #[test]
    fn location_id_constructs_from_string() {
        let id = LocationId("shelf-a".to_string());
        assert_eq!(id.0, "shelf-a");
    }

    #[test]
    fn tool_id_constructs_from_string() {
        let id = ToolId("gripper-1".to_string());
        assert_eq!(id.0, "gripper-1");
    }

    // ── Debug formatting ────────────────────────────────────────────────

    #[test]
    fn debug_format_is_readable() {
        let id = ObjectId("bolt-1".to_string());
        let debug = format!("{id:?}");
        assert!(debug.contains("bolt-1"));
    }

    // ── Serde round-trip ────────────────────────────────────────────────

    #[test]
    fn serde_round_trip_object_id() {
        let id = ObjectId("part-42".to_string());
        let json = serde_json::to_string(&id).expect("serialize");
        let decoded: ObjectId = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(id, decoded);
    }

    #[test]
    fn serde_round_trip_location_id() {
        let id = LocationId("shelf-a".to_string());
        let json = serde_json::to_string(&id).expect("serialize");
        let decoded: LocationId = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(id, decoded);
    }

    #[test]
    fn serde_round_trip_tool_id() {
        let id = ToolId("gripper-1".to_string());
        let json = serde_json::to_string(&id).expect("serialize");
        let decoded: ToolId = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(id, decoded);
    }

    // ── No geometry fields ──────────────────────────────────────────────

    #[test]
    fn no_geometry_fields_on_object_id() {
        let id = ObjectId("test".to_string());
        let ObjectId(inner) = &id;
        assert_eq!(inner, "test");
    }

    #[test]
    fn no_geometry_fields_on_location_id() {
        let id = LocationId("test".to_string());
        let LocationId(inner) = &id;
        assert_eq!(inner, "test");
    }

    #[test]
    fn no_geometry_fields_on_tool_id() {
        let id = ToolId("test".to_string());
        let ToolId(inner) = &id;
        assert_eq!(inner, "test");
    }

    // ── Equality and Clone ──────────────────────────────────────────────

    #[test]
    fn equality_comparison() {
        let a = ObjectId("same".to_string());
        let b = ObjectId("same".to_string());
        let c = ObjectId("different".to_string());
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn clone_produces_equal_value() {
        let original = ObjectId("clone-test".to_string());
        let cloned = original.clone();
        assert_eq!(original, cloned);
    }

    #[test]
    fn hash_set_deduplication() {
        use std::collections::HashSet;
        let mut set = HashSet::new();
        set.insert(ObjectId("dup".to_string()));
        set.insert(ObjectId("dup".to_string()));
        set.insert(ObjectId("unique".to_string()));
        assert_eq!(set.len(), 2);
    }
}
