use std::ops::Range;

use crate::ids::OperationId;
use crate::operation::motion_node::MotionRole;

/// Post-compilation record linking a waypoint range to its originating operation.
///
/// Created during compilation by grouping `MotionNode`s of the same operation
/// together — every entry has a known operation and role.
#[derive(Debug, Clone)]
pub struct MotionProvenance {
    pub waypoint_range: Range<usize>,
    pub operation_id: OperationId,
    pub role: MotionRole,
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Construction ──────────────────────────────────────

    #[test]
    fn provenance_holds_range_id_and_role() {
        let prov = MotionProvenance {
            waypoint_range: 0..5,
            operation_id: OperationId("1".to_string()),
            role: MotionRole::Execution,
        };

        assert_eq!(prov.waypoint_range, 0..5);
        assert_eq!(prov.operation_id, OperationId("1".to_string()));
        assert_eq!(prov.role, MotionRole::Execution);
    }

    #[test]
    fn provenance_with_different_values() {
        let prov = MotionProvenance {
            waypoint_range: 10..15,
            operation_id: OperationId("42".to_string()),
            role: MotionRole::Approach,
        };

        assert_eq!(prov.waypoint_range.start, 10);
        assert_eq!(prov.waypoint_range.end, 15);
        assert_eq!(prov.operation_id, OperationId("42".to_string()));
        assert_eq!(prov.role, MotionRole::Approach);
    }

    // ── Clone ─────────────────────────────────────────────

    #[test]
    fn provenance_is_clonable() {
        let prov = MotionProvenance {
            waypoint_range: 3..8,
            operation_id: OperationId("7".to_string()),
            role: MotionRole::Approach,
        };

        let cloned = prov.clone();
        assert_eq!(cloned.waypoint_range, prov.waypoint_range);
        assert_eq!(cloned.operation_id, prov.operation_id);
        assert_eq!(cloned.role, prov.role);
    }

    // ── Edge: single-waypoint range ───────────────────────

    #[test]
    fn provenance_accepts_single_waypoint_range() {
        let prov = MotionProvenance {
            waypoint_range: 5..6,
            operation_id: OperationId("99".to_string()),
            role: MotionRole::Interaction,
        };

        assert_eq!(prov.waypoint_range.len(), 1);
        assert_eq!(prov.waypoint_range, 5..6);
    }

    // ── Debug formatting ──────────────────────────────────

    #[test]
    fn provenance_debug_format() {
        let prov = MotionProvenance {
            waypoint_range: 0..3,
            operation_id: OperationId("1".to_string()),
            role: MotionRole::Transit,
        };

        let debug = format!("{:?}", prov);
        assert!(debug.contains("MotionProvenance"));
        assert!(debug.contains("0..3"));
        assert!(debug.contains("Transit"));
    }
}
