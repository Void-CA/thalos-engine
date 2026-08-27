use std::ops::Range;

use crate::operation::constraint_query::ConstraintQuery;
use crate::operation::operation::OperationConstraints;
use crate::operation::precision::PrecisionLevel;

/// A concrete `ConstraintQuery` backed by a sparse map of expanded node ranges.
///
/// Stores a `Vec<(Range<usize>, OperationConstraints)>` where each entry
/// maps a contiguous range of waypoint indices to the operation's constraints.
/// Linear scan over ~3-5 entries is faster than tree overhead at this scale.
pub struct RangeConstraintQuery {
    ranges: Vec<(Range<usize>, OperationConstraints)>,
}

impl RangeConstraintQuery {
    pub fn new(ranges: Vec<(Range<usize>, OperationConstraints)>) -> Self {
        Self { ranges }
    }

    fn constraints_at(&self, index: usize) -> Option<&OperationConstraints> {
        self.ranges
            .iter()
            .find(|(range, _)| range.contains(&index))
            .map(|(_, constraints)| constraints)
    }
}

impl ConstraintQuery for RangeConstraintQuery {
    fn can_relax_orientation(&self, waypoint_index: usize, max_angle: f64) -> bool {
        match self.constraints_at(waypoint_index) {
            Some(c) => match c.orientation_tolerance {
                Some(tol) => tol >= max_angle,
                None => true, // no orientation constraint → relax allowed
            },
            None => true, // no operation at this index → unconstrained
        }
    }

    fn can_modify_position(&self, waypoint_index: usize) -> bool {
        match self.constraints_at(waypoint_index) {
            Some(c) => c.position_tolerance.is_none(),
            None => true,
        }
    }

    fn max_position_error(&self, waypoint_index: usize) -> Option<f64> {
        self.constraints_at(waypoint_index)
            .and_then(|c| c.position_tolerance)
    }

    fn max_velocity(&self, waypoint_index: usize) -> Option<f64> {
        self.constraints_at(waypoint_index)
            .and_then(|c| c.velocity_limit)
    }

    fn required_precision(&self, waypoint_index: usize) -> PrecisionLevel {
        match self.constraints_at(waypoint_index) {
            Some(c) => match c.position_tolerance {
                Some(t) if t < 0.001 => PrecisionLevel::Critical,
                Some(t) if t < 0.01 => PrecisionLevel::High,
                Some(_) => PrecisionLevel::Normal,
                None => PrecisionLevel::None,
            },
            None => PrecisionLevel::None,
        }
    }

    fn can_modify_timing(&self, waypoint_index: usize) -> bool {
        match self.constraints_at(waypoint_index) {
            Some(c) => c.velocity_limit.is_none(),
            None => true,
        }
    }

    fn can_modify_joints(&self, waypoint_index: usize) -> bool {
        self.can_modify_position(waypoint_index)
    }

    fn can_modify_neighbors(&self, waypoint_index: usize) -> bool {
        match self.constraints_at(waypoint_index) {
            Some(c) => c.position_tolerance.is_none(),
            None => true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper: constraints with known tolerance ──────────

    fn pick_constraints() -> OperationConstraints {
        OperationConstraints {
            position_tolerance: Some(0.001),
            orientation_tolerance: Some(0.5_f64.to_radians()),
            ..Default::default()
        }
    }

    fn transit_constraints() -> OperationConstraints {
        OperationConstraints::default()
    }

    fn place_constraints() -> OperationConstraints {
        OperationConstraints {
            position_tolerance: Some(0.005),
            ..Default::default()
        }
    }

    // ── Construction ──────────────────────────────────────

    #[test]
    fn range_query_construction_with_empty_ranges() {
        let query = RangeConstraintQuery::new(vec![]);
        // Out-of-bounds should return None for any constraint
        assert!(query.max_position_error(0).is_none());
    }

    #[test]
    fn range_query_construction_with_single_range() {
        let constraints = pick_constraints();
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        // Waypoint 2 is inside the range
        assert!(!query.can_relax_orientation(2, 1.0_f64.to_radians()));
    }

    // ── can_relax_orientation ─────────────────────────────

    #[test]
    fn can_relax_orientation_returns_false_when_tolerance_tighter_than_angle() {
        // orientation_tolerance = 0.5°, max_angle = 1.0° → tolerance < angle → cannot relax
        let constraints = pick_constraints(); // orientation_tolerance = 0.5°
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert!(!query.can_relax_orientation(2, 1.0_f64.to_radians()));
    }

    #[test]
    fn can_relax_orientation_returns_true_when_tolerance_wider_than_angle() {
        // orientation_tolerance = 0.5°, max_angle = 0.1° → tolerance > angle → can relax
        let constraints = pick_constraints();
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert!(query.can_relax_orientation(2, 0.1_f64.to_radians()));
    }

    #[test]
    fn can_relax_orientation_returns_true_for_unconstrained_operation() {
        // No orientation_tolerance → unconstrained → can relax
        let constraints = transit_constraints();
        let query = RangeConstraintQuery::new(vec![(0..1, constraints)]);
        assert!(query.can_relax_orientation(0, 5.0_f64.to_radians()));
    }

    #[test]
    fn can_relax_orientation_returns_true_for_out_of_bounds_index() {
        let constraints = pick_constraints();
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        // Index 10 is outside any range → unconstrained
        assert!(query.can_relax_orientation(10, 10.0_f64.to_radians()));
    }

    // ── can_modify_position ──────────────────────────────

    #[test]
    fn can_modify_position_returns_false_when_position_tolerance_is_set() {
        let constraints = pick_constraints(); // position_tolerance = 0.001
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert!(!query.can_modify_position(2));
    }

    #[test]
    fn can_modify_position_returns_true_when_no_position_constraint() {
        let constraints = transit_constraints(); // all None
        let query = RangeConstraintQuery::new(vec![(0..1, constraints)]);
        assert!(query.can_modify_position(0));
    }

    #[test]
    fn can_modify_position_returns_true_for_out_of_bounds_index() {
        let constraints = pick_constraints();
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert!(query.can_modify_position(99));
    }

    // ── max_position_error ────────────────────────────────

    #[test]
    fn max_position_error_returns_tolerance_for_in_range_index() {
        let constraints = pick_constraints(); // position_tolerance = 0.001
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert_eq!(query.max_position_error(2), Some(0.001));
    }

    #[test]
    fn max_position_error_returns_none_for_unconstrained_operation() {
        let constraints = transit_constraints(); // all None
        let query = RangeConstraintQuery::new(vec![(0..1, constraints)]);
        assert_eq!(query.max_position_error(0), None);
    }

    #[test]
    fn max_position_error_returns_none_for_out_of_bounds_index() {
        let constraints = pick_constraints();
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert_eq!(query.max_position_error(99), None);
    }

    // ── max_velocity ──────────────────────────────────────

    #[test]
    fn max_velocity_returns_limit_when_set() {
        let constraints = OperationConstraints {
            velocity_limit: Some(0.5),
            ..Default::default()
        };
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert_eq!(query.max_velocity(2), Some(0.5));
    }

    #[test]
    fn max_velocity_returns_none_when_not_set() {
        let constraints = pick_constraints();
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert_eq!(query.max_velocity(2), None);
    }

    #[test]
    fn max_velocity_returns_none_for_out_of_bounds_index() {
        let constraints = OperationConstraints {
            velocity_limit: Some(0.5),
            ..Default::default()
        };
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert_eq!(query.max_velocity(99), None);
    }

    // ── required_precision ────────────────────────────────

    #[test]
    fn required_precision_is_critical_for_sub_mm_tolerance() {
        let constraints = OperationConstraints {
            position_tolerance: Some(0.0005),
            ..Default::default()
        };
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert_eq!(query.required_precision(2), PrecisionLevel::Critical);
    }

    #[test]
    fn required_precision_is_high_for_mm_tolerance() {
        let constraints = OperationConstraints {
            position_tolerance: Some(0.005),
            ..Default::default()
        };
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert_eq!(query.required_precision(2), PrecisionLevel::High);
    }

    #[test]
    fn required_precision_is_normal_for_cm_tolerance() {
        let constraints = OperationConstraints {
            position_tolerance: Some(0.05),
            ..Default::default()
        };
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert_eq!(query.required_precision(2), PrecisionLevel::Normal);
    }

    #[test]
    fn required_precision_is_none_for_unconstrained() {
        let constraints = transit_constraints();
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert_eq!(query.required_precision(2), PrecisionLevel::None);
    }

    #[test]
    fn required_precision_is_none_for_out_of_bounds_index() {
        let constraints = pick_constraints();
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert_eq!(query.required_precision(99), PrecisionLevel::None);
    }

    // ── can_modify_timing (velocity_limit.is_none()) ─────

    #[test]
    fn can_modify_timing_returns_false_when_velocity_limit_is_set() {
        let constraints = OperationConstraints {
            velocity_limit: Some(0.5),
            ..Default::default()
        };
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert!(!query.can_modify_timing(2));
    }

    #[test]
    fn can_modify_timing_returns_true_when_no_velocity_limit() {
        let constraints = pick_constraints(); // velocity_limit = None
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert!(query.can_modify_timing(2));
    }

    #[test]
    fn can_modify_timing_returns_true_for_out_of_bounds_index() {
        let constraints = OperationConstraints {
            velocity_limit: Some(0.5),
            ..Default::default()
        };
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert!(query.can_modify_timing(99));
    }

    // ── can_modify_joints (delegates to can_modify_position) ─

    #[test]
    fn can_modify_joints_returns_false_when_position_tolerance_is_set() {
        let constraints = pick_constraints(); // position_tolerance = 0.001
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert!(!query.can_modify_joints(2));
    }

    #[test]
    fn can_modify_joints_returns_true_when_no_position_constraint() {
        let constraints = transit_constraints(); // all None
        let query = RangeConstraintQuery::new(vec![(0..1, constraints)]);
        assert!(query.can_modify_joints(0));
    }

    #[test]
    fn can_modify_joints_returns_true_for_out_of_bounds_index() {
        let constraints = pick_constraints();
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert!(query.can_modify_joints(99));
    }

    // ── can_modify_neighbors (position_tolerance.is_none()) ─

    #[test]
    fn can_modify_neighbors_returns_false_when_position_tolerance_is_set() {
        let constraints = pick_constraints(); // position_tolerance = 0.001
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert!(!query.can_modify_neighbors(2));
    }

    #[test]
    fn can_modify_neighbors_returns_true_when_no_position_constraint() {
        let constraints = transit_constraints(); // all None
        let query = RangeConstraintQuery::new(vec![(0..1, constraints)]);
        assert!(query.can_modify_neighbors(0));
    }

    #[test]
    fn can_modify_neighbors_returns_true_for_out_of_bounds_index() {
        let constraints = pick_constraints();
        let query = RangeConstraintQuery::new(vec![(0..5, constraints)]);
        assert!(query.can_modify_neighbors(99));
    }

    // ── Multi-range queries (triangulation) ───────────────

    #[test]
    fn query_uses_correct_range_when_multiple_operations() {
        let ranges = vec![
            (0..5, pick_constraints()),
            (5..9, place_constraints()),
            (9..10, transit_constraints()),
        ];
        let query = RangeConstraintQuery::new(ranges);

        // Waypoint 2 → Pick (orientation_tolerance = 0.5°)
        assert!(!query.can_relax_orientation(2, 1.0_f64.to_radians()));

        // Waypoint 7 → Place (position_tolerance = 0.005)
        assert!(!query.can_modify_position(7));

        // Waypoint 9 → Transit (unconstrained)
        assert!(query.can_relax_orientation(9, 5.0_f64.to_radians()));
    }

    #[test]
    fn query_respects_range_boundaries_exactly() {
        let ranges = vec![(0..5, pick_constraints()), (5..9, place_constraints())];
        let query = RangeConstraintQuery::new(ranges);

        // Waypoint 4 is the last Pick index
        assert_eq!(query.max_position_error(4), Some(0.001));
        // Waypoint 5 is the first Place index
        assert_eq!(query.max_position_error(5), Some(0.005));
    }
}
