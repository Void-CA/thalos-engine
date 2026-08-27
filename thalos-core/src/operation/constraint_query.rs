use crate::operation::precision::PrecisionLevel;

/// Permission-only query interface.
///
/// Operators ask "can I do X?" at each waypoint — they never query
/// "what operation is this?". This prevents operators from making
/// decisions based on operation identity rather than constraint data.
pub trait ConstraintQuery: Send + Sync {
    /// May the operator relax orientation at this waypoint by at most `max_angle`?
    fn can_relax_orientation(&self, waypoint_index: usize, max_angle: f64) -> bool;

    /// May the operator modify the position at this waypoint?
    fn can_modify_position(&self, waypoint_index: usize) -> bool;

    /// Maximum allowed position error at this waypoint, if constrained.
    fn max_position_error(&self, waypoint_index: usize) -> Option<f64>;

    /// Maximum allowed velocity at this waypoint, if constrained.
    fn max_velocity(&self, waypoint_index: usize) -> Option<f64>;

    /// Required precision level for this waypoint.
    fn required_precision(&self, waypoint_index: usize) -> PrecisionLevel;

    /// May the operator modify the timestamp of waypoint `i`?
    ///
    /// Default: `true` — unconstrained, operators may adjust timing freely.
    fn can_modify_timing(&self, _waypoint_index: usize) -> bool {
        true
    }

    /// May the operator modify the joint values at waypoint `i`?
    ///
    /// Default: `true` — unconstrained, operators may adjust joints freely.
    fn can_modify_joints(&self, _waypoint_index: usize) -> bool {
        true
    }

    /// May the operator insert/remove waypoints adjacent to waypoint `i`?
    ///
    /// Default: `true` — unconstrained, operators may add/remove waypoints freely.
    fn can_modify_neighbors(&self, _waypoint_index: usize) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Test harness: a mock ConstraintQuery ──────────────

    struct MockConstraintQuery {
        orientation_allowed: bool,
        position_allowed: bool,
        precision: PrecisionLevel,
    }

    impl MockConstraintQuery {
        fn new(
            orientation_allowed: bool,
            position_allowed: bool,
            precision: PrecisionLevel,
        ) -> Self {
            Self {
                orientation_allowed,
                position_allowed,
                precision,
            }
        }
    }

    impl ConstraintQuery for MockConstraintQuery {
        fn can_relax_orientation(&self, _waypoint_index: usize, max_angle: f64) -> bool {
            // For testing: orientation_allowed AND max_angle must be meaningful
            self.orientation_allowed && max_angle > 0.0
        }

        fn can_modify_position(&self, _waypoint_index: usize) -> bool {
            self.position_allowed
        }

        fn max_position_error(&self, _waypoint_index: usize) -> Option<f64> {
            // Return stricter tolerance for higher precision levels
            match self.precision {
                PrecisionLevel::Critical => Some(0.001),
                PrecisionLevel::High => Some(0.01),
                PrecisionLevel::Normal => Some(0.1),
                PrecisionLevel::None => None,
            }
        }

        fn max_velocity(&self, _waypoint_index: usize) -> Option<f64> {
            // None for this mock (unconstrained velocity)
            None
        }

        fn required_precision(&self, _waypoint_index: usize) -> PrecisionLevel {
            self.precision
        }
    }

    // ── Trait method tests ────────────────────────────────

    #[test]
    fn can_relax_orientation_returns_true_when_allowed() {
        let query = MockConstraintQuery::new(true, false, PrecisionLevel::Normal);
        assert!(query.can_relax_orientation(0, 5.0_f64.to_radians()));
    }

    #[test]
    fn can_relax_orientation_returns_false_when_disallowed() {
        let query = MockConstraintQuery::new(false, false, PrecisionLevel::Critical);
        assert!(!query.can_relax_orientation(0, 1.0_f64.to_radians()));
    }

    #[test]
    fn can_modify_position_returns_true_when_allowed() {
        let query = MockConstraintQuery::new(false, true, PrecisionLevel::Normal);
        assert!(query.can_modify_position(5));
    }

    #[test]
    fn can_modify_position_returns_false_when_disallowed() {
        let query = MockConstraintQuery::new(false, false, PrecisionLevel::Normal);
        assert!(!query.can_modify_position(5));
    }

    #[test]
    fn max_position_error_scales_with_precision() {
        let critical = MockConstraintQuery::new(false, false, PrecisionLevel::Critical);
        let none = MockConstraintQuery::new(false, false, PrecisionLevel::None);

        assert_eq!(critical.max_position_error(0), Some(0.001));
        assert_eq!(none.max_position_error(0), None);
    }

    #[test]
    fn max_velocity_is_optional() {
        let query = MockConstraintQuery::new(false, false, PrecisionLevel::Normal);
        // Currently returns None — tests that the method exists and is callable
        assert!(query.max_velocity(0).is_none());
    }

    #[test]
    fn required_precision_returns_stored_level() {
        let query = MockConstraintQuery::new(false, false, PrecisionLevel::Critical);
        assert_eq!(query.required_precision(0), PrecisionLevel::Critical);
    }

    // ── New constraint methods (default true) ─────────────

    #[test]
    fn can_modify_timing_defaults_to_true() {
        let query = MockConstraintQuery::new(false, false, PrecisionLevel::Critical);
        assert!(query.can_modify_timing(0));
    }

    #[test]
    fn can_modify_timing_works_for_any_index() {
        let query = MockConstraintQuery::new(true, true, PrecisionLevel::Normal);
        assert!(query.can_modify_timing(99));
    }

    #[test]
    fn can_modify_joints_defaults_to_true() {
        let query = MockConstraintQuery::new(false, false, PrecisionLevel::Critical);
        assert!(query.can_modify_joints(5));
    }

    #[test]
    fn can_modify_joints_works_for_any_index() {
        let query = MockConstraintQuery::new(true, true, PrecisionLevel::Normal);
        assert!(query.can_modify_joints(100));
    }

    #[test]
    fn can_modify_neighbors_defaults_to_true() {
        let query = MockConstraintQuery::new(false, false, PrecisionLevel::Critical);
        assert!(query.can_modify_neighbors(3));
    }

    #[test]
    fn can_modify_neighbors_works_for_any_index() {
        let query = MockConstraintQuery::new(true, true, PrecisionLevel::Normal);
        assert!(query.can_modify_neighbors(0));
    }

    // ── Trait object safety ───────────────────────────────

    #[test]
    fn constraint_query_is_object_safe() {
        let query = MockConstraintQuery::new(true, true, PrecisionLevel::Normal);
        let trait_obj: &dyn ConstraintQuery = &query;
        // dyn dispatch must work
        assert!(trait_obj.can_relax_orientation(0, 1.0));
        assert!(trait_obj.can_modify_position(0));
    }
}
