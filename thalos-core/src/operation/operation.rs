use crate::ids::OperationId;
use crate::spatial::pose::Pose;
use thalos_math::UnitVector3;

/// Taxonomic classification of an operation.
///
/// Describes what the operation IS, not how to execute it.
/// Used for diagnostic logging and operator-level routing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum OperationType {
    Pick,
    Place,
    Transit,
}

/// Physical constraint envelope for an operation.
///
/// All fields are optional — `None` means "unconstrained".
/// Default implementation sets all fields to `None`.
#[derive(Debug, Clone)]
pub struct OperationConstraints {
    /// Maximum allowed position error (meters).
    pub position_tolerance: Option<f64>,
    /// Maximum allowed orientation error (radians).
    pub orientation_tolerance: Option<f64>,
    /// Maximum allowed joint deviation (radians).
    pub joint_deviation_limit: Option<f64>,
    /// Maximum velocity (m/s for linear, rad/s for joints).
    pub velocity_limit: Option<f64>,
    /// Preferred approach direction for the operation.
    pub approach_direction: Option<UnitVector3>,
    /// Preferred retreat direction after the operation.
    pub retreat_direction: Option<UnitVector3>,
}

impl Default for OperationConstraints {
    fn default() -> Self {
        Self {
            position_tolerance: None,
            orientation_tolerance: None,
            joint_deviation_limit: None,
            velocity_limit: None,
            approach_direction: None,
            retreat_direction: None,
        }
    }
}

/// A semantic work unit in the Operation IR (ADR-002).
///
/// Each operation carries a unique ID, a target pose, and a set of constraints.
/// Operations are concrete enums (not a trait) — the model is still stabilizing.
/// A trait should be introduced only when third-party operation types emerge.
///
/// # Variants
///
/// * `Pick` — Grasp an object at `target_pose`.
/// * `Place` — Release an object at `target_pose`.
/// * `Transit` — Free motion to `target_pose` (no contact semantics).
#[derive(Debug, Clone)]
pub enum Operation {
    Pick {
        id: OperationId,
        target_pose: Pose,
        constraints: OperationConstraints,
    },
    Place {
        id: OperationId,
        target_pose: Pose,
        constraints: OperationConstraints,
    },
    Transit {
        id: OperationId,
        target_pose: Pose,
        constraints: OperationConstraints,
    },
}

impl Operation {
    /// Returns the unique identifier of this operation.
    pub fn id(&self) -> OperationId {
        match self {
            Operation::Pick { id, .. }
            | Operation::Place { id, .. }
            | Operation::Transit { id, .. } => id.clone(),
        }
    }

    /// Returns the taxonomic type of this operation.
    pub fn operation_type(&self) -> OperationType {
        match self {
            Operation::Pick { .. } => OperationType::Pick,
            Operation::Place { .. } => OperationType::Place,
            Operation::Transit { .. } => OperationType::Transit,
        }
    }

    /// Returns the target pose of this operation.
    pub fn target_pose(&self) -> &Pose {
        match self {
            Operation::Pick { target_pose, .. }
            | Operation::Place { target_pose, .. }
            | Operation::Transit { target_pose, .. } => target_pose,
        }
    }

    /// Returns the constraints of this operation.
    pub fn constraints(&self) -> &OperationConstraints {
        match self {
            Operation::Pick { constraints, .. }
            | Operation::Place { constraints, .. }
            | Operation::Transit { constraints, .. } => constraints,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Operation construction ─────────────────────────────

    #[test]
    fn pick_operation_has_id_type_and_constraints() {
        let op = Operation::Pick {
            id: OperationId("42".to_string()),
            target_pose: Pose::new(
                crate::spatial::frame::FrameId::World,
                crate::spatial::frame::FrameId::Id(1),
                thalos_math::Transform3D::identity(),
            ),
            constraints: OperationConstraints::default(),
        };

        assert_eq!(op.id(), OperationId("42".to_string()));
        assert_eq!(op.operation_type(), OperationType::Pick);
        assert!(op.constraints().position_tolerance.is_none());
    }

    #[test]
    fn place_operation_has_id_type_and_constraints() {
        let op = Operation::Place {
            id: OperationId("7".to_string()),
            target_pose: Pose::new(
                crate::spatial::frame::FrameId::World,
                crate::spatial::frame::FrameId::Id(1),
                thalos_math::Transform3D::identity(),
            ),
            constraints: OperationConstraints::default(),
        };

        assert_eq!(op.id(), OperationId("7".to_string()));
        assert_eq!(op.operation_type(), OperationType::Place);
        assert!(op.constraints().orientation_tolerance.is_none());
    }

    #[test]
    fn transit_operation_has_id_type_and_constraints() {
        let op = Operation::Transit {
            id: OperationId("99".to_string()),
            target_pose: Pose::new(
                crate::spatial::frame::FrameId::World,
                crate::spatial::frame::FrameId::Id(1),
                thalos_math::Transform3D::identity(),
            ),
            constraints: OperationConstraints::default(),
        };

        assert_eq!(op.id(), OperationId("99".to_string()));
        assert_eq!(op.operation_type(), OperationType::Transit);
        assert!(op.constraints().velocity_limit.is_none());
    }

    // ── OperationId newtype ────────────────────────────────

    #[test]
    fn operation_id_is_comparable_and_clonable() {
        let a = OperationId("1".to_string());
        let b = OperationId("2".to_string());
        let c = OperationId("1".to_string());

        assert_ne!(a, b);
        assert_eq!(a, c);
    }

    // ── OperationConstraints defaults ──────────────────────

    #[test]
    fn constraints_default_all_none() {
        let c = OperationConstraints::default();

        assert!(c.position_tolerance.is_none());
        assert!(c.orientation_tolerance.is_none());
        assert!(c.joint_deviation_limit.is_none());
        assert!(c.velocity_limit.is_none());
        assert!(c.approach_direction.is_none());
        assert!(c.retreat_direction.is_none());
    }

    #[test]
    fn constraints_with_explicit_values() {
        let c = OperationConstraints {
            position_tolerance: Some(0.01),
            orientation_tolerance: Some(0.5_f64.to_radians()),
            ..Default::default()
        };

        assert_eq!(c.position_tolerance, Some(0.01));
        assert_eq!(c.orientation_tolerance, Some(0.5_f64.to_radians()));
        assert!(c.joint_deviation_limit.is_none());
    }

    // ── OperationType enum ─────────────────────────────────

    #[test]
    fn operation_type_variants_are_distinct() {
        assert_ne!(OperationType::Pick, OperationType::Place);
        assert_ne!(OperationType::Pick, OperationType::Transit);
        assert_ne!(OperationType::Place, OperationType::Transit);
    }

    #[test]
    fn operation_type_exhaustive_debug_format() {
        let pick = format!("{:?}", OperationType::Pick);
        let place = format!("{:?}", OperationType::Place);
        let transit = format!("{:?}", OperationType::Transit);

        assert!(!pick.is_empty());
        assert!(!place.is_empty());
        assert!(!transit.is_empty());
    }

    // ── Operation trait method consistency ─────────────────

    #[test]
    fn operations_round_trip_through_methods() {
        let id = OperationId("17".to_string());
        let op = Operation::Pick {
            id: id.clone(),
            target_pose: Pose::new(
                crate::spatial::frame::FrameId::World,
                crate::spatial::frame::FrameId::Id(1),
                thalos_math::Transform3D::identity(),
            ),
            constraints: OperationConstraints::default(),
        };

        assert_eq!(op.id(), id);
        assert_eq!(op.operation_type(), OperationType::Pick);
        assert_eq!(
            op.target_pose().target_id(),
            crate::spatial::frame::FrameId::Id(1)
        );
    }
}
