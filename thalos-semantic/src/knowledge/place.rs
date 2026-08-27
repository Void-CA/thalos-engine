use thalos_core::motion::MotionPose;

/// A placement plan returned by the `KnowledgeProvider` for a given object
/// and destination location.
///
/// Contains only geometric frames — no motion instructions, trajectories,
/// or constraints exist on this type. Lowering converts these frames into
/// `ExecutionInstruction` values.
#[derive(Debug, Clone, PartialEq)]
pub struct PlacementPlan {
    /// The pose where the object is released.
    pub drop_frame: MotionPose,
    /// The pose to approach from before placing.
    pub approach_frame: MotionPose,
    /// The pose to retract to after placing.
    pub retreat_frame: MotionPose,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_pose(x: f64, y: f64, z: f64) -> MotionPose {
        MotionPose {
            position: [x, y, z],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "world".into(),
        }
    }

    // ── Field access ────────────────────────────────────────────────────

    #[test]
    fn placement_plan_has_drop_frame() {
        let plan = PlacementPlan {
            drop_frame: sample_pose(1.0, 2.0, 3.0),
            approach_frame: sample_pose(0.0, 0.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 0.0),
        };
        assert_eq!(plan.drop_frame.position, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn placement_plan_has_approach_frame() {
        let plan = PlacementPlan {
            drop_frame: sample_pose(0.0, 0.0, 0.0),
            approach_frame: sample_pose(10.0, 20.0, 30.0),
            retreat_frame: sample_pose(0.0, 0.0, 0.0),
        };
        assert_eq!(plan.approach_frame.position, [10.0, 20.0, 30.0]);
    }

    #[test]
    fn placement_plan_has_retreat_frame() {
        let plan = PlacementPlan {
            drop_frame: sample_pose(0.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 0.0, 0.0),
            retreat_frame: sample_pose(5.0, 5.0, 5.0),
        };
        assert_eq!(plan.retreat_frame.position, [5.0, 5.0, 5.0]);
    }

    // ── No motion instruction fields ─────────────────────────────────────

    #[test]
    fn placement_plan_has_no_motion_instruction_fields() {
        let plan = PlacementPlan {
            drop_frame: sample_pose(0.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 0.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 0.0),
        };
        let PlacementPlan {
            drop_frame: _,
            approach_frame: _,
            retreat_frame: _,
        } = plan;
    }

    #[test]
    fn placement_plan_fields_are_motion_poses() {
        let plan = PlacementPlan {
            drop_frame: sample_pose(1.0, 2.0, 3.0),
            approach_frame: sample_pose(4.0, 5.0, 6.0),
            retreat_frame: sample_pose(7.0, 8.0, 9.0),
        };
        assert_eq!(plan.drop_frame.orientation, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(plan.drop_frame.frame, "world");
        assert_eq!(plan.approach_frame.frame, "world");
        assert_eq!(plan.retreat_frame.frame, "world");
    }

    // ── Clone + Debug ───────────────────────────────────────────────────

    #[test]
    fn placement_plan_clone_equals_original() {
        let plan = PlacementPlan {
            drop_frame: sample_pose(1.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 1.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 1.0),
        };
        let cloned = plan.clone();
        assert_eq!(plan, cloned);
    }

    #[test]
    fn placement_plan_debug_format() {
        let plan = PlacementPlan {
            drop_frame: sample_pose(0.0, 0.0, 0.0),
            approach_frame: sample_pose(0.0, 0.0, 0.0),
            retreat_frame: sample_pose(0.0, 0.0, 0.0),
        };
        let debug = format!("{plan:?}");
        assert!(debug.contains("PlacementPlan"));
    }
}
