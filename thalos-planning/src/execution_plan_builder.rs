use thalos_core::execution::plan::{
    BuilderError, ExecutionInstruction, ExecutionPlan, ExecutionSegment, ExecutionWaypoint,
};
use thalos_core::motion::segment::MotionSegment;

use crate::motion::program::CompiledPlan;

/// Pure builder: `CompiledPlan` → [`ExecutionPlan`].
///
/// Performs no I/O. Each `PlannedSegment` maps 1:1 to an `ExecutionSegment`
/// and each `TrajectoryPoint` in `merged_trajectory` maps to one
/// `ExecutionWaypoint`. Segments are never merged, split, or reclassified.
pub struct ExecutionPlanBuilder;

impl ExecutionPlanBuilder {
    pub fn build(plan: &CompiledPlan) -> Result<ExecutionPlan, BuilderError> {
        let segments = plan
            .segments
            .iter()
            .enumerate()
            .map(|(idx, seg)| ExecutionSegment {
                index: idx,
                planned_segment_index: idx,
                instruction: match &seg.source {
                    MotionSegment::MoveJ { .. } => ExecutionInstruction::MoveJ,
                    MotionSegment::MoveL { .. } => ExecutionInstruction::MoveL,
                    MotionSegment::MoveLPosition { .. } => ExecutionInstruction::MoveL,
                },
                waypoint_range: seg.waypoint_range.clone(),
            })
            .collect();

        let waypoints = plan
            .merged_trajectory
            .waypoints()
            .iter()
            .map(|tp| ExecutionWaypoint {
                joints: tp.joints().to_vec(),
                timestamp: tp.timestamp(),
            })
            .collect();

        Ok(ExecutionPlan {
            waypoints,
            segments,
            duration: plan.duration,
    repeat_count: 1,
        })
    }
}

#[cfg(test)]
mod tests {
    use std::ops::Range;

    use thalos_core::execution::plan::ExecutionInstruction;
    use thalos_core::ids::OperationId;
    use thalos_core::motion::segment::MotionSegment;
    use thalos_core::prelude::{Trajectory, TrajectoryPoint};
    use thalos_core::spatial::frame::FrameId;
    use thalos_core::spatial::pose::Pose;
    use thalos_math::Transform3D;

    use crate::execution_plan_builder::ExecutionPlanBuilder;
    use crate::motion::program::{CompiledPlan, PlannedSegment};

    fn movej(origin: &str, joints: Vec<f64>) -> MotionSegment {
        MotionSegment::MoveJ {
            origin: OperationId(origin.to_string()),
            target: joints,
            max_velocity: Some(500.0),
            max_acceleration: Some(1000.0),
        }
    }

    fn movel(origin: &str) -> MotionSegment {
        MotionSegment::MoveL {
            origin: OperationId(origin.to_string()),
            frame: FrameId::World,
            target_pose: Pose::new(FrameId::World, FrameId::Id(1), Transform3D::identity()),
            max_velocity: Some(200.0),
        }
    }

    fn planned(
        source: MotionSegment,
        waypoint_range: Range<usize>,
        time_range: Range<f64>,
    ) -> PlannedSegment {
        PlannedSegment {
            origin: source.origin().clone(),
            source,
            trajectory: Trajectory::new(vec![]),
            waypoint_range,
            time_range,
            operation_id: None,
            role: None,
        }
    }

    /// 3 segments (MoveJ, MoveL, MoveJ) with non-uniform timestamps
    /// 0.0, 0.3, 1.0, 1.5, 2.5. Duration = 2.5 (last waypoint).
    fn three_segment_plan() -> CompiledPlan {
        let merged = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.3, 0.1], 0.3),
            TrajectoryPoint::new(vec![0.6, 0.4], 1.0),
            TrajectoryPoint::new(vec![0.9, 0.7], 1.5),
            TrajectoryPoint::new(vec![1.2, 1.0], 2.5),
        ]);
        CompiledPlan::new(
            merged,
            vec![
                planned(movej("op-0", vec![0.3, 0.1]), 0..2, 0.0..0.3),
                planned(movel("op-1"), 2..4, 0.3..1.5),
                planned(movej("op-2", vec![1.2, 1.0]), 4..5, 1.5..2.5),
            ],
        )
    }

    /// 2 segments: MoveJ then MoveL.
    fn movej_then_movel_plan() -> CompiledPlan {
        let merged = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, 1.0], 1.0),
            TrajectoryPoint::new(vec![1.0, 2.0], 2.0),
        ]);
        CompiledPlan::new(
            merged,
            vec![
                planned(movej("op-j", vec![0.5, 1.0]), 0..2, 0.0..1.0),
                planned(movel("op-l"), 2..3, 1.0..2.0),
            ],
        )
    }

    #[test]
    fn compiled_plan_preserves_segment_boundaries() {
        let plan = three_segment_plan();
        let execution = ExecutionPlanBuilder::build(&plan).expect("build should succeed");
        assert_eq!(execution.segments.len(), plan.segments.len());
        let provenance: Vec<usize> = execution
            .segments
            .iter()
            .map(|s| s.planned_segment_index)
            .collect();
        assert_eq!(provenance, vec![0, 1, 2]);
    }

    #[test]
    fn segment_count_preserved() {
        let execution = ExecutionPlanBuilder::build(&three_segment_plan()).expect("build");
        assert_eq!(execution.segments.len(), 3);
    }

    #[test]
    fn movej_and_movel_are_preserved() {
        let execution = ExecutionPlanBuilder::build(&movej_then_movel_plan()).expect("build");
        assert_eq!(execution.segments.len(), 2);
        assert!(matches!(
            execution.segments[0].instruction,
            ExecutionInstruction::MoveJ
        ));
        assert!(matches!(
            execution.segments[1].instruction,
            ExecutionInstruction::MoveL
        ));
    }

    #[test]
    fn compiled_plan_generates_real_timestamps() {
        let execution = ExecutionPlanBuilder::build(&three_segment_plan()).expect("build");
        let ts: Vec<f64> = execution.waypoints.iter().map(|w| w.timestamp).collect();
        assert_eq!(ts, vec![0.0, 0.3, 1.0, 1.5, 2.5]);
        // Deltas differ → NOT uniformly spaced.
        let deltas: Vec<f64> = ts.windows(2).map(|w| w[1] - w[0]).collect();
        assert_ne!(deltas[0], deltas[1]);
        assert_ne!(deltas[1], deltas[2]);
    }

    #[test]
    fn manifest_duration_matches_plan() {
        let plan = three_segment_plan();
        let execution = ExecutionPlanBuilder::build(&plan).expect("build");
        assert_eq!(plan.duration, 2.5);
        assert_eq!(execution.duration, plan.duration);
        assert_eq!(execution.duration, 2.5);
    }

    #[test]
    fn builder_is_pure() {
        let plan = three_segment_plan();
        let a = ExecutionPlanBuilder::build(&plan).expect("build");
        let b = ExecutionPlanBuilder::build(&plan).expect("build");
        assert_eq!(a, b);
        assert_eq!(a.segments, b.segments);
        assert_eq!(a.waypoints, b.waypoints);
        // Input is only borrowed — unchanged after build.
        assert_eq!(plan.segments.len(), 3);
        assert_eq!(plan.merged_trajectory.len(), 5);
        assert_eq!(plan.duration, 2.5);
    }

    #[test]
    fn execution_segment_retains_planned_segment_index() {
        let execution = ExecutionPlanBuilder::build(&three_segment_plan()).expect("build");
        assert_eq!(execution.segments.len(), 3);
        for (i, seg) in execution.segments.iter().enumerate() {
            assert_eq!(seg.planned_segment_index, i);
        }
    }

    /// Early regression gate: `CompiledPlan → ExecutionPlan` preserves the
    /// full execution structure — segment count, MoveJ/MoveL types, total
    /// duration, waypoint ranges, and absolute timestamps.
    #[test]
    fn compiled_plan_round_trip_preserves_execution_structure() {
        let plan = three_segment_plan();
        let execution = ExecutionPlanBuilder::build(&plan).expect("build");
        assert_eq!(execution.segments.len(), plan.segments.len());
        assert_eq!(execution.segments.len(), 3);
        assert!(matches!(
            execution.segments[0].instruction,
            ExecutionInstruction::MoveJ
        ));
        assert!(matches!(
            execution.segments[1].instruction,
            ExecutionInstruction::MoveL
        ));
        assert!(matches!(
            execution.segments[2].instruction,
            ExecutionInstruction::MoveJ
        ));
        assert_eq!(execution.duration, plan.duration);
        assert_eq!(execution.duration, 2.5);
        let ranges: Vec<_> = execution
            .segments
            .iter()
            .map(|s| s.waypoint_range.clone())
            .collect();
        assert_eq!(ranges, vec![0..2, 2..4, 4..5]);
        assert_eq!(execution.waypoints.len(), plan.waypoint_count);
        assert_eq!(execution.waypoints.len(), 5);
        let ts: Vec<f64> = execution.waypoints.iter().map(|w| w.timestamp).collect();
        assert_eq!(ts, vec![0.0, 0.3, 1.0, 1.5, 2.5]);
        assert_eq!(execution.waypoints[4].joints, vec![1.2, 1.0]);
    }
}
