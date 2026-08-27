//! Tests for the logical → temporal event transformation (IR-3).
//!
//! The `TimelineScheduler` walks the IR-1 instruction stream alongside the
//! `CompiledPlan` segment timing and assigns absolute `at_time` (from plan
//! start, `t=0`) to the logical `RuntimeProgram` events produced by the
//! resolver (I5: trajectory/events separation — timing lives only here).

use std::time::Duration;

use thalos_core::{
    execution::program::{ExecutionInstruction, ExecutionMetadata, ExecutionProgram},
    execution::runtime::{RuntimeAction, RuntimeEvent, RuntimeProgram},
    ids::OperationId,
    motion::segment::MotionSegment,
    motion::target::{MotionPose, MotionProfile, MotionTarget, OutputChannel, OutputValue},
    prelude::Trajectory,
    trajectory::TrajectoryPoint,
};

use super::TimelineScheduler;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn origin(op: &str) -> OperationId {
    OperationId(op.to_string())
}

fn sample_metadata() -> ExecutionMetadata {
    ExecutionMetadata {
        schema_version: 1,
        source_project: "test".into(),
    }
}

fn sample_pose(x: f64) -> MotionPose {
    MotionPose {
        position: [x, 0.0, 0.0],
        orientation: [0.0, 0.0, 0.0, 1.0],
        frame: "world".into(),
    }
}

fn default_profile() -> MotionProfile {
    MotionProfile {
        max_velocity: 500.0,
        max_acceleration: 1000.0,
        max_jerk: None,
    }
}

/// Build an `ExecutionProgram` from instructions.
fn program_with(instructions: Vec<ExecutionInstruction>) -> ExecutionProgram {
    ExecutionProgram {
        instructions,
        metadata: sample_metadata(),
    }
}

/// Build a `CompiledPlan` with one MoveJ segment spanning `[0.0, duration]`
/// and a linear joint trajectory, plus an optional second segment spanning
/// `[start, end]`.
fn compiled_with_segments(segments: Vec<(f64, f64)>) -> crate::motion::program::CompiledPlan {
    let mut planned = Vec::new();
    for (i, (start, end)) in segments.iter().enumerate() {
        let source = MotionSegment::MoveJ {
            origin: origin(&format!("op-seg-{i}")),
            target: vec![end - start],
            max_velocity: Some(500.0),
            max_acceleration: Some(1000.0),
        };
        let trajectory = Trajectory::new(vec![
            TrajectoryPoint::new(vec![*start], *start),
            TrajectoryPoint::new(vec![*end], *end),
        ]);
        planned.push(crate::motion::program::PlannedSegment {
            origin: origin(&format!("op-seg-{i}")),
            source,
            trajectory,
            waypoint_range: 2 * i..2 * i + 2,
            time_range: *start..*end,
            operation_id: None,
            role: None,
        });
    }
    let merged = Trajectory::new(
        planned
            .iter()
            .flat_map(|s| s.trajectory.waypoints().to_vec())
            .collect(),
    );
    crate::motion::program::CompiledPlan::new(merged, planned)
}

fn set_output_event(op: &str) -> RuntimeEvent {
    RuntimeEvent {
        at_time: Duration::ZERO, // logical — no timing yet
        operation_id: origin(op),
        action: RuntimeAction::SetOutput {
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        },
    }
}

fn delay_event(op: &str, duration: Duration) -> RuntimeEvent {
    RuntimeEvent {
        at_time: Duration::ZERO,
        operation_id: origin(op),
        action: RuntimeAction::Delay(duration),
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn empty_program_produces_empty_runtime_program() {
    let program = program_with(vec![]);
    let compiled = compiled_with_segments(vec![]);
    let logical = RuntimeProgram { events: vec![] };

    let temporal = TimelineScheduler::new().schedule(&program, &compiled, logical);
    assert!(temporal.events.is_empty(), "no instructions → no events");
}

#[test]
fn set_output_after_motion_gets_segment_end_time() {
    // MoveJ [0, 1.0] then SetOutput → at_time must be 1.0s (absolute).
    let program = program_with(vec![
        ExecutionInstruction::MoveJ {
            origin: origin("op-j"),
            target: MotionTarget::Pose(sample_pose(1.0)),
            profile: default_profile(),
        },
        ExecutionInstruction::SetOutput {
            origin: origin("op-out"),
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        },
    ]);
    let compiled = compiled_with_segments(vec![(0.0, 1.0)]);
    let logical = RuntimeProgram {
        events: vec![set_output_event("op-out")],
    };

    let temporal = TimelineScheduler::new().schedule(&program, &compiled, logical);
    assert_eq!(temporal.events.len(), 1);
    assert_eq!(
        temporal.events[0].at_time,
        Duration::from_secs_f64(1.0),
        "SetOutput after a 1s segment fires at absolute t=1.0s"
    );
}

#[test]
fn delay_contributes_duration_to_timeline_cursor() {
    // MoveJ [0, 1.0], Delay(500ms), SetOutput.
    // Delay fires at 1.0s; the SetOutput fires 500ms later → at_time = 1.5s.
    let program = program_with(vec![
        ExecutionInstruction::MoveJ {
            origin: origin("op-j"),
            target: MotionTarget::Pose(sample_pose(1.0)),
            profile: default_profile(),
        },
        ExecutionInstruction::Delay {
            origin: origin("op-wait"),
            duration: Duration::from_millis(500),
        },
        ExecutionInstruction::SetOutput {
            origin: origin("op-out"),
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        },
    ]);
    let compiled = compiled_with_segments(vec![(0.0, 1.0)]);
    let logical = RuntimeProgram {
        events: vec![
            delay_event("op-wait", Duration::from_millis(500)),
            set_output_event("op-out"),
        ],
    };

    let temporal = TimelineScheduler::new().schedule(&program, &compiled, logical);
    assert_eq!(temporal.events.len(), 2);
    assert_eq!(
        temporal.events[0].at_time,
        Duration::from_secs_f64(1.0),
        "Delay starts when the preceding segment ends"
    );
    assert_eq!(
        temporal.events[1].at_time,
        Duration::from_secs_f64(1.5),
        "post-delay event fires at absolute 1.5s (delay duration added to cursor)"
    );
}

#[test]
fn spec_scenario_delay_then_post_delay_absolute() {
    // Spec scenario (runtime-event-timeline): Delay at at_time 1.0s with
    // 500ms → trajectory holds until clock 1.5s → SetOutput at 2.0s fires
    // exactly at 2.0s from plan start (not 0.5s after the delay).
    let program = program_with(vec![
        ExecutionInstruction::MoveJ {
            origin: origin("op-move-1"),
            target: MotionTarget::Pose(sample_pose(1.0)),
            profile: default_profile(),
        },
        ExecutionInstruction::Delay {
            origin: origin("op-wait"),
            duration: Duration::from_millis(500),
        },
        ExecutionInstruction::MoveL {
            origin: origin("op-move-2"),
            target: MotionTarget::Pose(sample_pose(2.0)),
            profile: default_profile(),
        },
        ExecutionInstruction::SetOutput {
            origin: origin("op-out"),
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        },
    ]);
    // Trajectory: MoveJ [0,1.0], MoveL [1.0,1.5] (0.5s each in traj time).
    let compiled = compiled_with_segments(vec![(0.0, 1.0), (1.0, 1.5)]);
    let logical = RuntimeProgram {
        events: vec![
            delay_event("op-wait", Duration::from_millis(500)),
            set_output_event("op-out"),
        ],
    };

    let temporal = TimelineScheduler::new().schedule(&program, &compiled, logical);
    assert_eq!(temporal.events.len(), 2);
    assert_eq!(
        temporal.events[0].at_time,
        Duration::from_secs_f64(1.0),
        "Delay fires at absolute 1.0s"
    );
    assert_eq!(
        temporal.events[1].at_time,
        Duration::from_secs_f64(2.0),
        "post-delay SetOutput fires at absolute 2.0s, not 0.5s after the delay"
    );
}

#[test]
fn at_time_independent_of_segment_ordering() {
    // Spec scenario: event at at_time = 5.0s in a plan where the preceding
    // segment ends at 3.0s → fires at t=5.0s from plan start, not 2.0s
    // after the prior segment.
    let program = program_with(vec![
        ExecutionInstruction::MoveJ {
            origin: origin("op-move"),
            target: MotionTarget::Pose(sample_pose(3.0)),
            profile: default_profile(),
        },
        ExecutionInstruction::Delay {
            origin: origin("op-wait"),
            duration: Duration::from_secs(2),
        },
        ExecutionInstruction::SetOutput {
            origin: origin("op-out"),
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        },
    ]);
    let compiled = compiled_with_segments(vec![(0.0, 3.0)]);
    let logical = RuntimeProgram {
        events: vec![
            delay_event("op-wait", Duration::from_secs(2)),
            set_output_event("op-out"),
        ],
    };

    let temporal = TimelineScheduler::new().schedule(&program, &compiled, logical);
    assert_eq!(
        temporal.events[0].at_time,
        Duration::from_secs(3),
        "Delay starts at segment end (3.0s)"
    );
    assert_eq!(
        temporal.events[1].at_time,
        Duration::from_secs(5),
        "SetOutput fires at absolute 5.0s (segment end + delay), not 2.0s after the segment"
    );
}

#[test]
fn events_are_sorted_by_absolute_time() {    // Even if the logical events arrive in program order with zero at_time,
    // the scheduler output must be sorted by at_time (spec: RuntimeProgram
    // Structure). Cursor is monotonic, so this holds by construction.
    let program = program_with(vec![
        ExecutionInstruction::Delay {
            origin: origin("op-wait-1"),
            duration: Duration::from_millis(500),
        },
        ExecutionInstruction::MoveJ {
            origin: origin("op-j"),
            target: MotionTarget::Pose(sample_pose(1.0)),
            profile: default_profile(),
        },
        ExecutionInstruction::SetOutput {
            origin: origin("op-out"),
            channel: OutputChannel {
                name: "gripper".into(),
                channel_type: "digital".into(),
            },
            value: OutputValue::Bool(true),
        },
    ]);
    let compiled = compiled_with_segments(vec![(0.0, 1.0)]);
    let logical = RuntimeProgram {
        events: vec![
            delay_event("op-wait-1", Duration::from_millis(500)),
            set_output_event("op-out"),
        ],
    };

    let temporal = TimelineScheduler::new().schedule(&program, &compiled, logical);
    assert_eq!(temporal.events.len(), 2);
    let times: Vec<Duration> = temporal.events.iter().map(|e| e.at_time).collect();
    assert_eq!(
        times,
        vec![Duration::ZERO, Duration::from_secs_f64(1.5)],
        "events must be ordered by at_time (Delay fires at t=0, cursor→0.5s; MoveJ adds 1s → SetOutput at 1.5s)"
    );
}
