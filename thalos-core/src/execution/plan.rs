//! Execution IR — the pure data contract between planning output and
//! manifest generation.
//!
//! [`ExecutionPlan`] is the third IR in `thalos_core::execution`. It is
//! immutable after construction: all fields are public and read-only, and no
//! builder, mutator, or interior-mutability API is exposed. It carries
//! planning output (a trajectory with absolute timestamps, an ordered segment
//! list with `MoveJ`/`MoveL` instructions, and the total duration) with NO
//! planning, interpolation, transport, hardware, or runtime-event state.

use std::ops::Range;

use serde::{Deserialize, Serialize};

/// A single move instruction, preserved 1:1 from the source `MotionSegment`.
/// The system MUST NOT merge, split, or reclassify segments.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ExecutionInstruction {
    /// Joint-space move to a target configuration.
    MoveJ,
    /// Cartesian linear move to a target pose.
    MoveL,
}

/// One executed move, mapped 1:1 from a `PlannedSegment`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExecutionSegment {
    /// Ordinal of this segment within `ExecutionPlan.segments`.
    pub index: usize,
    /// Position of the source `PlannedSegment` in `CompiledPlan.segments`.
    ///
    /// Provenance invariant: preserved through the whole chain even though
    /// the ESP32 wire protocol never transmits it. Downstream builders derive
    /// segment identity from this, never by re-inferring structure.
    pub planned_segment_index: usize,
    /// The move instruction, derived from the source `MotionSegment`.
    pub instruction: ExecutionInstruction,
    /// Indices into `ExecutionPlan.waypoints` covered by this segment.
    pub waypoint_range: Range<usize>,
}

/// A single execution snapshot: joint positions at an absolute timestamp.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExecutionWaypoint {
    /// Joint positions, copied from `TrajectoryPoint.joints`.
    pub joints: Vec<f64>,
    /// Absolute time in seconds, matching `TrajectoryPoint.timestamp`.
    /// Monotonically non-decreasing across the plan.
    pub timestamp: f64,
}

/// Immutable execution IR: the ordered trajectory, its 1:1 segments, and the
/// total duration. Fields are readable without mutation; no mutator API.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExecutionPlan {
    /// Ordered waypoints with absolute timestamps (seconds) and joints.
    pub waypoints: Vec<ExecutionWaypoint>,
    /// Ordered segments, 1:1 with the source `PlannedSegment`s.
    pub segments: Vec<ExecutionSegment>,
    /// Total duration in seconds, copied from `CompiledPlan.duration`.
    pub duration: f64,
    /// Firmware-side repeat count (v3): `1` = single pass (default). The
    /// SceneService sets it to the `Repeat { count }` mode ONLY for hardware
    /// backends — the ESP32 executor loops the trajectory back-to-back with NO
    /// re-upload between passes. Simulation/Replay keep 1 and repeat via the
    /// host completion gate.
    pub repeat_count: u32,
}

/// Error produced by the pure builders of the execution chain
/// (`ExecutionPlanBuilder`, `ExecutionManifestBuilder`).
#[derive(Debug, thiserror::Error)]
pub enum BuilderError {
    /// Two consecutive waypoints share a timestamp but differ in position —
    /// never collapsed silently.
    #[error("duplicate timestamp {t} with different positions at waypoint {index}")]
    DedupConflict { index: usize, t: f64 },
    /// Validation failed (mirrors `firmware/esp32/src/validator.cpp` rules).
    #[error("validation failed: {0}")]
    Validation(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The Execution IR MUST be immutable after construction: every field is
    /// readable through shared references, no builder or interior-mutability
    /// API is exposed, and the value can be shared across threads unchanged.
    #[test]
    fn execution_plan_is_immutable() {
        let plan = ExecutionPlan {
            waypoints: vec![ExecutionWaypoint {
                joints: vec![0.0, 1.0],
                timestamp: 0.0,
            }],
            segments: vec![ExecutionSegment {
                index: 0,
                planned_segment_index: 0,
                instruction: ExecutionInstruction::MoveJ,
                waypoint_range: 0..1,
            }],
            duration: 0.0,
            repeat_count: 1,
        };

        assert_eq!(plan.waypoints.len(), 1);
        assert_eq!(plan.waypoints[0].joints, vec![0.0, 1.0]);
        assert_eq!(plan.waypoints[0].timestamp, 0.0);
        assert_eq!(plan.segments.len(), 1);
        assert_eq!(plan.segments[0].index, 0);
        assert_eq!(plan.segments[0].planned_segment_index, 0);
        assert_eq!(plan.segments[0].instruction, ExecutionInstruction::MoveJ);
        assert_eq!(plan.segments[0].waypoint_range, 0..1);
        assert_eq!(plan.duration, 0.0);

        // Clone yields an equal value; mutating the clone leaves the original
        // untouched — values are shared, not moved.
        let clone = plan.clone();
        assert_eq!(clone, plan);
        assert_eq!(clone.waypoints[0].joints, plan.waypoints[0].joints);

        // No interior mutability: an `ExecutionPlan` is freely shareable
        // across threads (`Send + Sync` fails to compile for Cell/RefCell/Rc).
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<ExecutionPlan>();
        assert_send_sync::<ExecutionSegment>();
        assert_send_sync::<ExecutionWaypoint>();
        assert_send_sync::<ExecutionInstruction>();
    }
}
