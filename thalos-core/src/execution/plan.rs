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
pub enum PlanInstruction {
    /// Joint-space move to a target configuration.
    MoveJ,
    /// Cartesian linear move to a target pose.
    MoveL,
    /// Cartesian circular move through a via point.
    MoveC,
    /// Temporal step: hold the current configuration for `seconds`.
    Delay { seconds: f64 },
    /// Operational step: set an output channel to a value (no geometry).
    SetOutput { channel: String, value: bool },
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
    pub instruction: PlanInstruction,
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
    /// Program identifier for provenance tracking.
    pub program_id: Option<String>,
    /// Revision counter of the source program when this plan was built.
    pub program_revision: Option<u64>,
    /// Cryptographic fingerprint (SHA-256) of the source code that generated this plan.
    pub source_fingerprint: Option<String>,
    /// Robot identifier targeted by this plan.
    pub robot_id: Option<String>,
}

impl ExecutionPlan {
    pub fn with_provenance(
        mut self,
        program_id: impl Into<String>,
        program_revision: u64,
        source_fingerprint: impl Into<String>,
        robot_id: Option<String>,
    ) -> Self {
        self.program_id = Some(program_id.into());
        self.program_revision = Some(program_revision);
        self.source_fingerprint = Some(source_fingerprint.into());
        self.robot_id = robot_id;
        self
    }

    /// Checks if the plan is stale with respect to a current program revision and source fingerprint.
    pub fn is_stale_for(&self, current_revision: u64, current_fingerprint: &str) -> bool {
        if let Some(rev) = self.program_revision
            && rev != current_revision {
            return true;
        }
        if let Some(ref fp) = self.source_fingerprint
            && fp != current_fingerprint {
            return true;
        }
        false
    }

    /// The plan's own end time (last waypoint timestamp). `0.0` if empty.
    pub fn end_time(&self) -> f64 {
        self.waypoints
            .iter()
            .map(|w| w.timestamp)
            .fold(0.0_f64, f64::max)
    }

    /// Index of the waypoint the plan requires at `elapsed_secs`.
    ///
    /// This is the SINGLE plan-derived definition of "expected": both the
    /// dispatch path and tick evaluation resolve the expected state through it.
    ///
    /// Returns `None` for an empty plan, or once `elapsed_secs` is strictly past
    /// the plan end (the FINAL waypoint is still current AT the end).
    pub fn expected_waypoint_index_at(&self, elapsed_secs: f64) -> Option<usize> {
        if self.waypoints.is_empty() || elapsed_secs > self.end_time() {
            return None;
        }
        let idx = self
            .waypoints
            .partition_point(|w| w.timestamp <= elapsed_secs);
        Some(idx.saturating_sub(1).min(self.waypoints.len() - 1))
    }

    /// Expected joint configuration at `elapsed_secs`, derived from the plan.
    pub fn expected_joints_at(&self, elapsed_secs: f64) -> Option<Vec<f64>> {
        self.expected_waypoint_index_at(elapsed_secs)
            .map(|i| self.waypoints[i].joints.clone())
    }
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
                instruction: PlanInstruction::MoveJ,
                waypoint_range: 0..1,
            }],
            duration: 0.0,
            repeat_count: 1,
            program_id: Some("prog-1".to_string()),
            program_revision: Some(12),
            source_fingerprint: Some("hash-abc".to_string()),
            robot_id: Some("robot-1".to_string()),
        };

        assert_eq!(plan.waypoints.len(), 1);
        assert_eq!(plan.waypoints[0].joints, vec![0.0, 1.0]);
        assert_eq!(plan.waypoints[0].timestamp, 0.0);
        assert_eq!(plan.segments.len(), 1);
        assert_eq!(plan.segments[0].index, 0);
        assert_eq!(plan.segments[0].planned_segment_index, 0);
        assert_eq!(plan.segments[0].instruction, PlanInstruction::MoveJ);
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
        assert_send_sync::<PlanInstruction>();
    }

    /// The expected state is PLAN-DERIVED: it is a pure function of the plan and
    /// the elapsed time, and it exists both when the plan still has work and at
    /// its final waypoint — but not past the end.
    #[test]
    fn expected_state_is_derived_from_the_plan() {
        let plan = ExecutionPlan {
            waypoints: vec![
                ExecutionWaypoint {
                    joints: vec![0.0],
                    timestamp: 0.0,
                },
                ExecutionWaypoint {
                    joints: vec![2.0],
                    timestamp: 2.0,
                },
            ],
            segments: vec![ExecutionSegment {
                index: 0,
                planned_segment_index: 0,
                instruction: PlanInstruction::MoveL,
                waypoint_range: 0..2,
            }],
            duration: 2.0,
            repeat_count: 1,
            program_id: None,
            program_revision: None,
            source_fingerprint: None,
            robot_id: None,
        };

        assert_eq!(plan.end_time(), 2.0);
        assert_eq!(plan.expected_joints_at(1.5), Some(vec![0.0]));
        // Final waypoint is current AT the end, and past the end there is none.
        assert_eq!(plan.expected_joints_at(2.0), Some(vec![2.0]));
        assert_eq!(plan.expected_joints_at(2.5), None);
    }
}
