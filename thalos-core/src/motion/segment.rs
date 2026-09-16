use crate::ids::OperationId;
use crate::spatial::frame::FrameId;
use crate::spatial::pose::Pose;
use serde::{Deserialize, Serialize};

/// A single movement command in a motion program.
///
/// This represents the *intent* — what the user wants to happen, not the
/// planned result. The `PlanCompiler` transforms this into a `PlannedSegment`.
///
/// Every variant carries an `origin: OperationId` linking it to the source
/// IR-0 operation. Origin MUST survive every transformation from IR-0 through
/// IR-3 unchanged (invariant I2).
///
/// # Extensibility
///
/// New variants (e.g. `Wait`, `SetTool`, `IO`) can be added without changing
/// the compiler — only the dispatcher needs a new arm.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MotionSegment {
    /// Joint-space move to a target configuration.
    MoveJ {
        /// The IR-0 operation this segment was derived from.
        origin: OperationId,
        target: Vec<f64>,
        max_velocity: Option<f64>,
        max_acceleration: Option<f64>,
    },
    /// Cartesian linear move to a target pose.
    MoveL {
        /// The IR-0 operation this segment was derived from.
        origin: OperationId,
        frame: FrameId,
        target_pose: Pose,
        max_velocity: Option<f64>,
    },
    /// Cartesian linear move to a target **position** — orientation is
    /// unconstrained (resolved from the current configuration via
    /// `IKGoal::Position`). Required for robots that cannot reach a full
    /// 6-DOF pose (e.g. SCARA: yaw-only).
    MoveLPosition {
        /// The IR-0 operation this segment was derived from.
        origin: OperationId,
        frame: FrameId,
        target_position: [f64; 3],
        max_velocity: Option<f64>,
    },
    /// Cartesian **circular** move through a via point to a target position.
    ///
    /// The arc is defined by three points in world space: the current TCP
    /// position (start), `via_position`, and `target_position`. Orientation is
    /// unconstrained (position-only IK), matching `MoveLPosition`.
    MoveC {
        /// The IR-0 operation this segment was derived from.
        origin: OperationId,
        frame: FrameId,
        via_position: [f64; 3],
        target_position: [f64; 3],
        max_velocity: Option<f64>,
    },
    /// Temporal step: hold the current configuration for `seconds`.
    ///
    /// Non-geometric but temporal — it MUST appear in the plan so the total
    /// duration and the timeline represent the program faithfully.
    Delay {
        origin: OperationId,
        seconds: f64,
    },
    /// Operational step: set an output channel to a value.
    ///
    /// Non-geometric and non-temporal (zero duration) — it produces no
    /// waypoints but MUST appear in the plan so no instruction is dropped.
    SetOutput {
        origin: OperationId,
        channel: String,
        value: bool,
    },
}

impl MotionSegment {
    /// The `OperationId` this segment was derived from (invariant I2).
    pub fn origin(&self) -> &OperationId {
        match self {
            MotionSegment::MoveJ { origin, .. }
            | MotionSegment::MoveL { origin, .. }
            | MotionSegment::MoveLPosition { origin, .. }
            | MotionSegment::MoveC { origin, .. }
            | MotionSegment::Delay { origin, .. }
            | MotionSegment::SetOutput { origin, .. } => origin,
        }
    }
}
