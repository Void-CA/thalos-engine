//! Tier-1 motion intent types.
//!
//! A [`MotionRequest`] carries only the data its own intention needs. A joint
//! move holds joint values; a linear move holds a cartesian target. No variant
//! exposes the IK solver, the context, the planner or any per-stage
//! configuration — that knowledge lives in the engine (spec C-2).

use std::fmt;

use thalos_core::kinematics::inverse::IkError;
use thalos_core::trajectory::Trajectory;
use thalos_planning::error::PlanningError;

use crate::intent::kinematics::CartesianTarget;

/// Motion execution profile (Tier 2). Optional per request.
///
/// `Default` mirrors the engine's longstanding runtime/analysis values
/// (`1.0 / 0.5 / 0.01`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionProfile {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub time_step: f64,
}

impl Default for MotionProfile {
    fn default() -> Self {
        Self {
            max_velocity: 1.0,
            max_acceleration: 0.5,
            time_step: 0.01,
        }
    }
}

/// A joint-space target: "move the joints to these values".
#[derive(Debug, Clone)]
pub struct JointMove {
    target: Vec<f64>,
    profile: MotionProfile,
}

impl JointMove {
    /// Target joint values. The count must match the robot DOF.
    pub fn to(target: Vec<f64>) -> Self {
        Self {
            target,
            profile: MotionProfile::default(),
        }
    }

    /// Override the motion profile.
    pub fn with_profile(mut self, profile: MotionProfile) -> Self {
        self.profile = profile;
        self
    }

    pub fn target(&self) -> &[f64] {
        &self.target
    }

    pub fn profile(&self) -> MotionProfile {
        self.profile
    }
}

/// A cartesian target: "go linearly to here".
///
/// It holds an intention (a target) and, at most, a Tier-2 profile — never the
/// IK configuration, the sampling policy or the planner. The engine decides how
/// to reach it; that `IK → joint planning` pipeline is not visible here.
#[derive(Debug, Clone)]
pub struct LinearMove {
    target: CartesianTarget,
    profile: MotionProfile,
}

impl LinearMove {
    /// Move linearly to a cartesian target.
    pub fn to(target: impl Into<CartesianTarget>) -> Self {
        Self {
            target: target.into(),
            profile: MotionProfile::default(),
        }
    }

    /// Override the motion profile.
    pub fn with_profile(mut self, profile: MotionProfile) -> Self {
        self.profile = profile;
        self
    }

    pub fn target(&self) -> &CartesianTarget {
        &self.target
    }

    pub fn profile(&self) -> MotionProfile {
        self.profile
    }
}

/// A motion intention.
///
/// `#[non_exhaustive]`: later slices add variants (e.g. `Circular`) without
/// breaking downstream matches.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum MotionRequest {
    /// Move in joint space. Needs no IK.
    Joint(JointMove),
    /// Move linearly to a cartesian target. The engine resolves it internally.
    Linear(LinearMove),
}

/// The kind of motion a [`MotionPlan`] represents (Tier 1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionKind {
    Joint,
    Linear,
}

/// The plan a motion request produced (Tier 1).
///
/// Wraps the resulting trajectory opaquely so the underlying Tier 3 type can
/// evolve without breaking this surface. A `MotionPlan` is a **value**: it can
/// be produced once and passed across layers (e.g. to
/// [`prepare_execution`](crate::intent::prepare_execution)) without re-planning.
#[derive(Debug, Clone)]
pub struct MotionPlan {
    trajectory: Trajectory,
    kind: MotionKind,
}

impl MotionPlan {
    pub(crate) fn new(trajectory: Trajectory, kind: MotionKind) -> Self {
        Self { trajectory, kind }
    }

    /// The motion kind that produced this plan.
    pub fn kind(&self) -> MotionKind {
        self.kind
    }

    /// Access the underlying trajectory (escape hatch to Tier 3).
    pub fn trajectory(&self) -> &Trajectory {
        &self.trajectory
    }

    pub fn waypoint_count(&self) -> usize {
        self.trajectory.len()
    }

    pub fn duration(&self) -> f64 {
        self.trajectory.duration()
    }
}

/// Error produced by a motion request.
#[derive(Debug, Clone)]
pub enum MotionError {
    /// The start or target joint count does not match the robot DOF.
    DofMismatch { expected: usize, got: usize },
    /// A cartesian target the robot cannot reach (IK did not converge).
    Unreachable { final_error: f64 },
    /// The underlying kinematic solver reported an error.
    Kinematics(IkError),
    /// The underlying planner rejected the request.
    Planning(PlanningError),
}

impl fmt::Display for MotionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MotionError::DofMismatch { expected, got } => {
                write!(f, "joint count mismatch: expected {expected} DOF, got {got}")
            }
            MotionError::Unreachable { final_error } => {
                write!(f, "target unreachable (final error {final_error:.6})")
            }
            MotionError::Kinematics(e) => write!(f, "{e}"),
            MotionError::Planning(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for MotionError {}

impl From<IkError> for MotionError {
    fn from(e: IkError) -> Self {
        MotionError::Kinematics(e)
    }
}

impl From<PlanningError> for MotionError {
    fn from(e: PlanningError) -> Self {
        MotionError::Planning(e)
    }
}
