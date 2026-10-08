//! **Tier 1 — intention-oriented API** (spec `docs/API-INTENT-SPEC.md`).
//!
//! The consumer expresses *what* it wants without assembling the mechanism.
//! The Tier 3 types (`SerialChain`, `DampedLeastSquaresSolver`, `MoveJPlanner`,
//! `PlanCompiler`, …) remain public and reachable, but they stop being the
//! entry point.
//!
//! The receiver is [`Manipulator`], a **stateless** capability value: it holds
//! the robot model, never pipeline state (no active plan, no session, no current
//! joints). Execution state belongs to the application.
//!
//! Implemented slices: D1 (joint motion), D2 (cartesian/linear motion), D3
//! (motion plan → execution preparation), D4 (inverse kinematics) and D5 (robot
//! loading). The main path is complete:
//!
//! ```text
//! Manipulator::from_urdf ─► Manipulator ─► intent ─► MotionPlan ─► PreparedPlan
//! ```
//!
//! Program planning (`Plan` from `.thls` source) is deliberately deferred until
//! there is evidence that it is needed.

pub mod execution;
pub mod kinematics;
pub mod manipulator;
pub mod motion;

pub use execution::{PreparedPlan, prepare_execution};
pub use kinematics::{CartesianTarget, IkRequest, IkSolution};
pub use manipulator::{Manipulator, RobotLoadError};

/// The world TCP pose produced by forward kinematics (a plain value type,
/// re-exported so the consumer needs no internal path).
pub use thalos_core::kinematics::context::TcpPose;
pub use motion::{
    JointMove, LinearMove, MotionError, MotionKind, MotionPlan, MotionProfile, MotionRequest,
};
