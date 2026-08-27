//! Optimization operators — concrete implementations of the
//! [`TrajectoryOperator`] trait.
//!
//! Each operator implements a specific optimization strategy that can
//! be applied to a problem region within a trajectory.

pub mod joint_centering;
pub mod retime;
pub mod sampling;

pub mod nullspace;
pub mod orientation_relaxation;

pub use joint_centering::JointCenteringOperator;
pub use nullspace::NullSpaceOptimization;
pub use orientation_relaxation::OrientationRelaxation;
pub use retime::Retime;
pub use sampling::AdaptiveSampling;
