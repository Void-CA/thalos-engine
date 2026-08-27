use crate::robot::joint::JointKind;
use thiserror::Error;

/// Errors returned by the inverse kinematics solvers.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum IkError {
    /// The robot contains a joint kind the IK solvers cannot actuate
    /// (e.g. `Floating` or `Planar`). These joints consume more than one
    /// DOF in the `q` vector and are filtered out by the clamp loop.
    #[error("unsupported joint type for IK: {0}")]
    UnsupportedJointType(JointKind),
}
