use crate::models::{RobotModel, RobotSpec};

#[derive(Debug, thiserror::Error)]
pub enum RobotModelError {
    #[error("robot model and spec do not match")]
    ModelSpecMismatch { model: RobotModel, spec: RobotSpec },

    #[error("invalid robot id: {id}")]
    InvalidRobotId { id: String },
}
