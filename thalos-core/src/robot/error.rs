use crate::spatial::frame::FrameId;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum RobotBuilderError {
    #[error("Frame '{0}' not found in registry")]
    FrameNotFound(FrameId),

    #[error("End effector not defined")]
    EndEffectorNotDefined,
}
