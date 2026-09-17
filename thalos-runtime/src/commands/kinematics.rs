use crate::engine::core::kinematics::inverse::{IKGoal, IKResult};
use crate::engine::core::prelude::{FrameId, Pose};
use crate::engine::math::Vector3;

use crate::{RuntimeError, commands::handler::ExecutableCommand, robot::SceneRuntime};

#[derive(Debug, Clone)]
pub enum KinematicsCommand {
    MoveToPosition { frame: FrameId, target: Vector3 },
    MoveToPose { frame: FrameId, target: Pose },
}

impl ExecutableCommand for KinematicsCommand {
    type Output = IKResult;

    fn execute(&self, runtime: &mut SceneRuntime) -> Result<IKResult, RuntimeError> {
        match self {
            Self::MoveToPosition { frame, target } => Ok(runtime.solve_and_apply_ik(
                *frame,
                IKGoal::Position(*target),
            )?),
            Self::MoveToPose { frame, target } => {
                Ok(runtime.solve_and_apply_ik(*frame, IKGoal::Pose(target.clone()))?)
            }
        }
    }
}
