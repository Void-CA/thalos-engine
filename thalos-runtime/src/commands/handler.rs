use crate::{RuntimeError, robot::SceneRuntime};

pub trait ExecutableCommand {
    type Output;

    fn execute(&self, runtime: &mut SceneRuntime) -> Result<Self::Output, RuntimeError>;
}
