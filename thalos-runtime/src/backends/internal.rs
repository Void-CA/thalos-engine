use thalos_engine::core::models::RobotModel;

use crate::error::RuntimeError;

use super::RobotBackend;

/// Default backend that resolves robots from the built-in catalog.
pub struct InternalBackend;

impl RobotBackend for InternalBackend {
    fn resolve_model(&self, id: &str) -> Result<RobotModel, RuntimeError> {
        Ok(RobotModel::from_id(id)?)
    }
}
