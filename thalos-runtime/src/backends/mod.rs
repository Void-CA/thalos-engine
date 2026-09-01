pub mod controller;
pub mod esp32;
mod internal;
pub mod manager;
pub mod transport;

pub use internal::InternalBackend;
pub use manager::{BackendEntry, BackendManager};

use thalos_engine::core::models::RobotModel;

use crate::error::RuntimeError;

/// Strategy for resolving robot models from identifiers.
///
/// The default implementation is [`InternalBackend`], which resolves against
/// the built-in robot catalog in `thalos-core`. Custom backends can integrate
/// external sources (hardware discovery, config files, network, etc.).
pub trait RobotBackend: Send + Sync {
    /// Resolve a robot model by its string identifier.
    fn resolve_model(&self, id: &str) -> Result<RobotModel, RuntimeError>;
}
