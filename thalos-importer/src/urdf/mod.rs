pub mod attr;
pub mod elements;
pub mod error;
pub mod parser;

pub use error::UrdfError;
pub use parser::parse_robot;

use thalos_models::Robot;
use crate::error::ImportError;

/// Import a URDF XML string into a native [`Robot`](thalos_models::Robot) domain model.
pub fn import_urdf(source: &str) -> Result<Robot, ImportError> {
    parse_robot(source).map_err(ImportError::from)
}
