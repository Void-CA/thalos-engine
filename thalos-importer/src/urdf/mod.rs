pub mod attr;
pub mod elements;
pub mod error;
pub mod parser;

pub use error::UrdfError;
pub use parser::parse;

use thalos_models::Robot;
use crate::error::ImportError;
use crate::import_urdf_resolved;
use crate::assets::resolver::Resolution;

/// Import a URDF XML string into a native [`Robot`](thalos_models::Robot) domain model.
///
/// This is the public facade of the importer. Internally it:
/// 1. Parses XML into an [`ImportedCandidate`]
/// 2. Validates and normalizes the candidate into a domain [`Robot`]
///
/// Callers receive a fully normalized robot; the intermediate
/// representation is an implementation detail.
///
/// This is equivalent to calling [`import_urdf_resolved`] with an empty
/// [`Resolution`](crate::assets::resolver::Resolution).
pub fn import_urdf(source: &str) -> Result<Robot, ImportError> {
    let result = import_urdf_resolved(source, &Resolution::default())?;
    Ok(result.robot)
}
