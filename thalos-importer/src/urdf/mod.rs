pub mod attr;
pub mod elements;
pub mod error;
pub mod parser;

pub use error::UrdfError;
pub use parser::parse;

use thalos_models::Robot;
use crate::candidate::ImportedCandidate;
use crate::error::ImportError;
use crate::normalize::{CandidateNormalizer, Normalizer};

/// Import a URDF XML string into a native [`Robot`](thalos_models::Robot) domain model.
///
/// This is the public facade of the importer. Internally it:
/// 1. Parses XML into an [`ImportedCandidate`]
/// 2. Validates and normalizes the candidate into a domain [`Robot`]
///
/// Callers receive a fully normalized robot; the intermediate
/// representation is an implementation detail.
pub fn import_urdf(source: &str) -> Result<Robot, ImportError> {
    let candidate: ImportedCandidate = parse(source).map_err(ImportError::from)?;
    let normalizer = CandidateNormalizer::new();
    let result = normalizer.normalize(&candidate)?;
    Ok(result.robot)
}
