//! URDF format parser and exporter.
//!
//! Currently only the parser is implemented. See the
//! [`parser`] module for usage.

pub mod attr;
pub mod elements;
pub mod error;
pub mod parser;

// Re-export for convenient access.
pub use error::UrdfError;
