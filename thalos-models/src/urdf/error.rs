//! URDF parser error type.

use std::fmt;

/// Errors that can occur during URDF parsing.
#[derive(Debug, Clone)]
pub enum UrdfError {
    /// Invalid XML or I/O error.
    Xml(String),
    /// Required attribute is missing on an element.
    MissingAttribute { element: String, attribute: String },
    /// Required child element is missing.
    MissingElement { parent: String, child: String },
    /// A numeric value could not be parsed.
    ParseFloat { value: String, source: String },
    /// A space-separated tuple (xyz, rpy, rgba, …) has the wrong
    /// number of components.
    TupleLength {
        element: String,
        expected: usize,
        got: usize,
    },
    /// Unknown joint type string.
    UnknownJointType(String),
    /// Zero axis vector (must be non-zero for 1-DOF joints).
    ZeroAxis,
    /// Required attribute `name` is missing or empty.
    UnnamedElement(String),
}

impl fmt::Display for UrdfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            UrdfError::Xml(msg) => write!(f, "XML error: {msg}"),
            UrdfError::MissingAttribute { element, attribute } => {
                write!(f, "<{element}> is missing required attribute `{attribute}`")
            }
            UrdfError::MissingElement { parent, child } => {
                write!(f, "<{parent}> is missing required child <{child}>")
            }
            UrdfError::ParseFloat { value, source } => {
                write!(f, "cannot parse float `{value}`: {source}")
            }
            UrdfError::TupleLength {
                element,
                expected,
                got,
            } => {
                write!(f, "<{element}>: expected {expected} values, got {got}")
            }
            UrdfError::UnknownJointType(t) => {
                write!(f, "unknown joint type `{t}`")
            }
            UrdfError::ZeroAxis => {
                write!(f, "joint axis must be a non-zero vector")
            }
            UrdfError::UnnamedElement(e) => {
                write!(f, "<{e}> is missing the required `name` attribute")
            }
        }
    }
}

impl std::error::Error for UrdfError {}

impl From<quick_xml::Error> for UrdfError {
    fn from(e: quick_xml::Error) -> Self {
        UrdfError::Xml(e.to_string())
    }
}
