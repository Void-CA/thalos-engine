use std::fmt;

#[derive(Debug, Clone)]
pub enum ImportError {
    Urdf(String),
    UnsupportedFormat(String),
    MissingRootLink,
}

impl fmt::Display for ImportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ImportError::Urdf(msg) => write!(f, "URDF import error: {msg}"),
            ImportError::UnsupportedFormat(fmt_str) => write!(f, "unsupported format: {fmt_str}"),
            ImportError::MissingRootLink => write!(f, "missing root link in imported model"),
        }
    }
}

impl std::error::Error for ImportError {}

impl From<crate::urdf::error::UrdfError> for ImportError {
    fn from(e: crate::urdf::error::UrdfError) -> Self {
        ImportError::Urdf(e.to_string())
    }
}
