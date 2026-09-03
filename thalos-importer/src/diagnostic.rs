use serde::{Deserialize, Serialize};

/// Stable identifier for import diagnostic categories.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DiagnosticCode {
    MissingJointLimit,
    UnresolvedMeshReference,
    UnresolvedParentLink,
    AmbiguousJointType,
    MalformedGeometry,
    Custom(String),
}

/// A structured diagnostic message emitted during import or normalization.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ImportDiagnostic {
    Info {
        code: DiagnosticCode,
        message: String,
    },
    Warning {
        code: DiagnosticCode,
        message: String,
    },
    Error {
        code: DiagnosticCode,
        message: String,
    },
}

impl ImportDiagnostic {
    pub fn info(code: DiagnosticCode, message: impl Into<String>) -> Self {
        Self::Info {
            code,
            message: message.into(),
        }
    }

    pub fn warning(code: DiagnosticCode, message: impl Into<String>) -> Self {
        Self::Warning {
            code,
            message: message.into(),
        }
    }

    pub fn error(code: DiagnosticCode, message: impl Into<String>) -> Self {
        Self::Error {
            code,
            message: message.into(),
        }
    }

    pub fn is_error(&self) -> bool {
        matches!(self, Self::Error { .. })
    }
}
