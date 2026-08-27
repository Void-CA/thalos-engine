use serde::{Deserialize, Serialize};
use std::fmt;

// ---------------------------------------------------------------------------
// ID newtypes — String-backed, serde-compatible, type-safe identifiers
// ---------------------------------------------------------------------------

macro_rules! id_newtype {
    ($name:ident) => {
        #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(pub String);

        impl $name {
            /// View the inner string as a `&str`.
            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                self.0.fmt(f)
            }
        }
    };
}

/// Unique identifier for an operation.
///
/// String-backed for JSON readability. Single source of truth
/// used across all crates — eliminates conversion at crate boundaries.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct OperationId(pub String);

impl OperationId {
    /// View the inner string as a `&str`.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for OperationId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

// ---------------------------------------------------------------------------
// Semantic resource identifiers
// ---------------------------------------------------------------------------

id_newtype!(ObjectId);
id_newtype!(LocationId);
id_newtype!(ToolId);
id_newtype!(TaskDocumentId);

// ---------------------------------------------------------------------------
// Analysis artifact identifiers — anchors for observations (spec I3)
// ---------------------------------------------------------------------------
// Defined in core so the analysis model never depends on planning/runtime
// crates for identity (design D1, cycle-free layering).

id_newtype!(RobotId);
id_newtype!(SceneId);
id_newtype!(SemanticProgramId);
id_newtype!(MotionPlanId);
id_newtype!(ExecutionSessionId);
