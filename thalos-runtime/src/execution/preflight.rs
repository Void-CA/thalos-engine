use serde::{Deserialize, Serialize};

/// PreflightCheckKind (ADR-014)
/// Domain categories for preflight readiness checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PreflightCheckKind {
    Plan,
    Resource,
    Robot,
    Transport,
    Safety,
}

/// PreflightStatus (ADR-014)
/// Outcome status of an individual check.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PreflightStatus {
    Passed,
    Failed,
    Warning,
    Skipped,
}

/// PreflightCheck (ADR-014)
/// Granular report entry detailing status and diagnostic feedback.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PreflightCheck {
    pub kind: PreflightCheckKind,
    pub status: PreflightStatus,
    pub message: Option<String>,
}

impl PreflightCheck {
    pub fn pass(kind: PreflightCheckKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            status: PreflightStatus::Passed,
            message: Some(message.into()),
        }
    }

    pub fn fail(kind: PreflightCheckKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            status: PreflightStatus::Failed,
            message: Some(message.into()),
        }
    }

    pub fn skip(kind: PreflightCheckKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            status: PreflightStatus::Skipped,
            message: Some(message.into()),
        }
    }
}

/// ExecutionPreflight (ADR-014)
/// Immutable evaluation report determining whether an ExecutionRequest can be safely dispatched.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionPreflight {
    pub checks: Vec<PreflightCheck>,
    pub can_dispatch: bool,
}

impl ExecutionPreflight {
    pub fn new(checks: Vec<PreflightCheck>) -> Self {
        let can_dispatch = !checks.iter().any(|c| c.status == PreflightStatus::Failed);
        Self { checks, can_dispatch }
    }
}
