//! Unit tests for `WorkspaceError` enum.
//!
//! Covers: Display impl (per `#[error("...")]`), Debug impl, PartialEq,
//! and that the four variants exist with the documented fields.

use crate::analysis::workspace::WorkspaceError;

#[test]
fn invalid_sample_count_displays_value() {
    let err = WorkspaceError::InvalidSampleCount(0);
    assert_eq!(err.to_string(), "sample count must be > 0, got 0");
}

#[test]
fn invalid_tolerance_displays_value() {
    let err = WorkspaceError::InvalidTolerance(-1.0);
    assert_eq!(err.to_string(), "tolerance must be >= 0, got -1");
}

#[test]
fn invalid_point_displays_message() {
    let err = WorkspaceError::InvalidPoint("x is NaN".to_string());
    assert_eq!(err.to_string(), "point has non-finite coordinate: x is NaN");
}

#[test]
fn empty_workspace_displays_message() {
    let err = WorkspaceError::EmptyWorkspace;
    assert_eq!(err.to_string(), "workspace is empty");
}

#[test]
fn variants_compare_equal_with_same_fields() {
    assert_eq!(
        WorkspaceError::InvalidSampleCount(0),
        WorkspaceError::InvalidSampleCount(0),
    );
    assert_ne!(
        WorkspaceError::InvalidSampleCount(0),
        WorkspaceError::InvalidSampleCount(1),
    );
}

#[test]
fn debug_impl_does_not_panic() {
    let _ = format!("{:?}", WorkspaceError::EmptyWorkspace);
    let _ = format!("{:?}", WorkspaceError::InvalidTolerance(-0.5));
}
