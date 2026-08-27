use std::fmt;

/// Errors that can occur during trajectory optimization.
#[derive(Debug, Clone)]
pub enum OptimizationError {
    /// An operator failed during `apply()`.
    OperatorFailed {
        operator: &'static str,
        reason: String,
    },
    /// No applicable operator could be found for a region.
    NoApplicableOperator,
    /// Maximum number of iterations was reached without convergence.
    MaxIterationsReached(u32),
    /// A kinematics-related error occurred (e.g. IK failure).
    Kinematics(String),
    /// An evaluation/scoring error occurred.
    Evaluation(String),
    /// The specified region is invalid or out of bounds.
    InvalidRegion(String),
}

impl fmt::Display for OptimizationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            OptimizationError::OperatorFailed { operator, reason } => {
                write!(f, "operator '{}' failed: {}", operator, reason)
            }
            OptimizationError::NoApplicableOperator => {
                write!(f, "no applicable operator found for region")
            }
            OptimizationError::MaxIterationsReached(max) => {
                write!(
                    f,
                    "maximum iterations ({}) reached without convergence",
                    max
                )
            }
            OptimizationError::Kinematics(msg) => {
                write!(f, "kinematics error: {}", msg)
            }
            OptimizationError::Evaluation(msg) => {
                write!(f, "evaluation error: {}", msg)
            }
            OptimizationError::InvalidRegion(msg) => {
                write!(f, "invalid region: {}", msg)
            }
        }
    }
}

impl std::error::Error for OptimizationError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_operator_failed() {
        let err = OptimizationError::OperatorFailed {
            operator: "test_op",
            reason: "something went wrong".into(),
        };
        let msg = err.to_string();
        assert!(msg.contains("test_op"));
        assert!(msg.contains("something went wrong"));
    }

    #[test]
    fn display_no_applicable_operator() {
        let err = OptimizationError::NoApplicableOperator;
        assert_eq!(err.to_string(), "no applicable operator found for region");
    }

    #[test]
    fn display_max_iterations_reached() {
        let err = OptimizationError::MaxIterationsReached(42);
        assert!(err.to_string().contains("42"));
        assert!(err.to_string().contains("maximum iterations"));
    }

    #[test]
    fn display_kinematics() {
        let err = OptimizationError::Kinematics("IK solver failed".into());
        assert!(err.to_string().contains("kinematics"));
        assert!(err.to_string().contains("IK solver"));
    }

    #[test]
    fn display_evaluation() {
        let err = OptimizationError::Evaluation("score out of range".into());
        assert!(err.to_string().contains("evaluation"));
        assert!(err.to_string().contains("score out of range"));
    }

    #[test]
    fn display_invalid_region() {
        let err = OptimizationError::InvalidRegion("empty range".into());
        assert!(err.to_string().contains("invalid region"));
        assert!(err.to_string().contains("empty range"));
    }

    #[test]
    fn error_impl_is_send() {
        fn assert_send<T: Send>() {}
        assert_send::<OptimizationError>();
    }
}
