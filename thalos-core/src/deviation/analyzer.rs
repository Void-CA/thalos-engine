use thiserror::Error;
use super::expected::{ExpectedTrajectory, ObservedState};
use super::kinematic::{EnvelopeStatus, KinematicDeviation, KinematicError};
use super::policy::TolerancePolicy;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum DeviationAnalysisError {
    #[error("No expected state available at timestamp {timestamp_ns} (out of bounds)")]
    OutOfBoundsTimestamp { timestamp_ns: u64 },

    #[error("DOF mismatch between expected state ({expected_dof}) and observed state ({observed_dof})")]
    DimensionMismatch { expected_dof: usize, observed_dof: usize },
}

/// Pure domain service performing deterministic kinematic comparison and envelope validation.
pub struct DeviationAnalyzer;

impl DeviationAnalyzer {
    pub fn analyze(
        trajectory: &impl ExpectedTrajectory,
        observed: &ObservedState,
        tolerance_policy: &impl TolerancePolicy,
    ) -> Result<KinematicDeviation, DeviationAnalysisError> {
        let expected = trajectory
            .sample_at(observed.sampled_at_ns)
            .ok_or(DeviationAnalysisError::OutOfBoundsTimestamp {
                timestamp_ns: observed.sampled_at_ns,
            })?;

        if expected.joint_positions.len() != observed.joint_positions.len() {
            return Err(DeviationAnalysisError::DimensionMismatch {
                expected_dof: expected.joint_positions.len(),
                observed_dof: observed.joint_positions.len(),
            });
        }

        let error = KinematicError::compute(&expected, observed);

        let mut envelope = EnvelopeStatus::WithinTolerance;

        for (i, (&pos_err, &vel_err)) in error
            .joint_position_errors
            .iter()
            .zip(error.joint_velocity_errors.iter())
            .enumerate()
        {
            let tol = tolerance_policy.joint_tolerance(i);

            // Strict envelope rule: |e| <= tolerance is WithinTolerance. |e| > tolerance is Violated.
            if pos_err.abs() > tol.position || vel_err.abs() > tol.velocity {
                envelope = EnvelopeStatus::Violated;
                break;
            }
        }

        if envelope == EnvelopeStatus::WithinTolerance {
            if let (Some(cart_err), Some(cart_tol)) = (
                error.cartesian_position_error,
                tolerance_policy.cartesian_position_tolerance(),
            ) {
                if cart_err.abs() > cart_tol {
                    envelope = EnvelopeStatus::Violated;
                }
            }
        }

        Ok(KinematicDeviation {
            robot_id: observed.robot_id.clone(),
            sampled_at_ns: observed.sampled_at_ns,
            expected,
            observed: observed.clone(),
            error,
            envelope,
            severity: None,
        })
    }
}
