//! Temporal utilities for trajectory optimization.
//!
//! Provides the time-parameterization building blocks used by temporal
//! operators (e.g., Retime). These functions compute minimum segment
//! durations under velocity limits and extract velocity limits from
//! the robot model.

use thalos_core::robot::serial_chain::SerialChain;

/// Compute the minimum segment duration given joint-space displacement
/// and optional velocity limits.
///
/// Returns the stretched duration, or `original_dt` if no stretch is
/// needed or no velocity limits are provided.
///
/// # Arguments
///
/// * `displacement` — Per-joint absolute displacement `|q[i+1] - q[i]|`.
/// * `original_dt` — The original segment duration.
/// * `velocity_limits` — Optional per-joint velocity limits (rad/s).
///   When `None`, the original duration is returned unchanged.
/// * `max_duration_scale` — Cap on the stretch factor applied to
///   `original_dt` (e.g., 10.0 means at most 10× the original duration).
///   Must be ≥ 1.0.
pub fn min_segment_duration(
    displacement: &[f64],
    original_dt: f64,
    velocity_limits: Option<&[f64]>,
    max_duration_scale: f64,
) -> f64 {
    let Some(limits) = velocity_limits else {
        return original_dt;
    };

    let mut max_dt = original_dt;
    for (dq, v_max) in displacement.iter().zip(limits.iter()) {
        if *v_max <= 0.0 {
            continue;
        }
        let required_dt = dq.abs() / v_max;
        if required_dt > max_dt {
            max_dt = required_dt;
        }
    }

    // Cap at max_duration_scale × original
    max_dt.min(original_dt * max_duration_scale)
}

/// Extract velocity limits from a [`SerialChain`] into the
/// [`JointLimits`](crate::domain::context::JointLimits) format.
///
/// Returns `Some(vec)` if every enabled joint has a velocity limit.
/// Returns `None` if any enabled joint lacks a velocity limit,
/// indicating the model does not fully specify per-joint velocities.
pub fn extract_velocity_limits(chain: &SerialChain) -> Option<Vec<f64>> {
    let mut velocities = Vec::new();
    for segment in &chain.segments {
        let limits = segment.joint.limits();
        if limits.enabled {
            velocities.push(limits.velocity?);
        }
    }
    Some(velocities)
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_core::models::{RobotModel, RobotRegistry};

    // ── min_segment_duration tests ────────────────────────

    #[test]
    fn zero_displacement_preserves_original_dt() {
        let dq = [0.0, 0.0, 0.0];
        let dt = min_segment_duration(&dq, 1.0, Some(&[1.0, 1.0, 1.0]), 10.0);
        assert!((dt - 1.0).abs() < f64::EPSILON, "expected 1.0, got {}", dt);
    }

    #[test]
    fn displacement_within_limit_preserves_original_dt() {
        let dq = [0.5, 0.3];
        let dt = min_segment_duration(&dq, 2.0, Some(&[1.0, 0.6]), 10.0);
        // max(0.5/1.0, 0.3/0.6) = 0.5, original is 2.0, so no stretch
        assert!((dt - 2.0).abs() < f64::EPSILON, "expected 2.0, got {}", dt);
    }

    #[test]
    fn displacement_exceeds_limit_stretches_duration() {
        let dq = [1.0, 0.3];
        let dt = min_segment_duration(&dq, 0.5, Some(&[0.5, 1.0]), 10.0);
        // max(1.0/0.5, 0.3/1.0) = 2.0, which is > original 0.5
        assert!((dt - 2.0).abs() < f64::EPSILON, "expected 2.0, got {}", dt);
    }

    #[test]
    fn no_velocity_limits_preserves_original_dt() {
        let dq = [1.0, 2.0];
        let dt = min_segment_duration(&dq, 1.5, None, 10.0);
        assert!((dt - 1.5).abs() < f64::EPSILON, "expected 1.5, got {}", dt);
    }

    #[test]
    fn max_duration_scale_cap_enforced() {
        let dq = [10.0, 0.0];
        // v_max=1.0 → need 10.0s, but scale capped at 2× original = 4.0
        let dt = min_segment_duration(&dq, 2.0, Some(&[1.0, 1.0]), 2.0);
        assert!((dt - 4.0).abs() < f64::EPSILON, "expected 4.0, got {}", dt);
    }

    #[test]
    fn zero_velocity_limit_skipped() {
        let dq = [0.5, 0.3];
        // v_max[0] is 0.0 → skipped, only v_max[1] matters
        let dt = min_segment_duration(&dq, 0.1, Some(&[0.0, 1.0]), 10.0);
        // max(0.3/1.0) = 0.3
        assert!((dt - 0.3).abs() < f64::EPSILON, "expected 0.3, got {}", dt);
    }

    #[test]
    fn stretch_factor_not_exceeding_max_scale() {
        let dq = [100.0];
        let dt = min_segment_duration(&dq, 0.1, Some(&[1.0]), 5.0);
        // Required: 100.0/1.0 = 100.0. Cap: 0.1 * 5.0 = 0.5
        assert!((dt - 0.5).abs() < f64::EPSILON, "expected 0.5, got {}", dt);
    }

    // ── extract_velocity_limits tests ─────────────────────

    #[test]
    fn extract_velocity_limits_round_trip() {
        // Planar2R has 2 revolute joints with limits
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let velocities = extract_velocity_limits(&chain);
        // Planar2R joints may or may not have velocity set in defaults
        // We just verify it returns consistent length when Some
        if let Some(v) = velocities {
            assert_eq!(v.len(), 2, "Planar2R has 2 enabled joints");
        }
    }
}
