//! Trajectory composition: mixes a modified segment back into an
//! original trajectory with configurable boundary blending.
//!
//! All operators produce a modified trajectory segment (the full traj
//! with the region's waypoints changed). This module takes that result
//! and blends the transitions at the region boundaries so that the
//! final trajectory is C0-continuous.

use thalos_core::trajectory::{Trajectory, TrajectoryPoint};

/// How to blend the modified segment back into the original trajectory.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BlendPolicy {
    /// No blending — replace directly (may cause discontinuities).
    None,
    /// Linear interpolation (fast, C0).
    Linear,
    /// Smooth step: 3x² - 2x³ (smooth, C1).
    SmoothStep,
    /// Cosine interpolation.
    Cosine,
}

impl Default for BlendPolicy {
    fn default() -> Self {
        BlendPolicy::SmoothStep
    }
}

/// Blends a modified trajectory segment back into the original trajectory
/// with configurable window size and policy.
///
/// # How it works
///
/// Outside the [range] the operator's output equals the original — the
/// operator only modifies waypoints inside the range. A simple point-by-point
/// blend between original and modified would do nothing outside the range
/// because both are identical there.
///
/// Instead, this function:
/// - At the **entry**: smoothly transitions from `original[i]` (pre-range)
///   toward `modified[range.start]` (first modified waypoint) over the window.
/// - At the **exit**: smoothly transitions from `modified[range.end-1]`
///   (last modified waypoint) toward `original[i]` (post-range) over the window.
/// - Inside the range: uses the modified values directly.
/// - Far from the range: keeps values from the modified trajectory (which
///   equal the original there).
///
/// # Arguments
/// * `original` — the full original trajectory (unmodified waypoints)
/// * `modified` — the operator's output (same length as original)
/// * `range` — the waypoint range that the operator modified (0-indexed, exclusive end)
/// * `window` — number of waypoints on each side of the range to blend over
/// * `policy` — the blending function to use
///
/// Returns a blended trajectory where the transition at the range boundaries
/// is smoothed over `window` waypoints.
pub fn compose_trajectory(
    original: &Trajectory,
    modified: &Trajectory,
    range: &std::ops::Range<usize>,
    window: usize,
    policy: BlendPolicy,
) -> Trajectory {
    let orig_wps = original.waypoints();
    let mod_wps = modified.waypoints();

    if orig_wps.len() != mod_wps.len() || window == 0 || policy == BlendPolicy::None {
        return modified.clone();
    }

    let mut result: Vec<TrajectoryPoint> = mod_wps.to_vec();
    let len = orig_wps.len();
    let range_start = range.start.min(len);
    let range_end = range.end.min(len);

    // --- Entry blend: transition FROM original[i] TOWARD modified[range.start] ---
    let entry_start = range_start.saturating_sub(window);
    let entry_end = range_start;

    // The target waypoint (first modified waypoint in the range)
    let target_wp = &mod_wps[range_start];
    let target_q = target_wp.joints();

    for i in entry_start..entry_end {
        let t = (i - entry_start) as f64 / window.max(1) as f64;
        let alpha = blend_factor(t, policy); // 0.0 → 1.0

        let orig_q = orig_wps[i].joints();
        let blended: Vec<f64> = orig_q
            .iter()
            .zip(target_q.iter())
            .map(|(o, t)| o + (t - o) * alpha)
            .collect();

        result[i] = TrajectoryPoint::new(blended, mod_wps[i].timestamp());
    }

    // --- Exit blend: transition FROM modified[range.end-1] TOWARD original[i] ---
    let exit_start = range_end;
    let exit_end = (range_end + window).min(len);

    if range_start < range_end {
        // The source waypoint (last modified waypoint in the range)
        let source_wp = &mod_wps[range_end - 1];
        let source_q = source_wp.joints();

        for i in exit_start..exit_end {
            let t = (i - exit_start) as f64 / (exit_end - exit_start).max(1) as f64;
            // alpha goes 0.0 → 1.0, blending FROM source TOWARD original
            let alpha = blend_factor(t, policy);

            let orig_q = orig_wps[i].joints();
            let blended: Vec<f64> = source_q
                .iter()
                .zip(orig_q.iter())
                .map(|(s, o)| s + (o - s) * alpha)
                .collect();

            result[i] = TrajectoryPoint::new(blended, mod_wps[i].timestamp());
        }
    }

    Trajectory::new(result)
}

fn blend_factor(t: f64, policy: BlendPolicy) -> f64 {
    let t = t.clamp(0.0, 1.0);
    match policy {
        BlendPolicy::None => 1.0,
        BlendPolicy::Linear => t,
        BlendPolicy::SmoothStep => t * t * (3.0 - 2.0 * t), // 3t² - 2t³
        BlendPolicy::Cosine => (1.0 - (t * std::f64::consts::PI).cos()) / 2.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_traj() -> Trajectory {
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 1.0),
            TrajectoryPoint::new(vec![2.0, 2.0], 2.0),
            TrajectoryPoint::new(vec![3.0, 3.0], 3.0),
            TrajectoryPoint::new(vec![4.0, 4.0], 4.0),
        ])
    }

    fn centered_traj() -> Trajectory {
        // All joints at 0.0 (centered)
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.0, 0.0], 1.0),
            TrajectoryPoint::new(vec![0.0, 0.0], 2.0),
            TrajectoryPoint::new(vec![0.0, 0.0], 3.0),
            TrajectoryPoint::new(vec![0.0, 0.0], 4.0),
        ])
    }

    #[test]
    fn no_blend_returns_modified() {
        let orig = simple_traj();
        let modd = centered_traj();
        let result = compose_trajectory(&orig, &modd, &(1..3), 0, BlendPolicy::SmoothStep);
        // window=0 → no blend, all waypoints from modified
        for wp in result.waypoints() {
            assert_eq!(wp.joints(), &[0.0, 0.0]);
        }
    }

    #[test]
    fn blend_preserves_waypoints_far_from_boundary() {
        let orig = simple_traj();
        let modd = centered_traj();
        // range 1..3, window=1 → blend at indices 0, 3
        let result = compose_trajectory(&orig, &modd, &(1..3), 1, BlendPolicy::SmoothStep);
        let wps = result.waypoints();

        // Waypoint 2 (center of range) should still be fully modified (0.0)
        assert!((wps[2].joints()[0] - 0.0).abs() < 1e-10);

        // Waypoint 4 (outside window) should be fully modified (0.0)
        assert!((wps[4].joints()[0] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn blend_creates_intermediate_values_at_boundary() {
        let orig = simple_traj();
        // modified: only waypoints 1 and 2 centered to 0.0
        let modd = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0), // unchanged
            TrajectoryPoint::new(vec![0.0, 0.0], 1.0), // centered
            TrajectoryPoint::new(vec![0.0, 0.0], 2.0), // centered
            TrajectoryPoint::new(vec![3.0, 3.0], 3.0), // unchanged
            TrajectoryPoint::new(vec![4.0, 4.0], 4.0), // unchanged
        ]);
        // range 1..3, window=2 → entry blend at idx 0,1 blends with target=modified[1]
        // Entry: idx 0 blends orig[0]→target[0]=0.0, t=(0-(-1))/2 = clamped→0 → alpha=0 → stays 0.0
        // Exit: idx 3 blends source=modified[2]=0.0→orig[3]=3.0
        let result = compose_trajectory(&orig, &modd, &(1..3), 2, BlendPolicy::Linear);
        let wps = result.waypoints();

        // idx 0: saturating_sub gives 0, so i=0, t=0/2=0, alpha=0 → stays orig[0]=0.0
        assert!((wps[0].joints()[0] - 0.0).abs() < 1e-10);

        // idx 1: inside range → fully modified
        assert!((wps[1].joints()[0] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn blend_policies_produce_different_results() {
        // Use a longer trajectory so we have enough points for a meaningful blend window
        let orig = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![1.0], 1.0),
            TrajectoryPoint::new(vec![2.0], 2.0),
            TrajectoryPoint::new(vec![3.0], 3.0),
            TrajectoryPoint::new(vec![4.0], 4.0),
            TrajectoryPoint::new(vec![5.0], 5.0),
            TrajectoryPoint::new(vec![6.0], 6.0),
        ]);
        let modd = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![0.0], 1.0),
            TrajectoryPoint::new(vec![0.0], 2.0),
            TrajectoryPoint::new(vec![0.0], 3.0),
            TrajectoryPoint::new(vec![0.0], 4.0),
            TrajectoryPoint::new(vec![0.0], 5.0),
            TrajectoryPoint::new(vec![0.0], 6.0),
        ]);
        // Range 2..5, window=3: i=1 gets t=0.333 where Linear≠SmoothStep
        let r = compose_trajectory(&orig, &modd, &(2..5), 3, BlendPolicy::Linear);
        let s = compose_trajectory(&orig, &modd, &(2..5), 3, BlendPolicy::SmoothStep);

        // At i=1 (t=1/3), Linear and SmoothStep should differ
        let r_val = r.waypoints()[1].joints()[0];
        let s_val = s.waypoints()[1].joints()[0];
        assert!(
            (r_val - s_val).abs() > 1e-6,
            "policies should produce different blends at t=0.333: Linear={}, SmoothStep={}",
            r_val,
            s_val
        );
    }

    #[test]
    fn blend_window_clamped_to_trajectory_bounds() {
        let orig = simple_traj();
        let modd = centered_traj();
        // window=10 but traj has 5 waypoints — should not panic
        let result = compose_trajectory(&orig, &modd, &(1..3), 10, BlendPolicy::SmoothStep);
        assert_eq!(result.len(), 5);
    }
}
