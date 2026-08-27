//! AdaptiveSampling — error-driven waypoint insertion operator.
//!
//! Inserts waypoints where interpolation error or local curvature exceeds
//! configurable thresholds. Uses a greedy max-heap approach: always
//! subdivide the worst segment first.
//!
//! # Algorithm
//!
//! ```text
//! apply(robot, trajectory, region, ctx):
//!   guard: region waypoints < 2 → return clone
//!   collect region waypoints → working Vec
//!   for each segment: compute error + curvature → push MaxHeap
//!   LOOP while heap not empty AND len < max_points:
//!     pop worst segment (i, error, curvature)
//!     if error ≤ error_threshold AND curvature ≤ curvature_threshold → break
//!     insert midpoint at i+1 in working vec
//!     push left and right child segments into heap
//!   return Trajectory::new(working vec replacing range)
//! ```
//!
//! # Invariants
//!
//! - Pure-additive: only inserts waypoints, never mutates existing ones
//! - Preserves start and end waypoints
//! - Hard cap on total waypoints via `max_points`

use thalos_core::{
    analysis::region::{ProblemRegion, RegionKind},
    evaluation::PlanMetrics,
    operation::ConstraintQuery,
    robot::serial_chain::SerialChain,
    trajectory::{Trajectory, TrajectoryPoint},
};

use crate::{
    domain::{
        TrajectoryOperator,
        context::OptimizationContext,
        operator::{Invariant, OperatorFamily, OptimizationObjective},
    },
    error::OptimizationError,
};

// ── Struct ─────────────────────────────────────────────────

/// Error-driven waypoint insertion operator.
///
/// Inserts midpoints into segments where the joint-space L2 distance
/// between adjacent waypoints exceeds `error_threshold`, or where
/// local curvature between consecutive segments exceeds
/// `curvature_threshold`.
///
/// The operator uses a max-heap priority queue to greedily subdivide
/// the worst segment first, stopping when both thresholds are met
/// or the point budget is exhausted.
pub struct AdaptiveSampling {
    /// Hard cap on total waypoints in the output trajectory.
    pub max_points: usize,
    /// L2 joint-space distance threshold for error-driven insertion.
    pub error_threshold: f64,
    /// Angular threshold (radians) for curvature-driven insertion.
    pub curvature_threshold: f64,
    /// Minimum segment length (L2 joint-space) below which no insertion occurs.
    pub min_segment_length: f64,
}

impl AdaptiveSampling {
    /// Create a new `AdaptiveSampling` operator.
    pub fn new(
        max_points: usize,
        error_threshold: f64,
        curvature_threshold: f64,
        min_segment_length: f64,
    ) -> Self {
        Self {
            max_points,
            error_threshold,
            curvature_threshold,
            min_segment_length,
        }
    }

    /// Default maximum waypoints (1000).
    pub const DEFAULT_MAX_POINTS: usize = 1000;

    /// Default error threshold (0.01 radians in normalized joint space).
    pub const DEFAULT_ERROR_THRESHOLD: f64 = 0.01;

    /// Default curvature threshold (~5.7°).
    pub const DEFAULT_CURVATURE_THRESHOLD: f64 = 0.1;

    /// Default minimum segment length (1e-6 radians in normalized joint space).
    pub const DEFAULT_MIN_SEGMENT_LENGTH: f64 = 1e-6;
}

// ── Helper functions ────────────────────────────────────────

/// Compute the L2 norm of the joint-space difference between two waypoints.
fn compute_joint_l2(a: &TrajectoryPoint, b: &TrajectoryPoint) -> f64 {
    a.joints()
        .iter()
        .zip(b.joints().iter())
        .map(|(aj, bj)| (aj - bj).powi(2))
        .sum::<f64>()
        .sqrt()
}

/// Compute the curvature (angle in radians) at `curr` between the segments
/// `prev→curr` and `curr→next`.
///
/// Returns 0.0 for collinear or degenerate (zero-length) segments.
fn compute_curvature(
    prev: &TrajectoryPoint,
    curr: &TrajectoryPoint,
    next: &TrajectoryPoint,
) -> f64 {
    let v1: Vec<f64> = curr
        .joints()
        .iter()
        .zip(prev.joints().iter())
        .map(|(c, p)| c - p)
        .collect();
    let v2: Vec<f64> = next
        .joints()
        .iter()
        .zip(curr.joints().iter())
        .map(|(n, c)| n - c)
        .collect();

    let dot: f64 = v1.iter().zip(v2.iter()).map(|(a, b)| a * b).sum();
    let norm1: f64 = v1.iter().map(|a| a.powi(2)).sum::<f64>().sqrt();
    let norm2: f64 = v2.iter().map(|a| a.powi(2)).sum::<f64>().sqrt();

    if norm1 < 1e-12 || norm2 < 1e-12 {
        return 0.0;
    }

    (dot / (norm1 * norm2)).clamp(-1.0, 1.0).acos()
}

/// Linearly interpolate the midpoint between two waypoints (t = 0.5).
fn lerp_midpoint(a: &TrajectoryPoint, b: &TrajectoryPoint) -> TrajectoryPoint {
    let joints: Vec<f64> = a
        .joints()
        .iter()
        .zip(b.joints().iter())
        .map(|(aj, bj)| (aj + bj) / 2.0)
        .collect();
    let timestamp = (a.timestamp() + b.timestamp()) / 2.0;
    TrajectoryPoint::new(joints, timestamp)
}

// ── TrajectoryOperator impl ─────────────────────────────────

impl TrajectoryOperator for AdaptiveSampling {
    fn id(&self) -> &'static str {
        "adaptive_sampling"
    }

    fn family(&self) -> OperatorFamily {
        OperatorFamily::Sampling
    }

    fn objective(&self) -> OptimizationObjective {
        OptimizationObjective::Continuity
    }

    fn invariants(&self) -> &'static [Invariant] {
        &[
            Invariant::PreserveExistingWaypoints,
            Invariant::PreserveStart,
            Invariant::PreserveEnd,
        ]
    }

    fn applicability(&self, region: &ProblemRegion) -> f32 {
        // AdaptiveSampling optimizes continuity/sampling, not collisions.
        // For collision regions, applicability is low by design.
        if region.kind == RegionKind::Collision {
            return 0.2;
        }
        if region.waypoint_count() >= 2 {
            0.8
        } else {
            0.0
        }
    }

    fn estimate_improvement(&self, region: &ProblemRegion, _metrics: &PlanMetrics) -> f32 {
        if region.waypoint_count() >= 2 {
            0.5
        } else {
            0.0
        }
    }

    fn estimate_cost(&self) -> f32 {
        0.3
    }

    fn apply(
        &self,
        _robot: &SerialChain,
        trajectory: &Trajectory,
        region: &ProblemRegion,
        _ctx: &OptimizationContext,
        constraints: Option<&dyn ConstraintQuery>,
    ) -> Result<Trajectory, OptimizationError> {
        let range = &region.waypoint_range;
        let all_wps = trajectory.waypoints();

        // Guard: must have at least 2 waypoints in the region
        if range.len() < 2 {
            return Ok(trajectory.clone());
        }

        // Extract the region's waypoints into a working vec. Each working
        // waypoint keeps the ORIGINAL absolute index it had in the input
        // trajectory, so constraint queries stay valid after insertions.
        let mut wps: Vec<TrajectoryPoint> = all_wps[range.clone()].to_vec();
        let mut orig_idx: Vec<usize> = (range.start..range.end).collect();

        // Iterative scan: find the worst segment, subdivide it, repeat.
        // This avoids the stale-index problems of a heap-based approach
        // while still being O(n·k) — fine for typical trajectory sizes.
        loop {
            if wps.len() >= self.max_points {
                break;
            }

            // Find the segment with the worst error (primary) and curvature (secondary)
            let mut worst_idx = usize::MAX;
            let mut worst_error = -1.0;
            let mut worst_curvature = -1.0;

            for i in 0..wps.len().saturating_sub(1) {
                // Constraint-aware guard: only subdivide segments whose
                // endpoint waypoints both allow neighbor modification.
                if !constraints.is_none_or(|c| {
                    c.can_modify_neighbors(orig_idx[i]) && c.can_modify_neighbors(orig_idx[i + 1])
                }) {
                    continue;
                }

                let error = compute_joint_l2(&wps[i], &wps[i + 1]);

                // Skip segments shorter than min_segment_length
                if error < self.min_segment_length {
                    continue;
                }

                let curvature = if i > 0 {
                    compute_curvature(&wps[i - 1], &wps[i], &wps[i + 1])
                } else {
                    0.0
                };

                // Compare: higher error is worse
                if error > worst_error || (error == worst_error && curvature > worst_curvature) {
                    worst_idx = i;
                    worst_error = error;
                    worst_curvature = curvature;
                }
            }

            // Termination: nothing subdividable remains (every candidate
            // segment is constrained or below the thresholds).
            if worst_idx == usize::MAX {
                break;
            }

            // Termination: the worst segment is within both thresholds
            if worst_error <= self.error_threshold && worst_curvature <= self.curvature_threshold {
                break;
            }

            // Subdivide the worst segment at its midpoint. The inserted
            // midpoint inherits the segment start's original index — it
            // was only inserted because both endpoints were modifiable.
            let mid = lerp_midpoint(&wps[worst_idx], &wps[worst_idx + 1]);
            wps.insert(worst_idx + 1, mid);
            orig_idx.insert(worst_idx + 1, orig_idx[worst_idx]);
        }

        // Build the full trajectory by replacing the region's waypoints
        let mut result_wps: Vec<TrajectoryPoint> = all_wps.to_vec();
        result_wps.splice(range.clone(), wps);

        Ok(Trajectory::new(result_wps))
    }
}

// ── Unit tests ─────────────────────────────────────────────

#[cfg(test)]
mod unit_tests {
    use super::*;
    use thalos_core::{
        analysis::region::{RegionId, RegionKind, RegionSeverity},
        models::{RobotModel, RobotRegistry},
    };

    // ── Test helpers ──────────────────────────────────────

    fn two_waypoint_region() -> ProblemRegion {
        ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            0..2,
        )
    }

    fn three_waypoint_region() -> ProblemRegion {
        ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            0..3,
        )
    }

    fn test_robot() -> SerialChain {
        RobotRegistry::create_default(RobotModel::Planar2R)
    }

    fn test_ctx() -> OptimizationContext {
        OptimizationContext::default()
    }

    // ── 1. Identity ───────────────────────────────────────

    #[test]
    fn identity_id_and_family() {
        let op = AdaptiveSampling::new(100, 0.01, 0.1, 1e-6);
        assert_eq!(op.id(), "adaptive_sampling");
        assert_eq!(op.family(), OperatorFamily::Sampling);
    }

    // ── 2. Applicability gate ─────────────────────────────

    #[test]
    fn applicability_below_two_waypoints_is_zero() {
        let op = AdaptiveSampling::new(100, 0.01, 0.1, 1e-6);
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            0..1, // only 1 waypoint
        );
        assert_eq!(op.applicability(&region), 0.0);
    }

    #[test]
    fn applicability_with_two_or_more_is_positive() {
        let op = AdaptiveSampling::new(100, 0.01, 0.1, 1e-6);
        let region = two_waypoint_region();
        assert!(op.applicability(&region) > 0.0);
    }

    // ── 3. Error-driven insertion ─────────────────────────

    #[test]
    fn error_driven_insertion_adds_waypoint() {
        // Two waypoints far apart (L2 ≈ 14.14) → error above threshold 8.0 → insert
        // After insertion: sub-segments have L2 ≈ 7.07 < 8.0 → stop after 1 insertion
        let op = AdaptiveSampling::new(100, 8.0, 10.0, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![10.0, 10.0], 1.0),
        ]);
        let region = two_waypoint_region();
        let ctx = test_ctx();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        // Initial error ≈ 14.14 > 8.0 → insert (1 waypoint added)
        // Sub-segment errors ≈ 7.07 < 8.0 → stop
        assert_eq!(
            result.len(),
            3,
            "expected 1 insertion, got {}",
            result.len()
        );
    }

    // ── 4. Low-error segment ──────────────────────────────

    #[test]
    fn low_error_segment_no_insertion() {
        // Two waypoints close together → error below threshold
        let op = AdaptiveSampling::new(100, 10.0, 10.0, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.01, 0.01], 1.0),
        ]);
        let region = two_waypoint_region();
        let ctx = test_ctx();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        assert_eq!(result.len(), traj.len(), "expected no insertion");
    }

    // ── 5. Curvature insertion ────────────────────────────

    #[test]
    fn high_curvature_triggers_insertion() {
        // Three waypoints with a gentle turn, low segment errors
        // A=[0,0], B=[1,0.1], C=[2,0]
        // Segment AB: error=√(1.01)≈1.005, curvature=0
        // Segment BC: error=√(1.01)≈1.005, curvature≈0.199 rad (~11.4°)
        // With error_threshold=5.0 and curvature_threshold=0.15:
        //   - Both segments have error 1.005 < 5.0 (below error threshold)
        //   - Segment BC has curvature 0.199 > 0.15 (above curvature threshold)
        //   - So BC should trigger curvature-based insertion
        let op = AdaptiveSampling::new(10, 5.0, 0.15, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 0.1], 1.0),
            TrajectoryPoint::new(vec![2.0, 0.0], 2.0),
        ]);
        let region = three_waypoint_region();
        let ctx = test_ctx();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        // The curvature trigger should insert at least 1 waypoint
        assert!(
            result.len() > traj.len(),
            "expected insertion from curvature trigger, got same length"
        );
    }

    // ── 6. Straight path ──────────────────────────────────

    #[test]
    fn straight_path_no_curvature_insertion() {
        // Collinear segments → all curvature values near 0
        let op = AdaptiveSampling::new(100, 5.0, 0.1, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 1.0),
            TrajectoryPoint::new(vec![2.0, 2.0], 2.0),
        ]);
        let region = three_waypoint_region();
        let ctx = test_ctx();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        // Both segments have error ≈1.414 < 5.0, and curvature≈0 < 0.1
        // No insertions should occur
        assert_eq!(
            result.len(),
            traj.len(),
            "expected no curvature insertion for collinear path"
        );
    }

    // ── 7. Budget cap ─────────────────────────────────────

    #[test]
    fn budget_cap_is_enforced() {
        // Strict thresholds will trigger many insertions
        let op = AdaptiveSampling::new(5, 0.001, 0.001, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![5.0, 5.0], 1.0),
            TrajectoryPoint::new(vec![10.0, 0.0], 2.0),
        ]);
        let region = three_waypoint_region();
        let ctx = test_ctx();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        // max_points=5, input has 3 inside region + possibly outside region
        // The operator caps region waypoints at max_points
        // Total output should be <= (full_traj_len - region_len + max_points)
        // Since the full traj IS the region here, output <= 5
        assert!(
            result.len() <= 5,
            "expected at most 5 waypoints, got {}",
            result.len()
        );
    }

    // ── 8. Waypoint preservation ──────────────────────────

    #[test]
    fn original_waypoints_byte_identical() {
        let op = AdaptiveSampling::new(100, 0.5, 10.0, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![5.0, 5.0], 1.0),
            TrajectoryPoint::new(vec![10.0, 0.0], 2.0),
        ]);
        let region = three_waypoint_region();
        let ctx = test_ctx();

        let original_joints: Vec<Vec<f64>> = traj
            .waypoints()
            .iter()
            .map(|wp| wp.joints().to_vec())
            .collect();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();

        // All original waypoints should appear in order in the output
        let result_joints: Vec<&[f64]> = result.waypoints().iter().map(|wp| wp.joints()).collect();

        let mut result_idx = 0;
        for orig in &original_joints {
            while result_idx < result_joints.len() && result_joints[result_idx] != orig.as_slice() {
                result_idx += 1;
            }
            assert!(
                result_idx < result_joints.len(),
                "original waypoint {:?} not found in order",
                orig
            );
        }
    }

    // ── 9. Endpoints preserved ────────────────────────────

    #[test]
    fn endpoints_are_preserved() {
        let op = AdaptiveSampling::new(100, 0.5, 10.0, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![5.0, 5.0], 1.0),
            TrajectoryPoint::new(vec![10.0, 0.0], 2.0),
        ]);
        let region = three_waypoint_region();
        let ctx = test_ctx();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();

        let first_input = traj.waypoints().first().unwrap();
        let last_input = traj.waypoints().last().unwrap();
        let first_output = result.waypoints().first().unwrap();
        let last_output = result.waypoints().last().unwrap();

        assert_eq!(first_input.joints(), first_output.joints());
        assert_eq!(last_input.joints(), last_output.joints());
    }

    // ── 10. Dense trajectory (no-op) ──────────────────────

    #[test]
    fn dense_trajectory_is_no_op() {
        // All segments well below both thresholds
        let op = AdaptiveSampling::new(100, 10.0, 10.0, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.005, 0.005], 0.1),
            TrajectoryPoint::new(vec![0.01, 0.01], 0.2),
            TrajectoryPoint::new(vec![0.015, 0.015], 0.3),
        ]);
        // Region covering all 4 waypoints
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            0..4,
        );
        let ctx = test_ctx();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        assert_eq!(
            result.len(),
            traj.len(),
            "expected no-op for dense trajectory"
        );
        for (orig, res) in traj.waypoints().iter().zip(result.waypoints().iter()) {
            assert_eq!(orig.joints(), res.joints());
        }
    }

    // ── 11. Start/end preservation (single-segment guard) ─

    #[test]
    fn single_waypoint_region_returns_clone() {
        let op = AdaptiveSampling::new(100, 0.01, 0.1, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![10.0, 10.0], 1.0),
        ]);
        // Region with only 1 waypoint
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            1..2,
        );
        let ctx = test_ctx();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        assert_eq!(result.len(), traj.len());
        for (orig, res) in traj.waypoints().iter().zip(result.waypoints().iter()) {
            assert_eq!(orig.joints(), res.joints());
        }
    }

    // ── 12. Zero-headroom returns input ────────────────────

    #[test]
    fn zero_headroom_returns_input() {
        // max_points == current waypoint count → no insertions possible
        let op = AdaptiveSampling::new(3, 0.001, 0.001, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 1.0),
            TrajectoryPoint::new(vec![2.0, 2.0], 2.0),
        ]);
        let region = three_waypoint_region();
        let ctx = test_ctx();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        assert_eq!(result.len(), 3, "expected no insertion with zero headroom");
        for (orig, res) in traj.waypoints().iter().zip(result.waypoints().iter()) {
            assert_eq!(orig.joints(), res.joints());
        }
    }

    // ── 13. Collision region applicability ─────────────────

    #[test]
    fn collision_region_low_applicability() {
        let op = AdaptiveSampling::new(100, 0.01, 0.1, 1e-6);
        let collision_region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Collision,
            RegionSeverity::Warning,
            0..3,
        );
        assert!(
            op.applicability(&collision_region) <= 0.3,
            "expected ≤ 0.3 for collision region, got {}",
            op.applicability(&collision_region)
        );
    }

    // ── 14. estimate_cost constant ────────────────────────

    #[test]
    fn estimate_cost_is_constant() {
        let op = AdaptiveSampling::new(100, 0.01, 0.1, 1e-6);
        assert!((op.estimate_cost() - 0.3).abs() < f32::EPSILON);
    }

    // ── 15. ConstraintQuery neighbor guard (2.6) ──────────

    use thalos_core::operation::PrecisionLevel;

    /// Mock query: only `can_modify_neighbors` is overridden; every other
    /// guard returns `true`.
    struct NeighborsMock {
        allowed: Vec<bool>,
    }

    impl ConstraintQuery for NeighborsMock {
        fn can_relax_orientation(&self, _i: usize, _a: f64) -> bool {
            true
        }
        fn can_modify_position(&self, _i: usize) -> bool {
            true
        }
        fn max_position_error(&self, _i: usize) -> Option<f64> {
            None
        }
        fn max_velocity(&self, _i: usize) -> Option<f64> {
            None
        }
        fn required_precision(&self, _i: usize) -> PrecisionLevel {
            PrecisionLevel::None
        }
        fn can_modify_neighbors(&self, i: usize) -> bool {
            self.allowed.get(i).copied().unwrap_or(true)
        }
    }

    #[test]
    fn constrained_neighbor_segment_not_subdivided() {
        // Segment [0,0]→[10,10]: L2 ≈ 14.14 > error_threshold 8.0 → would
        // insert a midpoint, but waypoint 0 forbids neighbor modification.
        let op = AdaptiveSampling::new(100, 8.0, 10.0, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![10.0, 10.0], 1.0),
        ]);
        let region = two_waypoint_region(); // 0..2
        let ctx = test_ctx();

        let mock = NeighborsMock {
            allowed: vec![false, true],
        };
        let result = op.apply(&robot, &traj, &region, &ctx, Some(&mock)).unwrap();

        assert_eq!(
            result.len(),
            2,
            "no insertion in segment adjacent to constrained waypoint 0"
        );
        // Original waypoints remain byte-identical.
        assert_eq!(result.waypoints()[0].joints(), traj.waypoints()[0].joints());
        assert_eq!(result.waypoints()[1].joints(), traj.waypoints()[1].joints());
    }

    #[test]
    fn unconstrained_high_error_segment_inserts() {
        // Same segment with both endpoints free → exactly 1 insertion
        // (sub-segments 7.07 < 8.0 stop further subdivision).
        let op = AdaptiveSampling::new(100, 8.0, 10.0, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![10.0, 10.0], 1.0),
        ]);
        let region = two_waypoint_region();
        let ctx = test_ctx();

        let free = NeighborsMock {
            allowed: vec![true, true],
        };
        let result = op.apply(&robot, &traj, &region, &ctx, Some(&free)).unwrap();

        assert_eq!(
            result.len(),
            3,
            "1 insertion expected when both endpoints are free"
        );
    }

    #[test]
    fn constrained_middle_waypoint_blocks_adjacent_segments() {
        // A=[0,0] B=[5,5] C=[10,10]: both segments have L2 ≈ 7.07 >
        // error_threshold 3.0 → would insert, but the middle waypoint
        // forbids neighbor modification → both segments stay intact.
        let op = AdaptiveSampling::new(100, 3.0, 10.0, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![5.0, 5.0], 1.0),
            TrajectoryPoint::new(vec![10.0, 10.0], 2.0),
        ]);
        let region = three_waypoint_region(); // 0..3
        let ctx = test_ctx();

        let mock = NeighborsMock {
            allowed: vec![true, false, true],
        };
        let result = op.apply(&robot, &traj, &region, &ctx, Some(&mock)).unwrap();

        assert_eq!(
            result.len(),
            3,
            "no insertion in segments adjacent to constrained waypoint 1"
        );
    }
}

// ── Benchmarks ──────────────────────────────────────────────

#[cfg(test)]
mod benchmarks {
    use super::*;
    use thalos_core::{
        analysis::region::{RegionId, RegionKind, RegionSeverity},
        models::{RobotModel, RobotRegistry},
    };

    fn test_robot() -> SerialChain {
        RobotRegistry::create_default(RobotModel::Planar2R)
    }

    fn test_ctx() -> OptimizationContext {
        OptimizationContext::default()
    }

    /// Generate a high-curvature trajectory (coarse sine wave).
    fn high_curvature_trajectory() -> Trajectory {
        Trajectory::new(
            (0..=500)
                .step_by(50)
                .map(|i| {
                    let t = i as f64 * 0.04;
                    TrajectoryPoint::new(vec![t, (t * 0.8).sin() * 3.0], t)
                })
                .collect(),
        )
    }

    /// Compute max segment error (max L2 joint-space distance across all segments).
    /// This is the primary metric AdaptiveSampling directly optimizes — it subdivides
    /// segments until all are below error_threshold.
    fn max_segment_error(traj: &Trajectory) -> f64 {
        let wps = traj.waypoints();
        if wps.len() < 2 {
            return 0.0;
        }
        let mut max_err = 0.0;
        for i in 0..wps.len() - 1 {
            let err = compute_joint_l2(&wps[i], &wps[i + 1]);
            if err > max_err {
                max_err = err;
            }
        }
        max_err
    }

    /// Benchmark: measure max-segment-error reduction on a high-curvature trajectory.
    ///
    /// The spec criterion is: interpolation error decreases ≥ 30%.
    /// AdaptiveSampling directly minimizes max segment error by subdividing
    /// high-error segments until all are below error_threshold.
    #[test]
    fn benchmark_interpolation_error_reduction() {
        let op = AdaptiveSampling::new(500, 0.3, 0.15, 1e-6);
        let robot = test_robot();
        let ctx = test_ctx();

        let traj = high_curvature_trajectory();
        let before_max = max_segment_error(&traj);
        let before_count = traj.len();

        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            0..traj.len(),
        );

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        let after_max = max_segment_error(&result);
        let after_count = result.len();

        let reduction_pct = if before_max > 0.0 {
            ((before_max - after_max) / before_max) * 100.0
        } else {
            0.0
        };

        println!("\n═══ Benchmark: AdaptiveSampling ─────────────────");
        println!("  Waypoints:          {} → {}", before_count, after_count);
        println!("  Max segment error:  {:.4} → {:.4}", before_max, after_max);
        println!("  Reduction:          {:.1}%", reduction_pct);
        println!(
            "  Result:             {}",
            if reduction_pct >= 30.0 {
                "✅ PASS"
            } else {
                "❌ FAIL"
            }
        );
        println!("────────────────────────────────────────────\n");

        // The operator guarantees all segments are below error_threshold.
        // Before, max error ≈ 4.3 (high-curvature sine wave).
        // After, max error ≤ 0.3 (error_threshold) → reduction ≥ 93%.
        assert!(
            after_max <= 0.31, // slightly above threshold for floating point tolerance
            "AdaptiveSampling should reduce max segment error to ≤ error_threshold (0.3), got {:.4}",
            after_max
        );
        assert!(
            reduction_pct >= 30.0,
            "AdaptiveSampling should reduce max segment error by ≥30%, got {:.1}%",
            reduction_pct
        );
        assert!(
            after_count >= before_count,
            "Waypoint count should not decrease"
        );
    }
    /// Benchmark: verify AdaptiveSampling does not degrade already-dense trajectories.
    #[test]
    fn benchmark_dense_trajectory_no_degradation() {
        let op = AdaptiveSampling::new(500, 0.2, 0.15, 1e-6);
        let robot = test_robot();
        let ctx = test_ctx();

        // Dense straight-line trajectory — already well-sampled
        let traj = Trajectory::new(
            (0..=100)
                .map(|i| {
                    let t = i as f64 * 0.1;
                    TrajectoryPoint::new(vec![t, t * 0.1], t)
                })
                .collect(),
        );

        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            0..traj.len(),
        );

        let before_count = traj.len();
        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        let after_count = result.len();

        println!("\n═══ Benchmark: Dense trajectory ────────────────");
        println!(
            "  Waypoints:     {} → {} ({})",
            before_count,
            after_count,
            if before_count == after_count {
                "unchanged ✅"
            } else {
                "CHANGED ❌"
            }
        );
        println!("────────────────────────────────────────────\n");

        // Dense trajectory should remain unchanged (already well-sampled)
        assert_eq!(
            before_count, after_count,
            "Dense trajectory should not gain waypoints, got {} → {}",
            before_count, after_count
        );
        for (orig, res) in traj.waypoints().iter().zip(result.waypoints().iter()) {
            assert_eq!(orig.joints(), res.joints());
        }
    }
}

// ── Integration tests ───────────────────────────────────────

#[cfg(test)]
mod integration_tests {
    use super::*;
    use crate::{
        PlanMetrics,
        pipeline::{BlendPolicy, compose_trajectory},
    };
    use thalos_core::{
        analysis::region::{RegionId, RegionKind, RegionSeverity},
        evaluation::{CollisionMetrics, JointSafetyMetrics, ManipulabilityMetrics},
        models::{RobotModel, RobotRegistry},
    };

    fn test_robot() -> SerialChain {
        RobotRegistry::create_default(RobotModel::Planar2R)
    }

    fn test_ctx() -> OptimizationContext {
        OptimizationContext::default()
    }

    fn test_metrics() -> PlanMetrics {
        PlanMetrics::new(
            0.0,
            0,
            ManipulabilityMetrics::new(0.0, 0.0, 0, 0),
            JointSafetyMetrics::new(1.0, 0.0, 0),
            CollisionMetrics::new(1.0, 0, 0),
            0.0,
            0.0,
        )
    }

    fn two_waypoint_region() -> ProblemRegion {
        ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            0..2,
        )
    }

    // ── 1. Pipeline ranking ───────────────────────────────

    #[test]
    fn sampling_operator_ranked_for_sufficient_waypoints() {
        let op = AdaptiveSampling::new(100, 0.01, 0.1, 1e-6);
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            0..3,
        );
        let _metrics = test_metrics();

        let applicability = op.applicability(&region);
        assert!(
            applicability >= 0.7,
            "expected >= 0.7 for region with >= 2 waypoints, got {}",
            applicability
        );
    }

    // ── 2. Composition with length change ─────────────────

    #[test]
    fn composition_with_different_length_returns_modified_directly() {
        let op = AdaptiveSampling::new(100, 0.5, 10.0, 1e-6);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![10.0, 10.0], 1.0),
        ]);
        let region = two_waypoint_region();
        let ctx = test_ctx();

        let modified = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        // Lengths differ → compose_trajectory returns modified directly
        let composed = compose_trajectory(
            &traj,
            &modified,
            &region.waypoint_range,
            5,
            BlendPolicy::SmoothStep,
        );
        assert_eq!(composed.len(), modified.len());
    }

    // ── 3. Idempotency ────────────────────────────────────

    #[test]
    fn second_pass_is_no_op() {
        // Use a straight-line trajectory (no curvature) with moderate threshold
        // so first pass fully converges
        let op = AdaptiveSampling::new(100, 5.0, 10.0, 1e-6);
        let robot = test_robot();
        // Single segment: [0,0]→[5,5], L2≈7.07 > 5.0 → will subdivide once
        // Sub-segments have L2≈3.54 < 5.0 → stop at 3 waypoints
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![5.0, 5.0], 1.0),
        ]);
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            0..2,
        );
        let ctx = test_ctx();

        let first = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        assert_eq!(first.len(), 3, "expected exactly 1 insertion in first pass");

        // Second pass on the result (new region covering the full trajectory)
        let second_region = ProblemRegion::new(
            RegionId(1),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            0..first.len(),
        );
        let second = op
            .apply(&robot, &first, &second_region, &ctx, None)
            .unwrap();

        // After first pass, all errors should be below threshold
        // so second pass should not add new waypoints
        assert_eq!(
            second.len(),
            first.len(),
            "second pass should not add waypoints"
        );
    }

    // ── 4. Objective/invariant declaration ────────────────

    #[test]
    fn objective_returns_continuity() {
        let op = AdaptiveSampling::new(100, 0.01, 0.1, 1e-6);
        assert_eq!(op.objective(), OptimizationObjective::Continuity);
    }

    #[test]
    fn invariants_contain_existing_waypoints() {
        let op = AdaptiveSampling::new(100, 0.01, 0.1, 1e-6);
        let invs = op.invariants();
        assert!(invs.contains(&Invariant::PreserveExistingWaypoints));
        assert!(invs.contains(&Invariant::PreserveStart));
        assert!(invs.contains(&Invariant::PreserveEnd));
    }

    // ── 5. Backward compat ────────────────────────────────

    #[test]
    fn backward_compat_default_trait_methods() {
        // This test verifies that M9.0 JointCenteringOperator still
        // compiles and returns default values for the new trait methods.
        let op = crate::operators::JointCenteringOperator::new(0.3);
        assert_eq!(op.objective(), OptimizationObjective::Feasibility);
        assert!(op.invariants().is_empty());
    }
}
