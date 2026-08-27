//! Retime — per-segment timestamp stretching operator.
//!
//! Enforces per-joint velocity limits by stretching segment durations
//! while preserving joint values and waypoint count. First implementation
//! of `OperatorFamily::Temporal`.
//!
//! # Algorithm
//!
//! ```text
//! apply(robot, trajectory, region, ctx):
//!   guard: region waypoints < 2 → return clone
//!   extract region waypoints → working vec
//!   get velocity limits from ctx.joint_limits.velocity
//!       or use self.default_velocity as fallback
//!   for each segment: compute |dq|, determine required dt via
//!       temporal::min_segment_duration()
//!   forward-propagate new timestamps
//!   splice back into trajectory (joint values unchanged)
//! ```
//!
//! # Invariants
//!
//! - Joint values are never modified (only timestamps change)
//! - Waypoint count is preserved
//! - First waypoint timestamp is preserved
//! - Output timestamps are strictly increasing

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
    temporal,
};

// ── Struct ─────────────────────────────────────────────────

/// Per-segment timestamp stretching operator.
///
/// Enforces per-joint velocity limits by stretching segment durations
/// while preserving joint values and waypoint count. Only timestamps
/// are modified — joint positions remain byte-identical.
///
/// # When to use
///
/// Apply to regions with `Velocity` or `Tracking` kind where joint
/// velocities exceed configured limits. The operator stretches segment
/// durations so peak joint velocity is bounded by the limit.
pub struct Retime {
    /// Fallback velocity limit (rad/s) when per-joint limits are not
    /// available via `ctx.joint_limits.velocity`.
    pub default_velocity: f64,
    /// Cap on the stretch factor applied to the original segment
    /// duration. For example, `10.0` means at most 10× the original
    /// duration, no matter how extreme the velocity violation.
    pub max_duration_scale: f64,
}

impl Retime {
    /// Create a new `Retime` operator with the given parameters.
    ///
    /// * `default_velocity` — fallback velocity limit (rad/s) used when
    ///   `ctx.joint_limits.velocity` is `None`.
    /// * `max_duration_scale` — cap on stretch factor (must be ≥ 1.0).
    pub fn new(default_velocity: f64, max_duration_scale: f64) -> Self {
        Self {
            default_velocity,
            max_duration_scale,
        }
    }

    /// Default fallback velocity limit: 3.0 rad/s.
    pub const DEFAULT_VELOCITY: f64 = 3.0;

    /// Default max duration scale: 10.0× original duration.
    pub const DEFAULT_MAX_DURATION_SCALE: f64 = 10.0;
}

// ── TrajectoryOperator impl ─────────────────────────────────

impl TrajectoryOperator for Retime {
    fn id(&self) -> &'static str {
        "retime"
    }

    fn family(&self) -> OperatorFamily {
        OperatorFamily::Temporal
    }

    fn objective(&self) -> OptimizationObjective {
        OptimizationObjective::Feasibility
    }

    fn invariants(&self) -> &'static [Invariant] {
        &[Invariant::PreserveJointPath]
    }

    fn applicability(&self, region: &ProblemRegion) -> f32 {
        if region.waypoint_count() < 2 {
            return 0.0;
        }
        match region.kind {
            RegionKind::Velocity | RegionKind::Tracking => 0.8,
            _ => 0.5,
        }
    }

    fn estimate_improvement(&self, region: &ProblemRegion, _metrics: &PlanMetrics) -> f32 {
        if region.waypoint_count() < 2 {
            return 0.0;
        }
        match region.kind {
            RegionKind::Velocity | RegionKind::Tracking => 0.8,
            _ => 0.3,
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
        ctx: &OptimizationContext,
        constraints: Option<&dyn ConstraintQuery>,
    ) -> Result<Trajectory, OptimizationError> {
        let range = &region.waypoint_range;
        let all_wps = trajectory.waypoints();

        // Guard: must have at least 2 waypoints for at least one segment
        if range.len() < 2 {
            return Ok(trajectory.clone());
        }

        // Extract the region's waypoints
        let region_wps: Vec<&TrajectoryPoint> = all_wps[range.clone()].iter().collect();

        // Resolve velocity limits: prefer per-joint limits from context,
        // fall back to a uniform vector using self.default_velocity.
        let velocity_limits: Option<Vec<f64>> = ctx.joint_limits.velocity.clone().or_else(|| {
            if !region_wps.is_empty() {
                let joint_count = region_wps[0].joints().len();
                Some(vec![self.default_velocity; joint_count])
            } else {
                None
            }
        });
        let velocity_limits_ref = velocity_limits.as_deref();

        // Store original timestamps separately — we need the original dt
        // for each segment as the baseline for min_segment_duration, even
        // after forward-propagating previous stretches.
        let original_timestamps: Vec<f64> = region_wps.iter().map(|wp| wp.timestamp()).collect();
        let mut new_timestamps = original_timestamps.clone();

        // Pre-pass: mark which waypoints have locked timing. Locked waypoints
        // are hard anchors — their original timestamp is preserved — and free
        // stretches must never overshoot the next anchor, or the output would
        // lose its strictly-increasing invariant (see retime.rs invariants).
        let locked: Vec<bool> = (0..region_wps.len())
            .map(|k| !constraints.is_none_or(|c| c.can_modify_timing(range.start + k)))
            .collect();

        // Backward ceiling pass: compute a per-index hard ceiling so that free
        // stretches between anchors keep strictly increasing timestamps. Each
        // waypoint's ceiling is capped by the next locked anchor, and each
        // free waypoint inherits a strictly smaller ceiling than its successor
        // (ceiling[k] = previous representable value below ceiling[k+1]). This
        // avoids the collapse where two consecutive free waypoints would both
        // clamp to the same anchor value and produce equal (non-strictly-
        // increasing) timestamps. The step is a true nextafter-down, not
        // f64::EPSILON, which is absolute and rounds back to the anchor for
        // magnitudes >= 2.0.
        fn next_down(v: f64) -> f64 {
            f64::from_bits(v.to_bits() - 1)
        }
        let mut ceiling: Vec<f64> = vec![f64::INFINITY; region_wps.len()];
        for k in (0..region_wps.len()).rev() {
            if locked[k] {
                ceiling[k] = original_timestamps[k];
            } else if k + 1 < region_wps.len() {
                ceiling[k] = next_down(ceiling[k + 1]);
            }
        }

        // Stretch each segment independently, forward-propagating timestamps
        for i in 0..region_wps.len() - 1 {
            // Constraint-aware guard: preserve the ORIGINAL timestamp of
            // waypoints whose timing is locked, and stretch around them.
            if locked[i + 1] {
                new_timestamps[i + 1] = original_timestamps[i + 1];
                continue;
            }

            let dq: Vec<f64> = region_wps[i + 1]
                .joints()
                .iter()
                .zip(region_wps[i].joints().iter())
                .map(|(a, b)| (a - b).abs())
                .collect();

            // Use the ORIGINAL segment duration as the baseline —
            // min_segment_duration caps the stretch against original_dt * max_duration_scale
            let original_dt = original_timestamps[i + 1] - original_timestamps[i];

            let new_dt = temporal::min_segment_duration(
                &dq,
                original_dt,
                velocity_limits_ref,
                self.max_duration_scale,
            );

            // Hard ceiling: never overshoot the next locked anchor. Otherwise
            // earlier stretches can push past a later locked waypoint and the
            // timestamps become non-monotonic (e.g. [0.0, 4.0, 2.0]).
            new_timestamps[i + 1] = (new_timestamps[i] + new_dt).min(ceiling[i + 1]);
        }

        // Build new waypoints with adjusted timestamps (joint values unchanged)
        let new_wps: Vec<TrajectoryPoint> = all_wps[range.clone()]
            .iter()
            .zip(new_timestamps.iter())
            .map(|(wp, t)| TrajectoryPoint::new(wp.joints().to_vec(), *t))
            .collect();

        // Replace the region waypoints in the full trajectory
        let mut result_wps = all_wps.to_vec();
        result_wps.splice(range.clone(), new_wps);

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

    fn velocity_region(range: std::ops::Range<usize>) -> ProblemRegion {
        ProblemRegion::new(
            RegionId(0),
            RegionKind::Velocity,
            RegionSeverity::Warning,
            range,
        )
    }

    fn two_wp_velocity_region() -> ProblemRegion {
        velocity_region(0..2)
    }

    fn three_wp_velocity_region() -> ProblemRegion {
        velocity_region(0..3)
    }

    fn four_wp_velocity_region() -> ProblemRegion {
        velocity_region(0..4)
    }

    fn test_robot() -> SerialChain {
        RobotRegistry::create_default(RobotModel::Planar2R)
    }

    fn ctx_with_velocity(limits: Vec<f64>) -> OptimizationContext {
        OptimizationContext {
            joint_limits: crate::domain::context::JointLimits {
                lower: vec![],
                upper: vec![],
                velocity: Some(limits),
                acceleration: None,
            },
            ..OptimizationContext::default()
        }
    }

    // ── 1. Identity ───────────────────────────────────────

    #[test]
    fn identity_returns_correct_values() {
        let op = Retime::new(Retime::DEFAULT_VELOCITY, Retime::DEFAULT_MAX_DURATION_SCALE);
        assert_eq!(op.id(), "retime");
        assert_eq!(op.family(), OperatorFamily::Temporal);
        assert_eq!(op.objective(), OptimizationObjective::Feasibility);
        assert_eq!(op.invariants(), &[Invariant::PreserveJointPath]);
    }

    // ── 2. Joint values preserved ─────────────────────────

    #[test]
    fn joint_values_preserved() {
        let op = Retime::new(3.0, 10.0);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 0.5),
            TrajectoryPoint::new(vec![2.0, 0.0], 1.0),
        ]);
        let region = three_wp_velocity_region();
        let ctx = ctx_with_velocity(vec![0.5, 0.5]);

        let original_joints: Vec<Vec<f64>> = traj
            .waypoints()
            .iter()
            .map(|wp| wp.joints().to_vec())
            .collect();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();

        let result_joints: Vec<Vec<f64>> = result
            .waypoints()
            .iter()
            .map(|wp| wp.joints().to_vec())
            .collect();

        assert_eq!(
            original_joints, result_joints,
            "joint values must be byte-identical"
        );
    }

    // ── 3. Timestamps increase ────────────────────────────

    #[test]
    fn timestamps_increase() {
        let op = Retime::new(3.0, 10.0);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![2.0, 2.0], 0.5),
            TrajectoryPoint::new(vec![4.0, 0.0], 1.0),
        ]);
        let region = three_wp_velocity_region();
        let ctx = ctx_with_velocity(vec![1.0, 1.0]);

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();

        let timestamps: Vec<f64> = result.waypoints().iter().map(|wp| wp.timestamp()).collect();
        for i in 1..timestamps.len() {
            assert!(
                timestamps[i] > timestamps[i - 1],
                "timestamps[{}]={} must be > timestamps[{}]={}",
                i,
                timestamps[i],
                i - 1,
                timestamps[i - 1]
            );
        }
    }

    // ── 4. Velocity limit enforced ────────────────────────

    #[test]
    fn velocity_limit_enforced() {
        let op = Retime::new(3.0, 10.0);
        let robot = test_robot();
        // dq = |1.0 - 0.0| = 1.0 per joint, dt = 0.5 → velocity = 2.0 rad/s > v_max = 1.0
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 0.5),
        ]);
        let region = two_wp_velocity_region();
        let ctx = ctx_with_velocity(vec![1.0, 1.0]);

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();

        let new_dt = result.waypoints()[1].timestamp() - result.waypoints()[0].timestamp();
        // Required: max(1.0/1.0, 1.0/1.0) = 1.0 → stretched from 0.5 to 1.0
        assert!(
            (new_dt - 1.0).abs() < f64::EPSILON,
            "expected dt=1.0 for v_max=1.0, dq=1.0, got {}",
            new_dt
        );
    }

    // ── 5. Within limit unchanged ─────────────────────────

    #[test]
    fn within_limit_unchanged() {
        let op = Retime::new(3.0, 10.0);
        let robot = test_robot();
        // dq = |0.5 - 0.0| = 0.5 per joint, dt = 1.0 → velocity = 0.5 rad/s < v_max = 1.0
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, 0.5], 1.0),
        ]);
        let region = two_wp_velocity_region();
        let ctx = ctx_with_velocity(vec![1.0, 1.0]);

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();

        let new_dt = result.waypoints()[1].timestamp() - result.waypoints()[0].timestamp();
        assert!(
            (new_dt - 1.0).abs() < f64::EPSILON,
            "expected unchanged dt=1.0, got {}",
            new_dt
        );
    }

    // ── 6. Max duration scale cap ─────────────────────────

    #[test]
    fn max_duration_scale_cap() {
        let op = Retime::new(3.0, 2.0); // cap at 2× original
        let robot = test_robot();
        // dq = |10.0 - 0.0| = 10.0, v_max = 1.0 → need 10.0 s
        // cap = 0.5 × 2.0 = 1.0
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0], 0.0),
            TrajectoryPoint::new(vec![10.0], 0.5),
        ]);
        let region = velocity_region(0..2);
        let ctx = ctx_with_velocity(vec![1.0]);

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();

        let new_dt = result.waypoints()[1].timestamp() - result.waypoints()[0].timestamp();
        assert!(
            (new_dt - 1.0).abs() < f64::EPSILON,
            "expected capped dt=1.0 (2×0.5), got {}",
            new_dt
        );
    }

    // ── 7. Single waypoint returns clone ──────────────────

    #[test]
    fn single_waypoint_returns_clone() {
        let op = Retime::new(3.0, 10.0);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 1.0),
        ]);
        // Region with only 1 waypoint (index 1)
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Velocity,
            RegionSeverity::Warning,
            1..2,
        );
        let ctx = ctx_with_velocity(vec![1.0, 1.0]);

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        assert_eq!(result.len(), traj.len());
        for (orig, res) in traj.waypoints().iter().zip(result.waypoints().iter()) {
            assert_eq!(orig.joints(), res.joints());
            assert!((orig.timestamp() - res.timestamp()).abs() < f64::EPSILON);
        }
    }

    // ── 8. Timestamps preserved outside region ────────────

    #[test]
    fn timestamps_preserved_outside_region() {
        let op = Retime::new(3.0, 10.0);
        let robot = test_robot();
        // 4 waypoints: region covers middle 2 (indices 1..3)
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, 0.5], 1.0),
            TrajectoryPoint::new(vec![2.0, 2.0], 2.0), // large dq → stretched
            TrajectoryPoint::new(vec![2.5, 2.5], 3.0),
        ]);
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Velocity,
            RegionSeverity::Warning,
            1..3, // waypoints 1 and 2
        );
        let ctx = ctx_with_velocity(vec![0.5, 0.5]);

        let original_wps = traj.waypoints().to_vec();
        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();

        // Waypoint 0 outside region → fully unchanged
        assert_eq!(result.waypoints()[0].joints(), original_wps[0].joints());
        assert!(
            (result.waypoints()[0].timestamp() - original_wps[0].timestamp()).abs() < f64::EPSILON
        );
        // Waypoint 3 outside region → fully unchanged
        assert_eq!(result.waypoints()[3].joints(), original_wps[3].joints());
        assert!(
            (result.waypoints()[3].timestamp() - original_wps[3].timestamp()).abs() < f64::EPSILON
        );
        // Inside region: wp1 (index 0 in region) timestamp preserved,
        // wp2 (index 1 in region) timestamp stretched
        assert!(
            result.waypoints()[2].timestamp() > original_wps[2].timestamp(),
            "region waypoint timestamp should be stretched"
        );
    }

    // ── 9. Applicability ──────────────────────────────────

    #[test]
    fn applicability_single_waypoint_is_zero() {
        let op = Retime::new(3.0, 10.0);
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Velocity,
            RegionSeverity::Warning,
            0..1,
        );
        assert_eq!(op.applicability(&region), 0.0);
    }

    #[test]
    fn applicability_velocity_region_is_high() {
        let op = Retime::new(3.0, 10.0);
        let region = two_wp_velocity_region();
        assert!((op.applicability(&region) - 0.8).abs() < f32::EPSILON);
    }

    #[test]
    fn applicability_tracking_region_is_high() {
        let op = Retime::new(3.0, 10.0);
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Tracking,
            RegionSeverity::Warning,
            0..2,
        );
        assert!((op.applicability(&region) - 0.8).abs() < f32::EPSILON);
    }

    #[test]
    fn applicability_other_region_is_medium() {
        let op = Retime::new(3.0, 10.0);
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Singularity,
            RegionSeverity::Warning,
            0..2,
        );
        assert!((op.applicability(&region) - 0.5).abs() < f32::EPSILON);
    }

    // ── 10. estimate_cost constant ────────────────────────

    #[test]
    fn estimate_cost_is_constant() {
        let op = Retime::new(3.0, 10.0);
        assert!((op.estimate_cost() - 0.3).abs() < f32::EPSILON);
    }

    // ── 11. estimate_improvement ──────────────────────────

    fn zero_metrics() -> PlanMetrics {
        use thalos_core::evaluation::{
            CollisionMetrics, JointSafetyMetrics, ManipulabilityMetrics,
        };
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

    #[test]
    fn estimate_improvement_velocity_region_is_high() {
        let op = Retime::new(3.0, 10.0);
        let region = two_wp_velocity_region();
        let metrics = zero_metrics();
        assert!((op.estimate_improvement(&region, &metrics) - 0.8).abs() < f32::EPSILON);
    }

    #[test]
    fn estimate_improvement_single_wp_is_zero() {
        let op = Retime::new(3.0, 10.0);
        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Velocity,
            RegionSeverity::Warning,
            0..1,
        );
        let metrics = zero_metrics();
        assert!((op.estimate_improvement(&region, &metrics) - 0.0).abs() < f32::EPSILON);
    }

    // ── 12. Default velocity fallback ─────────────────────

    #[test]
    fn default_velocity_fallback_used_when_no_limits() {
        let op = Retime::new(1.0, 10.0); // default_velocity = 1.0 rad/s
        let robot = test_robot();
        // dq = |2.0 - 0.0| = 2.0 per joint, dt = 0.5 → would need 2.0 s with v_max=1.0
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![2.0, 2.0], 0.5),
        ]);
        let region = two_wp_velocity_region();
        // No velocity limits in ctx → falls back to self.default_velocity = 1.0
        let ctx = OptimizationContext::default();

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();

        let new_dt = result.waypoints()[1].timestamp() - result.waypoints()[0].timestamp();
        // Required: max(2.0/1.0, 2.0/1.0) = 2.0
        assert!(
            (new_dt - 2.0).abs() < f64::EPSILON,
            "expected dt=2.0 with default_velocity=1.0, dq=2.0, got {}",
            new_dt
        );
    }

    // ── 13. ConstraintQuery timing guard (2.4) ────────────

    use thalos_core::operation::PrecisionLevel;

    /// Mock query: only `can_modify_timing` is overridden; every other
    /// guard returns `true`.
    struct TimingMock {
        allowed: Vec<bool>,
    }

    impl ConstraintQuery for TimingMock {
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
        fn can_modify_timing(&self, i: usize) -> bool {
            self.allowed.get(i).copied().unwrap_or(true)
        }
    }

    #[test]
    fn constrained_waypoint_timing_preserved() {
        let op = Retime::new(3.0, 10.0);
        let robot = test_robot();
        // dq = 1.0 per joint, dt = 0.5 → would stretch to dt=1.0 with v_max=1.0
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 0.5),
            TrajectoryPoint::new(vec![2.0, 2.0], 1.0),
        ]);
        let region = three_wp_velocity_region(); // 0..3
        let ctx = ctx_with_velocity(vec![1.0, 1.0]);

        // Waypoint 1 (absolute index 1) cannot have its timing modified.
        let mock = TimingMock {
            allowed: vec![true, false, true],
        };

        let result = op
            .apply(&robot, &traj, &region, &ctx, Some(&mock))
            .unwrap();

        let t0 = result.waypoints()[0].timestamp();
        let t1 = result.waypoints()[1].timestamp();
        let t2 = result.waypoints()[2].timestamp();

        // wp0 is never modified (first waypoint preserved by invariant).
        assert!((t0 - 0.0).abs() < f64::EPSILON, "wp0 timestamp, got {t0}");
        // wp1 keeps its ORIGINAL timestamp despite the velocity violation.
        assert!(
            (t1 - 0.5).abs() < f64::EPSILON,
            "constrained wp1 timestamp must be preserved, got {t1}"
        );
        // wp2 is retimed around the constrained waypoint: dt(wp1→wp2)=1.0.
        assert!(
            (t2 - 1.5).abs() < f64::EPSILON,
            "wp2 must be retimed around constrained wp1, got {t2}"
        );
    }

    #[test]
    fn unconstrained_waypoints_retimed_with_guard_allowed() {
        let op = Retime::new(3.0, 10.0);
        let robot = test_robot();
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 1.0], 0.5),
            TrajectoryPoint::new(vec![2.0, 2.0], 1.0),
        ]);
        let region = three_wp_velocity_region();
        let ctx = ctx_with_velocity(vec![1.0, 1.0]);

        // All waypoints free → identical to the no-constraint behavior.
        let mock = TimingMock {
            allowed: vec![true, true, true],
        };
        let with_query = op
            .apply(&robot, &traj, &region, &ctx, Some(&mock))
            .unwrap();
        let without_query = op.apply(&robot, &traj, &region, &ctx, None).unwrap();

        for (a, b) in with_query
            .waypoints()
            .iter()
            .zip(without_query.waypoints().iter())
        {
            assert!(
                (a.timestamp() - b.timestamp()).abs() < f64::EPSILON,
                "all-free guard must match None behavior"
            );
        }
        // Sanity: all waypoints were actually stretched (both paths).
        let t1 = with_query.waypoints()[1].timestamp();
        let t2 = with_query.waypoints()[2].timestamp();
        assert!((t1 - 1.0).abs() < f64::EPSILON, "wp1 stretched, got {t1}");
        assert!((t2 - 2.0).abs() < f64::EPSILON, "wp2 stretched, got {t2}");
    }

    #[test]
    fn locked_final_waypoint_keeps_timestamps_monotonic() {
        let op = Retime::new(3.0, 10.0);
        let robot = test_robot();
        // Fast-moving joints that would stretch each segment to dt=4.0 under
        // v_max=1.0, with a LOCKED final waypoint at the original timestamp.
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![4.0, 4.0], 1.0),
            TrajectoryPoint::new(vec![8.0, 8.0], 2.0),
        ]);
        let region = three_wp_velocity_region(); // 0..3
        let ctx = ctx_with_velocity(vec![1.0, 1.0]);

        // Final waypoint (absolute index 2) cannot have its timing modified.
        let mock = TimingMock {
            allowed: vec![true, true, false],
        };

        let result = op
            .apply(&robot, &traj, &region, &ctx, Some(&mock))
            .unwrap();

        let t: Vec<f64> = result.waypoints().iter().map(|wp| wp.timestamp()).collect();

        // Regression: earlier free stretches must never overshoot the locked
        // final anchor, otherwise timestamps become non-monotonic ([0,4,2]).
        for pair in t.windows(2) {
            assert!(
                pair[1] > pair[0],
                "timestamps must be strictly increasing, got {t:?}"
            );
        }
        // The locked final waypoint keeps its original timestamp.
        assert!(
            (t[2] - 2.0).abs() < f64::EPSILON,
            "locked final wp timestamp must be preserved, got {}",
            t[2]
        );
        // The free stretch is capped just below the anchor, not left at dt=4.0.
        assert!(
            t[1] < 2.0,
            "free wp1 must not overshoot the locked anchor, got {}",
            t[1]
        );
    }

    #[test]
    fn multiple_free_waypoints_before_anchor_keep_monotonic_timestamps() {
        let op = Retime::new(3.0, 10.0);
        let robot = test_robot();
        // Two consecutive free waypoints before a locked final anchor. Each
        // segment would stretch to dt=4.0 under v_max=1.0, but the anchor at
        // t=3.0 caps both; without per-index ceilings they would collapse to
        // the same value and break strict monotonicity.
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![4.0, 4.0], 1.0),
            TrajectoryPoint::new(vec![8.0, 8.0], 2.0),
            TrajectoryPoint::new(vec![12.0, 12.0], 3.0),
        ]);
        let region = four_wp_velocity_region(); // 0..4
        let ctx = ctx_with_velocity(vec![1.0, 1.0]);

        let mock = TimingMock {
            allowed: vec![true, true, true, false],
        };

        let result = op
            .apply(&robot, &traj, &region, &ctx, Some(&mock))
            .unwrap();

        let t: Vec<f64> = result.waypoints().iter().map(|wp| wp.timestamp()).collect();

        for pair in t.windows(2) {
            assert!(
                pair[1] > pair[0],
                "timestamps must be strictly increasing, got {t:?}"
            );
        }
        assert!(
            (t[3] - 3.0).abs() < f64::EPSILON,
            "locked final wp timestamp must be preserved, got {}",
            t[3]
        );
        assert!(
            t[1] < t[2],
            "free waypoints must not collapse to the same timestamp, got {t:?}"
        );
    }
}

// ── Integration tests ───────────────────────────────────────

#[cfg(test)]
mod integration_tests {
    use super::*;
    use crate::operators::JointCenteringOperator;
    use thalos_core::{
        analysis::region::{RegionId, RegionKind, RegionSeverity},
        models::{RobotModel, RobotRegistry},
    };

    fn test_robot() -> SerialChain {
        RobotRegistry::create_default(RobotModel::Planar2R)
    }

    fn joint_centering_ctx() -> OptimizationContext {
        OptimizationContext {
            joint_limits: crate::domain::context::JointLimits {
                lower: vec![-10.0, -10.0],
                upper: vec![10.0, 10.0],
                velocity: None,
                acceleration: None,
            },
            ..OptimizationContext::default()
        }
    }

    fn ctx_with_velocity(limits: Vec<f64>) -> OptimizationContext {
        OptimizationContext {
            joint_limits: crate::domain::context::JointLimits {
                lower: vec![-10.0, -10.0],
                upper: vec![10.0, 10.0],
                velocity: Some(limits),
                acceleration: None,
            },
            ..OptimizationContext::default()
        }
    }

    // ── 1. Composable with JointCentering ─────────────────

    #[test]
    fn composable_with_joint_centering() {
        let retime = Retime::new(3.0, 10.0);
        let centering = JointCenteringOperator::new(0.5);
        let robot = test_robot();

        // Start with a trajectory where joints are NOT centered
        // (lower=upper extremes → JointCentering will move them toward 0.0)
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![5.0, 5.0], 0.0),
            TrajectoryPoint::new(vec![8.0, 8.0], 0.5),
        ]);

        let region = ProblemRegion::new(
            RegionId(0),
            RegionKind::Velocity,
            RegionSeverity::Warning,
            0..2,
        );

        // Apply Retime first (stretches timestamps)
        let retime_ctx = ctx_with_velocity(vec![0.5, 0.5]);
        let retimed = retime
            .apply(&robot, &traj, &region, &retime_ctx, None)
            .unwrap();

        // Verify Retime actually stretched
        let retimed_dt = retimed.waypoints()[1].timestamp() - retimed.waypoints()[0].timestamp();
        assert!(
            retimed_dt > 0.5,
            "Retime should stretch dt, got {}",
            retimed_dt
        );

        // Then apply JointCentering to the retimed result
        let centered = centering
            .apply(&robot, &retimed, &region, &joint_centering_ctx(), None)
            .unwrap();

        // Verify both operators' effects are visible:
        // 1. Joints should be centered (moved toward 0.0 from [5.0, 5.0] / [8.0, 8.0])
        let first_joints = centered.waypoints()[0].joints();
        assert!(
            first_joints[0] < 5.0,
            "JointCentering should move joints toward center, got {}",
            first_joints[0]
        );

        // 2. Timestamps should still be stretched from Retime
        let centered_dt = centered.waypoints()[1].timestamp() - centered.waypoints()[0].timestamp();
        assert!(
            (centered_dt - retimed_dt).abs() < f64::EPSILON,
            "Retime timestamp stretch should survive JointCentering: expected dt={}, got {}",
            retimed_dt,
            centered_dt
        );

        // 3. Waypoint count preserved
        assert_eq!(centered.len(), traj.len());
    }
}
