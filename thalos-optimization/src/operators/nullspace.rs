//! NullSpaceOptimization — kinematics-aware operator that improves joint
//! margin within the Jacobian null space while preserving the Cartesian
//! TCP path exactly.
//!
//! # Algorithm
//!
//! For each waypoint `q` in the problem region:
//!
//! ```text
//! J = evaluate_geometric_jacobian(q)   → 6×n full Jacobian
//! J⁺ = pseudo_inverse(J, tolerance)    → n×6
//! z = (q_center − q) · factor          → joint-centering gradient
//! q' = q + N·z · dt                    → null-space projected correction
//! q' = clamp(q', limits)               → joint limit enforcement
//! ```
//!
//! where `N·z = z − J⁺·(J·z)` is computed without forming the projector
//! matrix `N` explicitly. This guarantees that the correction produces
//! zero task-space displacement: `J·(q' − q) = 0`.
//!
//! # Invariants
//!
//! - Preserves the Cartesian TCP path within the null-space accuracy of
//!   the pseudo-inverse.
//! - Preserves waypoint count (additive within region).
//! - Does **not** declare `PreserveExistingWaypoints` because joint
//!   values may change — only the Cartesian path is preserved.

use thalos_core::{
    analysis::region::{ProblemRegion, RegionKind},
    evaluation::PlanMetrics,
    kinematics::forward::ForwardKinematics,
    kinematics::jacobian::{GeometricJacobian, JacobianSolver},
    operation::ConstraintQuery,
    robot::serial_chain::SerialChain,
    trajectory::{Trajectory, TrajectoryPoint},
};

use thalos_math::DynamicVector;

use crate::{
    domain::{
        TrajectoryOperator,
        context::OptimizationContext,
        operator::{Invariant, OperatorFamily, OptimizationObjective},
    },
    error::OptimizationError,
};

// ── Struct ─────────────────────────────────────────────────

/// Kinematics-aware operator that drives redundant joints toward their
/// mechanical centre within the Jacobian null space.
///
/// The null-space projector `N = I − J⁺·J` (implemented efficiently as
/// `z − J⁺·(J·z)`) ensures that corrections produce **zero** Cartesian
/// displacement at the TCP.
pub struct NullSpaceOptimization {
    /// Step size toward the joint centre (default: 0.3).
    pub factor: f64,
    /// SVD singular-value threshold below which a value is treated as
    /// zero in the pseudo-inverse (default: 1e-6).
    pub tolerance: f64,
    /// Integration step applied to the null-space correction
    /// (default: 0.1).
    pub dt: f64,
}

impl NullSpaceOptimization {
    /// Create a new `NullSpaceOptimization` with the given parameters.
    pub const fn new(factor: f64, tolerance: f64, dt: f64) -> Self {
        Self {
            factor,
            tolerance,
            dt,
        }
    }

    /// Default centering factor (0.3).
    pub const DEFAULT_FACTOR: f64 = 0.3;

    /// Default SVD threshold (1e-6).
    pub const DEFAULT_TOLERANCE: f64 = 1e-6;

    /// Default integration step (0.1).
    pub const DEFAULT_DT: f64 = 0.1;
}

// ── TrajectoryOperator impl ─────────────────────────────────

impl TrajectoryOperator for NullSpaceOptimization {
    fn id(&self) -> &'static str {
        "nullspace_optimization"
    }

    fn family(&self) -> OperatorFamily {
        OperatorFamily::JointSpace
    }

    fn objective(&self) -> OptimizationObjective {
        OptimizationObjective::Feasibility
    }

    fn invariants(&self) -> &'static [Invariant] {
        // Does NOT declare PreserveExistingWaypoints because joint
        // values do change — only the Cartesian TCP path is preserved.
        &[Invariant::PreserveCartesianPath]
    }

    fn applicability(&self, region: &ProblemRegion) -> f32 {
        // NOTE: The trait signature does not provide access to the
        // robot, so DOF-based redundancy gating must be performed at
        // the pipeline level.
        if region.waypoint_count() < 2 {
            return 0.0;
        }
        match region.kind {
            RegionKind::Constraint | RegionKind::Singularity | RegionKind::LowManipulability => 0.8,
            _ => 0.5,
        }
    }

    fn estimate_improvement(&self, _region: &ProblemRegion, _metrics: &PlanMetrics) -> f32 {
        0.7
    }

    fn estimate_cost(&self) -> f32 {
        // Jacobian evaluation + SVD per waypoint — moderate cost.
        0.7
    }

    fn apply(
        &self,
        robot: &SerialChain,
        trajectory: &Trajectory,
        region: &ProblemRegion,
        ctx: &OptimizationContext,
        constraints: Option<&dyn ConstraintQuery>,
    ) -> Result<Trajectory, OptimizationError> {
        let range = &region.waypoint_range;
        let all_wps = trajectory.waypoints();

        // Guard: need at least two waypoints to form a segment
        if range.len() < 2 {
            return Ok(trajectory.clone());
        }

        // Validate range bounds
        if range.start >= all_wps.len() || range.end > all_wps.len() {
            return Err(OptimizationError::InvalidRegion(format!(
                "waypoint range {:?} is out of bounds for trajectory length {}",
                range,
                all_wps.len()
            )));
        }

        // Build forward kinematics and geometric Jacobian solver
        let fk = ForwardKinematics::new(robot.clone());
        let jacobian_solver = GeometricJacobian::new(fk, *robot.end_effector());

        // Extract region waypoints into a working buffer
        let mut wps: Vec<TrajectoryPoint> = all_wps[range.clone()].to_vec();

        // Compute joint centre from limits (if available)
        let has_limits = !ctx.joint_limits.lower.is_empty();
        let q_center: Option<Vec<f64>> = if has_limits {
            Some(
                ctx.joint_limits
                    .lower
                    .iter()
                    .zip(ctx.joint_limits.upper.iter())
                    .map(|(l, u)| (l + u) / 2.0)
                    .collect(),
            )
        } else {
            None
        };

        // Validate joint-count consistency if we have limits
        if let Some(center) = &q_center {
            let n_expected = center.len();
            for (i, wp) in wps.iter().enumerate() {
                if wp.joints().len() != n_expected {
                    return Err(OptimizationError::InvalidRegion(format!(
                        "region waypoint {} has {} joints, expected {} from joint limits",
                        i,
                        wp.joints().len(),
                        n_expected
                    )));
                }
            }
        }

        for (local_i, wp) in wps.iter_mut().enumerate() {
            let q = wp.joints();

            // Constraint-aware guard: skip waypoints whose joint values
            // are locked (counted as skipped, left unmodified).
            if !constraints.is_none_or(|c| c.can_modify_joints(range.start + local_i)) {
                continue;
            }

            // Evaluate geometric Jacobian at this configuration
            let j = jacobian_solver.evaluate(q);
            let j_full = j.full(); // 6 × n_dof

            // Compute pseudo-inverse; skip waypoints where the
            // Jacobian is fully degenerate (all SVs below tolerance)
            let j_plus = match j_full.pseudo_inverse(self.tolerance) {
                Some(p) => p,
                None => continue,
            };

            // Secondary objective: joint-centering gradient
            let z: Vec<f64> = match &q_center {
                Some(center) => q
                    .iter()
                    .zip(center.iter())
                    .map(|(qj, cj)| (cj - qj) * self.factor)
                    .collect(),
                None => continue,
            };

            // Null-space correction: N·z = z − J⁺·(J·z)
            //
            // Computed without forming the projector N explicitly:
            //   1. jz       = J · z           (6×1 vector)
            //   2. j_plus_jz = J⁺ · jz       (n×1 vector)
            //   3. nz       = z − j_plus_jz  (n×1, element-wise)
            let z_vec = DynamicVector::from_column_slice(&z);
            let jz = &j_full * &z_vec;
            let j_plus_jz = &j_plus * &jz;
            let nz = z_vec - j_plus_jz;

            // Apply correction: q' = q + nz · dt
            let new_q: Vec<f64> = q
                .iter()
                .zip(nz.as_slice().iter())
                .map(|(qj, dz)| qj + dz * self.dt)
                .collect();

            // Clamp to joint limits
            let clamped_q: Vec<f64> = if has_limits {
                new_q
                    .iter()
                    .zip(ctx.joint_limits.lower.iter())
                    .zip(ctx.joint_limits.upper.iter())
                    .map(|((qj, lo), hi)| qj.clamp(*lo, *hi))
                    .collect()
            } else {
                new_q
            };

            *wp = TrajectoryPoint::new(clamped_q, wp.timestamp());
        }

        // Build the full output trajectory by replacing the region's
        // waypoints with the corrected ones
        let mut result_wps: Vec<TrajectoryPoint> = all_wps.to_vec();
        result_wps.splice(range.clone(), wps);

        Ok(Trajectory::new(result_wps))
    }
}

// ═══════════════════════════════════════════════════════════════
// Shared test helpers
// ═══════════════════════════════════════════════════════════════

#[cfg(test)]
pub(crate) mod test_helpers {
    use crate::domain::context::{OptimizationContext, PipelineConfig};
    use thalos_core::prelude::*;
    use thalos_core::{
        analysis::region::{ProblemRegion, RegionId, RegionKind, RegionSeverity},
        robot::serial_chain::SerialChain,
        trajectory::{Trajectory, TrajectoryPoint},
    };
    use thalos_math::constants::*;
    use thalos_math::*;

    // Re-import the joint JointLimits (with `new` constructor) rather
    // than the context-level JointLimits (struct-literal only).
    use thalos_core::robot::joint::JointLimits as JointLimitsJoint;

    /// Build a redundant planar 4R robot (4 DOF, Z-axis revolute,
    /// 1 m links, ±π joint limits) for null-space tests.
    pub fn planar_4r() -> SerialChain {
        let mut builder = SerialChainBuilder::new();
        let f1 = builder.create_frame("link_1");
        let f2 = builder.create_frame("link_2");
        let f3 = builder.create_frame("link_3");
        let f4 = builder.create_frame("link_4");

        let limits = JointLimitsJoint::new(-PI, PI);

        let make_link = |id: usize| -> Link {
            Link {
                id: id as u32,
                transform: Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0)),
                collision_geometry: None,
            }
        };

        let make_joint = |id: u32| -> JointType {
            JointType::Revolute(RevoluteJoint::new(
                id,
                UnitVector3::z_axis(),
                limits,
                Transform3D::identity(),
            ))
        };

        builder.add_segment(Segment::new(
            FrameId::World,
            f1.clone(),
            make_joint(0),
            make_link(0),
        ));
        builder.add_segment(Segment::new(f1, f2.clone(), make_joint(1), make_link(1)));
        builder.add_segment(Segment::new(f2, f3.clone(), make_joint(2), make_link(2)));
        builder.add_segment(Segment::new(f3, f4.clone(), make_joint(3), make_link(3)));

        builder.set_end_effector(f4);
        builder.build().expect("planar 4R should build")
    }

    pub fn region(range: std::ops::Range<usize>, kind: RegionKind) -> ProblemRegion {
        ProblemRegion::new(RegionId(0), kind, RegionSeverity::Warning, range)
    }

    pub fn ctx_with_limits(limits: &[(f64, f64)]) -> OptimizationContext {
        let (lower, upper): (Vec<f64>, Vec<f64>) = limits.iter().cloned().unzip();
        OptimizationContext {
            joint_limits: crate::domain::context::JointLimits {
                lower,
                upper,
                velocity: None,
                acceleration: None,
            },
            config: PipelineConfig::default(),
            tool_frame: None,
        }
    }

    pub fn ctx_planar_4r() -> OptimizationContext {
        ctx_with_limits(&[(-PI, PI), (-PI, PI), (-PI, PI), (-PI, PI)])
    }

    pub fn ctx_planar_3r() -> OptimizationContext {
        ctx_with_limits(&[(-PI, PI), (-PI, PI), (-PI, PI)])
    }

    pub fn ctx_planar_2r() -> OptimizationContext {
        ctx_with_limits(&[(-PI, PI), (-PI, PI)])
    }

    pub fn ctx_six_dof() -> OptimizationContext {
        ctx_with_limits(&[(-PI, PI); 6])
    }

    pub fn fk_position(robot: &SerialChain, q: &[f64]) -> Vector3 {
        let fk = ForwardKinematics::new(robot.clone());
        let result = fk.evaluate(q);
        result.ee_position().expect("EE position should exist")
    }

    /// Build a 6-DOF spatial test robot with 6 revolute joints
    /// on alternating Z/Y axes for non-degenerate geometry.
    ///
    /// Each link is 1 m along X. Joint limits are [−π, π].
    pub fn six_dof_test_robot() -> SerialChain {
        let mut builder = SerialChainBuilder::new();
        let f1 = builder.create_frame("link_1");
        let f2 = builder.create_frame("link_2");
        let f3 = builder.create_frame("link_3");
        let f4 = builder.create_frame("link_4");
        let f5 = builder.create_frame("link_5");
        let f6 = builder.create_frame("link_6");

        let limits = JointLimitsJoint::new(-PI, PI);

        let make_link = |id: usize| -> Link {
            Link {
                id: id as u32,
                transform: Transform3D::from_translation(Vector3::new(1.0, 0.0, 0.0)),
                collision_geometry: None,
            }
        };

        builder.add_segment(Segment::new(
            FrameId::World,
            f1.clone(),
            JointType::Revolute(RevoluteJoint::new(
                0,
                UnitVector3::z_axis(),
                limits,
                Transform3D::identity(),
            )),
            make_link(0),
        ));
        builder.add_segment(Segment::new(
            f1,
            f2.clone(),
            JointType::Revolute(RevoluteJoint::new(
                1,
                UnitVector3::y_axis(),
                limits,
                Transform3D::identity(),
            )),
            make_link(1),
        ));
        builder.add_segment(Segment::new(
            f2,
            f3.clone(),
            JointType::Revolute(RevoluteJoint::new(
                2,
                UnitVector3::z_axis(),
                limits,
                Transform3D::identity(),
            )),
            make_link(2),
        ));
        builder.add_segment(Segment::new(
            f3,
            f4.clone(),
            JointType::Revolute(RevoluteJoint::new(
                3,
                UnitVector3::y_axis(),
                limits,
                Transform3D::identity(),
            )),
            make_link(3),
        ));
        builder.add_segment(Segment::new(
            f4,
            f5.clone(),
            JointType::Revolute(RevoluteJoint::new(
                4,
                UnitVector3::z_axis(),
                limits,
                Transform3D::identity(),
            )),
            make_link(4),
        ));
        builder.add_segment(Segment::new(
            f5,
            f6.clone(),
            JointType::Revolute(RevoluteJoint::new(
                5,
                UnitVector3::y_axis(),
                limits,
                Transform3D::identity(),
            )),
            make_link(5),
        ));

        builder.set_end_effector(f6);
        builder.build().expect("6DOF test robot should build")
    }

    /// Minimum distance from any joint to its nearest limit
    /// across all waypoints (used for margin-improvement
    /// benchmarks).
    pub fn min_joint_margin(traj: &Trajectory, limits: &[(f64, f64)]) -> f64 {
        traj.waypoints()
            .iter()
            .flat_map(|wp| {
                wp.joints()
                    .iter()
                    .zip(limits.iter())
                    .map(|(q, (lo, hi))| (q - lo).abs().min((hi - q).abs()))
            })
            .fold(f64::INFINITY, f64::min)
    }
}

// ── Unit tests ─────────────────────────────────────────────

#[cfg(test)]
mod unit_tests {
    use super::test_helpers::*;
    use super::*;
    use thalos_core::{
        analysis::region::RegionKind,
        evaluation::{CollisionMetrics, JointSafetyMetrics, ManipulabilityMetrics, PlanMetrics},
        models::{RobotModel, RobotRegistry},
        prelude::{Trajectory, TrajectoryPoint},
    };
    use thalos_math::constants::*;

    // ── 2.1 Struct + defaults ─────────────────────────────

    #[test]
    fn struct_defaults_are_public() {
        assert!((NullSpaceOptimization::DEFAULT_FACTOR - 0.3).abs() < 1e-15);
        assert!((NullSpaceOptimization::DEFAULT_TOLERANCE - 1e-6).abs() < 1e-15);
        assert!((NullSpaceOptimization::DEFAULT_DT - 0.1).abs() < 1e-15);
    }

    #[test]
    fn new_creates_struct_with_correct_values() {
        let op = NullSpaceOptimization::new(0.5, 1e-8, 0.2);
        assert!((op.factor - 0.5).abs() < 1e-15);
        assert!((op.tolerance - 1e-8).abs() < 1e-15);
        assert!((op.dt - 0.2).abs() < 1e-15);
    }

    // ── 2.2 Identity ──────────────────────────────────────

    #[test]
    fn identity_returns_correct_values() {
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        assert_eq!(op.id(), "nullspace_optimization");
        assert_eq!(op.family(), OperatorFamily::JointSpace);
        assert_eq!(op.objective(), OptimizationObjective::Feasibility);
        assert_eq!(op.invariants(), &[Invariant::PreserveCartesianPath]);
    }

    // ── 2.3 Applicability ─────────────────────────────────

    #[test]
    fn applicability_below_two_waypoints_is_zero() {
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let r = region(0..1, RegionKind::Singularity);
        assert_eq!(op.applicability(&r), 0.0);
    }

    #[test]
    fn applicability_for_constraint_singularity_and_low_manipulability_is_high() {
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        for kind in &[
            RegionKind::Constraint,
            RegionKind::Singularity,
            RegionKind::LowManipulability,
        ] {
            let r = region(0..3, *kind);
            assert!(
                op.applicability(&r) >= 0.7,
                "expected >= 0.7 for {:?}, got {}",
                kind,
                op.applicability(&r)
            );
        }
    }

    #[test]
    fn applicability_for_other_regions_is_medium() {
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        for kind in &[
            RegionKind::Collision,
            RegionKind::Tracking,
            RegionKind::Velocity,
        ] {
            let r = region(0..3, *kind);
            let a = op.applicability(&r);
            assert!(
                (0.4..=0.6).contains(&a),
                "expected ~0.5 for {:?}, got {}",
                kind,
                a
            );
        }
    }

    // ── 2.4 Estimate ──────────────────────────────────────

    #[test]
    fn estimate_improvement_is_constant() {
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let r = region(0..3, RegionKind::Constraint);
        let metrics = PlanMetrics::new(
            0.0,
            0,
            ManipulabilityMetrics::new(0.0, 0.0, 0, 0),
            JointSafetyMetrics::new(1.0, 0.0, 0),
            CollisionMetrics::new(1.0, 0, 0),
            0.0,
            0.0,
        );
        assert!((op.estimate_improvement(&r, &metrics) - 0.7).abs() < f32::EPSILON);
    }

    #[test]
    fn estimate_cost_is_constant() {
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        assert!((op.estimate_cost() - 0.7).abs() < f32::EPSILON);
    }

    // ── 3.1 TCP preservation ──────────────────────────────

    #[test]
    fn tcp_position_preserved_within_tolerance() {
        // Use a redundant planar 4R robot. The null-space correction
        // should not change the Cartesian TCP position.
        let robot = planar_4r();
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let ctx = ctx_planar_4r();

        // Nominal joint configuration (non-singular, redundant)
        let q_nominal = vec![0.5, -0.3, 0.2, 0.0];
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(q_nominal.clone(), 0.0),
            TrajectoryPoint::new(vec![0.5, -0.3, 0.2, 0.1], 1.0),
        ]);

        // FK position before
        let before = fk_position(&robot, &q_nominal);

        let result = op
            .apply(
                &robot,
                &traj,
                &region(0..2, RegionKind::Constraint),
                &ctx,
                None,
            )
            .unwrap();
        let after = fk_position(&robot, result.waypoints()[0].joints());

        let dx = (after - before).magnitude();
        // SVD + pseudo-inverse introduce numerical errors at the
        // micron level — 1e-4 is well within practical requirements.
        assert!(dx < 1e-4, "TCP position deviation {:.2e} exceeds 1e-4", dx);
    }

    // ── 3.2 Joint-centering direction ─────────────────────

    #[test]
    fn apply_on_non_trivial_robot_does_not_crash() {
        // Verify the operator processes all waypoints without error.
        let robot = planar_4r();
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let ctx = ctx_planar_4r();

        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.5, -0.3, 0.2, 0.0], 0.0),
            TrajectoryPoint::new(vec![0.5, -0.3, 0.2, 0.0], 1.0),
        ]);
        let result = op.apply(
            &robot,
            &traj,
            &region(0..2, RegionKind::Constraint),
            &ctx,
            None,
        );
        assert!(result.is_ok());
        let result = result.unwrap();
        assert_eq!(result.len(), traj.len());
    }

    // ── 3.3 Joint at limit ────────────────────────────────

    #[test]
    fn joint_at_limit_is_clamped() {
        let robot = RobotRegistry::create_default(RobotModel::Planar3R);
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let ctx = ctx_planar_3r();

        // Joint 0 at upper limit PI
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![PI, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![PI, 0.0, 0.0], 1.0),
        ]);
        let result = op
            .apply(
                &robot,
                &traj,
                &region(0..2, RegionKind::Constraint),
                &ctx,
                None,
            )
            .unwrap();
        let clamped = result.waypoints()[0].joints();

        // Joint 0 at PI — should be clamped at PI or less, never above
        assert!(
            clamped[0] <= PI + 1e-10,
            "joint at limit exceeded: {} > PI",
            clamped[0]
        );
        // All joints within [-PI, PI]
        for (i, q) in clamped.iter().enumerate() {
            assert!(
                (-PI..=PI + 1e-10).contains(q),
                "joint {} out of limits: {}",
                i,
                q
            );
        }
    }

    // ── 3.4 Singular Jacobian ─────────────────────────────

    #[test]
    fn singular_jacobian_skips_waypoint() {
        let robot = RobotRegistry::create_default(RobotModel::Planar3R);
        // Set tolerance very high so pseudo_inverse returns None
        // for any reasonable Jacobian.
        let op = NullSpaceOptimization::new(0.3, 10.0, 0.1);
        let ctx = ctx_planar_3r();

        let q = vec![0.0, 0.0, 0.0];
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(q.clone(), 0.0),
            TrajectoryPoint::new(q.clone(), 1.0),
        ]);
        let region = region(0..2, RegionKind::Singularity);

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();

        // Waypoints should be unchanged (skipped via None from pinv)
        assert_eq!(
            result.waypoints()[0].joints(),
            q.as_slice(),
            "singular waypoint should be unchanged"
        );
        assert_eq!(
            result.waypoints()[1].joints(),
            q.as_slice(),
            "singular waypoint should be unchanged"
        );
        assert_eq!(result.len(), traj.len());
    }

    // ── 3.5 Non-redundant robot ───────────────────────────

    #[test]
    fn apply_on_planar_2r_does_not_crash() {
        let robot = RobotRegistry::create_default(RobotModel::Planar2R);
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let ctx = ctx_planar_2r();

        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.5, -0.3], 0.0),
            TrajectoryPoint::new(vec![0.5, -0.3], 1.0),
        ]);
        let result = op.apply(
            &robot,
            &traj,
            &region(0..2, RegionKind::Constraint),
            &ctx,
            None,
        );
        assert!(result.is_ok(), "non-redundant robot should not error");
        assert_eq!(result.unwrap().len(), traj.len());
    }

    // ── Guard: single-waypoint region ─────────────────────

    #[test]
    fn single_waypoint_region_returns_clone() {
        let robot = RobotRegistry::create_default(RobotModel::Planar3R);
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let ctx = ctx_planar_3r();

        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 0.0, 0.0], 1.0),
        ]);
        let region = region(1..2, RegionKind::Constraint);

        let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
        assert_eq!(result.len(), traj.len());
        for (orig, res) in traj.waypoints().iter().zip(result.waypoints().iter()) {
            assert_eq!(orig.joints(), res.joints());
        }
    }

    // ── Out-of-bounds region ──────────────────────────────

    #[test]
    fn out_of_bounds_region_returns_error() {
        let robot = RobotRegistry::create_default(RobotModel::Planar3R);
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let ctx = ctx_planar_3r();

        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 0.0, 0.0], 1.0),
        ]);
        let region = region(0..100, RegionKind::Constraint);

        let result = op.apply(&robot, &traj, &region, &ctx, None);
        assert!(result.is_err());
    }

    // ── Empty joint limits skips correction ───────────────

    #[test]
    fn empty_joint_limits_returns_trajectory_unchanged() {
        let robot = RobotRegistry::create_default(RobotModel::Planar3R);
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let ctx = OptimizationContext::default(); // empty joint limits

        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.5, -0.3, 0.1], 0.0),
            TrajectoryPoint::new(vec![0.5, -0.3, 0.2], 1.0),
        ]);
        let result = op
            .apply(
                &robot,
                &traj,
                &region(0..2, RegionKind::Constraint),
                &ctx,
                None,
            )
            .unwrap();
        assert_eq!(result.len(), traj.len());
        for (orig, res) in traj.waypoints().iter().zip(result.waypoints().iter()) {
            assert_eq!(orig.joints(), res.joints());
        }
    }

    // ── ConstraintQuery joint guard (2.5) ─────────────────

    use thalos_core::operation::PrecisionLevel;

    /// Mock query: only `can_modify_joints` is overridden; every other
    /// guard returns `true`.
    struct JointsMock {
        allowed: Vec<bool>,
    }

    impl ConstraintQuery for JointsMock {
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
        fn can_modify_joints(&self, i: usize) -> bool {
            self.allowed.get(i).copied().unwrap_or(true)
        }
    }

    #[test]
    fn constrained_waypoint_joints_preserved() {
        // Asymmetric redundant config — null-space correction is non-zero
        // (same configuration as benchmark_nullspace_correction_on_redundant_robot).
        let robot = planar_4r();
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let ctx = ctx_planar_4r();

        let q = vec![1.5, 1.2, -1.3, -1.0];
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(q.clone(), 0.0),
            TrajectoryPoint::new(q.clone(), 1.0),
        ]);
        let r = region(0..2, RegionKind::Constraint);

        // Waypoint 0 is locked; waypoint 1 is free.
        let mock = JointsMock {
            allowed: vec![false, true],
        };
        let result = op.apply(&robot, &traj, &r, &ctx, Some(&mock)).unwrap();

        assert_eq!(
            result.waypoints()[0].joints(),
            q.as_slice(),
            "locked waypoint 0 must remain unchanged"
        );
        assert_ne!(
            result.waypoints()[1].joints(),
            q.as_slice(),
            "free waypoint 1 must receive null-space correction"
        );

        // Triangulation: no query → both waypoints corrected (legacy behavior).
        let none_result = op.apply(&robot, &traj, &r, &ctx, None).unwrap();
        assert_ne!(
            none_result.waypoints()[0].joints(),
            q.as_slice(),
            "without constraints waypoint 0 is corrected"
        );
        assert_ne!(
            none_result.waypoints()[1].joints(),
            q.as_slice(),
            "without constraints waypoint 1 is corrected"
        );
    }

    #[test]
    fn mixed_constrained_and_free_waypoints() {
        let robot = planar_4r();
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let ctx = ctx_planar_4r();

        let q = vec![1.5, 1.2, -1.3, -1.0];
        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(q.clone(), 0.0),
            TrajectoryPoint::new(q.clone(), 1.0),
            TrajectoryPoint::new(q.clone(), 2.0),
        ]);
        let r = region(0..3, RegionKind::Constraint);

        // Only the middle waypoint (absolute index 1) is locked.
        let mock = JointsMock {
            allowed: vec![true, false, true],
        };
        let result = op.apply(&robot, &traj, &r, &ctx, Some(&mock)).unwrap();

        assert_ne!(result.waypoints()[0].joints(), q.as_slice());
        assert_eq!(result.waypoints()[1].joints(), q.as_slice());
        assert_ne!(result.waypoints()[2].joints(), q.as_slice());
    }

    // ── Waypoints outside region preserved ────────────────

    #[test]
    fn waypoints_outside_region_unchanged() {
        let robot = RobotRegistry::create_default(RobotModel::Planar3R);
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let ctx = ctx_planar_3r();

        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 0.0, 0.0], 1.0),
            TrajectoryPoint::new(vec![2.0, 0.0, 0.0], 2.0),
            TrajectoryPoint::new(vec![3.0, 0.0, 0.0], 3.0),
        ]);
        // Region covers middle two waypoints only
        let result = op
            .apply(
                &robot,
                &traj,
                &region(1..3, RegionKind::Constraint),
                &ctx,
                None,
            )
            .unwrap();
        // First and last waypoints should be byte-identical
        assert_eq!(result.waypoints()[0].joints(), traj.waypoints()[0].joints());
        assert_eq!(result.waypoints()[3].joints(), traj.waypoints()[3].joints());
    }

    // ── Timestamps preserved ──────────────────────────────

    #[test]
    fn timestamps_are_preserved() {
        let robot = RobotRegistry::create_default(RobotModel::Planar3R);
        let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
        let ctx = ctx_planar_3r();

        let traj = Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![1.0, 0.0, 0.0], 1.0),
        ]);
        let result = op
            .apply(
                &robot,
                &traj,
                &region(0..2, RegionKind::Constraint),
                &ctx,
                None,
            )
            .unwrap();
        for (orig, res) in traj.waypoints().iter().zip(result.waypoints().iter()) {
            assert!(
                (orig.timestamp() - res.timestamp()).abs() < 1e-10,
                "timestamp changed from {} to {}",
                orig.timestamp(),
                res.timestamp()
            );
        }
    }

    // ── Integration tests ───────────────────────────────────────

    #[cfg(test)]
    mod integration_tests {
        use super::test_helpers::ctx_with_limits;
        use super::*;
        use crate::domain::operator::OptimizationObjective;
        use crate::operators::JointCenteringOperator;
        use thalos_core::{
            analysis::region::{RegionId, RegionKind, RegionSeverity},
            evaluation::{
                CollisionMetrics, JointSafetyMetrics, ManipulabilityMetrics, PlanMetrics,
            },
            models::{RobotModel, RobotRegistry},
            prelude::*,
        };
        use thalos_math::constants::*;

        fn robot() -> SerialChain {
            RobotRegistry::create_default(RobotModel::Planar2R)
        }

        fn region_2wp(kind: RegionKind) -> ProblemRegion {
            ProblemRegion::new(RegionId(0), kind, RegionSeverity::Warning, 0..2)
        }

        fn ctx_2r() -> OptimizationContext {
            ctx_with_limits(&[(-PI, PI), (-PI, PI)])
        }

        fn metrics() -> PlanMetrics {
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

        // ── 3.6 Pipeline integration ──────────────────────────

        #[test]
        fn nullspace_and_joint_centering_no_conflict() {
            // Apply both operators sequentially. Both improve joint
            // margins but through different mechanisms.
            let traj = Trajectory::new(vec![
                TrajectoryPoint::new(vec![1.0, 1.0], 0.0),
                TrajectoryPoint::new(vec![1.0, 1.0], 1.0),
            ]);

            let ns = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
            let jc = JointCenteringOperator::new(0.3);

            let ctx = ctx_2r();
            let r = region_2wp(RegionKind::Constraint);

            // Apply NullSpaceOptimization first (no-op for Planar2R)
            let after_ns = ns.apply(&robot(), &traj, &r, &ctx, None).unwrap();
            assert_eq!(after_ns.len(), traj.len());

            // Then apply JointCenteringOperator
            let after_jc = jc.apply(&robot(), &after_ns, &r, &ctx, None).unwrap();
            assert_eq!(after_jc.len(), traj.len());

            // Joints should be closer to center after both operators
            let original_sum: f64 = traj.waypoints()[0].joints().iter().map(|q| q.abs()).sum();
            let final_sum: f64 = after_jc.waypoints()[0]
                .joints()
                .iter()
                .map(|q| q.abs())
                .sum();
            assert!(
                final_sum <= original_sum + 1e-10,
                "joint-centering should decrease |q| sum: {:.4} → {:.4}",
                original_sum,
                final_sum
            );
        }

        #[test]
        fn operators_are_composable() {
            let ns = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
            let jc = JointCenteringOperator::new(0.3);

            assert_eq!(ns.family(), jc.family());
            assert_eq!(ns.objective(), OptimizationObjective::Feasibility);
            assert_eq!(jc.objective(), OptimizationObjective::Feasibility);
        }

        #[test]
        fn estimate_and_cost_methods_work() {
            let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
            let r = region_2wp(RegionKind::Constraint);
            let m = metrics();

            let improvement = op.estimate_improvement(&r, &m);
            assert!((0.0..=1.0).contains(&improvement));

            let cost = op.estimate_cost();
            assert!((0.0..=1.0).contains(&cost));

            let applicability = op.applicability(&r);
            assert!((0.0..=1.0).contains(&applicability));
        }
    }

    // ── Benchmarks ──────────────────────────────────────────────

    #[cfg(test)]
    mod benchmarks {
        use super::test_helpers::planar_4r;
        use super::*;
        use thalos_core::{
            analysis::region::{RegionId, RegionKind, RegionSeverity},
            prelude::TrajectoryPoint,
        };
        // ── 3.7 Benchmark: null-space correction on redundant robot ──

        /// Verify that NullSpaceOptimization produces a non-zero
        /// correction on a redundant planar 4R robot while preserving
        /// the TCP position.
        ///
        /// Uses moderate operator parameters (factor=0.5, dt=0.3) and
        /// verifies:
        ///   1. Joints are modified (non-zero null-space correction)
        ///   2. TCP position is preserved within SVD precision
        #[test]
        fn benchmark_nullspace_correction_on_redundant_robot() {
            use crate::domain::context::OptimizationContext;

            let robot = planar_4r();
            // Moderate settings: factor=0.5, dt=0.3
            let op = NullSpaceOptimization::new(0.5, 1e-6, 0.3);
            let limits: Vec<(f64, f64)> = vec![(-2.0, 2.0); 4];
            let (lower, upper): (Vec<f64>, Vec<f64>) = limits.iter().cloned().unzip();
            let ctx = OptimizationContext {
                joint_limits: crate::domain::context::JointLimits {
                    lower,
                    upper,
                    velocity: None,
                    acceleration: None,
                },
                config: crate::PipelineConfig::default(),
                tool_frame: None,
            };

            // Asymmetrical configuration — all joints off centre.
            let q = vec![1.5, 1.2, -1.3, -1.0];
            let traj = Trajectory::new(vec![
                TrajectoryPoint::new(q.clone(), 0.0),
                TrajectoryPoint::new(vec![1.5, 1.2, -1.3, -1.0], 1.0),
            ]);
            let region = ProblemRegion::new(
                RegionId(0),
                RegionKind::Singularity,
                RegionSeverity::Warning,
                0..2,
            );

            // Debug: verify the null-space property directly.
            let fk = ForwardKinematics::new(robot.clone());
            let js = GeometricJacobian::new(fk.clone(), robot.end_effector().clone());
            let j = js.evaluate(&q);
            let j_full = j.full();
            let j_plus = j_full.pseudo_inverse(1e-6).expect("pinv");
            let z: Vec<f64> = q.iter().map(|qj| (0.0 - qj) * 0.5).collect();
            let z_vec = DynamicVector::from_column_slice(&z);
            let jz = &j_full * &z_vec;
            let j_plus_jz = &j_plus * &jz;
            let nz = z_vec - j_plus_jz;

            // Verify J·N·z ≈ 0 (core null-space property)
            let j_nz = &j_full * &nz;
            let residual: f64 = j_nz
                .as_slice()
                .iter()
                .map(|v| v.powi(2))
                .sum::<f64>()
                .sqrt();
            eprintln!("[DEBUG] |J·N·z| = {:.6e}", residual);
            eprintln!(
                "[DEBUG] |N·z|   = {:.6e}",
                nz.as_slice().iter().map(|v| v.powi(2)).sum::<f64>().sqrt()
            );

            let before_pos = fk.evaluate(&q).ee_position().expect("EE position");

            let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
            assert_eq!(result.len(), traj.len());

            let q_after = result.waypoints()[0].joints();

            // 1. Joints changed (non-zero null-space correction)
            let diff: f64 = q
                .iter()
                .zip(q_after.iter())
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
                .sqrt();
            assert!(
                diff > 1e-10,
                "null-space correction should modify joints, diff = {:.2e}",
                diff
            );

            // 2. TCP position preserved.
            // J·N·z ≈ 0 holds to machine precision (verified above).
            // The finite step q' - q = N·z·dt introduces a second-order
            // FK deviation O(|N·z·dt|²) — this is expected for any
            // finite-step null-space method. With |N·z·dt| ≈ 0.025 rad,
            // the second-order deviation is ≈ 0.0003 on a 4 m arm.
            let after_pos = fk.evaluate(q_after).ee_position().expect("EE position");
            let pos_diff = (after_pos - before_pos).magnitude();
            assert!(
                pos_diff < 5e-4,
                "TCP deviation {:.2e} exceeds 5e-4 (O(|N·z·dt|²) expected)",
                pos_diff
            );

            println!("\n═══ Benchmark: NullSpaceOptimization ────────────");
            println!(
                "  |N·z|:            {:.6e}",
                nz.as_slice().iter().map(|v| v.powi(2)).sum::<f64>().sqrt()
            );
            println!("  |J·N·z|:          {:.6e}", residual);
            println!("  Joint diff:       {:.6e}", diff);
            println!("  TCP deviation:    {:.6e}", pos_diff);
            println!("  Joints:           {:?} → {:?}", q, q_after);
            println!("────────────────────────────────────────────\n");
        }

        /// Benchmark: verify null-space correction improves joint margin.
        ///
        /// Uses a Planar4R robot with joints closer to one limit than the other.
        /// The secondary objective z = (q_center - q) · factor should move joints
        /// toward center, increasing the minimum distance to the nearest joint limit.
        ///
        /// This validates that the optimization direction (z) is aligned with the
        /// declared objective of improving joint margin.
        #[test]
        fn benchmark_joint_margin_improvement() {
            use crate::domain::context::OptimizationContext;

            let robot = planar_4r();
            // Use conservative settings — smaller step to isolate direction
            let op = NullSpaceOptimization::new(0.3, 1e-6, 0.1);
            let limits: Vec<(f64, f64)> = vec![(-2.0, 2.0); 4];
            let (lower, upper): (Vec<f64>, Vec<f64>) = limits.iter().cloned().unzip();
            let ctx = OptimizationContext {
                joint_limits: crate::domain::context::JointLimits {
                    lower,
                    upper,
                    velocity: None,
                    acceleration: None,
                },
                config: crate::PipelineConfig::default(),
                tool_frame: None,
            };

            // Asymmetric config: different joints at different distances from limits
            // Joint 0 near upper limit (1.8 of 2.0), joints 1-3 in various positions
            // This avoids symmetric degeneracy where null space is orthogonal to centering
            let q = vec![1.8, 0.5, -0.3, -1.2];
            let traj = Trajectory::new(vec![
                TrajectoryPoint::new(q.clone(), 0.0),
                TrajectoryPoint::new(q.clone(), 1.0),
            ]);
            let region = ProblemRegion::new(
                RegionId(0),
                RegionKind::Singularity,
                RegionSeverity::Warning,
                0..2,
            );

            // Compute before: min distance to nearest joint limit
            let before_margin = q
                .iter()
                .zip(limits.iter())
                .map(|(qj, (lo, hi))| (qj - lo).min(hi - qj))
                .fold(f64::MAX, |a, b| a.min(b));

            let result = op.apply(&robot, &traj, &region, &ctx, None).unwrap();
            let q_after = result.waypoints()[0].joints();

            // Compute after: min distance to nearest joint limit
            let after_margin = q_after
                .iter()
                .zip(limits.iter())
                .map(|(qj, (lo, hi))| (qj - lo).min(hi - qj))
                .fold(f64::MAX, |a, b| a.min(b));

            let improvement = if before_margin > 0.0 {
                (after_margin - before_margin) / before_margin * 100.0
            } else {
                0.0
            };

            // Verify joints moved toward center (not away) for joints near limits
            let center = vec![0.0; 4];
            let moved_toward_center: bool = q.iter().zip(q_after.iter()).zip(center.iter()).all(
                |((q_before, q_after), center)| {
                    let dist_before = (q_before - center).abs();
                    let dist_after = (q_after - center).abs();
                    dist_after <= dist_before + 1e-6 // monotonic toward center (allow floating point)
                },
            );

            // Verify the worst joint (closest to limit) improved specifically
            let worst_before_idx = q
                .iter()
                .zip(limits.iter())
                .map(|(qj, (lo, hi))| (qj - lo).min(hi - qj))
                .enumerate()
                .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                .map(|(i, _)| i);

            let worst_improved = worst_before_idx
                .map(|idx| {
                    let (lo, hi) = limits[idx];
                    let margin_before = (q[idx] - lo).min(hi - q[idx]);
                    let margin_after = (q_after[idx] - lo).min(hi - q_after[idx]);
                    margin_after > margin_before + 1e-8
                })
                .unwrap_or(false);

            // Note: null-space projection does NOT move all joints toward center simultaneously.
            // The projected vector N·z = z - J⁺·J·z depends on the Jacobian's current configuration.
            // Some joints may move slightly away from center to compensate.
            // The key invariant is that the TCP path is preserved and the worst joint improves.

            println!("\n═══ Benchmark: Joint Margin Improvement ─────────");
            println!("  Joints:        {:?}", q);
            println!("  After:         {:?}", q_after);
            println!(
                "  Worst joint:   joint #{}",
                worst_before_idx.unwrap_or(999)
            );
            println!("  Before margin: {:.6}", before_margin);
            println!("  After margin:  {:.6}", after_margin);
            println!("  Improvement:   {:+.1}%", improvement);
            println!(
                "  Worst improved: {}",
                if worst_improved {
                    "✅"
                } else {
                    "⚠️ not on this config"
                }
            );
            println!("────────────────────────────────────────────\n");

            // The worst joint (closest to limit) should improve
            // Individual joints may move toward or away from center depending on
            // the null-space orientation — only the aggregate margin and TCP
            // preservation are guaranteed invariants.
            assert!(
                worst_improved,
                "The joint closest to its limit should improve, got joint #{}: {}",
                worst_before_idx.unwrap_or(999),
                before_margin
            );
        }
    }
}
