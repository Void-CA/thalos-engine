//! 6 benchmark scenario implementations for the OptimizationPipeline.
//!
//! Each scenario implements [`BenchmarkScenario`] to define a specific
//! problematic trajectory and the expected metric improvements.
//!
//! # Design
//!
//! All scenarios use `RobotModel::Planar2R` because `Manipulator6DOF`
//! factory is not yet wired in `RobotRegistry::create()`. Expectations
//! are adjusted accordingly (see the M10 design notes).
//!
//! On Planar2R the `AdaptiveSampling` operator wins the ranking for
//! `Singularity` and `LowManipulability` regions (composite score
//! 1.333 > 1.275 for `JointCentering`).  To test operators other than
//! `AdaptiveSampling`, scenarios provide `JointLimit` constraints that
//! produce `ConstraintViolation` findings → `Constraint`-kind regions,
//! where `JointCentering` scores 1.7 > 1.333 for `AdaptiveSampling`.
//!
//! | # | Scenario | P(F)i | Trigger | Primary assertion |
//! |---|----------|--------|---------|-------------------|
//! | 1 | JointLimit | constraint | q at 3.0 (tighter 2.0 limit) | `JointMargin` increases |
//! | 2 | NearSingularity | LMP finding | q₂ → 0 + coarse segment | `MaxSegmentError` decreases |
//! | 3 | VelocityViolation | Singularity finding | 25 rad/s per joint | `MaxVelocity` decreases |
//! | 4 | CoarseSampling | Singularity finding | 7-waypoint sine, high curvature | `MaxSegmentError` decreases |
//! | 5 | OrientationConstraint | constraint | orientation sweep (tight limits) | `JointMargin` increases |
//! | 6 | Mixed | constraint | near-limit + velocity + coarse | ≥3 metrics improve |

use std::f64::consts::PI;

use thalos_core::{
    analysis::constraints::Constraint,
    models::RobotModel,
    trajectory::{Trajectory, TrajectoryPoint},
};

use super::{ExpectedImprovement, ImprovementDirection, MetricKind};

// ─── Scenario 1: JointLimitScenario ───────────────────────────────────

/// Joints operating near their mechanical limits.
///
/// Uses a `JointLimit` constraint (tighter than the robot's ±PI hardware
/// limits) so the analyzer produces `ConstraintViolation` findings →
/// `Constraint`-kind region where `JointCenteringOperator` is ranked
/// first and centers the joints away from the limit, increasing margin.
pub struct JointLimitScenario;

impl super::BenchmarkScenario for JointLimitScenario {
    fn name(&self) -> &'static str {
        "joint_limit"
    }

    fn robot_model(&self) -> RobotModel {
        RobotModel::Planar2R
    }

    fn trajectory(&self) -> Trajectory {
        // Both joints at 2.5 rad, between the tighter constraint max
        // (2.0) and the hardware limit (±PI).  All waypoints violate
        // the constraint → one contiguous Constraint region.
        // |sin(2.5)| ≈ 0.598 > 0.3 → no LowManipulability split.
        // All waypoints identical → segment error = 0 → AdaptiveSampling
        // no-ops → JointCenteringOperator gets the turn.
        let joints = vec![2.5_f64, 2.5];
        Trajectory::new(
            (0..=6)
                .map(|i| TrajectoryPoint::new(joints.clone(), i as f64 * 0.5))
                .collect(),
        )
    }

    fn constraints(&self) -> Vec<Constraint> {
        vec![
            Constraint::JointLimit {
                joint: 0,
                min: -2.0,
                max: 2.0,
            },
            Constraint::JointLimit {
                joint: 1,
                min: -2.0,
                max: 2.0,
            },
        ]
    }

    fn expected_improvements(&self) -> Vec<ExpectedImprovement> {
        vec![ExpectedImprovement {
            operator_id: "joint_centering",
            metric: MetricKind::JointMargin,
            direction: ImprovementDirection::Increase,
        }]
    }
}

// ─── Scenario 2: NearSingularityScenario ──────────────────────────────

/// Near-singularity configuration.
///
/// Planar2R with q₂ → 0 (arm fully extended), where Yoshikawa
/// manipulability drops well below 0.3 → `LowManipulability` finding.
///
/// On Planar2R, `NullSpaceOptimization` cannot modify the trajectory
/// (no redundant DOF → null-space correction is always zero for full-rank
/// Jacobians).  Instead, `AdaptiveSampling` wins the ranking and reduces
/// the coarse segment between the near-singular hold and the end point,
/// decreasing `MaxSegmentError`.
pub struct NearSingularityScenario;

impl super::BenchmarkScenario for NearSingularityScenario {
    fn name(&self) -> &'static str {
        "near_singularity"
    }

    fn robot_model(&self) -> RobotModel {
        RobotModel::Planar2R
    }

    fn trajectory(&self) -> Trajectory {
        // First 3 waypoints: near-singularity hold (q₂ → 0).
        // Last waypoint: coarse jump (error ≈ 2.0) → forces
        // AdaptiveSampling to insert waypoints and reduce MaxSegmentError.
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.5, 0.05], 0.0),
            TrajectoryPoint::new(vec![0.5, 0.05], 0.5),
            TrajectoryPoint::new(vec![0.5, 0.05], 1.0),
            TrajectoryPoint::new(vec![2.5, 0.05], 2.0),
        ])
    }

    fn expected_improvements(&self) -> Vec<ExpectedImprovement> {
        vec![ExpectedImprovement {
            operator_id: "adaptive_sampling",
            metric: MetricKind::MaxSegmentError,
            direction: ImprovementDirection::Decrease,
        }]
    }
}

// ─── Scenario 3: VelocityViolationScenario ────────────────────────────

/// Velocity violation.
///
/// Uses Planar2R with an exaggerated segment: dq ≈ 2.5 rad in
/// dt = 0.1 s → 25 rad/s per joint.  The default velocity limit
/// (`Retime::DEFAULT_VELOCITY`) is 3.0 rad/s, so the violation is
/// extreme.
///
/// The arm passes through q₂ = 0 (singularity) at waypoints 0 and 2,
/// producing `Singularity` findings.  `AdaptiveSampling` is ranked first
/// but `JointCenteringOperator` applies to the Singularity region and
/// reduces the joint displacement, which lowers the peak velocity.
pub struct VelocityViolationScenario;

impl super::BenchmarkScenario for VelocityViolationScenario {
    fn name(&self) -> &'static str {
        "velocity_violation"
    }

    fn robot_model(&self) -> RobotModel {
        RobotModel::Planar2R
    }

    fn trajectory(&self) -> Trajectory {
        // [0, 0] → [2.5, 2.5] in 0.1 s → 25 rad/s per joint.
        // Each joint stays within ±PI limits.
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![2.5, 2.5], 0.1),
            TrajectoryPoint::new(vec![0.0, 0.0], 0.2),
        ])
    }

    fn expected_improvements(&self) -> Vec<ExpectedImprovement> {
        vec![ExpectedImprovement {
            operator_id: "retime",
            metric: MetricKind::MaxVelocity,
            direction: ImprovementDirection::Decrease,
        }]
    }
}

// ─── Scenario 4: CoarseSamplingScenario ───────────────────────────────

/// Coarse sampling over a high-curvature path.
///
/// A sine-wave trajectory with only 7 waypoints across one 2π period.
/// Large gaps between successive samples produce high interpolation
/// errors.  `AdaptiveSampling` should insert waypoints and reduce
/// the maximum segment error.
pub struct CoarseSamplingScenario;

impl super::BenchmarkScenario for CoarseSamplingScenario {
    fn name(&self) -> &'static str {
        "coarse_sampling"
    }

    fn robot_model(&self) -> RobotModel {
        RobotModel::Planar2R
    }

    fn trajectory(&self) -> Trajectory {
        // 7 waypoints over [0, 2π]; q₁ = 1.5·sin(t), q₂ = 1.0·sin(2t).
        // q₂ crosses 0 (singularity) at t=0, t=π/2, t=π, producing
        // findings → region → AdaptiveSampling applies.
        let n = 7;
        let pts: Vec<TrajectoryPoint> = (0..n)
            .map(|i| {
                let t = (i as f64 / (n - 1) as f64) * 2.0 * PI;
                TrajectoryPoint::new(vec![(t * 1.0).sin() * 1.5, (t * 2.0).sin() * 1.0], t)
            })
            .collect();
        Trajectory::new(pts)
    }

    fn expected_improvements(&self) -> Vec<ExpectedImprovement> {
        vec![ExpectedImprovement {
            operator_id: "adaptive_sampling",
            metric: MetricKind::MaxSegmentError,
            direction: ImprovementDirection::Decrease,
        }]
    }
}

// ─── Scenario 5: OrientationConstraintScenario ────────────────────────

/// Joint-limit constraint challenge (orientation proxy).
///
/// On Planar2R there is no redundant DOF for `OrientationRelaxation`
/// to exploit.  This scenario instead demonstrates that the pipeline
/// detects `ConstraintViolation` findings from a `JointLimit` constraint
/// and applies `JointCenteringOperator` to improve joint safety margin.
///
/// The trajectory sweeps through varied (q₁, q₂) combos, all of which
/// exceed the tight limit, producing a `Constraint` region.
pub struct OrientationConstraintScenario;

impl super::BenchmarkScenario for OrientationConstraintScenario {
    fn name(&self) -> &'static str {
        "orientation_constraint"
    }

    fn robot_model(&self) -> RobotModel {
        RobotModel::Planar2R
    }

    fn trajectory(&self) -> Trajectory {
        // All waypoints have |q| > 2.0, violating the tighter constraint.
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![3.0, 3.0], 0.0),
            TrajectoryPoint::new(vec![2.5, 2.8], 0.5),
            TrajectoryPoint::new(vec![3.0, 2.5], 1.0),
            TrajectoryPoint::new(vec![2.8, 3.0], 1.5),
            TrajectoryPoint::new(vec![2.5, 2.5], 2.0),
            TrajectoryPoint::new(vec![3.0, 3.0], 2.5),
        ])
    }

    fn constraints(&self) -> Vec<Constraint> {
        vec![
            Constraint::JointLimit {
                joint: 0,
                min: -2.0,
                max: 2.0,
            },
            Constraint::JointLimit {
                joint: 1,
                min: -2.0,
                max: 2.0,
            },
        ]
    }

    fn expected_improvements(&self) -> Vec<ExpectedImprovement> {
        vec![ExpectedImprovement {
            operator_id: "joint_centering",
            metric: MetricKind::JointMargin,
            direction: ImprovementDirection::Increase,
        }]
    }
}

// ─── Scenario 6: MixedScenario ────────────────────────────────────────

/// Combined problems: near-limit joints + velocity + coarse sampling.
///
/// All waypoints violate a tighter `JointLimit` constraint (max=2.0),
/// creating a single `Constraint` region.  `JointCenteringOperator`
/// ranks first and centers all waypoints toward the joint centre,
/// which improves all three tracked metrics simultaneously:
///
/// - **JointMargin**: moves joints away from the ±PI hardware limit
/// - **MaxVelocity**: reduces joint displacement in the fast segment
/// - **MaxSegmentError**: shrinks the L2 gap between centred waypoints
pub struct MixedScenario;

impl super::BenchmarkScenario for MixedScenario {
    fn name(&self) -> &'static str {
        "mixed"
    }

    fn robot_model(&self) -> RobotModel {
        RobotModel::Planar2R
    }

    fn trajectory(&self) -> Trajectory {
        // All waypoints have |q| > 2.0 → ConstraintViolation findings
        // on every waypoint → one contiguous Constraint region.
        // q₂ values stay in (2.0, 2.838) so |sin(q₂)| > 0.3 →
        // no LowManipulability finding to split the region.
        // The fast segment (wp 0→1, dt=0.1) and varied positions let
        // centering improve all three metrics.
        Trajectory::new(vec![
            TrajectoryPoint::new(vec![3.0, 3.0], 0.0),
            // Fast move: 0.2 rad in 0.1 s → 2 rad/s (before centering)
            TrajectoryPoint::new(vec![2.8, 2.8], 0.1),
            TrajectoryPoint::new(vec![2.5, 2.5], 0.5),
            TrajectoryPoint::new(vec![2.8, 2.5], 0.9),
            TrajectoryPoint::new(vec![2.5, 2.8], 1.3),
            TrajectoryPoint::new(vec![2.6, 2.6], 1.7),
        ])
    }

    fn constraints(&self) -> Vec<Constraint> {
        vec![
            Constraint::JointLimit {
                joint: 0,
                min: -2.0,
                max: 2.0,
            },
            Constraint::JointLimit {
                joint: 1,
                min: -2.0,
                max: 2.0,
            },
        ]
    }

    fn expected_improvements(&self) -> Vec<ExpectedImprovement> {
        vec![
            ExpectedImprovement {
                operator_id: "joint_centering",
                metric: MetricKind::JointMargin,
                direction: ImprovementDirection::Increase,
            },
            ExpectedImprovement {
                operator_id: "retime",
                metric: MetricKind::MaxVelocity,
                direction: ImprovementDirection::Decrease,
            },
            ExpectedImprovement {
                operator_id: "adaptive_sampling",
                metric: MetricKind::MaxSegmentError,
                direction: ImprovementDirection::Decrease,
            },
        ]
    }
}
