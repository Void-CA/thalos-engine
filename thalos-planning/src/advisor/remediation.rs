//! Causal singularity remediation (design ADR-5 REVISION 3, M4).
//!
//! This module owns the joint-space singularity remediation operator and the
//! physical motion envelope.
//!
//! - [`SingularityResolveMaterializer`]: interior singular regions — the MoveJ
//!   path CROSSES the full extension mid-segment. The operator re-solves IK
//!   from the segment-start joints toward the SAME cartesian position, so
//!   `DampedLeastSquaresSolver` converges to the same-side elbow posture that
//!   reaches the IDENTICAL cartesian point without crossing the extension.
//!   This is a root-cause fix: a clean 1:1 MoveJ replacement, not a sampling
//!   trick or a blind perturbation.
//! - [`PhysicalEnvelope`]: the per-robot actuation ceiling the firmware
//!   safety-envelope commit consumes (P1 physical-limits contract).
//!
//! The tests in this module are the causal contract: the re-solved trajectory
//! MUST reach the same cartesian point with the elbow on the same side as the
//! segment start, on the REAL robot models.

use thalos_core::{
    analysis::action::ActionKind,
    kinematics::{
        forward::ForwardKinematics,
        inverse::{IKGoal, IKSolver},
    },
    motion::segment::MotionSegment,
    robot::joint::JointKind,
    robot::serial_chain::SerialChain,
};

use crate::feedback::materializer::{MaterializationError, ProposalMaterializer};
use crate::feedback::operator::ActionProposal;

/// Physical motion envelope: the per-robot actuation ceiling the departure
/// operator may NOT exceed (P1 physical-limits contract, 4R findings R1-1 /
/// R4-2).
///
/// ## Limit source
/// The catalog specs declare POSITION limits only — every toy model builds
/// its joints with `JointLimits::new(min, max)` leaving `velocity` and
/// `effort` as `None` (verified `models/*/spec.rs`, M1–M4). There is therefore
/// NO per-robot velocity/acceleration data on the chain. P1 defines a per-robot
/// SAFETY CEILING table keyed by the chain's actuated-joint signature
/// (`dof_count` + joint-kind sequence — the only robot identity the planner
/// holds; `SerialChain` carries no `RobotModel`), NOT a global constant.
///
/// The ceilings are sized to:
/// 1. cover the documented M3 departures (measured on the real Jacobians:
///    Planar3R ~22.5 rad/s / ~448 rad/s², Scara ~17.4 rad/s / ~269 rad/s²)
///    with headroom, so the permanent usability scenarios (24→1, 17→0) keep
///    passing; and
/// 2. bound extreme departures (the 4R finding: a straight-extension
///    departure needs ~61 rad/s / ~1667 rad/s² — unbounded on the old code).
///
/// Unknown chains (URDF-loaded robots, future models) fall back to the
/// conservative [`GENERIC_ENVELOPE`]. The documented extension point for real
/// robots is `JointLimits.velocity` / `JointLimits.effort` on the chain's
/// joints — when a spec or URDF declares them, this table is the override
/// site.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhysicalEnvelope {
    /// Maximum joint velocity (rad/s) the departure operator may request.
    pub max_velocity: f64,
    /// Maximum joint acceleration (rad/s²) the departure operator may request.
    pub max_acceleration: f64,
}

/// SCARA (4 dof: R-R-P-R) safety ceiling.
pub const SCARA_ENVELOPE: PhysicalEnvelope = PhysicalEnvelope {
    max_velocity: 25.0,
    max_acceleration: 600.0,
};

/// Planar3R / Manipulator3DOF (3 dof, all revolute) safety ceiling.
pub const PLANAR_3R_ENVELOPE: PhysicalEnvelope = PhysicalEnvelope {
    max_velocity: 30.0,
    max_acceleration: 900.0,
};

/// Planar2R (2 dof, both revolute) safety ceiling.
pub const PLANAR_2R_ENVELOPE: PhysicalEnvelope = PhysicalEnvelope {
    max_velocity: 20.0,
    max_acceleration: 500.0,
};

/// SingleRevolute (1 dof) safety ceiling.
pub const SINGLE_REVOLUTE_ENVELOPE: PhysicalEnvelope = PhysicalEnvelope {
    max_velocity: 15.0,
    max_acceleration: 400.0,
};

/// CylindricalRPP (R-P-P) safety ceiling.
pub const CYLINDRICAL_RPP_ENVELOPE: PhysicalEnvelope = PhysicalEnvelope {
    max_velocity: 20.0,
    max_acceleration: 500.0,
};

/// SphericalPolarRRP (R-R-P) safety ceiling.
pub const SPHERICAL_POLAR_RRP_ENVELOPE: PhysicalEnvelope = PhysicalEnvelope {
    max_velocity: 20.0,
    max_acceleration: 500.0,
};

/// Manipulator6DOF (6 dof, all revolute) safety ceiling.
pub const MANIPULATOR_6DOF_ENVELOPE: PhysicalEnvelope = PhysicalEnvelope {
    max_velocity: 25.0,
    max_acceleration: 600.0,
};

/// Generic ceiling for unknown chains (URDF-loaded robots, future models):
/// the most conservative entry in the table.
pub const GENERIC_ENVELOPE: PhysicalEnvelope = PhysicalEnvelope {
    max_velocity: 15.0,
    max_acceleration: 400.0,
};

impl PhysicalEnvelope {
    /// The envelope for the robot the chain describes (P1 per-robot limit
    /// source): keyed by the actuated-joint signature (`dof_count` + joint
    /// kinds in segment order).
    pub fn for_chain(chain: &SerialChain) -> Self {
        let kinds: Vec<JointKind> = chain
            .segments
            .iter()
            .filter(|s| s.joint.dof() > 0)
            .map(|s| s.joint.kind())
            .collect();
        Self::for_signature(chain.dof_count(), &kinds)
    }

    /// Envelope for a `(dof_count, joint-kind sequence)` signature. Unknown
    /// signatures fall back to [`GENERIC_ENVELOPE`].
    pub fn for_signature(dof: usize, kinds: &[JointKind]) -> Self {
        use JointKind::{Prismatic, Revolute};
        match (dof, kinds) {
            (4, [Revolute, Revolute, Prismatic, Revolute]) => SCARA_ENVELOPE,
            (6, [Revolute, Revolute, Revolute, Revolute, Revolute, Revolute]) => {
                MANIPULATOR_6DOF_ENVELOPE
            }
            (3, [Revolute, Revolute, Revolute]) => PLANAR_3R_ENVELOPE,
            (3, [Revolute, Prismatic, Prismatic]) => CYLINDRICAL_RPP_ENVELOPE,
            (3, [Revolute, Revolute, Prismatic]) => SPHERICAL_POLAR_RRP_ENVELOPE,
            (2, [Revolute, Revolute]) => PLANAR_2R_ENVELOPE,
            (1, [Revolute]) => SINGLE_REVOLUTE_ENVELOPE,
            _ => GENERIC_ENVELOPE,
        }
    }
}

/// Interior singular regions: re-solve IK from the segment-start joints toward
/// the SAME cartesian position (root-cause fix, design ADR-5 REVISION 3).
///
/// A MoveJ whose target crosses the full extension (the elbow joint passes
/// through 0 mid-segment) is repaired by re-solving inverse kinematics from
/// the PREVIOUS segment's joints toward the target's cartesian position.
/// `DampedLeastSquaresSolver` converges to the SAME-SIDE elbow posture, which
/// reaches the IDENTICAL cartesian point but does NOT cross the extension.
///
/// This is a clean 1:1 MoveJ replacement (same origin and motion limits, only
/// the target joints change) — not a 2-segment split and not a via-point.
/// MoveL targets carry no joint configuration to re-solve — a documented gap
/// surfaced honestly as [`MaterializationError::UnsupportedSegment`].
pub struct SingularityResolveMaterializer<'a> {
    ik_solver: &'a dyn IKSolver,
    segment_start_joints: &'a [f64],
}

impl<'a> SingularityResolveMaterializer<'a> {
    /// Creates the operator with the solver and the segment-start joints the
    /// IK is re-solved from (the deterministic context the compiler uses).
    pub fn new(ik_solver: &'a dyn IKSolver, segment_start_joints: &'a [f64]) -> Self {
        Self {
            ik_solver,
            segment_start_joints,
        }
    }
}

impl ProposalMaterializer for SingularityResolveMaterializer<'_> {
    fn name(&self) -> &'static str {
        "singularity_resolve_materializer"
    }

    fn materialize(
        &self,
        proposal: &ActionProposal,
        target: &MotionSegment,
    ) -> Result<Vec<MotionSegment>, MaterializationError> {
        if proposal.kind != ActionKind::Singularity {
            return Err(MaterializationError::UnsupportedProposal {
                kind: proposal.kind,
            });
        }
        let MotionSegment::MoveJ {
            origin,
            target: joints,
            max_velocity,
            max_acceleration,
        } = target
        else {
            return Err(MaterializationError::UnsupportedSegment);
        };

        let robot = self
            .ik_solver
            .robot()
            .ok_or(MaterializationError::UnsupportedProposal {
                kind: proposal.kind,
            })?;

        let fk = ForwardKinematics::new(robot.clone());
        let pos = fk
            .evaluate(joints)
            .ee_position()
            .ok_or(MaterializationError::IkFailure)?;

        let result = self
            .ik_solver
            .solve(self.segment_start_joints, IKGoal::Position(pos))
            .map_err(|_| MaterializationError::IkFailure)?;
        if !result.status.is_converged() {
            return Err(MaterializationError::IkFailure);
        }

        Ok(vec![MotionSegment::MoveJ {
            origin: origin.clone(),
            target: result.q,
            max_velocity: *max_velocity,
            max_acceleration: *max_acceleration,
        }])
    }
}

#[cfg(test)]
mod tests {
    use thalos_core::{
        analysis::action::{ActionImpact, ActionKind, ActionPriority},
        analysis::observation::ObservationId,
        ids::OperationId,
        kinematics::{
            forward::ForwardKinematics,
            inverse::DampedLeastSquaresSolver,
        },
        models::{RobotModel, RobotRegistry},
        motion::segment::MotionSegment,
        robot::serial_chain::SerialChain,
        spatial::frame::FrameId,
    };

    use crate::feedback::operator::ActionProposal;
    use crate::feedback::materializer::{MaterializationError, ProposalMaterializer};

    use super::{PhysicalEnvelope, SingularityResolveMaterializer};

    fn chain(model: RobotModel) -> SerialChain {
        RobotRegistry::create_default(model)
    }

    fn real_solver(chain: &SerialChain) -> DampedLeastSquaresSolver {
        let fk = ForwardKinematics::new(chain.clone());
        DampedLeastSquaresSolver::new(fk, *chain.end_effector(), 500, 1e-6, 0.1)
    }

    fn movej(target: Vec<f64>) -> MotionSegment {
        MotionSegment::MoveJ {
            origin: OperationId("op-j".to_string()),
            target,
            max_velocity: None,
            max_acceleration: None,
        }
    }

    fn move_l() -> MotionSegment {
        MotionSegment::MoveL {
            origin: OperationId("op-l".to_string()),
            frame: FrameId::World,
            target_pose: thalos_core::spatial::pose::Pose::new(
                FrameId::World,
                FrameId::Id(1),
                thalos_math::Transform3D::identity(),
            ),
            max_velocity: None,
        }
    }

    fn proposal(kind: ActionKind) -> ActionProposal {
        ActionProposal {
            kind,
            target_observation: ObservationId(1),
            priority: ActionPriority::High,
            impact: ActionImpact::High,
            parameters: std::collections::BTreeMap::new(),
        }
    }

    #[test]
    fn singularity_resolve_materializer_resolves_to_same_side_posture() {
        // Root-cause fix contract: for a MoveJ whose target crosses the full
        // extension (elbow +0.6), the materializer re-solves IK from the
        // bent home posture and returns a SINGLE MoveJ whose elbow joint
        // (index 1) is NEGATIVE (same side as home) and whose FK position
        // matches the bad target's FK position.
        let robot = chain(RobotModel::Scara);
        let home = vec![0.0, -1.31, -0.1, 0.0];
        let bad_target = vec![0.5, 0.6, -0.15, 0.0];

        let solver = real_solver(&robot);
        let materializer = SingularityResolveMaterializer::new(&solver, &home);

        let segments = materializer
            .materialize(
                &proposal(ActionKind::Singularity),
                &movej(bad_target.clone()),
            )
            .expect("resolve must materialize a MoveJ");

        assert_eq!(segments.len(), 1, "must be a clean 1:1 replacement");
        let MotionSegment::MoveJ { target, .. } = &segments[0] else {
            panic!("expected MoveJ, got {:?}", segments[0]);
        };

        assert!(
            target[1] < 0.0,
            "the re-solved elbow (index 1) must be NEGATIVE (same side as home), got {:?}",
            target
        );

        let fk = ForwardKinematics::new(robot.clone());
        let bad_pos = fk
            .evaluate(&bad_target)
            .ee_position()
            .expect("bad target FK position");
        let alt_pos = fk.evaluate(target).ee_position().expect("re-solved FK position");
        assert!(
            (bad_pos.x - alt_pos.x).abs() < 1e-3
                && (bad_pos.y - alt_pos.y).abs() < 1e-3
                && (bad_pos.z - alt_pos.z).abs() < 1e-3,
            "the re-solved posture must reach the SAME cartesian position, got bad={bad_pos:?} alt={alt_pos:?}"
        );
    }

    #[test]
    fn singularity_resolve_materializer_rejects_movel_and_wrong_kind() {
        let robot = chain(RobotModel::Scara);
        let solver = real_solver(&robot);
        let materializer = SingularityResolveMaterializer::new(&solver, &[0.0; 4]);

        match materializer.materialize(&proposal(ActionKind::Singularity), &move_l()) {
            Err(MaterializationError::UnsupportedSegment) => {}
            other => panic!("expected UnsupportedSegment, got {other:?}"),
        }
        match materializer.materialize(
            &proposal(ActionKind::Manipulability),
            &movej(vec![0.5, 0.5, 0.0, 0.0]),
        ) {
            Err(MaterializationError::UnsupportedProposal { .. }) => {}
            other => panic!("expected UnsupportedProposal, got {other:?}"),
        }
    }

    #[test]
    fn physical_envelope_is_per_robot_with_named_ceilings() {
        // P1 limit source: the envelope is per-robot (a SCARA and a Planar3R
        // have different actuation ceilings), keyed by the chain's
        // actuated-joint signature — the only robot identity the planner
        // holds (`SerialChain` carries no `RobotModel`). Unknown chains
        // (URDF-loaded robots) fall back to the conservative generic ceiling.
        use thalos_core::robot::joint::JointKind::{Prismatic, Revolute};

        assert_eq!(
            PhysicalEnvelope::for_signature(4, &[Revolute, Revolute, Prismatic, Revolute]),
            PhysicalEnvelope { max_velocity: 25.0, max_acceleration: 600.0 },
            "SCARA ceiling"
        );
        assert_eq!(
            PhysicalEnvelope::for_signature(3, &[Revolute, Revolute, Revolute]),
            PhysicalEnvelope { max_velocity: 30.0, max_acceleration: 900.0 },
            "Planar3R / Manipulator3DOF ceiling"
        );
        assert_eq!(
            PhysicalEnvelope::for_signature(2, &[Revolute, Revolute]),
            PhysicalEnvelope { max_velocity: 20.0, max_acceleration: 500.0 },
            "Planar2R ceiling"
        );
        assert_eq!(
            PhysicalEnvelope::for_signature(1, &[Revolute]),
            PhysicalEnvelope { max_velocity: 15.0, max_acceleration: 400.0 },
            "SingleRevolute ceiling"
        );
        assert_eq!(
            PhysicalEnvelope::for_signature(0, &[]),
            PhysicalEnvelope { max_velocity: 15.0, max_acceleration: 400.0 },
            "unknown signature must fall back to the conservative generic ceiling"
        );
        // The per-robot requirement: SCARA and Planar3R MUST differ.
        let scara = PhysicalEnvelope::for_chain(&chain(RobotModel::Scara));
        let p3r = PhysicalEnvelope::for_chain(&chain(RobotModel::Planar3R));
        assert_ne!(scara, p3r, "the envelope must be per-robot, never a global constant");
        assert!(scara.max_acceleration < p3r.max_acceleration);
    }
}
