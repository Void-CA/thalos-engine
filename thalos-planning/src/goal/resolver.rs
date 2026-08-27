use thalos_core::{
    kinematics::{
        forward::ForwardKinematics,
        inverse::{IKGoal, IKStatus},
        jacobian::{GeometricJacobian, JacobianSolver, ManipulabilityReport, SingularityReport},
    },
    spatial::pose::Pose,
};

use thalos_core::robot::state::RobotState;

use crate::{
    error::{IkFailureReason, PlanningError},
    motion::planner::PlanningContext,
};

use super::policy::PlanningPolicy;
use super::types::{
    GoalMetadata, JointGoal, MetricAction, ResolvedPoseGoal, ResolvedPositionGoal, ValidatedGoal,
};

#[derive(Debug, Clone)]
pub struct GoalResolverConfig {
    pub policy: PlanningPolicy,
    pub check_joint_limits: bool,
    pub strict_limits: bool,
}

impl Default for GoalResolverConfig {
    fn default() -> Self {
        Self {
            policy: PlanningPolicy::default(),
            check_joint_limits: true,
            strict_limits: true,
        }
    }
}

pub struct GoalResolver {
    pub config: GoalResolverConfig,
}

impl GoalResolver {
    pub fn new(config: GoalResolverConfig) -> Self {
        Self { config }
    }

    pub fn resolve_pose(
        &self,
        ctx: &PlanningContext,
        pose: &Pose,
    ) -> Result<ValidatedGoal<ResolvedPoseGoal>, PlanningError> {
        let q_start = ctx.current_state.positions().ok_or_else(|| {
            PlanningError::InvalidContext("Current state missing joint positions".into())
        })?;
        let ik_result = ctx
            .ik_solver
            .solve(&q_start, IKGoal::Pose(pose.clone()))?;

        match ik_result.status {
            IKStatus::Converged => {}
            IKStatus::MaxIterations => {
                return Err(PlanningError::IkFailed {
                    target_pose: pose.clone(),
                    reason: IkFailureReason::MaxIterationsReached,
                });
            }
        }

        let mut metadata = GoalMetadata::default();

        if self.config.check_joint_limits {
            self.validate_joint_limits(ctx, &ik_result.q)?;
        }

        let q = &ik_result.q;
        self.enrich_metadata(ctx, q, &mut metadata);
        let assessment = self.config.policy.evaluate(&metadata);

        Ok(ValidatedGoal {
            goal: ResolvedPoseGoal {
                pose: pose.clone(),
                state: RobotState::from_positions(ik_result.q),
            },
            metadata,
            assessment,
        })
    }

    /// Resolve a translation-only target via `IKGoal::Position` — orientation
    /// is unconstrained. Robots that cannot reach a full 6-DOF pose (e.g.
    /// SCARA, yaw-only) converge on a position goal when a full-pose goal
    /// would hit `MaxIterations`.
    pub fn resolve_position(
        &self,
        ctx: &PlanningContext,
        position: thalos_math::Vector3,
    ) -> Result<ValidatedGoal<ResolvedPositionGoal>, PlanningError> {
        let q_start = ctx.current_state.positions().ok_or_else(|| {
            PlanningError::InvalidContext("Current state missing joint positions".into())
        })?;
        let ik_result = ctx
            .ik_solver
            .solve(&q_start, IKGoal::Position(position))?;

        match ik_result.status {
            IKStatus::Converged => {}
            IKStatus::MaxIterations => {
                return Err(PlanningError::IkFailedPosition {
                    target_position: [position.x, position.y, position.z],
                    reason: IkFailureReason::MaxIterationsReached,
                });
            }
        }

        let mut metadata = GoalMetadata::default();

        if self.config.check_joint_limits {
            self.validate_joint_limits(ctx, &ik_result.q)?;
        }

        let q = &ik_result.q;
        self.enrich_metadata(ctx, q, &mut metadata);
        let assessment = self.config.policy.evaluate(&metadata);

        Ok(ValidatedGoal {
            goal: ResolvedPositionGoal {
                position,
                state: RobotState::from_positions(ik_result.q),
            },
            metadata,
            assessment,
        })
    }

    pub fn resolve_joint(
        &self,
        ctx: &PlanningContext,
        target: &[f64],
    ) -> Result<ValidatedGoal<JointGoal>, PlanningError> {
        let mut metadata = GoalMetadata::default();

        if self.config.check_joint_limits {
            self.validate_joint_limits(ctx, target)?;
        }

        self.enrich_metadata(ctx, target, &mut metadata);
        let assessment = self.config.policy.evaluate(&metadata);

        Ok(ValidatedGoal {
            goal: JointGoal(target.to_vec()),
            metadata,
            assessment,
        })
    }

    /// Populate metadata with singularity/manipulability when at least one
    /// policy metric is active. Avoids paying SVD cost when everything is `Ignore`.
    fn enrich_metadata(&self, ctx: &PlanningContext, q: &[f64], metadata: &mut GoalMetadata) {
        let active = !matches!(
            (
                self.config.policy.singularity,
                self.config.policy.manipulability
            ),
            (MetricAction::Ignore, MetricAction::Ignore)
        );

        if active {
            if let Some((singularity, manipulability)) = self.analyze_configuration(ctx, q) {
                metadata.singularity = Some(singularity);
                metadata.manipulability = Some(manipulability);
            }
        }
    }

    fn validate_joint_limits(&self, ctx: &PlanningContext, q: &[f64]) -> Result<(), PlanningError> {
        let mut joint_idx = 0;
        for segment in &ctx.robot.segments {
            if segment.joint.dof() == 0 {
                continue;
            }
            let limits = segment.joint.limits();

            // Joints without mechanical bounds (e.g. URDF continuous
            // without an explicit <limit>) cannot violate limits.
            if !limits.enabled {
                joint_idx += 1;
                continue;
            }

            let value = q[joint_idx];

            if self.config.strict_limits {
                if value < limits.min || value > limits.max {
                    return Err(PlanningError::JointLimitViolation {
                        joint_index: joint_idx,
                        value,
                        min: limits.min,
                        max: limits.max,
                    });
                }
            }
            joint_idx += 1;
        }
        Ok(())
    }

    fn analyze_configuration(
        &self,
        ctx: &PlanningContext,
        q: &[f64],
    ) -> Option<(SingularityReport, ManipulabilityReport)> {
        let fk = ForwardKinematics::new(ctx.robot.clone());
        let jac_solver = if let Some(tcp) = ctx.tcp {
            GeometricJacobian::with_tcp(fk, tcp.clone())
        } else {
            let ee = ctx.robot.end_effector().clone();
            GeometricJacobian::new(fk, ee)
        };
        let jacobian = jac_solver.evaluate(q);
        let singularity = SingularityReport::analyze(&jacobian);
        let manipulability = ManipulabilityReport::compute(&singularity);
        Some((singularity, manipulability))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::goal::types::ResolvedPositionGoal;
    use crate::motion::planner::PlanningContext;
    use thalos_core::{
        kinematics::inverse::DampedLeastSquaresSolver,
        models::{RobotModel, RobotRegistry},
        robot::state::RobotState,
    };
    use thalos_math::Vector3;

    /// `resolve_position` must drive IK with `IKGoal::Position` so a SCARA
    /// (4 DOF, all Z axes — yaw only) converges on a translation target even
    /// though it can never reach a full 6-DOF pose.
    #[test]
    fn resolve_position_converges_scara() {
        let robot = RobotRegistry::create_default(RobotModel::Scara);
        let state = RobotState::zero(4);
        let fk = ForwardKinematics::new(robot.clone());
        let solver =
            DampedLeastSquaresSolver::new(fk.clone(), robot.end_effector().clone(), 500, 1e-6, 0.1);
        let ctx = PlanningContext {
            robot: &robot,
            current_state: &state,
            ik_solver: &solver,
            tcp: None,
        };

        let resolver = GoalResolver::new(GoalResolverConfig {
            policy: PlanningPolicy::default(),
            check_joint_limits: true,
            strict_limits: true,
        });
        let target = Vector3::new(0.6, 0.5, 0.25);
        let validated = resolver
            .resolve_position(&ctx, target)
            .expect("position-only IK must converge on SCARA");

        let ResolvedPositionGoal { position, state } = &validated.goal;
        assert_eq!(position, &target);

        let q_eval = state.positions().expect("valid state positions");
        let ee = fk
            .evaluate(&q_eval)
            .ee_pose()
            .unwrap()
            .translation();
        let error = (ee - target).magnitude();
        assert!(
            error < 0.02,
            "resolved state EE error {error:.4} (target {target:?}, got {ee:?})"
        );
    }
}
