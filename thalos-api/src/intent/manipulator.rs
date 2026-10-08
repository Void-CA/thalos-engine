//! The Tier-1 receiver: [`Manipulator`].

use std::fmt;

use thalos_core::kinematics::context::{KinematicContext, TcpPose};
use thalos_core::kinematics::forward::ForwardKinematics;
use thalos_core::kinematics::inverse::{
    DampedLeastSquaresSolver, IKConfig, IKGoal, IKResult, IKSolver,
};
use thalos_core::robot::adapter::{self, AdapterError};
use thalos_core::robot::serial_chain::SerialChain;
use thalos_core::robot::state::RobotState;
use thalos_models::Robot as ModelRobot;
use thalos_planning::goal::{GoalMetadata, JointGoal, PlanningAssessment, ValidatedGoal};
use thalos_planning::motion::move_j::{MoveJConfig, MoveJPlanner};
use thalos_planning::motion::planner::JointPlanningContext;

use crate::intent::kinematics::{CartesianTarget, IkRequest, IkSolution};
use crate::intent::motion::{MotionError, MotionKind, MotionPlan, MotionProfile, MotionRequest};

/// Wrap a Tier 3 `IKResult` as a Tier 1 `IkSolution`.
fn solution_from(result: IKResult) -> IkSolution {
    IkSolution::new(
        result.q,
        result.status.is_converged(),
        result.iterations,
        result.final_error,
    )
}

/// Error loading a robot into a [`Manipulator`].
///
/// Tier 1 owns its error type: the underlying `AdapterError` is wrapped, not
/// exposed, so the mechanism can evolve without changing this surface.
#[derive(Debug)]
pub struct RobotLoadError {
    source: AdapterError,
}

impl RobotLoadError {
    /// A human-readable description of the load failure.
    pub fn message(&self) -> String {
        self.source.to_string()
    }
}

impl fmt::Display for RobotLoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "robot load failed: {}", self.source)
    }
}

impl std::error::Error for RobotLoadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(&self.source)
    }
}

impl From<AdapterError> for RobotLoadError {
    fn from(source: AdapterError) -> Self {
        Self { source }
    }
}

/// A robot expressed as a **capability**: something you can ask to move.
///
/// `Manipulator` is deliberately **stateless** with respect to the pipeline —
/// it holds the kinematic chain and nothing else. There is no active plan, no
/// execution session, no current joints and no cached solver. The current state
/// is supplied by the caller to each request, because live robot state belongs
/// to the application (see `ARCHITECTURE.md`).
#[derive(Debug, Clone)]
pub struct Manipulator {
    kinematics: KinematicContext,
}

impl Manipulator {
    /// Load a manipulator from a URDF **source string**.
    ///
    /// The engine performs no I/O: pass the URDF text, not a filesystem path.
    /// The end-effector is chosen automatically (the most-actuated tip). This is
    /// the intended entry point; [`from_chain`](Self::from_chain) remains as an
    /// escape hatch.
    pub fn from_urdf(source: &str) -> Result<Self, RobotLoadError> {
        Ok(Self::from_chain(adapter::from_urdf(source)?))
    }

    /// Build a manipulator from a structural robot model, choosing the tip
    /// automatically.
    pub fn from_model(model: &ModelRobot) -> Result<Self, RobotLoadError> {
        Ok(Self::from_chain(adapter::auto(model)?))
    }

    /// Build a manipulator over an existing kinematic chain (escape hatch).
    pub fn from_chain(chain: SerialChain) -> Self {
        Self {
            kinematics: KinematicContext::at_end_effector(chain),
        }
    }

    /// The end-effector frame name chosen for this manipulator.
    ///
    /// The common cases (default base frame, default end-effector) are handled
    /// without exposing `FrameId`. Selecting a different tip or resolving poses
    /// against a non-default frame remains a Tier 3 concern via
    /// [`chain`](Self::chain).
    pub fn end_effector_name(&self) -> Option<&str> {
        self.chain().end_effector_frame().map(|frame| frame.name())
    }

    /// The underlying kinematic chain (escape hatch to Tier 3).
    pub fn chain(&self) -> &SerialChain {
        self.kinematics.chain()
    }

    /// Number of actuated degrees of freedom.
    pub fn dof(&self) -> usize {
        self.chain().dof_count()
    }

    /// Forward kinematics: joint configuration → world TCP pose.
    ///
    /// Delegates to the shared kinematic authority ([`KinematicContext`]); the
    /// consumer does not build or name it. Errors only on a DOF mismatch.
    pub fn forward(&self, joints: &[f64]) -> Result<TcpPose, MotionError> {
        self.kinematics
            .tcp_pose(joints)
            .ok_or(MotionError::DofMismatch {
                expected: self.dof(),
                got: joints.len(),
            })
    }

    /// Plan a motion intention from a given start configuration.
    ///
    /// The start configuration is an explicit parameter — not `Manipulator`
    /// state — keeping the receiver free of execution state. (Named
    /// `plan_motion`, not `move`: `move` is a Rust keyword.)
    ///
    /// For a [`MotionRequest::Linear`], the engine resolves the cartesian target
    /// internally (inverse kinematics → joint planning). The caller never sees
    /// that pipeline.
    pub fn plan_motion(
        &self,
        from: &[f64],
        request: MotionRequest,
    ) -> Result<MotionPlan, MotionError> {
        let dof = self.dof();
        if from.len() != dof {
            return Err(MotionError::DofMismatch {
                expected: dof,
                got: from.len(),
            });
        }

        match request {
            MotionRequest::Joint(joint) => {
                if joint.target().len() != dof {
                    return Err(MotionError::DofMismatch {
                        expected: dof,
                        got: joint.target().len(),
                    });
                }
                self.plan_joint_move(from, joint.target().to_vec(), joint.profile(), MotionKind::Joint)
            }
            MotionRequest::Linear(linear) => {
                let target = self.solve_ik_to_joints(from, linear.target())?;
                self.plan_joint_move(from, target, linear.profile(), MotionKind::Linear)
            }
        }
    }

    /// Inverse kinematics: solve a cartesian target into joints.
    ///
    /// A non-converged solve is returned as an [`IkSolution`] with
    /// `converged() == false` (IK is inherently approximate — the solution is
    /// the answer to the question). Only a solver error is returned as `Err`.
    /// The seed defaults to zeros; use [`inverse_with`](Self::inverse_with) to
    /// provide a warm start or tolerances.
    pub fn inverse(&self, target: impl Into<CartesianTarget>) -> Result<IkSolution, MotionError> {
        let seed = vec![0.0; self.dof()];
        let result = self.run_ik(&seed, &target.into(), IKConfig::default())?;
        Ok(solution_from(result))
    }

    /// Inverse kinematics with a Tier-2 [`IkRequest`] (seed, tolerances).
    pub fn inverse_with(&self, request: IkRequest) -> Result<IkSolution, MotionError> {
        let seed = request
            .seed()
            .map(<[f64]>::to_vec)
            .unwrap_or_else(|| vec![0.0; self.dof()]);

        let mut config = IKConfig::default();
        if let Some(tolerance) = request.tolerance() {
            config.tolerance = tolerance;
        }
        if let Some(max_iterations) = request.max_iterations() {
            config.max_iterations = max_iterations;
        }
        if let Some(lambda) = request.lambda() {
            config.lambda = lambda;
        }

        let result = self.run_ik(&seed, request.target(), config)?;
        Ok(solution_from(result))
    }

    /// Run the IK solver. Private: Tier 1 does not expose the solver.
    fn run_ik(
        &self,
        seed: &[f64],
        target: &CartesianTarget,
        config: IKConfig,
    ) -> Result<IKResult, MotionError> {
        let solver = DampedLeastSquaresSolver::from_config(
            ForwardKinematics::new(self.chain().clone()),
            *self.chain().end_effector(),
            config,
        );
        let goal = match target {
            CartesianTarget::Position(position) => IKGoal::Position(*position),
            CartesianTarget::Pose(pose) => IKGoal::Pose(pose.clone()),
        };
        Ok(solver.solve(seed, goal)?)
    }

    /// Resolve a cartesian target into joints for planning, requiring
    /// convergence (a plan to an unreachable goal must fail).
    fn solve_ik_to_joints(
        &self,
        seed: &[f64],
        target: &CartesianTarget,
    ) -> Result<Vec<f64>, MotionError> {
        let result = self.run_ik(seed, target, IKConfig::default())?;
        if !result.status.is_converged() {
            return Err(MotionError::Unreachable {
                final_error: result.final_error,
            });
        }
        Ok(result.q)
    }

    /// Joint-space planning shared by both intents. Builds ONLY a joint context:
    /// no IK solver participates here.
    fn plan_joint_move(
        &self,
        from: &[f64],
        target: Vec<f64>,
        profile: MotionProfile,
        kind: MotionKind,
    ) -> Result<MotionPlan, MotionError> {
        let state = RobotState::from_positions(from.iter().copied());
        let context = JointPlanningContext {
            robot: self.chain(),
            current_state: &state,
        };

        let planner = MoveJPlanner::new(MoveJConfig {
            max_velocity: profile.max_velocity,
            max_acceleration: profile.max_acceleration,
            time_step: profile.time_step,
        });

        let goal = ValidatedGoal {
            goal: JointGoal::new(target),
            metadata: GoalMetadata::default(),
            assessment: PlanningAssessment::accepted(),
        };

        let trajectory = planner.plan_joint(&context, &goal)?;
        Ok(MotionPlan::new(trajectory, kind))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::intent::motion::{JointMove, LinearMove};
    use thalos_core::models::{RobotModel, RobotRegistry};
    use thalos_math::UnitQuaternion;

    fn robot() -> Manipulator {
        Manipulator::from_chain(RobotRegistry::create_default(RobotModel::Planar2R))
    }

    const PLANAR_2R_URDF: &str = r#"
        <robot name="planar_2r">
            <link name="base"/>
            <link name="link1"/>
            <link name="link2"/>
            <joint name="j1" type="revolute">
                <parent link="base"/><child link="link1"/>
                <axis xyz="0 0 1"/>
                <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
            </joint>
            <joint name="j2" type="revolute">
                <parent link="link1"/><child link="link2"/>
                <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
                <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
            </joint>
        </robot>
    "#;

    #[test]
    fn loads_from_urdf_without_naming_chain_types() {
        let robot = Manipulator::from_urdf(PLANAR_2R_URDF).expect("loads");
        assert_eq!(robot.dof(), 2);
        assert_eq!(robot.end_effector_name(), Some("link2"));
    }

    #[test]
    fn loads_from_a_structural_model() {
        let model = thalos_importer::import_urdf(PLANAR_2R_URDF).expect("import");
        let robot = Manipulator::from_model(&model).expect("loads");
        assert_eq!(robot.dof(), 2);
    }

    #[test]
    fn joint_move_plans_without_any_ik() {
        let robot = robot();
        let plan = robot
            .plan_motion(&[0.0, 0.0], MotionRequest::Joint(JointMove::to(vec![0.3, 0.4])))
            .expect("joint move plans");
        assert!(plan.waypoint_count() > 0);
        assert!(plan.duration() > 0.0);
    }

    #[test]
    fn linear_move_hides_the_ik_pipeline() {
        let robot = robot();
        let target = CartesianTarget::position(1.0, 0.5, 0.0);
        let plan = robot
            .plan_motion(&[0.0, 0.0], MotionRequest::Linear(LinearMove::to(target)))
            .expect("linear move plans");
        assert!(plan.waypoint_count() > 0);
    }

    #[test]
    fn unreachable_linear_target_errors() {
        let robot = robot();
        let target = CartesianTarget::position(50.0, 50.0, 0.0);
        let err = robot
            .plan_motion(&[0.0, 0.0], MotionRequest::Linear(LinearMove::to(target)))
            .unwrap_err();
        assert!(matches!(err, MotionError::Unreachable { .. }));
    }

    #[test]
    fn forward_kinematics_runs_on_the_receiver() {
        let robot = robot();
        let tcp = robot.forward(&[0.3, 0.4]).expect("fk");
        assert_eq!(tcp.position.len(), 3);
        // DOF mismatch is a typed error, not a panic.
        assert!(matches!(
            robot.forward(&[0.0]).unwrap_err(),
            MotionError::DofMismatch { expected: 2, got: 1 }
        ));
    }

    #[test]
    fn inverse_position_converges() {
        let robot = robot();
        let solution = robot
            .inverse(CartesianTarget::position(1.0, 0.5, 0.0))
            .expect("ik runs");
        assert!(solution.converged(), "err {}", solution.final_error());
        assert_eq!(solution.joints().len(), 2);
    }

    #[test]
    fn inverse_with_seed_and_tolerance() {
        let robot = robot();
        let solution = robot
            .inverse_with(
                IkRequest::new(CartesianTarget::position(1.0, 0.5, 0.0))
                    .with_seed(vec![0.2, 0.1])
                    .with_tolerance(1e-8),
            )
            .expect("ik runs");
        assert!(solution.converged());
    }

    #[test]
    fn inverse_pose_target_is_accepted() {
        let robot = robot();
        // A full pose on a 2-DOF arm cannot converge, but the intent is accepted
        // and returns a best-effort solution rather than an error.
        let solution = robot
            .inverse(CartesianTarget::pose(1.0, 0.5, 0.0, UnitQuaternion::identity()))
            .expect("ik runs");
        assert_eq!(solution.joints().len(), 2);
    }

    #[test]
    fn profile_override_is_honoured() {
        let robot = robot();
        let profile = MotionProfile {
            max_velocity: 2.0,
            ..Default::default()
        };
        let request = MotionRequest::Joint(JointMove::to(vec![0.1, 0.2]).with_profile(profile));
        assert!(robot.plan_motion(&[0.0, 0.0], request).is_ok());
    }

    #[test]
    fn start_dof_mismatch_is_an_error() {
        let robot = robot();
        let err = robot
            .plan_motion(&[0.0], MotionRequest::Joint(JointMove::to(vec![0.1, 0.2])))
            .unwrap_err();
        assert!(matches!(err, MotionError::DofMismatch { expected: 2, got: 1 }));
    }

    #[test]
    fn target_dof_mismatch_is_an_error() {
        let robot = robot();
        let err = robot
            .plan_motion(&[0.0, 0.0], MotionRequest::Joint(JointMove::to(vec![0.1])))
            .unwrap_err();
        assert!(matches!(err, MotionError::DofMismatch { expected: 2, got: 1 }));
    }
}
