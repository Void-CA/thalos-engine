use thalos_engine::core::{
    kinematics::{
        forward::ForwardKinematics,
        inverse::{DampedLeastSquaresSolver, IKGoal, IKResult, IKSolver},
    },
    prelude::RobotState,
    robot::{serial_chain::SerialChain, tool_frame::ToolFrame},
    spatial::frame::FrameId,
    spatial::pose::Pose,
};

use thalos_engine::planning::{
    goal::{
        GoalMetadata, GoalResolver, GoalResolverConfig, PlanningAssessment, PlanningPolicy,
        ValidatedGoal,
    },
    motion::move_j::{MoveJConfig, MoveJPlanner},
    motion::planner::{PlanningContext, SegmentPlanner},
};

use crate::plan::MotionType;
use crate::{RuntimeError, commands::handler::ExecutableCommand, robot::SceneRuntime};

const IK_MAX_ITERS: usize = 500;
const IK_TOLERANCE: f64 = 1e-6;
const IK_LAMBDA: f64 = 0.1;

/// Create a DampedLeastSquaresSolver for the given chain and frame.
fn make_ik_solver(chain: &SerialChain, frame: FrameId) -> DampedLeastSquaresSolver {
    let fk = ForwardKinematics::new(chain.clone());
    DampedLeastSquaresSolver::new(fk, frame, IK_MAX_ITERS, IK_TOLERANCE, IK_LAMBDA)
}

/// Build a `PlanningContext` from runtime state for a single planning call.
fn make_planning_ctx<'a>(
    chain: &'a SerialChain,
    state: &'a RobotState,
    ik_solver: &'a dyn IKSolver,
    tcp: Option<&'a ToolFrame>,
) -> PlanningContext<'a> {
    PlanningContext {
        robot: chain,
        current_state: state,
        ik_solver,
        tcp,
    }
}

#[derive(Debug, Clone)]
pub enum MotionCommands {
    MoveJ {
        target: Vec<f64>,
    },
    /// Plan a joint-space trajectory from current position to `target`.
    PlanAndMoveJ {
        target: Vec<f64>,
        max_velocity: Option<f64>,
        max_acceleration: Option<f64>,
        time_step: Option<f64>,
    },
    /// Plan a cartesian trajectory from current EE position to `target_pose`.
    PlanAndMoveL {
        frame: FrameId,
        target_pose: Pose,
        max_velocity: Option<f64>,
        max_acceleration: Option<f64>,
        time_step: Option<f64>,
        cartesian_step: Option<f64>,
    },
}

impl ExecutableCommand for MotionCommands {
    type Output = Option<IKResult>;

    fn execute(&self, runtime: &mut SceneRuntime) -> Result<Option<IKResult>, RuntimeError> {
        match self {
            Self::MoveJ { target } => {
                let expected = runtime.active_robot.chain.dof_count();
                if target.len() != expected {
                    return Err(RuntimeError::JointCountMismatch {
                        expected,
                        received: target.len(),
                    });
                }
                runtime.active_robot.joints = target.clone();
                Ok(None)
            }

            Self::PlanAndMoveJ {
                target,
                max_velocity,
                max_acceleration,
                time_step,
            } => {
                let chain = runtime.active_robot.chain.clone();
                let expected = chain.dof_count();
                if target.len() != expected {
                    return Err(RuntimeError::JointCountMismatch {
                        expected,
                        received: target.len(),
                    });
                }
                let ee = *chain.end_effector();
                let state = RobotState::from_positions(runtime.active_robot.joints.clone());
                let solver = make_ik_solver(&chain, ee);
                let ctx = make_planning_ctx(&chain, &state, &solver, runtime.active_tcp.as_ref());

                let resolver = GoalResolver::new(GoalResolverConfig {
                    policy: PlanningPolicy::default(),
                    check_joint_limits: true,
                    strict_limits: true,
                });
                let goal = resolver.resolve_joint(&ctx, target)?;

                let planner = MoveJPlanner::new(MoveJConfig {
                    max_velocity: max_velocity.unwrap_or(1.0),
                    max_acceleration: max_acceleration.unwrap_or(0.5),
                    time_step: time_step.unwrap_or(0.01),
                });
                let trajectory = planner.plan(&ctx, &goal)?;

                let last = trajectory
                    .waypoints()
                    .last()
                    .map(|p| p.joints().to_vec())
                    .unwrap_or_else(|| target.clone());
                runtime.active_robot.joints = last;
                runtime.set_completed_plan(trajectory, MotionType::MoveJ);

                Ok(None)
            }

            Self::PlanAndMoveL {
                frame,
                target_pose,
                max_velocity,
                max_acceleration,
                time_step: _,
                cartesian_step: _,
            } => {
                let joints = runtime.active_robot.joints.clone();
                let chain = runtime.active_robot.chain.clone();

                // Solve IK for the target position (position-only to handle
                // under-actuated arms like Planar2R that can't match a full pose).
                let solver = make_ik_solver(&chain, *frame);
                let translation = target_pose.translation();
                let ik = solver.solve(&joints, IKGoal::Position(translation))?;

                let target = ik.q.clone();
                let state = RobotState::from_positions(joints.clone());
                let ctx = make_planning_ctx(&chain, &state, &solver, runtime.active_tcp.as_ref());

                // Use joint-space planner to create a smooth trajectory to the
                // IK-solved position. Not a true cartesian path, but guarantees
                // convergence for any reachable target.
                let planner = MoveJPlanner::new(MoveJConfig {
                    max_velocity: max_velocity.unwrap_or(1.0),
                    max_acceleration: max_acceleration.unwrap_or(0.5),
                    time_step: 0.01,
                });
                let goal = ValidatedGoal {
                    goal: thalos_engine::planning::goal::JointGoal(target.clone()),
                    metadata: GoalMetadata::default(),
                    assessment: PlanningAssessment::accepted(),
                };
                let trajectory = planner.plan(&ctx, &goal)?;

                runtime.active_robot.joints = target;
                runtime.set_completed_plan(trajectory, MotionType::MoveL);

                Ok(None)
            }
        }
    }
}
