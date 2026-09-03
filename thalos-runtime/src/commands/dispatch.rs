use thalos_engine::core::{
    kinematics::inverse::IKResult,
    models::{RobotModel, RobotRegistry},
    prelude::ActiveRobot,
    robot::serial_chain::SerialChain,
    robot::tool_frame::ToolFrame,
};

use thalos_engine::models::Robot;

use crate::{
    RuntimeError,
    commands::{handler::ExecutableCommand, kinematics::KinematicsCommand, motion::MotionCommands},
    scene::JointMeta,
    robot::SceneRuntime,
};

#[derive(Debug, Clone)]
pub enum Command {
    SetJoints(Vec<f64>),
    LoadRobot(RobotModel),
    LoadUrdfRobot {
        name: String,
        joints_meta: Vec<JointMeta>,
        chain: SerialChain,
        /// The full URDF model — preserved for visual/collision rendering.
        robot: Robot,
        /// Canonical identity `urdf:<sha256-trunc-12>` (spec robot-identity R1).
        /// Computed in the API handler from the raw XML (design D2).
        robot_id: String,
    },
    Kinematics(KinematicsCommand),
    Motion(MotionCommands),
    /// Select or clear the active Tool Center Point (TCP) frame.
    ///
    /// When `Some(tool_frame)`, all analysis and IK default to this TCP.
    /// When `None`, the flange (`chain.end_effector`) is used as the default.
    SelectToolFrame(Option<ToolFrame>),
}

impl ExecutableCommand for Command {
    type Output = Option<IKResult>;

    fn execute(&self, runtime: &mut SceneRuntime) -> Result<Option<IKResult>, RuntimeError> {
        match self {
            Command::SetJoints(joints) => {
                let expected = runtime.active_robot.chain.dof_count();
                if joints.len() != expected {
                    return Err(RuntimeError::JointCountMismatch {
                        expected,
                        received: joints.len(),
                    });
                }
                runtime.active_robot.joints = joints.clone();
                Ok(None)
            }
            Command::LoadRobot(model) => {
                let dof = model.metadata().dof;
                let chain = RobotRegistry::create_default(*model);
                runtime.active_robot = ActiveRobot::new(Some(*model), chain, vec![0.0; dof]);
                runtime.robot_name = model.metadata().display_name.to_string();
                runtime.robot_id = model.metadata().id.to_string(); // spec R1.3
                runtime.joints_meta.clear();
                runtime.active_plan = None;
                runtime.scheduled_plan = None; // spec command-endpoints "Robot Change Cleanup"
                runtime.active_tcp = None; // Clear TCP when changing robot
                runtime.clear_command_history(); // stale inverses die with the robot
                Ok(None)
            }
            Command::LoadUrdfRobot {
                name,
                joints_meta,
                chain,
                robot,
                robot_id,
            } => {
                let dof = chain.dof_count();
                runtime.active_robot = ActiveRobot::new(None, chain.clone(), vec![0.0; dof]);
                runtime.robot_name = name.clone();
                runtime.robot_id = robot_id.clone();
                runtime.joints_meta = joints_meta.clone();
                runtime.robot_source = Some(robot.clone());
                runtime.active_plan = None;
                runtime.scheduled_plan = None; // spec command-endpoints "Robot Change Cleanup"
                runtime.active_tcp = None; // Clear TCP when changing robot
                runtime.clear_command_history(); // stale inverses die with the robot
                Ok(None)
            }
            Command::Kinematics(cmd) => cmd.execute(runtime).map(Some),
            Command::Motion(cmd) => cmd.execute(runtime),
            Command::SelectToolFrame(tool_frame) => {
                runtime.select_tool_frame(tool_frame.clone())?;
                Ok(None)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_engine::core::trajectory::TrajectoryPoint;
    use thalos_engine::planning::motion::program::CompiledPlan;
    use thalos_engine::planning::program_edit::ProgramEdit;

    use crate::services::command_history::CommandMetrics;

    fn test_runtime() -> SceneRuntime {
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);
        let active_robot = ActiveRobot::new(Some(RobotModel::Planar2R), chain, vec![0.0; 2]);
        SceneRuntime::new(active_robot, "test-bot".into())
    }

    /// A VALID compiled plan: two waypoints, non-zero duration, target `[t, t]`.
    fn compiled_plan(t: f64) -> CompiledPlan {
        let points = vec![
            TrajectoryPoint::new(vec![0.0, 0.0], 0.0),
            TrajectoryPoint::new(vec![t, t], 1.0),
        ];
        CompiledPlan::new(thalos_engine::core::trajectory::Trajectory::new(points), vec![])
    }

    /// A MoveWaypoint edit — seeds the history with a stored inverse.
    fn recorded_edit() -> (ProgramEdit, ProgramEdit) {
        let cmd = ProgramEdit::MoveWaypoint {
            segment_index: 0,
            new_target: vec![2.0, 2.0],
            old_target: Some(vec![1.0, 1.0]),
        };
        (cmd.clone(), cmd.inverse())
    }

    /// Seed a runtime with one applied command + a scheduled plan — the stale
    /// state a robot change must invalidate (spec "Robot Change Cleanup").
    fn seeded_runtime() -> SceneRuntime {
        let mut runtime = test_runtime();
        runtime.schedule_plan(compiled_plan(1.0));
        let (cmd, inverse) = recorded_edit();
        runtime.record_applied_command(cmd, inverse, CommandMetrics::new(0.4, 0.6), Vec::new());
        assert_eq!(runtime.history_len(), 1, "setup: one applied command");
        assert!(runtime.scheduled_plan.is_some(), "setup: a scheduled plan");
        runtime
    }

    #[test]
    fn load_robot_clears_command_history_and_scheduled_plan() {
        // Spec command-endpoints "Robot Change Cleanup": a robot change must
        // clear BOTH the command history (stale inverses) and the scheduled
        // plan — undo from a different robot's history is invalid.
        let mut runtime = seeded_runtime();

        Command::LoadRobot(RobotModel::Planar2R)
            .execute(&mut runtime)
            .expect("catalog robot load must succeed");

        assert_eq!(
            runtime.history_len(),
            0,
            "LoadRobot must clear the command history"
        );
        assert!(
            runtime.scheduled_plan.is_none(),
            "LoadRobot must clear the scheduled plan"
        );
        assert!(
            runtime.active_plan.is_none(),
            "LoadRobot must clear the active plan"
        );
    }

    #[test]
    fn load_urdf_robot_clears_command_history_and_scheduled_plan() {
        // Triangulation — the URDF import arm carries the same cleanup
        // contract as the catalog arm.
        let mut runtime = seeded_runtime();
        let chain = RobotRegistry::create_default(RobotModel::Planar2R);

        Command::LoadUrdfRobot {
            name: "test-urdf".into(),
            joints_meta: vec![],
            chain,
            robot: Robot::new("test-urdf", "base"),
            robot_id: "urdf:test".into(),
        }
        .execute(&mut runtime)
        .expect("URDF robot load must succeed");

        assert_eq!(
            runtime.history_len(),
            0,
            "LoadUrdfRobot must clear the command history"
        );
        assert!(
            runtime.scheduled_plan.is_none(),
            "LoadUrdfRobot must clear the scheduled plan"
        );
        assert!(
            runtime.active_plan.is_none(),
            "LoadUrdfRobot must clear the active plan"
        );
    }

    #[test]
    fn set_joints_rejects_wrong_dof_count() {
        let mut runtime = test_runtime(); // Planar2R = 2 DOF

        let err = Command::SetJoints(vec![1.0, 2.0, 3.0])
            .execute(&mut runtime)
            .unwrap_err();

        match err {
            RuntimeError::JointCountMismatch { expected, received } => {
                assert_eq!(expected, 2);
                assert_eq!(received, 3);
            }
            other => panic!("expected JointCountMismatch, got {other:?}"),
        }

        // Joints must remain unchanged after the rejected command.
        assert_eq!(runtime.active_robot.joints, vec![0.0, 0.0]);
    }

    #[test]
    fn movej_rejects_wrong_dof_count() {
        let mut runtime = test_runtime(); // Planar2R = 2 DOF

        let err = Command::Motion(MotionCommands::MoveJ {
            target: vec![1.0, 2.0, 3.0],
        })
        .execute(&mut runtime)
        .unwrap_err();

        match err {
            RuntimeError::JointCountMismatch { expected, received } => {
                assert_eq!(expected, 2);
                assert_eq!(received, 3);
            }
            other => panic!("expected JointCountMismatch, got {other:?}"),
        }

        // Joints must remain unchanged after the rejected command.
        assert_eq!(runtime.active_robot.joints, vec![0.0, 0.0]);
    }

    #[test]
    fn movej_accepts_correct_dof_count() {
        let mut runtime = test_runtime(); // Planar2R = 2 DOF

        Command::Motion(MotionCommands::MoveJ {
            target: vec![1.0, 2.0],
        })
        .execute(&mut runtime)
        .expect("MoveJ with correct DOF must succeed");

        assert_eq!(runtime.active_robot.joints, vec![1.0, 2.0]);
    }
}
