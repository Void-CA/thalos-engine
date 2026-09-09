//! Motion command — requests trajectory execution by a robot/controller.
//!
//! The resource assumes trajectory generation and servo control.
//! Thalos observes convergence through `RobotObservation`.
//!
//! `MotionKind` is deliberately minimal for MVP:
//! - `Joint`: joint-space movement (preserves `RobotCommand::MoveJoints` semantics)
//! - `Cartesian`: Cartesian pose movement
//! - `Stop`: immediate halt
//!
//! Future variants (`Home`, `Jog`, `Pause`, `Resume`) are excluded from MVP
//! because some of these may be controller-internal operations, not Thalos
//! motion commands. They will be introduced with clearer semantics.

use serde::{Deserialize, Serialize};

use crate::motion::target::{MotionPose, MotionProfile};
use crate::resource::ResourceRef;

/// A command requesting motion execution by a robot or motion controller.
///
/// The target resource assumes full responsibility for:
/// - Trajectory generation
/// - Servo control
/// - Safety monitoring
///
/// Thalos evaluates convergence by observing the resource's output
/// (joint positions, TCP pose, controller state).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MotionCommand {
    pub target: ResourceRef,
    pub kind: MotionKind,
    /// Optional motion profile. If absent, the controller uses its defaults.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub profile: Option<MotionProfile>,
}

/// The type of motion being requested.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum MotionKind {
    /// Joint-space movement to target positions.
    Joint {
        positions_rad: Vec<f64>,
        velocities_rad_s: Option<Vec<f64>>,
    },
    /// Cartesian pose movement.
    Cartesian {
        pose: MotionPose,
    },
    /// Immediate stop — resource halts all motion.
    Stop,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resource::ResourceKind;

    #[test]
    fn joint_command_preserves_robot_semantics() {
        let cmd = MotionCommand {
            target: ResourceRef::new("robot-01", ResourceKind::Robot),
            kind: MotionKind::Joint {
                positions_rad: vec![0.5, 0.2, -0.3],
                velocities_rad_s: Some(vec![0.1, 0.1, 0.1]),
            },
            profile: Some(MotionProfile {
                max_velocity: 1.0,
                max_acceleration: 2.0,
                max_jerk: Some(4.0),
            }),
        };
        match &cmd.kind {
            MotionKind::Joint {
                positions_rad,
                velocities_rad_s,
            } => {
                assert_eq!(positions_rad.len(), 3);
                assert!(velocities_rad_s.is_some());
            }
            _ => panic!("expected Joint"),
        }
    }

    #[test]
    fn cartesian_command_uses_existing_motion_pose() {
        let pose = MotionPose {
            position: [1.0, 2.0, 3.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "world".into(),
        };
        let cmd = MotionCommand {
            target: ResourceRef::new("robot-01", ResourceKind::Robot),
            kind: MotionKind::Cartesian { pose: pose.clone() },
            profile: None,
        };
        match &cmd.kind {
            MotionKind::Cartesian { pose: p } => {
                assert_eq!(*p, pose);
            }
            _ => panic!("expected Cartesian"),
        }
    }

    #[test]
    fn stop_command_no_payload() {
        let cmd = MotionCommand {
            target: ResourceRef::new("robot-01", ResourceKind::Robot),
            kind: MotionKind::Stop,
            profile: None,
        };
        assert_eq!(cmd.kind, MotionKind::Stop);
    }

    #[test]
    fn serde_round_trip_all_kinds() {
        let commands = vec![
            MotionCommand {
                target: ResourceRef::new("r1", ResourceKind::Robot),
                kind: MotionKind::Joint {
                    positions_rad: vec![1.0],
                    velocities_rad_s: None,
                },
                profile: None,
            },
            MotionCommand {
                target: ResourceRef::new("r1", ResourceKind::Robot),
                kind: MotionKind::Cartesian {
                    pose: MotionPose {
                        position: [0.0; 3],
                        orientation: [0.0, 0.0, 0.0, 1.0],
                        frame: "base".into(),
                    },
                },
                profile: None,
            },
            MotionCommand {
                target: ResourceRef::new("r1", ResourceKind::Robot),
                kind: MotionKind::Stop,
                profile: None,
            },
        ];
        for cmd in commands {
            let json = serde_json::to_string(&cmd).expect("serialize");
            let back: MotionCommand = serde_json::from_str(&json).expect("deserialize");
            assert_eq!(cmd, back);
        }
    }
}
