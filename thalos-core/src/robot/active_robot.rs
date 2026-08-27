use crate::models::RobotModel;

use super::serial_chain::SerialChain;

/// A loaded robot with its kinematic identity, chain description,
/// and current joint configuration.
///
/// This is the canonical representation of "the robot that is currently
/// loaded" across all subsystems: planning, execution, visualisation,
/// analysis, and runtime.
///
/// `model` is a catalog-membership tag (ADR-003), not a kinematic identity:
/// `Some(RobotModel::X)` = internal catalog robot (UI presets, examples);
/// `None` = robot loaded from an external URDF, whose identity is carried by
/// `robot_name`/`robot_source`/`joints_meta`/`chain`.
#[derive(Debug, Clone)]
pub struct ActiveRobot {
    pub model: Option<RobotModel>,
    pub chain: SerialChain,
    pub joints: Vec<f64>,
}

impl ActiveRobot {
    pub fn new(model: Option<RobotModel>, chain: SerialChain, joints: Vec<f64>) -> Self {
        Self {
            model,
            chain,
            joints,
        }
    }
}
