use serde::{Deserialize, Serialize};

/// Operational action/intention target for robot hardware/simulator execution (L0 Domain).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RobotAction {
    MoveJoints {
        positions_rad: Vec<f64>,
        velocities_rad_s: Option<Vec<f64>>,
    },
    Stop,
}

pub type RobotCommand = RobotAction;
