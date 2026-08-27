pub mod instruction;
pub mod target;

pub use instruction::*;
pub use target::*;

use serde::{Deserialize, Serialize};
use crate::ids::{ProgramName, RobotId};

/// Single Source of Truth for user robot operation logic (ADR-001).
///
/// Contains targets and body instructions without direct coupling to Scene,
/// ExecutionPlan, or runtime state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RobotProgram {
    pub name: ProgramName,
    pub robot: RobotId,
    pub targets: Vec<Target>,
    pub body: Vec<Instruction>,
}

impl RobotProgram {
    pub fn new(
        name: ProgramName,
        robot: RobotId,
        targets: Vec<Target>,
        body: Vec<Instruction>,
    ) -> Self {
        Self {
            name,
            robot,
            targets,
            body,
        }
    }
}
