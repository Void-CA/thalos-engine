use serde::{Deserialize, Serialize};
use thiserror::Error;
use thalos_core::ids::{ProgramName, RobotId, TargetId};
use thalos_core::program::{
    ControlInstruction, Instruction, MotionInstruction, RobotProgram, Target,
};
use crate::operation::{HomeOp, MoveToOp, SemanticOperation, WaitOp};
use crate::resource::LocationId;

/// Errors occurring during pure program normalization (`normalize`).
#[derive(Debug, Clone, PartialEq, Eq, Error, Serialize, Deserialize)]
pub enum NormalizeError {
    #[error("Target '{0}' referenced in instruction body is missing from program targets")]
    UndefinedTarget(TargetId),

    #[error("Program name cannot be empty")]
    EmptyProgramName,

    #[error("Robot ID cannot be empty")]
    EmptyRobotId,

    #[error("Unsupported or malformed skill call parameter for skill '{0}'")]
    InvalidSkillCallParameter(String),
}

/// Intermediate Representation (IR) derived from a `RobotProgram`.
///
/// `SemanticIr` is a pure, normalized representation of what the user program
/// intends to execute. It contains NO scene spatial resolution, kinematics, or
/// runtime state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SemanticIr {
    pub name: ProgramName,
    pub robot: RobotId,
    pub targets: Vec<Target>,
    pub operations: Vec<SemanticOperation>,
}

impl SemanticIr {
    pub fn new(
        name: ProgramName,
        robot: RobotId,
        targets: Vec<Target>,
        operations: Vec<SemanticOperation>,
    ) -> Self {
        Self {
            name,
            robot,
            targets,
            operations,
        }
    }

    pub fn from_operations(operations: Vec<SemanticOperation>) -> Self {
        Self {
            name: ProgramName("unnamed_program".into()),
            robot: RobotId("default_robot".into()),
            targets: Vec::new(),
            operations,
        }
    }
}

impl From<&crate::program::SemanticProgram> for SemanticIr {
    fn from(program: &crate::program::SemanticProgram) -> Self {
        Self::from_operations(program.operations.clone())
    }
}

impl From<crate::program::SemanticProgram> for SemanticIr {
    fn from(program: crate::program::SemanticProgram) -> Self {
        Self::from_operations(program.operations)
    }
}


/// Pure normalization pass: `RobotProgram → SemanticIr`.
///
/// Validates structural program invariants (e.g. referenced targets exist) and
/// desugars high-level `Instruction` items into normalized `SemanticOperation` items.
///
/// Purity invariant: This function NEVER accesses `Scene`, hardware drivers, or
/// external services.
pub fn normalize(program: &RobotProgram) -> Result<SemanticIr, NormalizeError> {
    if program.name.as_str().trim().is_empty() {
        return Err(NormalizeError::EmptyProgramName);
    }

    if program.robot.as_str().trim().is_empty() {
        return Err(NormalizeError::EmptyRobotId);
    }

    // Verify all target references in instructions match a target defined in the program.
    let defined_targets: std::collections::HashSet<&TargetId> =
        program.targets.iter().map(|t| &t.id).collect();

    let mut operations = Vec::new();

    for (idx, inst) in program.body.iter().enumerate() {
        let origin = thalos_core::ids::OperationId(format!("op-{}", idx + 1));

        match inst {
            Instruction::Motion(motion) => match motion {
                MotionInstruction::Move { target }
                | MotionInstruction::MoveJoint { target }
                | MotionInstruction::MoveLinear { target }
                | MotionInstruction::Approach { target, .. } => {
                    if !defined_targets.contains(target) {
                        return Err(NormalizeError::UndefinedTarget(target.clone()));
                    }
                    operations.push(SemanticOperation::MoveTo(MoveToOp {
                        origin,
                        destination: LocationId(target.as_str().to_string()),
                        tool: None,
                    }));
                }
                MotionInstruction::Retract { .. } => {
                    operations.push(SemanticOperation::Home(HomeOp { origin }));
                }
            },
            Instruction::Skill(skill_call) => {
                operations.push(SemanticOperation::Skill(crate::operation::SkillCallOp {
                    origin,
                    skill_call: skill_call.clone(),
                }));
            }
            Instruction::Control(control) => match control {
                ControlInstruction::Wait { duration } => {
                    operations.push(SemanticOperation::Wait(WaitOp {
                        origin,
                        duration: *duration,
                    }));
                }
                ControlInstruction::WaitSignal { .. } | ControlInstruction::SetSignal { .. } => {
                    operations.push(SemanticOperation::Wait(WaitOp {
                        origin,
                        duration: std::time::Duration::ZERO,
                    }));
                }
            },
        }
    }

    Ok(SemanticIr::new(
        program.name.clone(),
        program.robot.clone(),
        program.targets.clone(),
        operations,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use thalos_core::ids::{ProgramName, RobotId, SkillId, TargetId, TargetName};
    use thalos_core::program::{JointPosition, TargetReference, Value};

    #[test]
    fn test_normalize_is_pure_and_deterministic() {
        let target_1 = Target::new(
            TargetId("target-approach".to_string()),
            TargetName("pick_approach".to_string()),
            TargetReference::Joint {
                position: JointPosition::new(vec![0.0, 1.57, -1.57]),
            },
        );

        let program = RobotProgram::new(
            ProgramName("test_program".to_string()),
            RobotId("robot_arm_1".to_string()),
            vec![target_1],
            vec![
                Instruction::Motion(MotionInstruction::MoveJoint {
                    target: TargetId("target-approach".to_string()),
                }),
                Instruction::Skill(thalos_core::program::SkillCall::new(
                    SkillId("pick_object".to_string()),
                    vec![Value::Target(TargetId("target-approach".to_string()))],
                )),
                Instruction::Control(ControlInstruction::Wait {
                    duration: Duration::from_secs(2),
                }),
            ],
        );

        let ir1 = normalize(&program).expect("normalize pass 1");
        let ir2 = normalize(&program).expect("normalize pass 2");

        assert_eq!(ir1, ir2, "Normalize must be 100% deterministic");
        assert_eq!(ir1.operations.len(), 3);
    }

    #[test]
    fn test_normalize_detects_undefined_target() {
        let program = RobotProgram::new(
            ProgramName("invalid_program".to_string()),
            RobotId("robot_arm_1".to_string()),
            vec![], // No targets registered
            vec![Instruction::Motion(MotionInstruction::MoveJoint {
                target: TargetId("missing_target".to_string()),
            })],
        );

        let result = normalize(&program);
        assert_eq!(
            result,
            Err(NormalizeError::UndefinedTarget(TargetId(
                "missing_target".to_string()
            )))
        );
    }
}
