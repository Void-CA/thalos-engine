use serde::{Deserialize, Serialize};
use std::time::Duration;
use crate::ids::{SkillId, TargetId};

/// Core values passed to skill calls or instructions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Value {
    String(String),
    Number(f64),
    Boolean(bool),
    Target(TargetId),
}

/// Primitive motion instruction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MotionInstruction {
    MoveJoint { target: TargetId },
    MoveLinear { target: TargetId },
    Approach { target: TargetId, distance: f64 },
    Retract { distance: f64 },
}

/// Skill invocation instruction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SkillCall {
    pub skill: SkillId,
    pub arguments: Vec<Value>,
}

impl SkillCall {
    pub fn new(skill: SkillId, arguments: Vec<Value>) -> Self {
        Self { skill, arguments }
    }
}

/// Control flow instruction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ControlInstruction {
    Wait { duration: Duration },
    WaitSignal { signal_id: String },
    SetSignal { signal_id: String, value: bool },
}

/// Top-level instruction in a RobotProgram (ADR-001).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Instruction {
    Motion(MotionInstruction),
    Skill(SkillCall),
    Control(ControlInstruction),
}
