use serde::{Deserialize, Serialize};
use crate::ids::SkillId;
use crate::program::instruction::Instruction;

/// Parameter specification for a skill signature.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Parameter {
    pub name: String,
    pub param_type: String,
}

/// Pre/post condition stub (Role contract per ADR-001).
///
/// Evaluated against semantic state, physical state, or telemetry.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Condition {
    pub identifier: String,
    pub expected_value: String,
}

/// Program fragment for skills implemented as program compositions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProgramFragment {
    pub instructions: Vec<Instruction>,
}

/// Planner configuration for skills implemented via dynamic planning.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SkillPlanner {
    pub policy: String,
}

/// Identifier for native hardware/driver skills.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct NativeSkillId(pub String);

/// Implementation strategy for a RobotSkill (ADR-001).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SkillImplementation {
    Program(ProgramFragment),
    Planner(SkillPlanner),
    Native(NativeSkillId),
}

/// Declarative operational capability of a robot (ADR-001).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RobotSkill {
    pub id: SkillId,
    pub name: String,
    pub parameters: Vec<Parameter>,
    pub preconditions: Vec<Condition>,
    pub postconditions: Vec<Condition>,
    pub implementation: SkillImplementation,
}

impl RobotSkill {
    pub fn new(
        id: SkillId,
        name: String,
        parameters: Vec<Parameter>,
        preconditions: Vec<Condition>,
        postconditions: Vec<Condition>,
        implementation: SkillImplementation,
    ) -> Self {
        Self {
            id,
            name,
            parameters,
            preconditions,
            postconditions,
            implementation,
        }
    }
}

/// Registry of available RobotSkills for resolution during lowering.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SkillRegistry {
    global_skills: std::collections::HashMap<SkillId, RobotSkill>,
    robot_skills: std::collections::HashMap<(crate::ids::RobotId, SkillId), RobotSkill>,
}

impl SkillRegistry {
    pub fn new() -> Self {
        Self {
            global_skills: std::collections::HashMap::new(),
            robot_skills: std::collections::HashMap::new(),
        }
    }

    pub fn register(&mut self, skill: RobotSkill) {
        self.global_skills.insert(skill.id.clone(), skill);
    }

    pub fn register_for_robot(&mut self, robot: crate::ids::RobotId, skill: RobotSkill) {
        self.robot_skills.insert((robot, skill.id.clone()), skill);
    }

    pub fn get(&self, id: &SkillId) -> Option<&RobotSkill> {
        self.global_skills.get(id)
    }

    pub fn get_for_robot(&self, robot: &crate::ids::RobotId, id: &SkillId) -> Option<&RobotSkill> {
        self.robot_skills
            .get(&(robot.clone(), id.clone()))
            .or_else(|| self.global_skills.get(id))
    }

    pub fn contains(&self, id: &SkillId) -> bool {
        self.global_skills.contains_key(id) || self.robot_skills.keys().any(|(_, s)| s == id)
    }

    pub fn contains_for_robot(&self, robot: &crate::ids::RobotId, id: &SkillId) -> bool {
        self.robot_skills.contains_key(&(robot.clone(), id.clone()))
            || self.global_skills.contains_key(id)
    }

    pub fn is_empty(&self) -> bool {
        self.global_skills.is_empty() && self.robot_skills.is_empty()
    }

    pub fn len(&self) -> usize {
        self.global_skills.len() + self.robot_skills.len()
    }
}

