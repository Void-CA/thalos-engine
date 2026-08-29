use serde::{Deserialize, Serialize};
use crate::ids::SkillId;
use crate::program::instruction::Instruction;

/// Parameter specification for a skill signature.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Parameter {
    pub name: String,
    pub param_type: String,
}

use crate::ids::{ObjectId, TargetId};
use crate::robot::state::RobotState;

/// Pre/post condition specification for skill contracts (ADR-001 / Phase 2.5c).
///
/// Expresses declarative world expectations without coupling to specific hardware sensors.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Condition {
    GripperOpen,
    GripperClosed,
    ObjectAttached(ObjectId),
    AtTarget(TargetId),
    /// Experimental escape hatch for non-standard or external domain conditions.
    ///
    /// Note: `Custom` is an extensibility mechanism, not a core semantic guarantee.
    /// Standard compiler and evaluation workflows should prefer strongly-typed variants.
    Custom {
        identifier: String,
        expected_value: String,
    },
}

/// Result of evaluating a `Condition` against an observed state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConditionResult {
    Satisfied,
    Violated,
    Unknown,
}

/// Evaluator trait for resolving declarative `Condition` instances against `RobotState` or observations.
pub trait ConditionEvaluator {
    fn evaluate(&self, condition: &Condition, state: &RobotState) -> ConditionResult;
}

/// Declarative contract governing a `RobotSkill`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SkillContract {
    pub preconditions: Vec<Condition>,
    pub postconditions: Vec<Condition>,
}

/// Overall result of evaluating a `SkillContract` before or after execution.
#[derive(Debug, Clone, PartialEq)]
pub enum SkillEvaluationResult {
    Success,
    PreconditionViolation(Condition),
    PostconditionViolation(Condition),
    UnknownState(Condition),
}

impl SkillContract {
    pub fn new(preconditions: Vec<Condition>, postconditions: Vec<Condition>) -> Self {
        Self {
            preconditions,
            postconditions,
        }
    }

    pub fn evaluate_preconditions(
        &self,
        evaluator: &impl ConditionEvaluator,
        state: &RobotState,
    ) -> SkillEvaluationResult {
        for cond in &self.preconditions {
            match evaluator.evaluate(cond, state) {
                ConditionResult::Violated => return SkillEvaluationResult::PreconditionViolation(cond.clone()),
                ConditionResult::Unknown => return SkillEvaluationResult::UnknownState(cond.clone()),
                ConditionResult::Satisfied => {}
            }
        }
        SkillEvaluationResult::Success
    }

    pub fn evaluate_postconditions(
        &self,
        evaluator: &impl ConditionEvaluator,
        state: &RobotState,
    ) -> SkillEvaluationResult {
        for cond in &self.postconditions {
            match evaluator.evaluate(cond, state) {
                ConditionResult::Violated => return SkillEvaluationResult::PostconditionViolation(cond.clone()),
                ConditionResult::Unknown => return SkillEvaluationResult::UnknownState(cond.clone()),
                ConditionResult::Satisfied => {}
            }
        }
        SkillEvaluationResult::Success
    }
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

/// Declarative functional capability of a robot declaring support for a specific skill.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SkillCapability {
    pub skill: SkillId,
}

impl SkillCapability {
    pub fn new(skill: SkillId) -> Self {
        Self { skill }
    }
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

    pub fn contract(&self) -> SkillContract {
        SkillContract::new(self.preconditions.clone(), self.postconditions.clone())
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::ObjectId;

    struct MockTelemetryEvaluator {
        gripper_open: Option<bool>,
        attached_objects: std::collections::HashSet<ObjectId>,
    }

    impl ConditionEvaluator for MockTelemetryEvaluator {
        fn evaluate(&self, condition: &Condition, _state: &RobotState) -> ConditionResult {
            match condition {
                Condition::GripperOpen => match self.gripper_open {
                    Some(true) => ConditionResult::Satisfied,
                    Some(false) => ConditionResult::Violated,
                    None => ConditionResult::Unknown,
                },
                Condition::GripperClosed => match self.gripper_open {
                    Some(false) => ConditionResult::Satisfied,
                    Some(true) => ConditionResult::Violated,
                    None => ConditionResult::Unknown,
                },
                Condition::ObjectAttached(obj) => {
                    if self.attached_objects.contains(obj) {
                        ConditionResult::Satisfied
                    } else {
                        ConditionResult::Violated
                    }
                }
                Condition::AtTarget(_) => ConditionResult::Unknown,
                Condition::Custom { .. } => ConditionResult::Unknown,
            }
        }
    }

    #[test]
    fn contract_scenario_1_successful_execution() {
        let contract = SkillContract::new(
            vec![Condition::GripperOpen],
            vec![Condition::ObjectAttached(ObjectId("part_01".into()))],
        );

        let dummy_state = RobotState::zero(6);
        let mut attached = std::collections::HashSet::new();
        attached.insert(ObjectId("part_01".into()));

        let evaluator = MockTelemetryEvaluator {
            gripper_open: Some(true),
            attached_objects: attached,
        };

        assert_eq!(
            contract.evaluate_preconditions(&evaluator, &dummy_state),
            SkillEvaluationResult::Success
        );
        assert_eq!(
            contract.evaluate_postconditions(&evaluator, &dummy_state),
            SkillEvaluationResult::Success
        );
    }

    #[test]
    fn contract_scenario_2_semantic_postcondition_violation() {
        let part = ObjectId("part_01".into());
        let contract = SkillContract::new(
            vec![Condition::GripperOpen],
            vec![Condition::ObjectAttached(part.clone())],
        );

        let dummy_state = RobotState::zero(6);
        // Trajectories succeeded, but gripper failed to grasp object (attached_objects is empty)
        let evaluator = MockTelemetryEvaluator {
            gripper_open: Some(true),
            attached_objects: std::collections::HashSet::new(),
        };

        assert_eq!(
            contract.evaluate_preconditions(&evaluator, &dummy_state),
            SkillEvaluationResult::Success
        );
        assert_eq!(
            contract.evaluate_postconditions(&evaluator, &dummy_state),
            SkillEvaluationResult::PostconditionViolation(Condition::ObjectAttached(part))
        );
    }

    #[test]
    fn contract_scenario_3_insufficient_information_unknown_precondition() {
        let contract = SkillContract::new(
            vec![Condition::GripperOpen],
            vec![Condition::ObjectAttached(ObjectId("part_01".into()))],
        );

        let dummy_state = RobotState::zero(6);
        // No telemetry received for gripper state
        let evaluator = MockTelemetryEvaluator {
            gripper_open: None,
            attached_objects: std::collections::HashSet::new(),
        };

        assert_eq!(
            contract.evaluate_preconditions(&evaluator, &dummy_state),
            SkillEvaluationResult::UnknownState(Condition::GripperOpen)
        );
    }
}

