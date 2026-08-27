use thalos_core::ids::{OperationId, ProgramName, RobotId, SkillId};
use thalos_core::robot::state::RobotState;
use thalos_core::skill::{
    ConditionEvaluator, SkillEvaluationResult, SkillRegistry,
};
use crate::ir::SemanticIr;
use crate::knowledge::LoweringError;
use crate::lowering::{ExecutionProgram, LoweringContext, SemanticLowering};
use crate::operation::{SemanticOperation, SkillCallOp};

/// Execution and evaluation record for a single skill invocation within the runtime.
#[derive(Debug, Clone, PartialEq)]
pub struct SkillExecutionRecord {
    pub skill: SkillId,
    pub origin: OperationId,
    pub precondition_result: SkillEvaluationResult,
    pub lowered_program: Option<ExecutionProgram>,
    pub postcondition_result: Option<SkillEvaluationResult>,
}

impl SkillExecutionRecord {
    /// Returns true if preconditions passed, lowering produced instructions, and postconditions passed.
    pub fn is_success(&self) -> bool {
        self.precondition_result == SkillEvaluationResult::Success
            && self.lowered_program.is_some()
            && self.postcondition_result == Some(SkillEvaluationResult::Success)
    }

    /// Returns true if postconditions failed after successfully lowering and executing motion/IO.
    pub fn is_semantic_deviation(&self) -> bool {
        self.precondition_result == SkillEvaluationResult::Success
            && self.lowered_program.is_some()
            && matches!(
                self.postcondition_result,
                Some(SkillEvaluationResult::PostconditionViolation(_))
            )
    }
}

/// Orchestrator for skill execution and contract evaluation (ADR-001 / Phase 2.5c vertical slice).
///
/// Integrates contract evaluation with program lowering:
/// `SkillCall → resolve RobotSkill → evaluate preconditions → lower ProgramFragment → evaluate postconditions`
pub struct SkillExecutionEngine;

impl SkillExecutionEngine {
    /// Execute and evaluate a single `SkillCallOp`.
    pub fn execute_skill_op(
        skill_op: &SkillCallOp,
        robot: &RobotId,
        registry: &SkillRegistry,
        evaluator: &impl ConditionEvaluator,
        pre_state: &RobotState,
        post_state: &RobotState,
        ctx: &LoweringContext,
    ) -> Result<SkillExecutionRecord, LoweringError> {
        let skill_id = &skill_op.skill_call.skill;
        let skill = registry
            .get_for_robot(robot, skill_id)
            .ok_or_else(|| LoweringError::UnknownSkill(skill_id.clone()))?;

        let contract = skill.contract();

        // 1. Evaluate Preconditions against pre-execution RobotState
        let precondition_result = contract.evaluate_preconditions(evaluator, pre_state);
        if precondition_result != SkillEvaluationResult::Success {
            return Ok(SkillExecutionRecord {
                skill: skill_id.clone(),
                origin: skill_op.origin.clone(),
                precondition_result,
                lowered_program: None,
                postcondition_result: None,
            });
        }

        // 2. Lower Implementation into ExecutionProgram
        let single_op_ir = SemanticIr::new(
            ProgramName("skill_execution".into()),
            robot.clone(),
            vec![],
            vec![SemanticOperation::Skill(skill_op.clone())],
        );
        let lowered_program = SemanticLowering::lower(&single_op_ir, ctx)?;

        // 3. Evaluate Postconditions against post-execution RobotState
        let postcondition_result = contract.evaluate_postconditions(evaluator, post_state);

        Ok(SkillExecutionRecord {
            skill: skill_id.clone(),
            origin: skill_op.origin.clone(),
            precondition_result: SkillEvaluationResult::Success,
            lowered_program: Some(lowered_program),
            postcondition_result: Some(postcondition_result),
        })
    }

    /// Execute and evaluate all skill operations in a `SemanticIr` program.
    pub fn execute_program(
        ir: &SemanticIr,
        evaluator: &impl ConditionEvaluator,
        pre_state: &RobotState,
        post_state: &RobotState,
        ctx: &LoweringContext,
    ) -> Result<Vec<SkillExecutionRecord>, LoweringError> {
        let registry = ctx
            .skills
            .ok_or_else(|| LoweringError::UnknownSkill(SkillId("missing_registry".into())))?;

        let mut records = Vec::new();
        for op in &ir.operations {
            if let SemanticOperation::Skill(skill_op) = op {
                let record = Self::execute_skill_op(
                    skill_op,
                    &ir.robot,
                    registry,
                    evaluator,
                    pre_state,
                    post_state,
                    ctx,
                )?;
                records.push(record);
            }
        }

        Ok(records)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;
    use thalos_core::ids::ObjectId;
    use thalos_core::program::{Instruction, RobotProgram, SkillCall, Value};
    use thalos_core::skill::{Condition, ConditionResult, NativeSkillId, RobotSkill, SkillImplementation};
    use crate::knowledge::MockKnowledgeProvider;

    struct TestTelemetryEvaluator {
        gripper_open: Option<bool>,
        attached_objects: HashSet<ObjectId>,
    }

    impl ConditionEvaluator for TestTelemetryEvaluator {
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

    fn setup_pick_program_and_registry() -> (RobotProgram, SkillRegistry) {
        let mut registry = SkillRegistry::new();
        let part = ObjectId("part_01".into());

        let pick_skill = RobotSkill {
            id: SkillId("pick".into()),
            name: "Pick Part".into(),
            parameters: vec![],
            preconditions: vec![Condition::GripperOpen],
            postconditions: vec![Condition::ObjectAttached(part)],
            implementation: SkillImplementation::Native(NativeSkillId("scara_pick_driver".into())),
        };
        registry.register(pick_skill);

        let program = RobotProgram {
            name: ProgramName("PickTestProgram".into()),
            robot: RobotId("scara_1".into()),
            targets: vec![],
            body: vec![Instruction::Skill(SkillCall {
                skill: SkillId("pick".into()),
                arguments: vec![Value::String("part_01".into())],
            })],
        };

        (program, registry)
    }

    fn mock_provider() -> MockKnowledgeProvider {
        use thalos_core::motion::MotionPose;
        use crate::knowledge::grasp::GraspPlan;

        let pose = MotionPose {
            position: [1.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
            frame: "world".into(),
        };
        let grasp_plan = GraspPlan {
            grasp_frame: pose.clone(),
            approach_frame: pose.clone(),
            retreat_frame: pose,
            preferred_tool: None,
        };
        MockKnowledgeProvider::new().with_grasp_ok(ObjectId("part_01".into()), grasp_plan)
    }

    #[test]
    fn execution_vertical_slice_success() {
        let (program, registry) = setup_pick_program_and_registry();
        let ir = crate::ir::normalize(&program).unwrap();

        let provider = mock_provider();
        let ctx = LoweringContext::new(&provider).with_skills(&registry);

        let pre_state = RobotState::zero(6);
        let post_state = RobotState::zero(6);

        let mut attached = HashSet::new();
        attached.insert(ObjectId("part_01".into()));

        let evaluator = TestTelemetryEvaluator {
            gripper_open: Some(true),
            attached_objects: attached,
        };

        let records = SkillExecutionEngine::execute_program(
            &ir, &evaluator, &pre_state, &post_state, &ctx,
        ).unwrap();

        assert_eq!(records.len(), 1);
        let rec = &records[0];
        assert!(rec.is_success());
        assert_eq!(rec.precondition_result, SkillEvaluationResult::Success);
        assert_eq!(rec.postcondition_result, Some(SkillEvaluationResult::Success));
        assert!(rec.lowered_program.is_some());
    }

    #[test]
    fn execution_vertical_slice_precondition_violation() {
        let (program, registry) = setup_pick_program_and_registry();
        let ir = crate::ir::normalize(&program).unwrap();

        let provider = mock_provider();
        let ctx = LoweringContext::new(&provider).with_skills(&registry);

        let pre_state = RobotState::zero(6);
        let post_state = RobotState::zero(6);

        // Gripper is ALREADY closed prior to pick -> Precondition Violation
        let evaluator = TestTelemetryEvaluator {
            gripper_open: Some(false),
            attached_objects: HashSet::new(),
        };

        let records = SkillExecutionEngine::execute_program(
            &ir, &evaluator, &pre_state, &post_state, &ctx,
        ).unwrap();

        assert_eq!(records.len(), 1);
        let rec = &records[0];
        assert!(!rec.is_success());
        assert_eq!(
            rec.precondition_result,
            SkillEvaluationResult::PreconditionViolation(Condition::GripperOpen)
        );
        // Lowering and execution are SKIPPED
        assert!(rec.lowered_program.is_none());
        assert!(rec.postcondition_result.is_none());
    }

    #[test]
    fn execution_vertical_slice_semantic_deviation_postcondition_violation() {
        let (program, registry) = setup_pick_program_and_registry();
        let ir = crate::ir::normalize(&program).unwrap();

        let provider = mock_provider();
        let ctx = LoweringContext::new(&provider).with_skills(&registry);

        let pre_state = RobotState::zero(6);
        let post_state = RobotState::zero(6);

        // Precondition is satisfied, trajectory lowering/execution succeeds, but postcondition fails (part slipped)
        let evaluator = TestTelemetryEvaluator {
            gripper_open: Some(true),
            attached_objects: HashSet::new(),
        };

        let records = SkillExecutionEngine::execute_program(
            &ir, &evaluator, &pre_state, &post_state, &ctx,
        ).unwrap();

        assert_eq!(records.len(), 1);
        let rec = &records[0];
        assert!(!rec.is_success());
        assert!(rec.is_semantic_deviation());
        assert_eq!(rec.precondition_result, SkillEvaluationResult::Success);
        assert!(rec.lowered_program.is_some(), "ExecutionProgram was generated and executed");
        assert_eq!(
            rec.postcondition_result,
            Some(SkillEvaluationResult::PostconditionViolation(Condition::ObjectAttached(
                ObjectId("part_01".into())
            )))
        );
    }

    #[test]
    fn execution_vertical_slice_unknown_precondition_state() {
        let (program, registry) = setup_pick_program_and_registry();
        let ir = crate::ir::normalize(&program).unwrap();

        let provider = mock_provider();
        let ctx = LoweringContext::new(&provider).with_skills(&registry);

        let pre_state = RobotState::zero(6);
        let post_state = RobotState::zero(6);

        // Telemetry missing for gripper
        let evaluator = TestTelemetryEvaluator {
            gripper_open: None,
            attached_objects: HashSet::new(),
        };

        let records = SkillExecutionEngine::execute_program(
            &ir, &evaluator, &pre_state, &post_state, &ctx,
        ).unwrap();

        assert_eq!(records.len(), 1);
        let rec = &records[0];
        assert!(!rec.is_success());
        assert_eq!(
            rec.precondition_result,
            SkillEvaluationResult::UnknownState(Condition::GripperOpen)
        );
        assert!(rec.lowered_program.is_none());
    }
}
