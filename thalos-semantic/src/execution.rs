use thalos_core::ids::{OperationId, ProgramName, RobotId, SkillId};
use thalos_core::robot::state::RobotState;
use thalos_core::skill::{
    ConditionEvaluator, SkillContract, SkillEvaluationResult, SkillRegistry,
};
use crate::ir::SemanticIr;
use crate::knowledge::LoweringError;
use crate::lowering::{ExecutionProgram, LoweringContext, SemanticLowering};
use crate::operation::{SemanticOperation, SkillCallOp};

/// Result of compiling/lowering a skill implementation prior to physical execution.
#[derive(Debug, Clone, PartialEq)]
pub enum ExecutionOutcome {
    /// Program was successfully compiled/lowered and executed by runtime.
    Success(ExecutionProgram),
    /// Execution or lowering failed prior to completion (e.g. IK failure, collision).
    Failure(LoweringError),
}

/// Execution and evaluation record for a single skill invocation within the runtime (Fase 2.5d).
#[derive(Debug, Clone, PartialEq)]
pub struct SkillExecutionRecord {
    pub skill: SkillId,
    pub origin: OperationId,
    pub precondition_result: SkillEvaluationResult,
    pub execution_outcome: Option<ExecutionOutcome>,
    pub postcondition_result: Option<SkillEvaluationResult>,
}

impl SkillExecutionRecord {
    /// Returns true if preconditions passed, execution completed successfully, and postconditions passed.
    pub fn is_success(&self) -> bool {
        self.precondition_result == SkillEvaluationResult::Success
            && matches!(self.execution_outcome, Some(ExecutionOutcome::Success(_)))
            && self.postcondition_result == Some(SkillEvaluationResult::Success)
    }

    /// Returns true if lowering or execution failed (e.g., IK/kinematic error). NOT a semantic deviation!
    pub fn is_execution_failure(&self) -> bool {
        matches!(self.execution_outcome, Some(ExecutionOutcome::Failure(_)))
    }

    /// Returns true if execution completed cleanly, BUT postconditions failed (e.g. part dropped).
    /// This is the precise definition of a **Semantic Deviation**.
    pub fn is_semantic_deviation(&self) -> bool {
        self.precondition_result == SkillEvaluationResult::Success
            && matches!(self.execution_outcome, Some(ExecutionOutcome::Success(_)))
            && matches!(
                self.postcondition_result,
                Some(SkillEvaluationResult::PostconditionViolation(_))
            )
    }
}

/// A skill call that has passed preconditions and been successfully lowered, ready for execution & post-eval.
#[derive(Debug, Clone)]
pub struct SkillExecutionPrepared {
    pub skill: SkillId,
    pub origin: OperationId,
    pub contract: SkillContract,
    pub lowered_program: ExecutionProgram,
}

/// Orchestrator for skill execution with explicit 3-phase lifecycle (Phase 2.5d):
///
/// 1. `prepare` (resolve skill, check preconditions, lower program)
/// 2. `execute` (dispatch ExecutionProgram to motion/runtime)
/// 3. `evaluate_postconditions` (evaluate postconditions against observed RobotState)
pub struct SkillExecutionEngine;

impl SkillExecutionEngine {
    /// Phase 1: Prepare skill execution by checking preconditions and lowering implementation.
    ///
    /// If preconditions fail or lowering fails, returns `Err(SkillExecutionRecord)` containing the early failure.
    pub fn prepare_skill_op(
        skill_op: &SkillCallOp,
        robot: &RobotId,
        registry: &SkillRegistry,
        evaluator: &impl ConditionEvaluator,
        pre_state: &RobotState,
        ctx: &LoweringContext,
    ) -> Result<SkillExecutionPrepared, SkillExecutionRecord> {
        let skill_id = &skill_op.skill_call.skill;
        let skill = match registry.get_for_robot(robot, skill_id) {
            Some(s) => s,
            None => {
                return Err(SkillExecutionRecord {
                    skill: skill_id.clone(),
                    origin: skill_op.origin.clone(),
                    precondition_result: SkillEvaluationResult::UnknownState(
                        thalos_core::skill::Condition::Custom {
                            identifier: "skill_found".into(),
                            expected_value: "true".into(),
                        },
                    ),
                    execution_outcome: None,
                    postcondition_result: None,
                });
            }
        };

        let contract = skill.contract();

        // 1. Check Preconditions against pre_state
        let precondition_result = contract.evaluate_preconditions(evaluator, pre_state);
        if precondition_result != SkillEvaluationResult::Success {
            return Err(SkillExecutionRecord {
                skill: skill_id.clone(),
                origin: skill_op.origin.clone(),
                precondition_result,
                execution_outcome: None,
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

        match SemanticLowering::lower(&single_op_ir, ctx) {
            Ok(lowered_program) => Ok(SkillExecutionPrepared {
                skill: skill_id.clone(),
                origin: skill_op.origin.clone(),
                contract,
                lowered_program,
            }),
            Err(err) => Err(SkillExecutionRecord {
                skill: skill_id.clone(),
                origin: skill_op.origin.clone(),
                precondition_result: SkillEvaluationResult::Success,
                execution_outcome: Some(ExecutionOutcome::Failure(err)),
                postcondition_result: None,
            }),
        }
    }

    /// Phase 3: Evaluate postconditions against post-execution observation (`post_state`).
    pub fn evaluate_postconditions(
        prepared: SkillExecutionPrepared,
        evaluator: &impl ConditionEvaluator,
        post_state: &RobotState,
    ) -> SkillExecutionRecord {
        let postcondition_result = prepared
            .contract
            .evaluate_postconditions(evaluator, post_state);

        SkillExecutionRecord {
            skill: prepared.skill,
            origin: prepared.origin,
            precondition_result: SkillEvaluationResult::Success,
            execution_outcome: Some(ExecutionOutcome::Success(prepared.lowered_program)),
            postcondition_result: Some(postcondition_result),
        }
    }

    /// Full 3-phase execution pipeline for a single `SkillCallOp`.
    pub fn execute_skill_op(
        skill_op: &SkillCallOp,
        robot: &RobotId,
        registry: &SkillRegistry,
        evaluator: &impl ConditionEvaluator,
        pre_state: &RobotState,
        post_state: &RobotState,
        ctx: &LoweringContext,
    ) -> SkillExecutionRecord {
        match Self::prepare_skill_op(skill_op, robot, registry, evaluator, pre_state, ctx) {
            Ok(prepared) => Self::evaluate_postconditions(prepared, evaluator, post_state),
            Err(early_record) => early_record,
        }
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
                );
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
    fn test_1_precondition_failure_skips_lowering_and_execution() {
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
        // Lowering, execution, and postconditions NEVER ran
        assert!(rec.execution_outcome.is_none());
        assert!(rec.postcondition_result.is_none());
    }

    #[test]
    fn test_2_execution_failure_does_not_count_as_semantic_deviation() {
        let (program, registry) = setup_pick_program_and_registry();
        let ir = crate::ir::normalize(&program).unwrap();

        // Empty knowledge provider without grasp plan -> Lowering Error
        let provider = MockKnowledgeProvider::new();
        let ctx = LoweringContext::new(&provider).with_skills(&registry);

        let pre_state = RobotState::zero(6);
        let post_state = RobotState::zero(6);

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
        assert!(rec.is_execution_failure(), "IK/lowering error is an Execution Failure");
        assert!(!rec.is_semantic_deviation(), "Execution failure is NOT a semantic deviation");
        assert_eq!(rec.precondition_result, SkillEvaluationResult::Success);
        assert!(rec.postcondition_result.is_none(), "Postcondition never evaluated because execution failed");
    }

    #[test]
    fn test_3_execution_success_and_postcondition_failure_is_semantic_deviation() {
        let (program, registry) = setup_pick_program_and_registry();
        let ir = crate::ir::normalize(&program).unwrap();

        let provider = mock_provider();
        let ctx = LoweringContext::new(&provider).with_skills(&registry);

        let pre_state = RobotState::zero(6);
        let post_state = RobotState::zero(6);

        // Precondition is satisfied, execution succeeds, but postcondition fails (part slipped)
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
        assert!(rec.is_semantic_deviation(), "Execution succeeded but postcondition failed = Semantic Deviation");
        assert_eq!(rec.precondition_result, SkillEvaluationResult::Success);
        assert!(matches!(rec.execution_outcome, Some(ExecutionOutcome::Success(_))));
        assert_eq!(
            rec.postcondition_result,
            Some(SkillEvaluationResult::PostconditionViolation(Condition::ObjectAttached(
                ObjectId("part_01".into())
            )))
        );
    }

    #[test]
    fn test_4_execution_success_and_postcondition_success() {
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
        assert!(!rec.is_execution_failure());
        assert!(!rec.is_semantic_deviation());
        assert_eq!(rec.precondition_result, SkillEvaluationResult::Success);
        assert_eq!(rec.postcondition_result, Some(SkillEvaluationResult::Success));
    }
}
