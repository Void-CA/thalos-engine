use std::time::Duration;
use thalos_core::prelude::*;
use thalos_semantic::{
    knowledge::{LoweringError, MockKnowledgeProvider},
    lowering::{LoweringContext, SemanticLowering},
    script::parse,
};

#[test]
fn same_program_two_robots_different_execution_plans() {
    let dsl = r#"
        program pick_task(robot = scara) {
            target approach_target = cartesian(x = 100mm, y = 50mm, z = 10mm)
            pick(part_01)
        }
    "#;

    // 1. Parse DSL into pure RobotProgram AST
    let program_a = parse(dsl).expect("parse failed");
    let program_b = parse(dsl).expect("parse failed");

    // Both ASTs are identical
    assert_eq!(program_a, program_b);

    // 2. Define Robot 1 (SCARA-01) with Pick Implementation A (Wait 100ms)
    let robot_scara_01 = RobotDefinition::new(
        RobotId("scara_01".into()),
        "SCARA Robot 01",
        None,
        vec![],
        RobotCapability::default(),
        vec![SkillCapability::new(SkillId("pick".into()))],
    );

    let pick_skill_a = RobotSkill::new(
        SkillId("pick".into()),
        "Pick A".into(),
        vec![],
        vec![],
        vec![],
        SkillImplementation::Program(ProgramFragment {
            instructions: vec![Instruction::Control(ControlInstruction::Wait {
                duration: Duration::from_millis(100),
            })],
        }),
    );

    // 3. Define Robot 2 (SCARA-02) with Pick Implementation B (SetSignal "gripper_close")
    let robot_scara_02 = RobotDefinition::new(
        RobotId("scara_02".into()),
        "SCARA Robot 02",
        None,
        vec![],
        RobotCapability::default(),
        vec![SkillCapability::new(SkillId("pick".into()))],
    );

    let pick_skill_b = RobotSkill::new(
        SkillId("pick".into()),
        "Pick B".into(),
        vec![],
        vec![],
        vec![],
        SkillImplementation::Program(ProgramFragment {
            instructions: vec![Instruction::Control(ControlInstruction::SetSignal {
                signal_id: "gripper_close".into(),
                value: true,
            })],
        }),
    );

    let mut registry = SkillRegistry::new();
    registry.register_for_robot(robot_scara_01.id.clone(), pick_skill_a);
    registry.register_for_robot(robot_scara_02.id.clone(), pick_skill_b);

    let provider = MockKnowledgeProvider::new();

    // 4. Normalize and Lower for SCARA-01
    let ir_a = thalos_semantic::ir::normalize(&program_a).expect("normalize failed");
    let ctx_a = LoweringContext::new(&provider)
        .with_robot(&robot_scara_01)
        .with_skills(&registry);
    let plan_a = SemanticLowering::lower(&ir_a, &ctx_a).expect("lowering A failed");

    // 5. Normalize and Lower for SCARA-02
    let ir_b = thalos_semantic::ir::normalize(&program_b).expect("normalize failed");
    let ctx_b = LoweringContext::new(&provider)
        .with_robot(&robot_scara_02)
        .with_skills(&registry);
    let plan_b = SemanticLowering::lower(&ir_b, &ctx_b).expect("lowering B failed");

    // 6. Verify architectural invariants
    // Program ASTs are equal
    assert_eq!(program_a, program_b);
    // Emitted execution plans differ according to robot capability implementations!
    assert_ne!(plan_a.instructions, plan_b.instructions);
}

#[test]
fn unsupported_skill_error_when_robot_lacks_capability() {
    let dsl = r#"
        program laser_weld(robot = gantry) {
            target point_a = cartesian(x = 0mm, y = 0mm, z = 0mm)
            weld_seam(point_a)
        }
    "#;

    let program = parse(dsl).expect("parse failed");
    let ir = thalos_semantic::ir::normalize(&program).expect("normalize failed");

    // Robot does NOT declare "weld_seam" capability
    let gantry_robot = RobotDefinition::new(
        RobotId("gantry_01".into()),
        "Gantry Robot",
        None,
        vec![],
        RobotCapability::default(),
        vec![SkillCapability::new(SkillId("pick".into()))],
    );

    let registry = SkillRegistry::new();
    let provider = MockKnowledgeProvider::new();
    let ctx = LoweringContext::new(&provider)
        .with_robot(&gantry_robot)
        .with_skills(&registry);

    let result = SemanticLowering::lower(&ir, &ctx);
    assert_eq!(
        result.unwrap_err(),
        LoweringError::UnsupportedSkill(SkillId("weld_seam".into()))
    );
}

#[test]
fn missing_skill_implementation_error_when_capability_declared_but_not_registered() {
    let dsl = r#"
        program inspect_task(robot = cobot) {
            target inspect_point = cartesian(x = 10mm, y = 10mm, z = 10mm)
            inspect_surface(inspect_point)
        }
    "#;

    let program = parse(dsl).expect("parse failed");
    let ir = thalos_semantic::ir::normalize(&program).expect("normalize failed");

    // Cobot declares "inspect_surface" capability, but SkillRegistry has no implementation for it
    let cobot = RobotDefinition::new(
        RobotId("cobot_01".into()),
        "Cobot 01",
        None,
        vec![],
        RobotCapability::default(),
        vec![SkillCapability::new(SkillId("inspect_surface".into()))],
    );

    let registry = SkillRegistry::new();
    let provider = MockKnowledgeProvider::new();
    let ctx = LoweringContext::new(&provider)
        .with_robot(&cobot)
        .with_skills(&registry);

    let result = SemanticLowering::lower(&ir, &ctx);
    assert_eq!(
        result.unwrap_err(),
        LoweringError::MissingSkillImplementation(SkillId("inspect_surface".into()))
    );
}
