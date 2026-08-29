use std::fs;
use thalos_lang::parse_source;
use thalos_math::Vector3;
use thalos_planning::input::PlanningInput;
use thalos_semantic::compiler::SemanticCompiler;
use thalos_semantic::model::{MotionKind, MotionTarget};
use thalos_semantic::resolver::SemanticResolver;

#[test]
fn test_e2e_thls_pipeline_from_fixture() {
    let fixture_path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../tests/fixtures/basic_motion.thls"
    );
    let source = fs::read_to_string(fixture_path).expect("Fixture file should exist");

    // 1. Parse .thls -> AST
    let ast = parse_source(&source).expect("Parsing .thls source should succeed");

    // 2. Compile AST -> SemanticProgram
    let sem_program = SemanticCompiler::compile(&ast).expect("Semantic compilation should succeed");
    assert_eq!(sem_program.targets.len(), 2);
    assert_eq!(sem_program.functions.len(), 1);

    // 3. Resolve SemanticProgram -> ResolvedProgram
    let resolved = SemanticResolver::resolve(&sem_program).expect("Resolution should succeed");

    // 4. Lower ResolvedProgram -> PlanningInput
    let planning_input = PlanningInput::from_resolved(&resolved);

    // 5. Assert PlanningInput integrity and provenance
    assert_eq!(planning_input.motions.len(), 3);

    // Motion #1: movej(home) -> Joints([0, 0, 0])
    let m1 = &planning_input.motions[0];
    assert_eq!(m1.kind, MotionKind::MoveJ);
    if let MotionTarget::Joints(ref j) = m1.target {
        assert_eq!(j.values, vec![0.0, 0.0, 0.0]);
    } else {
        panic!("Expected Joints target for home");
    }
    assert_eq!(m1.provenance.source_name, Some("home".to_string()));
    assert_eq!(m1.provenance.call_stack.len(), 1);
    assert_eq!(m1.provenance.call_stack[0].function, "main");

    // Motion #2: movej(pick) -> Position([0.420, 0.180, 0.080])
    let m2 = &planning_input.motions[1];
    assert_eq!(m2.kind, MotionKind::MoveJ);
    if let MotionTarget::Position(ref p) = m2.target {
        assert_eq!(p.point, Vector3::new(0.420, 0.180, 0.080));
    } else {
        panic!("Expected Position target for pick");
    }
    assert_eq!(m2.provenance.source_name, Some("pick".to_string()));
    assert_eq!(m2.provenance.call_stack.len(), 1);
    assert_eq!(m2.provenance.call_stack[0].function, "main");

    // Motion #3: movel(pick + [0mm, 0mm, 100mm]) -> Position([0.420, 0.180, 0.180])
    let m3 = &planning_input.motions[2];
    assert_eq!(m3.kind, MotionKind::MoveL);
    if let MotionTarget::Position(ref p) = m3.target {
        assert_eq!(p.point, Vector3::new(0.420, 0.180, 0.180));
    } else {
        panic!("Expected Position target for pick + offset");
    }
    assert_eq!(m3.provenance.call_stack.len(), 1);
    assert_eq!(m3.provenance.call_stack[0].function, "main");
}

#[test]
fn test_e2e_rejection_guard_movel_on_joints() {
    // movel(joints(...)) is invalid and must be rejected in thalos-semantic before reaching PlanningInput
    let source = "
    fn main() {
        movel(joints([0deg, 0deg, 0deg]))
    }
    ";

    let ast = parse_source(source).expect("AST parsing should succeed");

    let compile_res = SemanticCompiler::compile(&ast);
    assert!(compile_res.is_err(), "movel on joints MUST be rejected in thalos-semantic");
    let errors = compile_res.unwrap_err();
    assert!(
        errors.iter().any(|e| e.contains("movel expected a spatial target")),
        "Error message should clearly state movel spatial target requirement"
    );
}
