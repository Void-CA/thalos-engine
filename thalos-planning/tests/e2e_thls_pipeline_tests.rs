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

#[test]
fn test_e2e_boundary_semantic_validity_vs_physical_feasibility() {
    use thalos_core::kinematics::forward::ForwardKinematics;
    use thalos_core::kinematics::inverse::DampedLeastSquaresSolver;
    use thalos_core::models::{RobotModel, RobotRegistry};
    use thalos_core::robot::state::RobotState;
    use thalos_planning::error::PlanningError;
    use thalos_planning::motion::compiler::{DefaultPlannerDispatcher, PlanCompiler};
    use thalos_planning::motion::planner::SegmentPlanningContext;

    let chain = RobotRegistry::create_default(RobotModel::Scara);
    let state = RobotState::zero(chain.dof_count());
    let fk = ForwardKinematics::new(chain.clone());
    let ik_solver = DampedLeastSquaresSolver::new(fk, *chain.end_effector(), 500, 1e-6, 0.1);
    let ctx = SegmentPlanningContext {
        robot: &chain,
        current_state: &state,
        ik_solver: &ik_solver,
        tcp: None,
    };
    let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));

    // Case A: Semantically valid AND physically feasible target
    let source_feasible = "
    target reachable = position([1200mm, 300mm, 300mm])
    fn main() {
        movej(reachable)
    }
    ";
    let ast_a = parse_source(source_feasible).expect("Parser must accept reachable script");
    let sem_a = SemanticCompiler::compile(&ast_a).expect("Semantic compiler must accept reachable script");
    let res_a = SemanticResolver::resolve(&sem_a).expect("Resolver must resolve reachable script");
    let input_a = PlanningInput::from_resolved(&res_a);
    let plan_a = compiler
        .compile(&input_a.to_program(), &ctx)
        .expect("PlanCompiler must plan reachable target");
    assert_eq!(plan_a.segments.len(), 1);

    // Case B: Semantically valid BUT physically unreachable target (5000mm, 5000mm, 5000mm)
    let source_unreachable = "
    target unreachable = position([5000mm, 5000mm, 5000mm])
    fn main() {
        movej(unreachable)
    }
    ";
    let ast_b = parse_source(source_unreachable).expect("1. Parser MUST accept unreachable script");
    let sem_b = SemanticCompiler::compile(&ast_b).expect("2. Semantic compiler MUST accept unreachable script");
    let res_b = SemanticResolver::resolve(&sem_b).expect("3. Semantic resolver MUST resolve unreachable script");
    let input_b = PlanningInput::from_resolved(&res_b);
    let err_b = compiler
        .compile(&input_b.to_program(), &ctx)
        .expect_err("4. PlanCompiler MUST reject unreachable target");

    assert_eq!(err_b.segment_index, 0, "Error must pinpoint segment index 0");
    assert!(
        matches!(
            err_b.source,
            PlanningError::IkFailedPosition { .. }
                | PlanningError::IkFailed { .. }
                | PlanningError::Ik(_)
        ),
        "Error must be an IK failure from physical kinematics, got {:?}",
        err_b.source
    );
}

#[test]
fn test_e2e_canonical_program_pipeline() {
    use thalos_core::kinematics::forward::ForwardKinematics;
    use thalos_core::kinematics::inverse::DampedLeastSquaresSolver;
    use thalos_core::models::{RobotModel, RobotRegistry};
    use thalos_core::robot::state::RobotState;
    use thalos_planning::motion::compiler::{DefaultPlannerDispatcher, PlanCompiler};
    use thalos_planning::motion::planner::SegmentPlanningContext;

    let source = "
    const APPROACH = [0mm, 0mm, 100mm]

    target home = position([1800mm, 0mm, 500mm])
    target pick = position([1500mm, 300mm, 350mm])

    fn approach(p: Position) -> Position {
        p + APPROACH
    }

    fn pick_part(p: Position) {
        movej(home)
        movel(approach(p))
        movel(p)
        wait(150ms)
        movel(approach(p))
    }

    fn main() {
        pick_part(pick);
    }
    ";

    let ast = parse_source(source).expect("1. Parser MUST accept canonical script");
    let sem = SemanticCompiler::compile(&ast).expect("2. Semantic compiler MUST accept canonical script");
    assert_eq!(sem.targets.len(), 2);
    assert_eq!(sem.functions.len(), 3);

    let res = SemanticResolver::resolve(&sem).expect("3. Semantic resolver MUST resolve canonical script");
    let input = PlanningInput::from_resolved(&res);
    assert_eq!(input.motions.len(), 4); // movej(home), movel(approach(pick)), movel(pick), movel(approach(pick))

    let chain = RobotRegistry::create_default(RobotModel::Scara);
    let state = RobotState::zero(chain.dof_count());
    let fk = ForwardKinematics::new(chain.clone());
    let ik_solver = DampedLeastSquaresSolver::new(fk, *chain.end_effector(), 500, 1e-6, 0.1);
    let ctx = SegmentPlanningContext {
        robot: &chain,
        current_state: &state,
        ik_solver: &ik_solver,
        tcp: None,
    };
    let compiler = PlanCompiler::new(Box::new(DefaultPlannerDispatcher::default()));

    let plan = compiler
        .compile(&input.to_program(), &ctx)
        .expect("5. PlanCompiler MUST compile canonical program into PlannedProgram");
    assert_eq!(plan.segments.len(), 4);
}

#[test]
fn test_effect_purity_rejection_builtin_motion_in_pure_fn() {
    let source = "
    fn invalid(p: Position) -> Position {
        movej(p)
        p
    }
    ";

    let ast = parse_source(source).expect("AST parsing should succeed");
    let compile_res = SemanticCompiler::compile(&ast);
    assert!(compile_res.is_err(), "Pure function with movej MUST be rejected");
    let errors = compile_res.unwrap_err();
    assert!(
        errors.iter().any(|e| e.contains("cannot produce robotic/IO effects")),
        "Error message should mention robotic/IO effects prohibition, got {:?}",
        errors
    );
}

#[test]
fn test_effect_purity_rejection_routine_call_in_pure_fn() {
    let source = "
    fn close_gripper() {
        wait(150ms);
    }

    fn invalid(p: Position) -> Position {
        close_gripper();
        p + [0mm, 0mm, 100mm]
    }
    ";

    let ast = parse_source(source).expect("AST parsing should succeed");
    let compile_res = SemanticCompiler::compile(&ast);
    assert!(compile_res.is_err(), "Pure function with routine call MUST be rejected");
    let errors = compile_res.unwrap_err();
    assert!(
        errors.iter().any(|e| e.contains("cannot produce robotic/IO effects")),
        "Error message should mention robotic/IO effects prohibition, got {:?}",
        errors
    );
}
