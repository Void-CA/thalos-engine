use thalos_lang::ast::{Expr, FnDecl, Item, Param, Program, Statement, TargetDecl};
use thalos_lang::units::{DurationSeconds, LengthMeters};
use thalos_semantic::compiler::SemanticCompiler;
use thalos_semantic::model::{MotionKind, MotionTarget, ResolvedStatement};
use thalos_semantic::resolver::SemanticResolver;
use thalos_math::Vector3;

#[test]
fn test_parametric_function_resolution_and_provenance() {
    // target home = position([400mm, 0mm, 300mm])
    let home_decl = Item::Target(TargetDecl {
        name: "home".to_string(),
        pose: Expr::Call {
            callee: "position".to_string(),
            args: vec![Expr::Vector3([
                Box::new(Expr::Length(LengthMeters(0.400))),
                Box::new(Expr::Length(LengthMeters(0.0))),
                Box::new(Expr::Length(LengthMeters(0.300))),
            ])],
        },
    });

    // fn pick(target) { movej(target); }
    let pick_fn = Item::Function(FnDecl {
        name: "pick".to_string(),
        params: vec![Param {
            name: "target".to_string(),
            type_ann: None,
        }],
        return_type: None,
        body: vec![Statement::MoveJ {
            target: Expr::Identifier("target".to_string()),
        }],
        tail_expr: None,
    });

    // fn main() { pick(home); wait(200ms); }
    let main_fn = Item::Function(FnDecl {
        name: "main".to_string(),
        params: vec![],
        return_type: None,
        body: vec![
            Statement::Expr(Expr::Call {
                callee: "pick".to_string(),
                args: vec![Expr::Identifier("home".to_string())],
            }),
            Statement::Wait(Expr::Duration(DurationSeconds(0.2))),
        ],
        tail_expr: None,
    });

    let ast_program = Program {
        items: vec![home_decl, pick_fn, main_fn],
    };

    // 1. Compile AST -> SemanticProgram
    let sem_program = SemanticCompiler::compile(&ast_program).expect("Compile should succeed");

    // 2. Resolve SemanticProgram -> ResolvedProgram
    let resolved1 = SemanticResolver::resolve(&sem_program).expect("Resolve should succeed");
    let resolved2 = SemanticResolver::resolve(&sem_program).expect("Resolve should succeed");

    // Determinism test
    assert_eq!(resolved1, resolved2);

    // Statements assertion
    assert_eq!(resolved1.statements.len(), 2);

    // Verify resolved motion target
    if let ResolvedStatement::Motion(ref m) = resolved1.statements[0] {
        assert_eq!(m.kind, MotionKind::MoveJ);
        if let MotionTarget::Position(ref p) = m.target {
            assert_eq!(p.point, Vector3::new(0.400, 0.0, 0.300));
        } else {
            panic!("Expected Position target");
        }

        // Verify call_stack trace in Provenance
        assert_eq!(m.provenance.call_stack.len(), 2);
        assert_eq!(m.provenance.call_stack[0].function, "main");
        assert_eq!(m.provenance.call_stack[1].function, "pick");
    } else {
        panic!("Expected Motion statement");
    }

    // Verify resolved wait statement
    if let ResolvedStatement::Wait { seconds, ref provenance } = resolved1.statements[1] {
        assert_eq!(seconds, 0.2);
        assert_eq!(provenance.call_stack.len(), 1);
        assert_eq!(provenance.call_stack[0].function, "main");
    } else {
        panic!("Expected Wait statement");
    }
}
