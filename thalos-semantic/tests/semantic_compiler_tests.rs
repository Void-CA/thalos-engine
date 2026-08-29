use thalos_lang::ast::{BinaryOp, Expr, FnDecl, Item, Program, Statement, TargetDecl};
use thalos_lang::units::{DurationSeconds, LengthMeters};
use thalos_semantic::compiler::SemanticCompiler;
use thalos_semantic::model::{MotionKind, MotionTarget, SemanticStatement};
use thalos_math::Vector3;

#[test]
fn test_semantic_compiler_lowering() {
    // 1. Build AST
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

    // target approach = home + [0mm, 0mm, 100mm]
    let approach_decl = Item::Target(TargetDecl {
        name: "approach".to_string(),
        pose: Expr::Binary {
            left: Box::new(Expr::Identifier("home".to_string())),
            op: BinaryOp::Add,
            right: Box::new(Expr::Vector3([
                Box::new(Expr::Length(LengthMeters(0.0))),
                Box::new(Expr::Length(LengthMeters(0.0))),
                Box::new(Expr::Length(LengthMeters(0.100))),
            ])),
        },
    });

    // fn main() { movej(approach); movel(home); wait(500ms); }
    let main_fn = Item::Function(FnDecl {
        name: "main".to_string(),
        params: vec![],
        body: vec![
            Statement::MoveJ {
                target: Expr::Identifier("approach".to_string()),
            },
            Statement::MoveL {
                target: Expr::Identifier("home".to_string()),
            },
            Statement::Wait(Expr::Duration(DurationSeconds(0.5))),
        ],
    });

    let ast_program = Program {
        items: vec![home_decl, approach_decl, main_fn],
    };

    // 2. Compile AST to SemanticProgram
    let sem_program = SemanticCompiler::compile(&ast_program).expect("Compilation should succeed");

    // 3. Assertions
    assert_eq!(sem_program.targets.len(), 2);
    assert_eq!(sem_program.targets[0].name, "home");
    assert_eq!(sem_program.targets[1].name, "approach");

    // Verify constant-folded approach target: position [0.400, 0.0, 0.400]
    if let MotionTarget::Position(ref p) = sem_program.targets[1].value {
        assert_eq!(p.point, Vector3::new(0.400, 0.0, 0.400));
    } else {
        panic!("Expected Position target for approach");
    }

    // Verify main function statements
    assert_eq!(sem_program.functions.len(), 1);
    let main_sem = &sem_program.functions[0];
    assert_eq!(main_sem.name, "main");
    assert_eq!(main_sem.body.len(), 3);

    // Verify MoveJ statement carries source_name provenance
    if let SemanticStatement::Motion(ref m) = main_sem.body[0] {
        assert_eq!(m.kind, MotionKind::MoveJ);
        assert_eq!(m.provenance.source_name, Some("approach".to_string()));
    } else {
        panic!("Expected Motion statement for MoveJ");
    }

    // Verify Wait statement
    if let SemanticStatement::Wait { ref duration, .. } = main_sem.body[2] {
        assert_eq!(
            duration,
            &thalos_semantic::model::SemanticExpr::Constant(
                thalos_semantic::evaluator::CompileTimeValue::Duration(0.5)
            )
        );
    } else {
        panic!("Expected Wait statement");
    }
}
