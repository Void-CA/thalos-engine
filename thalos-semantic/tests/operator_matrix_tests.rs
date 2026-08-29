use thalos_lang::ast::{BinaryOp, Expr, Statement};
use thalos_semantic::builtins::register_builtins;
use thalos_semantic::checker::TypeChecker;
use thalos_semantic::operators::BinaryOpRule;
use thalos_semantic::scope::SymbolTable;
use thalos_semantic::symbols::{Symbol, SymbolKind};
use thalos_semantic::types::Type;

#[test]
fn test_operator_matrix_rules() {
    // Position + Vector3 -> Position
    assert_eq!(
        BinaryOpRule::infer(&Type::Position, BinaryOp::Add, &Type::Vector3).unwrap(),
        Type::Position
    );

    // Position - Position -> Vector3
    assert_eq!(
        BinaryOpRule::infer(&Type::Position, BinaryOp::Sub, &Type::Position).unwrap(),
        Type::Vector3
    );

    // Pose * Transform3D -> Pose
    assert_eq!(
        BinaryOpRule::infer(&Type::Pose, BinaryOp::Mul, &Type::Transform3D).unwrap(),
        Type::Pose
    );

    // Quaternion * Vector3 -> Vector3
    assert_eq!(
        BinaryOpRule::infer(&Type::Quaternion, BinaryOp::Mul, &Type::Vector3).unwrap(),
        Type::Vector3
    );

    // Negative tests: Position + Position is Error
    assert!(BinaryOpRule::infer(&Type::Position, BinaryOp::Add, &Type::Position).is_err());

    // Negative tests: Pose + Pose is Error
    assert!(BinaryOpRule::infer(&Type::Pose, BinaryOp::Add, &Type::Pose).is_err());
}

#[test]
fn test_type_checker_end_to_end() {
    let mut table = SymbolTable::new();
    register_builtins(&mut table);

    // Declare pick = Position, joints1 = Joints
    table.declare(Symbol::new("pick", SymbolKind::Target, Type::Position, None)).unwrap();
    table.declare(Symbol::new("q_home", SymbolKind::Target, Type::Joints { dimension: Some(6) }, None)).unwrap();

    let mut checker = TypeChecker::new(&mut table);

    // Valid: movej(q_home)
    checker.check_statement(&Statement::MoveJ {
        target: Expr::Identifier("q_home".to_string()),
    });
    assert!(checker.diagnostics.is_empty());

    // Valid: movel(pick)
    checker.check_statement(&Statement::MoveL {
        target: Expr::Identifier("pick".to_string()),
    });
    assert!(checker.diagnostics.is_empty());

    // Invalid: movel(q_home) -> Should generate semantic diagnostic!
    checker.check_statement(&Statement::MoveL {
        target: Expr::Identifier("q_home".to_string()),
    });
    assert_eq!(checker.diagnostics.len(), 1);
    assert!(checker.diagnostics[0].message.contains("movel expected a spatial target"));
}
