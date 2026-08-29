use thalos_lang::ast::{BinaryOp, Expr};
use thalos_lang::units::LengthMeters;
use thalos_semantic::builtins::register_builtins;
use thalos_semantic::evaluator::{CompileTimeValue, EvalResult, Evaluator, Position};
use thalos_semantic::scope::SymbolTable;
use thalos_math::Vector3;

#[test]
fn test_spatial_evaluator_constant_folding() {
    let mut table = SymbolTable::new();
    register_builtins(&mut table);
    let evaluator = Evaluator::new(&table);

    // 1. Evaluate position([420mm, 180mm, 80mm])
    let pick_expr = Expr::Call {
        callee: "position".to_string(),
        args: vec![Expr::Vector3([
            Box::new(Expr::Length(LengthMeters(0.420))),
            Box::new(Expr::Length(LengthMeters(0.180))),
            Box::new(Expr::Length(LengthMeters(0.080))),
        ])],
    };

    let pick_res = evaluator.eval_expr(&pick_expr);
    assert_eq!(
        pick_res,
        EvalResult::Value(CompileTimeValue::Position(Position {
            point: Vector3::new(0.420, 0.180, 0.080)
        }))
    );

    // 2. Evaluate (pick + [0mm, 0mm, 100mm]) + [50mm, 0mm, 0mm]
    let offset1 = Expr::Vector3([
        Box::new(Expr::Length(LengthMeters(0.0))),
        Box::new(Expr::Length(LengthMeters(0.0))),
        Box::new(Expr::Length(LengthMeters(0.100))),
    ]);
    let offset2 = Expr::Vector3([
        Box::new(Expr::Length(LengthMeters(0.050))),
        Box::new(Expr::Length(LengthMeters(0.0))),
        Box::new(Expr::Length(LengthMeters(0.0))),
    ]);

    let step1 = Expr::Binary {
        left: Box::new(pick_expr.clone()),
        op: BinaryOp::Add,
        right: Box::new(offset1),
    };

    let step2 = Expr::Binary {
        left: Box::new(step1),
        op: BinaryOp::Add,
        right: Box::new(offset2),
    };

    let evaluated = evaluator.eval_expr(&step2);
    assert_eq!(
        evaluated,
        EvalResult::Value(CompileTimeValue::Position(Position {
            point: Vector3::new(0.470, 0.180, 0.180)
        }))
    );
}

#[test]
fn test_position_subtraction_produces_vector3() {
    let table = SymbolTable::new();
    let evaluator = Evaluator::new(&table);

    let pos1 = Expr::Call {
        callee: "position".to_string(),
        args: vec![Expr::Vector3([
            Box::new(Expr::Length(LengthMeters(0.100))),
            Box::new(Expr::Length(LengthMeters(0.200))),
            Box::new(Expr::Length(LengthMeters(0.300))),
        ])],
    };

    let pos2 = Expr::Call {
        callee: "position".to_string(),
        args: vec![Expr::Vector3([
            Box::new(Expr::Length(LengthMeters(0.400))),
            Box::new(Expr::Length(LengthMeters(0.200))),
            Box::new(Expr::Length(LengthMeters(0.300))),
        ])],
    };

    let delta_expr = Expr::Binary {
        left: Box::new(pos2),
        op: BinaryOp::Sub,
        right: Box::new(pos1),
    };

    let res = evaluator.eval_expr(&delta_expr);
    if let EvalResult::Value(CompileTimeValue::Vector3(v)) = res {
        assert!((v.x - 0.300).abs() < 1e-6);
        assert!((v.y - 0.0).abs() < 1e-6);
        assert!((v.z - 0.0).abs() < 1e-6);
    } else {
        panic!("Expected CompileTimeValue::Vector3, got {:?}", res);
    }
}

#[test]
fn test_runtime_expression_returns_not_constant() {
    let table = SymbolTable::new();
    let evaluator = Evaluator::new(&table);

    let runtime_call = Expr::Call {
        callee: "get_current_tcp".to_string(),
        args: vec![],
    };

    let res = evaluator.eval_expr(&runtime_call);
    assert_eq!(res, EvalResult::NotConstant);
}

#[test]
fn test_euler_and_quaternion_evaluation() {
    use thalos_lang::units::AngleRadians;
    use thalos_math::UnitQuaternion;

    let mut table = SymbolTable::new();
    register_builtins(&mut table);
    let evaluator = Evaluator::new(&table);

    // euler(0deg, 0deg, 3.141592653589793rad)
    let euler_expr = Expr::Call {
        callee: "euler".to_string(),
        args: vec![
            Expr::Angle(AngleRadians(0.0)),
            Expr::Angle(AngleRadians(0.0)),
            Expr::Angle(AngleRadians(std::f64::consts::PI)),
        ],
    };

    let res = evaluator.eval_expr(&euler_expr);
    if let EvalResult::Value(CompileTimeValue::Quaternion(q)) = res {
        let expected = UnitQuaternion::from_euler(0.0, 0.0, std::f64::consts::PI);
        assert!((q.inner().w - expected.inner().w).abs() < 1e-6);
        assert!((q.inner().z - expected.inner().z).abs() < 1e-6);
    } else {
        panic!("Expected CompileTimeValue::Quaternion, got {:?}", res);
    }
}
