//! Exhaustive conformance between the type algebra (`operators::infer`) and the
//! value algebra (`algebra::binary_value`).
//!
//! The type domain is finite, so every representative pair × operator is
//! enumerated instead of sampling. Both sides must agree on acceptance and
//! rejection, and an accepted operation must produce a value of the inferred
//! type.

use thalos_lang::ast::BinaryOp;
use thalos_language_service::algebra::binary_value;
use thalos_language_service::evaluator::{CompileTimeValue as V, Pose, Position};
use thalos_language_service::operators::BinaryOpRule;
use thalos_math::{Transform3D, UnitQuaternion, Vector3};

fn representatives() -> Vec<(&'static str, V)> {
    let translation = Vector3::new(1.0, 2.0, 3.0);
    vec![
        ("Bool", V::Bool(true)),
        ("Int", V::Int(2)),
        ("Float", V::Float(2.5)),
        ("String", V::String("x".to_string())),
        ("Length", V::Length(1.5)),
        ("Angle", V::Angle(0.5)),
        ("Duration", V::Duration(2.0)),
        ("Speed", V::Speed(1.25)),
        ("AngularSpeed", V::AngularSpeed(0.75)),
        ("Vector3", V::Vector3(translation)),
        ("Quaternion", V::Quaternion(UnitQuaternion::identity())),
        ("Transform3D", V::Transform3D(Transform3D::from_translation(translation))),
        (
            "Position",
            V::Position(Position { point: translation }),
        ),
        (
            "Pose",
            V::Pose(Pose {
                transform: Transform3D::from_translation(translation),
            }),
        ),
        ("Joints<2>", V::Joints(vec![0.1, 0.2])),
    ]
}

fn operators() -> [BinaryOp; 10] {
    [
        BinaryOp::Add,
        BinaryOp::Sub,
        BinaryOp::Mul,
        BinaryOp::Div,
        BinaryOp::Gt,
        BinaryOp::Lt,
        BinaryOp::Gte,
        BinaryOp::Lte,
        BinaryOp::Eq,
        BinaryOp::Neq,
    ]
}

#[test]
fn type_and_value_algebra_agree_for_every_pair_and_operator() {
    let values = representatives();
    for (left_name, left) in &values {
        for (right_name, right) in &values {
            for operator in operators() {
                let expected = BinaryOpRule::infer(&left.get_type(), operator, &right.get_type());
                let actual = binary_value(operator, left, right);
                match (expected, actual) {
                    (Ok(expected_ty), Ok(value)) => assert_eq!(
                        value.get_type(),
                        expected_ty,
                        "{left_name} {operator:?} {right_name} type-checks as {expected_ty} but the value algebra produced {}",
                        value.get_type()
                    ),
                    (Err(_), Err(_)) => {}
                    (Ok(expected_ty), Err(error)) => panic!(
                        "{left_name} {operator:?} {right_name} type-checks as {expected_ty} but the value algebra rejects it: {error}"
                    ),
                    (Err(error), Ok(value)) => panic!(
                        "{left_name} {operator:?} {right_name} is rejected by the type algebra ({error}) but the value algebra produces {}",
                        value.get_type()
                    ),
                }
            }
        }
    }
}
