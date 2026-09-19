//! Shared value-level algebra for `BinaryOp`.
//!
//! This is the **single** source of truth for how two [`CompileTimeValue`]s
//! combine. The compile-time evaluator and the resolver both delegate here, so
//! constant folding and deferred resolution cannot diverge. The *type* algebra
//! lives in `operators.rs`; this module mirrors it exactly at the value level.
//!
//! Normative rules: `docs/language/thalos-dimensional-system.md`.
//!
//! ## `Number` promotion
//!
//! `Int`/`Float` are representations of the `Number` category. Arithmetic
//! promotes to `Float` when either operand is a `Float`; `/` always yields a
//! `Float` (no integer division). This is *numeric widening*, not a dimensional
//! coercion: no physical dimension is ever satisfied by a bare `Number`.

use thalos_lang::ast::BinaryOp;
use thalos_math::{Transform3D, Vector3};

use crate::evaluator::{CompileTimeValue, Pose, Position};

/// Combine two compile-time values under a binary operator.
pub fn binary_value(
    op: BinaryOp,
    lhs: &CompileTimeValue,
    rhs: &CompileTimeValue,
) -> Result<CompileTimeValue, String> {
    use CompileTimeValue::*;

    // Number arithmetic / comparisons (Int/Float with promotion).
    if let (Some(a), Some(b)) = (number_of(lhs), number_of(rhs)) {
        let both_int = matches!(lhs, Int(_)) && matches!(rhs, Int(_));
        return number_op(op, a, b, both_int);
    }

    match (lhs, rhs) {
        // ── Length ──────────────────────────────────────────────────
        (Length(a), Length(b)) => same_dimension(op, *a, *b, Length, true),
        (Length(a), Duration(b)) if op == BinaryOp::Div => Ok(Speed(a / b)),
        (Length(a), r) if number_of(r).is_some() => {
            scale(op, *a, number_of(r).unwrap(), Length)
        }
        (l, Length(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(Length(number_of(l).unwrap() * b))
        }

        // ── Angle ───────────────────────────────────────────────────
        (Angle(a), Angle(b)) => same_dimension(op, *a, *b, Angle, true),
        (Angle(a), Duration(b)) if op == BinaryOp::Div => Ok(AngularSpeed(a / b)),
        (Angle(a), r) if number_of(r).is_some() => scale(op, *a, number_of(r).unwrap(), Angle),
        (l, Angle(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(Angle(number_of(l).unwrap() * b))
        }

        // ── Duration ────────────────────────────────────────────────
        (Duration(a), Duration(b)) => same_dimension(op, *a, *b, Duration, true),
        (Duration(a), r) if number_of(r).is_some() => {
            scale(op, *a, number_of(r).unwrap(), Duration)
        }
        (l, Duration(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(Duration(number_of(l).unwrap() * b))
        }

        // ── Speed (Length / Duration) ───────────────────────────────
        (Speed(a), Speed(b)) => same_dimension(op, *a, *b, Speed, false),
        (Speed(a), Duration(b)) if op == BinaryOp::Mul => Ok(Length(a * b)),
        (Duration(a), Speed(b)) if op == BinaryOp::Mul => Ok(Length(a * b)),
        (Speed(a), r) if number_of(r).is_some() => scale(op, *a, number_of(r).unwrap(), Speed),
        (l, Speed(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(Speed(number_of(l).unwrap() * b))
        }

        // ── AngularSpeed (Angle / Duration) ─────────────────────────
        (AngularSpeed(a), AngularSpeed(b)) => same_dimension(op, *a, *b, AngularSpeed, false),
        (AngularSpeed(a), Duration(b)) if op == BinaryOp::Mul => Ok(Angle(a * b)),
        (Duration(a), AngularSpeed(b)) if op == BinaryOp::Mul => Ok(Angle(a * b)),
        (AngularSpeed(a), r) if number_of(r).is_some() => {
            scale(op, *a, number_of(r).unwrap(), AngularSpeed)
        }
        (l, AngularSpeed(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(AngularSpeed(number_of(l).unwrap() * b))
        }

        // ── Vector3 ─────────────────────────────────────────────────
        (Vector3(a), Vector3(b)) => match op {
            BinaryOp::Add => Ok(Vector3(*a + *b)),
            BinaryOp::Sub => Ok(Vector3(*a - *b)),
            _ => Err(invalid(op, lhs, rhs)),
        },
        (Vector3(a), r) if number_of(r).is_some() => {
            let f = number_of(r).unwrap();
            match op {
                BinaryOp::Mul => Ok(Vector3(*a * f)),
                BinaryOp::Div => {
                    if f == 0.0 {
                        return Err("division by zero".to_string());
                    }
                    Ok(Vector3(*a * (1.0 / f)))
                }
                _ => Err(invalid(op, lhs, rhs)),
            }
        }
        (l, Vector3(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(Vector3(number_of(l).unwrap() * *b))
        }

        // ── Position / Pose / Transform ─────────────────────────────
        (Position(p), Vector3(v)) => match op {
            BinaryOp::Add => Ok(Position(Position { point: p.point + *v })),
            BinaryOp::Sub => Ok(Position(Position { point: p.point - *v })),
            _ => Err(invalid(op, lhs, rhs)),
        },
        (Position(p1), Position(p2)) if op == BinaryOp::Sub => Ok(Vector3(p1.point - p2.point)),
        (Pose(p), Vector3(v)) => match op {
            BinaryOp::Add => Ok(Pose(Pose {
                transform: Transform3D::from_translation_rotation(
                    p.transform.translation + *v,
                    p.transform.rotation,
                ),
            })),
            BinaryOp::Sub => Ok(Pose(Pose {
                transform: Transform3D::from_translation_rotation(
                    p.transform.translation - *v,
                    p.transform.rotation,
                ),
            })),
            _ => Err(invalid(op, lhs, rhs)),
        },
        (Pose(p), Transform3D(t)) if op == BinaryOp::Mul => Ok(Pose(Pose {
            transform: p.transform.compose(t),
        })),
        (Transform3D(a), Transform3D(b)) if op == BinaryOp::Mul => Ok(Transform3D(a.compose(b))),

        // ── Quaternion ──────────────────────────────────────────────
        (Quaternion(a), Quaternion(b)) if op == BinaryOp::Mul => Ok(Quaternion(*a * *b)),
        (Quaternion(q), Vector3(v)) if op == BinaryOp::Mul => Ok(Vector3(q.rotate_vector(*v))),

        // ── Bool ────────────────────────────────────────────────────
        (Bool(a), Bool(b)) => match op {
            BinaryOp::Eq => Ok(Bool(a == b)),
            BinaryOp::Neq => Ok(Bool(a != b)),
            _ => Err(invalid(op, lhs, rhs)),
        },

        _ => Err(invalid(op, lhs, rhs)),
    }
}

fn number_of(value: &CompileTimeValue) -> Option<f64> {
    match value {
        CompileTimeValue::Int(i) => Some(*i as f64),
        CompileTimeValue::Float(f) => Some(*f),
        _ => None,
    }
}

fn number_op(op: BinaryOp, a: f64, b: f64, both_int: bool) -> Result<CompileTimeValue, String> {
    use CompileTimeValue::{Bool, Float, Int};
    match op {
        BinaryOp::Add => Ok(arithmetic(a + b, op, both_int)),
        BinaryOp::Sub => Ok(arithmetic(a - b, op, both_int)),
        BinaryOp::Mul => Ok(arithmetic(a * b, op, both_int)),
        BinaryOp::Div => {
            if b == 0.0 {
                return Err("division by zero".to_string());
            }
            // `/` always yields a Float: no integer division surprises.
            Ok(Float(a / b))
        }
        BinaryOp::Gt => Ok(Bool(a > b)),
        BinaryOp::Lt => Ok(Bool(a < b)),
        BinaryOp::Gte => Ok(Bool(a >= b)),
        BinaryOp::Lte => Ok(Bool(a <= b)),
        BinaryOp::Eq => Ok(Bool(a == b)),
        BinaryOp::Neq => Ok(Bool(a != b)),
        _ => Err(format!("Invalid numeric operation {op:?}")),
    }
}

fn arithmetic(value: f64, _op: BinaryOp, both_int: bool) -> CompileTimeValue {
    if both_int {
        CompileTimeValue::Int(value as i64)
    } else {
        CompileTimeValue::Float(value)
    }
}

fn same_dimension(
    op: BinaryOp,
    a: f64,
    b: f64,
    make: fn(f64) -> CompileTimeValue,
    ordered: bool,
) -> Result<CompileTimeValue, String> {
    match op {
        BinaryOp::Add => Ok(make(a + b)),
        BinaryOp::Sub => Ok(make(a - b)),
        BinaryOp::Eq => Ok(CompileTimeValue::Bool(a == b)),
        BinaryOp::Neq => Ok(CompileTimeValue::Bool(a != b)),
        BinaryOp::Gt if ordered => Ok(CompileTimeValue::Bool(a > b)),
        BinaryOp::Lt if ordered => Ok(CompileTimeValue::Bool(a < b)),
        BinaryOp::Gte if ordered => Ok(CompileTimeValue::Bool(a >= b)),
        BinaryOp::Lte if ordered => Ok(CompileTimeValue::Bool(a <= b)),
        _ => Err(format!(
            "Invalid operation {op:?} between same-dimension quantities"
        )),
    }
}

fn scale(
    op: BinaryOp,
    quantity: f64,
    factor: f64,
    make: fn(f64) -> CompileTimeValue,
) -> Result<CompileTimeValue, String> {
    match op {
        BinaryOp::Mul => Ok(make(quantity * factor)),
        BinaryOp::Div => {
            if factor == 0.0 {
                return Err("division by zero".to_string());
            }
            Ok(make(quantity / factor))
        }
        _ => Err("Invalid scaling operation".to_string()),
    }
}

fn invalid(op: BinaryOp, lhs: &CompileTimeValue, rhs: &CompileTimeValue) -> String {
    format!(
        "Invalid binary operation {op:?} between {:?} and {:?}",
        lhs.get_type(),
        rhs.get_type()
    )
}
