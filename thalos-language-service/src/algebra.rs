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
use thalos_math::Transform3D;

use crate::evaluator::{CompileTimeValue, Pose, Position};

// Alias the enum so its variant names (`Transform3D`, `Position`, ...) do not
// shadow the struct/type names imported above.
use CompileTimeValue as V;

/// Combine two compile-time values under a binary operator.
pub fn binary_value(
    op: BinaryOp,
    lhs: &CompileTimeValue,
    rhs: &CompileTimeValue,
) -> Result<CompileTimeValue, String> {
    // Number arithmetic / comparisons (Int/Float with promotion).
    if let (Some(a), Some(b)) = (number_of(lhs), number_of(rhs)) {
        let both_int = matches!(lhs, V::Int(_)) && matches!(rhs, V::Int(_));
        return number_op(op, a, b, both_int);
    }

    match (lhs, rhs) {
        // ── Length ──────────────────────────────────────────────────
        (V::Length(a), V::Length(b)) => same_dimension(op, *a, *b, V::Length, true),
        (V::Length(a), V::Duration(b)) if op == BinaryOp::Div => Ok(V::Speed(a / b)),
        (V::Length(a), r) if number_of(r).is_some() => scale(op, *a, number_of(r).unwrap(), V::Length),
        (l, V::Length(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(V::Length(number_of(l).unwrap() * b))
        }

        // ── Angle ───────────────────────────────────────────────────
        (V::Angle(a), V::Angle(b)) => same_dimension(op, *a, *b, V::Angle, true),
        (V::Angle(a), V::Duration(b)) if op == BinaryOp::Div => Ok(V::AngularSpeed(a / b)),
        (V::Angle(a), r) if number_of(r).is_some() => scale(op, *a, number_of(r).unwrap(), V::Angle),
        (l, V::Angle(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(V::Angle(number_of(l).unwrap() * b))
        }

        // ── Duration ────────────────────────────────────────────────
        (V::Duration(a), V::Duration(b)) => same_dimension(op, *a, *b, V::Duration, true),
        (V::Duration(a), r) if number_of(r).is_some() => {
            scale(op, *a, number_of(r).unwrap(), V::Duration)
        }
        (l, V::Duration(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(V::Duration(number_of(l).unwrap() * b))
        }

        // ── Speed (Length / Duration) ───────────────────────────────
        (V::Speed(a), V::Speed(b)) => same_dimension(op, *a, *b, V::Speed, false),
        (V::Speed(a), V::Duration(b)) if op == BinaryOp::Mul => Ok(V::Length(a * b)),
        (V::Duration(a), V::Speed(b)) if op == BinaryOp::Mul => Ok(V::Length(a * b)),
        (V::Speed(a), r) if number_of(r).is_some() => scale(op, *a, number_of(r).unwrap(), V::Speed),
        (l, V::Speed(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(V::Speed(number_of(l).unwrap() * b))
        }

        // ── AngularSpeed (Angle / Duration) ─────────────────────────
        (V::AngularSpeed(a), V::AngularSpeed(b)) => {
            same_dimension(op, *a, *b, V::AngularSpeed, false)
        }
        (V::AngularSpeed(a), V::Duration(b)) if op == BinaryOp::Mul => Ok(V::Angle(a * b)),
        (V::Duration(a), V::AngularSpeed(b)) if op == BinaryOp::Mul => Ok(V::Angle(a * b)),
        (V::AngularSpeed(a), r) if number_of(r).is_some() => {
            scale(op, *a, number_of(r).unwrap(), V::AngularSpeed)
        }
        (l, V::AngularSpeed(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(V::AngularSpeed(number_of(l).unwrap() * b))
        }

        // ── Vector3 ─────────────────────────────────────────────────
        (V::Vector3(a), V::Vector3(b)) => match op {
            BinaryOp::Add => Ok(V::Vector3(*a + *b)),
            BinaryOp::Sub => Ok(V::Vector3(*a - *b)),
            _ => Err(invalid(op, lhs, rhs)),
        },
        (V::Vector3(a), r) if number_of(r).is_some() => {
            let f = number_of(r).unwrap();
            match op {
                BinaryOp::Mul => Ok(V::Vector3(*a * f)),
                BinaryOp::Div => {
                    if f == 0.0 {
                        return Err("division by zero".to_string());
                    }
                    Ok(V::Vector3(*a * (1.0 / f)))
                }
                _ => Err(invalid(op, lhs, rhs)),
            }
        }
        (l, V::Vector3(b)) if op == BinaryOp::Mul && number_of(l).is_some() => {
            Ok(V::Vector3(*b * number_of(l).unwrap()))
        }

        // ── Position / Pose / Transform ─────────────────────────────
        (V::Position(p), V::Vector3(v)) => match op {
            BinaryOp::Add => Ok(V::Position(Position { point: p.point + *v })),
            BinaryOp::Sub => Ok(V::Position(Position { point: p.point - *v })),
            _ => Err(invalid(op, lhs, rhs)),
        },
        (V::Position(p1), V::Position(p2)) if op == BinaryOp::Sub => Ok(V::Vector3(p1.point - p2.point)),
        (V::Pose(p), V::Vector3(v)) => match op {
            BinaryOp::Add => Ok(V::Pose(Pose {
                transform: Transform3D::from_translation_rotation(
                    p.transform.translation + *v,
                    p.transform.rotation,
                ),
            })),
            BinaryOp::Sub => Ok(V::Pose(Pose {
                transform: Transform3D::from_translation_rotation(
                    p.transform.translation - *v,
                    p.transform.rotation,
                ),
            })),
            _ => Err(invalid(op, lhs, rhs)),
        },
        (V::Pose(p), V::Transform3D(t)) if op == BinaryOp::Mul => Ok(V::Pose(Pose {
            transform: p.transform.compose(t),
        })),
        (V::Transform3D(a), V::Transform3D(b)) if op == BinaryOp::Mul => Ok(V::Transform3D(a.compose(b))),

        // ── Quaternion ──────────────────────────────────────────────
        (V::Quaternion(a), V::Quaternion(b)) if op == BinaryOp::Mul => Ok(V::Quaternion(*a * *b)),
        (V::Quaternion(q), V::Vector3(v)) if op == BinaryOp::Mul => Ok(V::Vector3(q.rotate_vector(*v))),

        // ── Bool ────────────────────────────────────────────────────
        (V::Bool(a), V::Bool(b)) => match op {
            BinaryOp::Eq => Ok(V::Bool(a == b)),
            BinaryOp::Neq => Ok(V::Bool(a != b)),
            _ => Err(invalid(op, lhs, rhs)),
        },

        _ => Err(invalid(op, lhs, rhs)),
    }
}

fn number_of(value: &CompileTimeValue) -> Option<f64> {
    match value {
        V::Int(i) => Some(*i as f64),
        V::Float(f) => Some(*f),
        _ => None,
    }
}

fn number_op(op: BinaryOp, a: f64, b: f64, both_int: bool) -> Result<CompileTimeValue, String> {
    match op {
        BinaryOp::Add => Ok(arithmetic(a + b, both_int)),
        BinaryOp::Sub => Ok(arithmetic(a - b, both_int)),
        BinaryOp::Mul => Ok(arithmetic(a * b, both_int)),
        BinaryOp::Div => {
            if b == 0.0 {
                return Err("division by zero".to_string());
            }
            // `/` always yields a Float: no integer division surprises.
            Ok(V::Float(a / b))
        }
        BinaryOp::Gt => Ok(V::Bool(a > b)),
        BinaryOp::Lt => Ok(V::Bool(a < b)),
        BinaryOp::Gte => Ok(V::Bool(a >= b)),
        BinaryOp::Lte => Ok(V::Bool(a <= b)),
        BinaryOp::Eq => Ok(V::Bool(a == b)),
        BinaryOp::Neq => Ok(V::Bool(a != b)),
    }
}

fn arithmetic(value: f64, both_int: bool) -> CompileTimeValue {
    if both_int {
        V::Int(value as i64)
    } else {
        V::Float(value)
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
        BinaryOp::Eq => Ok(V::Bool(a == b)),
        BinaryOp::Neq => Ok(V::Bool(a != b)),
        BinaryOp::Gt if ordered => Ok(V::Bool(a > b)),
        BinaryOp::Lt if ordered => Ok(V::Bool(a < b)),
        BinaryOp::Gte if ordered => Ok(V::Bool(a >= b)),
        BinaryOp::Lte if ordered => Ok(V::Bool(a <= b)),
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
