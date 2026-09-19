//! Type algebra for `BinaryOp` — the single source of truth for *what type*
//! results from combining two types.
//!
//! The value-level counterpart is `algebra.rs`; both must agree. Normative
//! rules: `docs/language/thalos-dimensional-system.md`.
//!
//! `Number` is the semantic category implemented by `Int`/`Float`. Arithmetic
//! promotes to `Float` when either operand is a `Float`; `/` always yields a
//! `Float`. This is numeric widening, never a dimensional coercion.

use thalos_lang::ast::BinaryOp;
use crate::types::Type;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BinaryOpRule;

impl BinaryOpRule {
    pub fn infer(lhs: &Type, op: BinaryOp, rhs: &Type) -> Result<Type, String> {
        // ── Number (Int/Float with promotion) ────────────────────────
        if let (Some(a), Some(b)) = (number_kind(lhs), number_kind(rhs)) {
            return Ok(number_result(op, a, b));
        }

        match (lhs, op, rhs) {
            // ── Length ───────────────────────────────────────────────
            (Type::Length, BinaryOp::Add | BinaryOp::Sub, Type::Length) => Ok(Type::Length),
            (Type::Length, BinaryOp::Mul, n) if is_number(n) => Ok(Type::Length),
            (n, BinaryOp::Mul, Type::Length) if is_number(n) => Ok(Type::Length),
            (Type::Length, BinaryOp::Div, n) if is_number(n) => Ok(Type::Length),
            (Type::Length, BinaryOp::Div, Type::Length) => Ok(Type::Float),
            (Type::Length, BinaryOp::Div, Type::Duration) => Ok(Type::Speed),
            (Type::Length, BinaryOp::Gt | BinaryOp::Lt | BinaryOp::Gte | BinaryOp::Lte | BinaryOp::Eq | BinaryOp::Neq, Type::Length) => Ok(Type::Bool),

            // ── Angle ────────────────────────────────────────────────
            (Type::Angle, BinaryOp::Add | BinaryOp::Sub, Type::Angle) => Ok(Type::Angle),
            (Type::Angle, BinaryOp::Mul, n) if is_number(n) => Ok(Type::Angle),
            (n, BinaryOp::Mul, Type::Angle) if is_number(n) => Ok(Type::Angle),
            (Type::Angle, BinaryOp::Div, n) if is_number(n) => Ok(Type::Angle),
            (Type::Angle, BinaryOp::Div, Type::Angle) => Ok(Type::Float),
            (Type::Angle, BinaryOp::Div, Type::Duration) => Ok(Type::AngularSpeed),
            (Type::Angle, BinaryOp::Gt | BinaryOp::Lt | BinaryOp::Gte | BinaryOp::Lte | BinaryOp::Eq | BinaryOp::Neq, Type::Angle) => Ok(Type::Bool),

            // ── Duration ─────────────────────────────────────────────
            (Type::Duration, BinaryOp::Add | BinaryOp::Sub, Type::Duration) => Ok(Type::Duration),
            (Type::Duration, BinaryOp::Mul, n) if is_number(n) => Ok(Type::Duration),
            (n, BinaryOp::Mul, Type::Duration) if is_number(n) => Ok(Type::Duration),
            (Type::Duration, BinaryOp::Div, n) if is_number(n) => Ok(Type::Duration),
            (Type::Duration, BinaryOp::Div, Type::Duration) => Ok(Type::Float),
            (Type::Duration, BinaryOp::Gt | BinaryOp::Lt | BinaryOp::Gte | BinaryOp::Lte | BinaryOp::Eq | BinaryOp::Neq, Type::Duration) => Ok(Type::Bool),

            // ── Speed (Length / Duration) ────────────────────────────
            (Type::Speed, BinaryOp::Add | BinaryOp::Sub, Type::Speed) => Ok(Type::Speed),
            (Type::Speed, BinaryOp::Mul, n) if is_number(n) => Ok(Type::Speed),
            (n, BinaryOp::Mul, Type::Speed) if is_number(n) => Ok(Type::Speed),
            (Type::Speed, BinaryOp::Div, n) if is_number(n) => Ok(Type::Speed),
            (Type::Speed, BinaryOp::Mul, Type::Duration) => Ok(Type::Length),
            (Type::Duration, BinaryOp::Mul, Type::Speed) => Ok(Type::Length),
            // Ordered comparisons of derived speeds are deferred; only equality.
            (Type::Speed, BinaryOp::Eq | BinaryOp::Neq, Type::Speed) => Ok(Type::Bool),

            // ── AngularSpeed (Angle / Duration) ──────────────────────
            (Type::AngularSpeed, BinaryOp::Add | BinaryOp::Sub, Type::AngularSpeed) => {
                Ok(Type::AngularSpeed)
            }
            (Type::AngularSpeed, BinaryOp::Mul, n) if is_number(n) => Ok(Type::AngularSpeed),
            (n, BinaryOp::Mul, Type::AngularSpeed) if is_number(n) => Ok(Type::AngularSpeed),
            (Type::AngularSpeed, BinaryOp::Div, n) if is_number(n) => Ok(Type::AngularSpeed),
            (Type::AngularSpeed, BinaryOp::Mul, Type::Duration) => Ok(Type::Angle),
            (Type::Duration, BinaryOp::Mul, Type::AngularSpeed) => Ok(Type::Angle),
            (Type::AngularSpeed, BinaryOp::Eq | BinaryOp::Neq, Type::AngularSpeed) => Ok(Type::Bool),

            // ── Position operations ──────────────────────────────────
            (Type::Position, BinaryOp::Add, Type::Vector3) => Ok(Type::Position),
            (Type::Position, BinaryOp::Sub, Type::Vector3) => Ok(Type::Position),
            (Type::Position, BinaryOp::Sub, Type::Position) => Ok(Type::Vector3),

            // ── Pose operations ──────────────────────────────────────
            (Type::Pose, BinaryOp::Add, Type::Vector3) => Ok(Type::Pose),
            (Type::Pose, BinaryOp::Sub, Type::Vector3) => Ok(Type::Pose),
            (Type::Pose, BinaryOp::Mul, Type::Transform3D) => Ok(Type::Pose),

            // ── Transform3D operations ───────────────────────────────
            (Type::Transform3D, BinaryOp::Mul, Type::Transform3D) => Ok(Type::Transform3D),

            // ── Quaternion operations ────────────────────────────────
            (Type::Quaternion, BinaryOp::Mul, Type::Quaternion) => Ok(Type::Quaternion),
            (Type::Quaternion, BinaryOp::Mul, Type::Vector3) => Ok(Type::Vector3),

            // ── Vector3 operations ───────────────────────────────────
            (Type::Vector3, BinaryOp::Add, Type::Vector3) => Ok(Type::Vector3),
            (Type::Vector3, BinaryOp::Sub, Type::Vector3) => Ok(Type::Vector3),
            (Type::Vector3, BinaryOp::Mul, n) if is_number(n) => Ok(Type::Vector3),
            (n, BinaryOp::Mul, Type::Vector3) if is_number(n) => Ok(Type::Vector3),
            (Type::Vector3, BinaryOp::Div, n) if is_number(n) => Ok(Type::Vector3),

            // ── Bool equality ────────────────────────────────────────
            (Type::Bool, BinaryOp::Eq | BinaryOp::Neq, Type::Bool) => Ok(Type::Bool),

            _ => Err(format!(
                "Invalid binary operation '{:?}' between types {:?} and {:?}",
                op, lhs, rhs
            )),
        }
    }
}

/// `Some(is_float)` for the numeric representations of `Number`.
fn number_kind(ty: &Type) -> Option<bool> {
    match ty {
        Type::Int => Some(false),
        Type::Float => Some(true),
        _ => None,
    }
}

fn is_number(ty: &Type) -> bool {
    number_kind(ty).is_some()
}

fn number_result(op: BinaryOp, a_is_float: bool, b_is_float: bool) -> Type {
    match op {
        // Division always yields a Float (no integer division).
        BinaryOp::Div => Type::Float,
        BinaryOp::Gt | BinaryOp::Lt | BinaryOp::Gte | BinaryOp::Lte | BinaryOp::Eq | BinaryOp::Neq => {
            Type::Bool
        }
        _ => {
            if a_is_float || b_is_float {
                Type::Float
            } else {
                Type::Int
            }
        }
    }
}
