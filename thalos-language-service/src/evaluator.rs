use serde::{Deserialize, Serialize};
use thalos_lang::ast::{Arg, Expr, UnaryOp};
use thalos_math::{Quaternion, Transform3D, UnitQuaternion, Vector3};
use crate::checker::SemanticDiagnostic;
use crate::registry::{registry, ConstructorId, ConstructorShape, ConstructorSpec};
use crate::scope::SymbolTable;
use crate::types::Type;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Position {
    pub point: Vector3,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub transform: Transform3D,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CompileTimeValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    String(String),
    Length(f64),
    Angle(f64),
    Duration(f64),
    /// `Length / Duration`, stored in m/s.
    Speed(f64),
    /// `Angle / Duration`, stored in rad/s.
    AngularSpeed(f64),
    Vector3(Vector3),
    Quaternion(UnitQuaternion),
    Transform3D(Transform3D),
    Position(Position),
    Pose(Pose),
    Joints(Vec<f64>),
}

impl CompileTimeValue {
    pub fn get_type(&self) -> Type {
        match self {
            CompileTimeValue::Bool(_) => Type::Bool,
            CompileTimeValue::Int(_) => Type::Int,
            CompileTimeValue::Float(_) => Type::Float,
            CompileTimeValue::String(_) => Type::String,
            CompileTimeValue::Length(_) => Type::Length,
            CompileTimeValue::Angle(_) => Type::Angle,
            CompileTimeValue::Duration(_) => Type::Duration,
            CompileTimeValue::Speed(_) => Type::Speed,
            CompileTimeValue::AngularSpeed(_) => Type::AngularSpeed,
            CompileTimeValue::Vector3(_) => Type::Vector3,
            CompileTimeValue::Quaternion(_) => Type::Quaternion,
            CompileTimeValue::Transform3D(_) => Type::Transform3D,
            CompileTimeValue::Position(_) => Type::Position,
            CompileTimeValue::Pose(_) => Type::Pose,
            CompileTimeValue::Joints(vals) => Type::Joints {
                dimension: crate::types::Dimension::Known(vals.len()),
            },
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum EvalResult {
    Value(CompileTimeValue),
    NotConstant,
    Error(SemanticDiagnostic),
}

pub struct Evaluator<'a> {
    pub symbol_table: &'a SymbolTable,
    pub target_values: Option<&'a std::collections::HashMap<String, CompileTimeValue>>,
}

impl<'a> Evaluator<'a> {
    pub fn new(symbol_table: &'a SymbolTable) -> Self {
        Self {
            symbol_table,
            target_values: None,
        }
    }

    pub fn with_target_values(
        symbol_table: &'a SymbolTable,
        target_values: &'a std::collections::HashMap<String, CompileTimeValue>,
    ) -> Self {
        Self {
            symbol_table,
            target_values: Some(target_values),
        }
    }

    fn eval_constructor_call(
        &self,
        callee: &str,
        args: &[Arg],
        overloads: &[ConstructorSpec],
    ) -> EvalResult {
        let mut values = Vec::with_capacity(args.len());
        for arg in args {
            match self.eval_expr(&arg.value) {
                EvalResult::Value(value) => values.push(value),
                EvalResult::NotConstant => return EvalResult::NotConstant,
                EvalResult::Error(error) => return EvalResult::Error(error),
            }
        }

        match &overloads[0].shape {
            ConstructorShape::ReceiverMethod => self.eval_offset_call(args),
            ConstructorShape::Variadic { param } => {
                if values.is_empty() {
                    return self.constructor_error(format!(
                        "{} requires at least one {}",
                        callee, param
                    ));
                }
                for value in &values {
                    if !param.accepts(&value.get_type()) {
                        return self.constructor_error(format!(
                            "{} component expected {}, found {}",
                            callee,
                            param,
                            value.get_type()
                        ));
                    }
                }
                self.build_constructor(overloads[0].id, values)
            }
            ConstructorShape::Fixed => {
                let matched = overloads.iter().find(|spec| {
                    spec.params.len() == values.len()
                        && spec
                            .params
                            .iter()
                            .zip(&values)
                            .all(|(param, value)| param.accepts(&value.get_type()))
                });
                match matched {
                    Some(spec) => self.build_constructor(spec.id, values),
                    None => self.constructor_error(format!(
                        "No matching overload for constructor '{}' with argument types {}",
                        callee,
                        values
                            .iter()
                            .map(|value| value.get_type().to_string())
                            .collect::<Vec<_>>()
                            .join(", ")
                    )),
                }
            }
        }
    }

    fn constructor_error(&self, message: String) -> EvalResult {
        EvalResult::Error(SemanticDiagnostic {
            message,
            span: None,
        })
    }

    fn build_constructor(&self, id: ConstructorId, values: Vec<CompileTimeValue>) -> EvalResult {
        match id {
            ConstructorId::Vector3 => match values.as_slice() {
                [
                    CompileTimeValue::Length(x),
                    CompileTimeValue::Length(y),
                    CompileTimeValue::Length(z),
                ] => EvalResult::Value(CompileTimeValue::Vector3(Vector3::new(*x, *y, *z))),
                _ => self.constructor_error(
                    "Vector3 requires three Length arguments (x, y, z)".to_string(),
                ),
            },
            ConstructorId::Position => match values.as_slice() {
                [CompileTimeValue::Vector3(point)] => {
                    EvalResult::Value(CompileTimeValue::Position(Position { point: *point }))
                }
                [
                    CompileTimeValue::Length(x),
                    CompileTimeValue::Length(y),
                    CompileTimeValue::Length(z),
                ] => EvalResult::Value(CompileTimeValue::Position(Position {
                    point: Vector3::new(*x, *y, *z),
                })),
                _ => self.constructor_error(
                    "Position requires (x, y, z) Lengths or a Vector3".to_string(),
                ),
            },
            ConstructorId::Pose => match values.as_slice() {
                [first, CompileTimeValue::Quaternion(rotation)] => {
                    let point = match first {
                        CompileTimeValue::Vector3(point) => *point,
                        CompileTimeValue::Position(position) => position.point,
                        other => {
                            return self.constructor_error(format!(
                                "pose expected Position/Vector3 as first argument, found {}",
                                other.get_type()
                            ));
                        }
                    };
                    EvalResult::Value(CompileTimeValue::Pose(Pose {
                        transform: Transform3D::from_translation_rotation(point, *rotation),
                    }))
                }
                _ => self.constructor_error(
                    "pose requires Position/Vector3 and Quaternion arguments".to_string(),
                ),
            },
            ConstructorId::Euler => match values.as_slice() {
                [
                    CompileTimeValue::Angle(roll),
                    CompileTimeValue::Angle(pitch),
                    CompileTimeValue::Angle(yaw),
                ] => EvalResult::Value(CompileTimeValue::Quaternion(
                    UnitQuaternion::from_euler(*roll, *pitch, *yaw),
                )),
                _ => self.constructor_error(
                    "euler requires 3 Angle arguments (roll, pitch, yaw)".to_string(),
                ),
            },
            ConstructorId::Quaternion => match values.as_slice() {
                [w, x, y, z] => match (
                    numeric_value(w),
                    numeric_value(x),
                    numeric_value(y),
                    numeric_value(z),
                ) {
                    (Some(w), Some(x), Some(y), Some(z)) => {
                        let q = Quaternion::new(w, x, y, z).normalize_or_identity();
                        EvalResult::Value(CompileTimeValue::Quaternion(
                            UnitQuaternion::from_quaternion_unchecked(q),
                        ))
                    }
                    _ => self.constructor_error(
                        "quaternion requires 4 numeric arguments (w, x, y, z)".to_string(),
                    ),
                },
                _ => self.constructor_error(
                    "quaternion requires 4 numeric arguments (w, x, y, z)".to_string(),
                ),
            },
            ConstructorId::Joints => {
                let mut angles = Vec::with_capacity(values.len());
                for value in &values {
                    match value {
                        CompileTimeValue::Angle(angle) => angles.push(*angle),
                        other => {
                            return self.constructor_error(format!(
                                "joints component expected Angle, found {}",
                                other.get_type()
                            ));
                        }
                    }
                }
                EvalResult::Value(CompileTimeValue::Joints(angles))
            }
            ConstructorId::Offset => self
                .constructor_error("offset is a receiver method, not a value constructor".to_string()),
        }
    }

    fn eval_offset_call(&self, args: &[Arg]) -> EvalResult {
        let Some(receiver_arg) = args.first() else {
            return self.constructor_error("offset requires a receiver argument".to_string());
        };
        let receiver = match self.eval_expr(&receiver_arg.value) {
            EvalResult::Value(value) => value,
            other => return other,
        };
        let canonical =
            match crate::offset::canonicalize_delta(&receiver.get_type(), &args[1..]) {
                Ok(canonical) => canonical,
                Err(message) => return self.constructor_error(message),
            };
        let mut delta = Vec::with_capacity(canonical.args.len());
        for arg in &canonical.args {
            match self.eval_expr(&arg.value) {
                EvalResult::Value(value) => delta.push(value),
                other => return other,
            }
        }
        match crate::offset::apply(&receiver, &delta) {
            Ok(value) => EvalResult::Value(value),
            Err(message) => self.constructor_error(message),
        }
    }

    pub fn eval_expr(&self, expr: &Expr) -> EvalResult {
        match expr {
            Expr::Number(n) => {
                if n.fract() == 0.0 {
                    EvalResult::Value(CompileTimeValue::Int(*n as i64))
                } else {
                    EvalResult::Value(CompileTimeValue::Float(*n))
                }
            }
            Expr::Boolean(b) => EvalResult::Value(CompileTimeValue::Bool(*b)),
            Expr::StringLiteral(s) => EvalResult::Value(CompileTimeValue::String(s.clone())),
            Expr::Length(l) => EvalResult::Value(CompileTimeValue::Length(l.0)),
            Expr::Angle(a) => EvalResult::Value(CompileTimeValue::Angle(a.0)),
            Expr::Duration(d) => EvalResult::Value(CompileTimeValue::Duration(d.0)),
            Expr::Vector3([x_expr, y_expr, z_expr]) => {
                // A Vector3 is a geometric displacement: every component must be
                // a `Length`. No implicit Number -> dimension coercion.
                let mut components = [0.0_f64; 3];
                for (index, component_expr) in [x_expr, y_expr, z_expr].into_iter().enumerate() {
                    match self.eval_expr(component_expr) {
                        EvalResult::Value(CompileTimeValue::Length(v)) => components[index] = v,
                        EvalResult::Value(other) => {
                            return EvalResult::Error(SemanticDiagnostic {
                                message: format!(
                                    "vector component expected Length, found {}",
                                    other.get_type()
                                ),
                                span: None,
                            });
                        }
                        other => return other,
                    }
                }
                EvalResult::Value(CompileTimeValue::Vector3(Vector3::new(
                    components[0],
                    components[1],
                    components[2],
                )))
            }
            Expr::Identifier(id) => {
                if let Some(values) = self.target_values
                    && let Some(val) = values.get(id) {
                        return EvalResult::Value(val.clone());
                    }
                if let Some(_symbols) = self.symbol_table.lookup(id) {
                    EvalResult::NotConstant
                } else {
                    EvalResult::Error(SemanticDiagnostic {
                        message: format!("Unknown symbol '{}'", id),
                        span: None,
                    })
                }
            }
            Expr::Call { callee, args } => {
                if let Some(overloads) = registry().constructor_overloads(callee) {
                    self.eval_constructor_call(callee, args, overloads)
                } else {
                    EvalResult::NotConstant
                }
            }
            Expr::Binary { left, op, right } => {
                let lhs = match self.eval_expr(left) {
                    EvalResult::Value(v) => v,
                    other => return other,
                };
                let rhs = match self.eval_expr(right) {
                    EvalResult::Value(v) => v,
                    other => return other,
                };

                // Single shared value algebra (also used by the resolver).
                match crate::algebra::binary_value(*op, &lhs, &rhs) {
                    Ok(value) => EvalResult::Value(value),
                    Err(message) => EvalResult::Error(SemanticDiagnostic {
                        message,
                        span: None,
                    }),
                }
            }
            // Prefix negation is additive inversion. The single value-level
            // implementation lives in `negate_value`, shared with the resolver
            // so compile-time folding and deferred resolution cannot diverge.
            Expr::Unary {
                op: UnaryOp::Neg,
                operand,
            } => match self.eval_expr(operand) {
                EvalResult::Value(value) => match negate_value(&value) {
                    Some(negated) => EvalResult::Value(negated),
                    None => EvalResult::Error(SemanticDiagnostic {
                        message: format!("Cannot negate value of type {}", value.get_type()),
                        span: None,
                    }),
                },
                other => other,
            },
            // Member access reads a component of a typed value. When the
            // receiver is a compile-time value (target/const) it folds here;
            // otherwise it stays symbolic and the resolver evaluates it.
            Expr::MemberAccess { object, member } => {
                match self.eval_expr(object) {
                    EvalResult::Value(value) => match read_member(&value, member) {
                        Some(member_value) => EvalResult::Value(member_value),
                        None => EvalResult::NotConstant,
                    },
                    other => other,
                }
            }
            _ => EvalResult::NotConstant,
        }
    }
}

/// Additive inversion for the value types that support it.
///
/// This is the **single** value-level implementation, shared by the compile-time
/// evaluator and the resolver, so folding and deferred resolution cannot diverge.
/// `Position`, `Pose`, `Joints`, `Bool`, ... have no additive inverse and yield
/// `None` (the checker rejects those earlier with a typed diagnostic).
fn numeric_value(value: &CompileTimeValue) -> Option<f64> {
    match value {
        CompileTimeValue::Int(value) => Some(*value as f64),
        CompileTimeValue::Float(value) => Some(*value),
        _ => None,
    }
}

pub fn negate_value(value: &CompileTimeValue) -> Option<CompileTimeValue> {
    match value {
        CompileTimeValue::Int(i) => Some(CompileTimeValue::Int(-i)),
        CompileTimeValue::Float(f) => Some(CompileTimeValue::Float(-f)),
        CompileTimeValue::Length(l) => Some(CompileTimeValue::Length(-l)),
        CompileTimeValue::Angle(a) => Some(CompileTimeValue::Angle(-a)),
        CompileTimeValue::Duration(d) => Some(CompileTimeValue::Duration(-d)),
        CompileTimeValue::Speed(s) => Some(CompileTimeValue::Speed(-s)),
        CompileTimeValue::AngularSpeed(s) => Some(CompileTimeValue::AngularSpeed(-s)),
        CompileTimeValue::Vector3(v) => Some(CompileTimeValue::Vector3(-*v)),
        _ => None,
    }
}

/// Read a named member from a compile-time value.
///
/// This is the **single** implementation of member extraction, shared by the
/// compile-time evaluator and the resolver, so accessor semantics cannot diverge.
///
/// `Vector3` components are read as `Length` to match the member schema; note
/// that a `Vector3` is not strongly unit-typed, so this is an approximation.
pub fn read_member(value: &CompileTimeValue, member: &str) -> Option<CompileTimeValue> {
    match value {
        CompileTimeValue::Position(p) => vector_component(&p.point, member),
        CompileTimeValue::Pose(p) => vector_component(&p.transform.translation, member),
        CompileTimeValue::Vector3(v) => vector_component(v, member),
        CompileTimeValue::Joints(values) => {
            let index = crate::member_schema::joint_index(member)?;
            values
                .get(index.checked_sub(1)?)
                .map(|v| CompileTimeValue::Angle(*v))
        }
        _ => None,
    }
}

fn vector_component(v: &Vector3, member: &str) -> Option<CompileTimeValue> {
    match member {
        "x" => Some(CompileTimeValue::Length(v.x)),
        "y" => Some(CompileTimeValue::Length(v.y)),
        "z" => Some(CompileTimeValue::Length(v.z)),
        _ => None,
    }
}
