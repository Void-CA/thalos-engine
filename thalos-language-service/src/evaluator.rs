use serde::{Deserialize, Serialize};
use thalos_lang::ast::{BinaryOp, Expr, UnaryOp};
use thalos_math::{Quaternion, Transform3D, UnitQuaternion, Vector3};
use crate::checker::SemanticDiagnostic;
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
                dimension: Some(vals.len()),
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
                                    "vector component expected Length, found {:?}",
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
            Expr::Call { callee, args } => match callee.as_str() {
                "position" => {
                    if args.len() == 1 {
                        match self.eval_expr(&args[0].value) {
                            EvalResult::Value(CompileTimeValue::Vector3(pt)) => {
                                EvalResult::Value(CompileTimeValue::Position(Position { point: pt }))
                            }
                            other => other,
                        }
                    } else {
                        EvalResult::Error(SemanticDiagnostic {
                            message: "position() requires 1 vector argument".to_string(),
                            span: None,
                        })
                    }
                }
                "pose" => {
                    if args.len() == 2 {
                        let pos_val = self.eval_expr(&args[0].value);
                        let rot_val = self.eval_expr(&args[1].value);
                        let pt = match pos_val {
                            EvalResult::Value(CompileTimeValue::Vector3(pt)) => Some(pt),
                            EvalResult::Value(CompileTimeValue::Position(p)) => Some(p.point),
                            _ => None,
                        };
                        let q = match rot_val {
                            EvalResult::Value(CompileTimeValue::Quaternion(q)) => Some(q),
                            _ => None,
                        };
                        match (pt, q) {
                            (Some(pt), Some(q)) => EvalResult::Value(CompileTimeValue::Pose(Pose {
                                transform: Transform3D::from_translation_rotation(pt, q),
                            })),
                            _ => EvalResult::Error(SemanticDiagnostic {
                                message: "pose() requires Position/Vector3 and Quaternion arguments".to_string(),
                                span: None,
                            }),
                        }
                    } else {
                        EvalResult::Error(SemanticDiagnostic {
                            message: "pose() requires 2 arguments".to_string(),
                            span: None,
                        })
                    }
                }
                "joints" => {
                    // A joint configuration is a sequence of `Angle` components;
                    // bare numbers and cartesian vectors are rejected.
                    let mut vals = Vec::new();
                    for arg in args {
                        match self.eval_expr(&arg.value) {
                            EvalResult::Value(CompileTimeValue::Angle(a)) => vals.push(a),
                            EvalResult::Error(err) => return EvalResult::Error(err),
                            EvalResult::NotConstant => return EvalResult::NotConstant,
                            EvalResult::Value(other) => {
                                return EvalResult::Error(SemanticDiagnostic {
                                    message: format!(
                                        "joints component expected Angle, found {:?}",
                                        other.get_type()
                                    ),
                                    span: None,
                                });
                            }
                        }
                    }
                    EvalResult::Value(CompileTimeValue::Joints(vals))
                }
                "euler" => {
                    if args.len() == 3 {
                        let r = match self.eval_expr(&args[0].value) {
                            EvalResult::Value(CompileTimeValue::Angle(a)) => a,
                            other => return other,
                        };
                        let p = match self.eval_expr(&args[1].value) {
                            EvalResult::Value(CompileTimeValue::Angle(a)) => a,
                            other => return other,
                        };
                        let y = match self.eval_expr(&args[2].value) {
                            EvalResult::Value(CompileTimeValue::Angle(a)) => a,
                            other => return other,
                        };
                        EvalResult::Value(CompileTimeValue::Quaternion(UnitQuaternion::from_euler(
                            r, p, y,
                        )))
                    } else {
                        EvalResult::Error(SemanticDiagnostic {
                            message: "euler() requires 3 Angle arguments (roll, pitch, yaw)"
                                .to_string(),
                            span: None,
                        })
                    }
                }
                "offset" => {
                    let Some(receiver_arg) = args.first() else {
                        return EvalResult::Error(SemanticDiagnostic {
                            message: "offset requires a receiver argument".to_string(),
                            span: None,
                        });
                    };
                    let receiver = match self.eval_expr(&receiver_arg.value) {
                        EvalResult::Value(value) => value,
                        other => return other,
                    };
                    let canonical =
                        match crate::offset::canonicalize_delta(&receiver.get_type(), &args[1..]) {
                            Ok(canonical) => canonical,
                            Err(message) => {
                                return EvalResult::Error(SemanticDiagnostic {
                                    message,
                                    span: None,
                                });
                            }
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
                        Err(message) => EvalResult::Error(SemanticDiagnostic {
                            message,
                            span: None,
                        }),
                    }
                }
                "quaternion" => {
                    if args.len() == 4 {
                        let w = match self.eval_expr(&args[0].value) {
                            EvalResult::Value(CompileTimeValue::Float(f)) => f,
                            EvalResult::Value(CompileTimeValue::Int(i)) => i as f64,
                            other => return other,
                        };
                        let x = match self.eval_expr(&args[1].value) {
                            EvalResult::Value(CompileTimeValue::Float(f)) => f,
                            EvalResult::Value(CompileTimeValue::Int(i)) => i as f64,
                            other => return other,
                        };
                        let y = match self.eval_expr(&args[2].value) {
                            EvalResult::Value(CompileTimeValue::Float(f)) => f,
                            EvalResult::Value(CompileTimeValue::Int(i)) => i as f64,
                            other => return other,
                        };
                        let z = match self.eval_expr(&args[3].value) {
                            EvalResult::Value(CompileTimeValue::Float(f)) => f,
                            EvalResult::Value(CompileTimeValue::Int(i)) => i as f64,
                            other => return other,
                        };
                        let q = Quaternion::new(w, x, y, z).normalize_or_identity();
                        EvalResult::Value(CompileTimeValue::Quaternion(
                            UnitQuaternion::from_quaternion_unchecked(q),
                        ))
                    } else {
                        EvalResult::Error(SemanticDiagnostic {
                            message: "quaternion() requires 4 float arguments (w, x, y, z)"
                                .to_string(),
                            span: None,
                        })
                    }
                }
                _ => EvalResult::NotConstant,
            },
            Expr::Binary { left, op, right } => {
                let lhs = match self.eval_expr(left) {
                    EvalResult::Value(v) => v,
                    other => return other,
                };
                let rhs = match self.eval_expr(right) {
                    EvalResult::Value(v) => v,
                    other => return other,
                };

                match (lhs, op, rhs) {
                    // Position + Vector3 -> Position
                    (CompileTimeValue::Position(p), BinaryOp::Add, CompileTimeValue::Vector3(v)) => {
                        EvalResult::Value(CompileTimeValue::Position(Position { point: p.point + v }))
                    }
                    // Position - Vector3 -> Position
                    (CompileTimeValue::Position(p), BinaryOp::Sub, CompileTimeValue::Vector3(v)) => {
                        EvalResult::Value(CompileTimeValue::Position(Position { point: p.point - v }))
                    }
                    // Position - Position -> Vector3
                    (CompileTimeValue::Position(p1), BinaryOp::Sub, CompileTimeValue::Position(p2)) => {
                        EvalResult::Value(CompileTimeValue::Vector3(p1.point - p2.point))
                    }
                    // Pose + Vector3 -> Pose
                    (CompileTimeValue::Pose(p), BinaryOp::Add, CompileTimeValue::Vector3(v)) => {
                        let new_t = Transform3D::from_translation_rotation(
                            p.transform.translation + v,
                            p.transform.rotation,
                        );
                        EvalResult::Value(CompileTimeValue::Pose(Pose { transform: new_t }))
                    }
                    // Vector3 + Vector3 -> Vector3
                    (CompileTimeValue::Vector3(v1), BinaryOp::Add, CompileTimeValue::Vector3(v2)) => {
                        EvalResult::Value(CompileTimeValue::Vector3(v1 + v2))
                    }
                    // Vector3 - Vector3 -> Vector3
                    (CompileTimeValue::Vector3(v1), BinaryOp::Sub, CompileTimeValue::Vector3(v2)) => {
                        EvalResult::Value(CompileTimeValue::Vector3(v1 - v2))
                    }
                    // Quaternion * Vector3 -> Vector3
                    (CompileTimeValue::Quaternion(q), BinaryOp::Mul, CompileTimeValue::Vector3(v)) => {
                        EvalResult::Value(CompileTimeValue::Vector3(q.rotate_vector(v)))
                    }
                    _ => EvalResult::Error(SemanticDiagnostic {
                        message: "Invalid binary operation during compile-time evaluation".to_string(),
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
                        message: format!("Cannot negate value of type {:?}", value.get_type()),
                        span: None,
                    }),
                },
                other => other,
            },
            // Member access reads a component of a typed value. When the
            // receiver is a compile-time value (target/const) it folds here;
            // otherwise it stays symbolic and the resolver evaluates it.
            Expr::MemberAccess { object, member } => {
                match self.eval_expr(&Expr::Identifier(object.clone())) {
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
