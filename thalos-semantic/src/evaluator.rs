use serde::{Deserialize, Serialize};
use thalos_lang::ast::{BinaryOp, Expr};
use thalos_math::{Transform3D, UnitQuaternion, Vector3};
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
                let x = match self.eval_expr(x_expr) {
                    EvalResult::Value(CompileTimeValue::Length(v)) => v,
                    EvalResult::Value(CompileTimeValue::Float(v)) => v,
                    EvalResult::Value(CompileTimeValue::Int(v)) => v as f64,
                    other => return other,
                };
                let y = match self.eval_expr(y_expr) {
                    EvalResult::Value(CompileTimeValue::Length(v)) => v,
                    EvalResult::Value(CompileTimeValue::Float(v)) => v,
                    EvalResult::Value(CompileTimeValue::Int(v)) => v as f64,
                    other => return other,
                };
                let z = match self.eval_expr(z_expr) {
                    EvalResult::Value(CompileTimeValue::Length(v)) => v,
                    EvalResult::Value(CompileTimeValue::Float(v)) => v,
                    EvalResult::Value(CompileTimeValue::Int(v)) => v as f64,
                    other => return other,
                };
                EvalResult::Value(CompileTimeValue::Vector3(Vector3::new(x, y, z)))
            }
            Expr::Identifier(id) => {
                if let Some(values) = self.target_values {
                    if let Some(val) = values.get(id) {
                        return EvalResult::Value(val.clone());
                    }
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
                        match self.eval_expr(&args[0]) {
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
                        let pos_val = self.eval_expr(&args[0]);
                        let rot_val = self.eval_expr(&args[1]);
                        match (pos_val, rot_val) {
                            (
                                EvalResult::Value(CompileTimeValue::Vector3(pt)),
                                EvalResult::Value(CompileTimeValue::Quaternion(q)),
                            ) => EvalResult::Value(CompileTimeValue::Pose(Pose {
                                transform: Transform3D::from_translation_rotation(pt, q),
                            })),
                            _ => EvalResult::Error(SemanticDiagnostic {
                                message: "pose() requires Vector3 and Quaternion arguments".to_string(),
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
                    let mut vals = Vec::new();
                    for arg in args {
                        match self.eval_expr(arg) {
                            EvalResult::Value(CompileTimeValue::Angle(a)) => vals.push(a),
                            EvalResult::Value(CompileTimeValue::Length(l)) => vals.push(l),
                            EvalResult::Value(CompileTimeValue::Float(f)) => vals.push(f),
                            other => return other,
                        }
                    }
                    EvalResult::Value(CompileTimeValue::Joints(vals))
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
            _ => EvalResult::NotConstant,
        }
    }
}
