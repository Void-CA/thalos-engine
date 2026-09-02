use serde::{Deserialize, Serialize};
use thalos_lang::ast::{Expr, Statement};
use thalos_lang::span::Span;
use crate::operators::BinaryOpRule;
use crate::scope::SymbolTable;
use crate::symbols::{Symbol, SymbolKind};
use crate::types::Type;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TypedExpr {
    pub expr: Expr,
    pub ty: Type,
    pub span: Option<Span>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SemanticDiagnostic {
    pub message: String,
    pub span: Option<Span>,
}

pub struct TypeChecker<'a> {
    pub symbol_table: &'a mut SymbolTable,
    pub diagnostics: Vec<SemanticDiagnostic>,
    pub current_fn_name: Option<String>,
    pub current_fn_return_type: Option<Type>,
}

impl<'a> TypeChecker<'a> {
    pub fn new(symbol_table: &'a mut SymbolTable) -> Self {
        Self {
            symbol_table,
            diagnostics: Vec::new(),
            current_fn_name: None,
            current_fn_return_type: None,
        }
    }

    fn check_purity_for_statement(&mut self, stmt_kind: &str) {
        if let Some(ref ret_ty) = self.current_fn_return_type {
            if *ret_ty != Type::Unit {
                let fn_name = self.current_fn_name.clone().unwrap_or_default();
                self.diagnostics.push(SemanticDiagnostic {
                    message: format!(
                        "Function '{}' has non-Unit return type {:?} and cannot produce robotic/IO effects through '{}'",
                        fn_name, ret_ty, stmt_kind
                    ),
                    span: None,
                });
            }
        }
    }

    pub fn infer_expr(&mut self, expr: &Expr) -> TypedExpr {
        match expr {
            Expr::Number(n) => TypedExpr {
                expr: expr.clone(),
                ty: if n.fract() == 0.0 { Type::Int } else { Type::Float },
                span: None,
            },
            Expr::Boolean(_) => TypedExpr {
                expr: expr.clone(),
                ty: Type::Bool,
                span: None,
            },
            Expr::StringLiteral(_) => TypedExpr {
                expr: expr.clone(),
                ty: Type::String,
                span: None,
            },
            Expr::Length(_) => TypedExpr {
                expr: expr.clone(),
                ty: Type::Length,
                span: None,
            },
            Expr::Angle(_) => TypedExpr {
                expr: expr.clone(),
                ty: Type::Angle,
                span: None,
            },
            Expr::Duration(_) => TypedExpr {
                expr: expr.clone(),
                ty: Type::Duration,
                span: None,
            },
            Expr::Identifier(id) => {
                if let Some(symbols) = self.symbol_table.lookup(id) {
                    TypedExpr {
                        expr: expr.clone(),
                        ty: symbols[0].ty.clone(),
                        span: None,
                    }
                } else {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("Unknown identifier '{}'", id),
                        span: None,
                    });
                    TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Unit,
                        span: None,
                    }
                }
            }
            Expr::Binary { left, op, right } => {
                let typed_left = self.infer_expr(left);
                let typed_right = self.infer_expr(right);

                match BinaryOpRule::infer(&typed_left.ty, *op, &typed_right.ty) {
                    Ok(res_ty) => TypedExpr {
                        expr: expr.clone(),
                        ty: res_ty,
                        span: None,
                    },
                    Err(err) => {
                        self.diagnostics.push(SemanticDiagnostic {
                            message: err,
                            span: None,
                        });
                        TypedExpr {
                            expr: expr.clone(),
                            ty: Type::Unit,
                            span: None,
                        }
                    }
                }
            }
            Expr::Vector3([x, y, z]) => {
                let _ = (self.infer_expr(x), self.infer_expr(y), self.infer_expr(z));
                TypedExpr {
                    expr: expr.clone(),
                    ty: Type::Vector3,
                    span: None,
                }
            }
            Expr::Call { callee, args } => {
                let arg_types: Vec<TypedExpr> = args.iter().map(|a| self.infer_expr(a)).collect();
                let param_types: Vec<Type> = arg_types.iter().map(|a| a.ty.clone()).collect();

                if let Some(symbols) = self.symbol_table.lookup(callee) {
                    // Overload matching
                    let mut matched_return = None;
                    for sym in symbols {
                        if let Type::Function(ref ft) = sym.ty {
                            if ft.params == param_types {
                                matched_return = Some(*ft.return_type.clone());
                                break;
                            }
                        }
                    }

                    if let Some(ret_ty) = matched_return {
                        TypedExpr {
                            expr: expr.clone(),
                            ty: ret_ty,
                            span: None,
                        }
                    } else {
                        self.diagnostics.push(SemanticDiagnostic {
                            message: format!(
                                "No matching overload for call '{}' with argument types {:?}",
                                callee, param_types
                            ),
                            span: None,
                        });
                        TypedExpr {
                            expr: expr.clone(),
                            ty: Type::Unit,
                            span: None,
                        }
                    }
                } else {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("Unknown function '{}'", callee),
                        span: None,
                    });
                    TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Unit,
                        span: None,
                    }
                }
            }
            Expr::MemberAccess { .. } => TypedExpr {
                expr: expr.clone(),
                ty: Type::Float,
                span: None,
            },
            _ => TypedExpr {
                expr: expr.clone(),
                ty: Type::Unit,
                span: None,
            },
        }
    }

    pub fn check_statement(&mut self, stmt: &Statement) {
        match stmt {
            Statement::If { condition, then_branch, else_branch } => {
                let typed_cond = self.infer_expr(condition);
                if typed_cond.ty != Type::Bool {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("if condition expected Bool, got {:?}", typed_cond.ty),
                        span: None,
                    });
                }
                for s in then_branch {
                    self.check_statement(s);
                }
                if let Some(else_stmts) = else_branch {
                    for s in else_stmts {
                        self.check_statement(s);
                    }
                }
            }
            Statement::Let { name, type_ann, value } => {
                let typed_val = self.infer_expr(value);
                if let Some(ann) = type_ann {
                    if let Some(expected_ty) = Type::from_name(ann) {
                        if expected_ty != typed_val.ty {
                            self.diagnostics.push(SemanticDiagnostic {
                                message: format!(
                                    "Type mismatch in let binding '{}': expected {:?}, got {:?}",
                                    name, expected_ty, typed_val.ty
                                ),
                                span: None,
                            });
                        }
                    } else {
                        self.diagnostics.push(SemanticDiagnostic {
                            message: format!("Unknown type annotation '{}' in let binding '{}'", ann, name),
                            span: None,
                        });
                    }
                }
                let _ = self.symbol_table.declare(Symbol::new(
                    name.clone(),
                    SymbolKind::Variable,
                    typed_val.ty,
                    None,
                ));
            }
            Statement::MoveJ { target } => {
                self.check_purity_for_statement("movej");
                let typed_target = self.infer_expr(target);
                if !typed_target.ty.is_target() {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("movej expected a target (Position, Pose, or Joints), got {:?}", typed_target.ty),
                        span: None,
                    });
                }
            }
            Statement::MoveL { target } => {
                self.check_purity_for_statement("movel");
                let typed_target = self.infer_expr(target);
                if !typed_target.ty.is_spatial_target() {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("movel expected a spatial target (Position or Pose), got {:?}", typed_target.ty),
                        span: None,
                    });
                }
            }
            Statement::Wait(dur_expr) => {
                self.check_purity_for_statement("wait");
                let typed_dur = self.infer_expr(dur_expr);
                if typed_dur.ty != Type::Duration {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("wait expected Duration, got {:?}", typed_dur.ty),
                        span: None,
                    });
                }
            }
            Statement::SetOutput { .. } => {
                self.check_purity_for_statement("set_output");
            }
            Statement::Expr(expr) => {
                let typed_expr = self.infer_expr(expr);
                if let Some(ref ret_ty) = self.current_fn_return_type {
                    if *ret_ty != Type::Unit && typed_expr.ty == Type::Unit {
                        let name = match expr {
                            Expr::Call { callee, .. } => callee.clone(),
                            Expr::MemberCall { object, method, .. } => format!("{}.{}", object, method),
                            _ => "side-effecting statement".to_string(),
                        };
                        let fn_name = self.current_fn_name.clone().unwrap_or_default();
                        self.diagnostics.push(SemanticDiagnostic {
                            message: format!(
                                "Function '{}' has non-Unit return type {:?} and cannot produce robotic/IO effects through '{}'",
                                fn_name, ret_ty, name
                            ),
                            span: None,
                        });
                    }
                }
            }
            _ => {}
        }
    }
}
