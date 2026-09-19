use serde::{Deserialize, Serialize};
use thalos_lang::ast::{Expr, Statement, UnaryOp};
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
    silent: bool,
}

impl<'a> TypeChecker<'a> {
    pub fn new(symbol_table: &'a mut SymbolTable) -> Self {
        Self {
            symbol_table,
            diagnostics: Vec::new(),
            current_fn_name: None,
            current_fn_return_type: None,
            silent: false,
        }
    }

    /// Record a diagnostic unless we are in a silent (type-collection) pass.
    fn push_diag(&mut self, message: String) {
        if !self.silent {
            self.diagnostics.push(SemanticDiagnostic {
                message,
                span: None,
            });
        }
    }

    /// Infer the type of an expression without emitting diagnostics.
    ///
    /// Used by tooling (hover) to reuse the exact same inference rules as the
    /// checker while a separate pass owns diagnostic emission.
    pub fn infer_expr_silent(&mut self, expr: &Expr) -> TypedExpr {
        let previous = self.silent;
        self.silent = true;
        let typed = self.infer_expr(expr);
        self.silent = previous;
        typed
    }

    fn check_purity_for_statement(&mut self, stmt_kind: &str) {
        if let Some(ref ret_ty) = self.current_fn_return_type
            && *ret_ty != Type::Unit {
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
                    self.push_diag(format!("Unknown identifier '{}'", id));
                    TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Error,
                        span: None,
                    }
                }
            }
            Expr::Binary { left, op, right } => {
                let typed_left = self.infer_expr(left);
                let typed_right = self.infer_expr(right);

                // If an operand already failed, don't pile on a second
                // diagnostic: the binary operation is not the root cause.
                if typed_left.ty.is_error() || typed_right.ty.is_error() {
                    return TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Error,
                        span: None,
                    };
                }

                match BinaryOpRule::infer(&typed_left.ty, *op, &typed_right.ty) {
                    Ok(res_ty) => TypedExpr {
                        expr: expr.clone(),
                        ty: res_ty,
                        span: None,
                    },
                    Err(err) => {
                        self.push_diag(err);
                        TypedExpr {
                            expr: expr.clone(),
                            ty: Type::Error,
                            span: None,
                        }
                    }
                }
            }
            Expr::Unary {
                op: UnaryOp::Neg,
                operand,
            } => {
                let typed_operand = self.infer_expr(operand);
                if typed_operand.ty.is_error() {
                    return TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Error,
                        span: None,
                    };
                }
                // Negation is additive inversion: it applies to values with an
                // additive inverse, not to every numerically-representable type.
                let negatable = matches!(
                    &typed_operand.ty,
                    Type::Int
                        | Type::Float
                        | Type::Length
                        | Type::Angle
                        | Type::Duration
                        | Type::Vector3
                );
                if !negatable {
                    self.push_diag(format!(
                        "Cannot negate value of type {:?}",
                        typed_operand.ty
                    ));
                    return TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Error,
                        span: None,
                    };
                }
                TypedExpr {
                    expr: expr.clone(),
                    ty: typed_operand.ty,
                    span: None,
                }
            }
            Expr::Vector3([x, y, z]) => {
                let elems = [self.infer_expr(x), self.infer_expr(y), self.infer_expr(z)];
                TypedExpr {
                    expr: expr.clone(),
                    ty: if elems.iter().any(|e| e.ty.is_error()) {
                        Type::Error
                    } else {
                        Type::Vector3
                    },
                    span: None,
                }
            }
            Expr::Call { callee, args } => {
                let arg_types: Vec<TypedExpr> =
                    args.iter().map(|a| self.infer_expr(&a.value)).collect();
                let param_types: Vec<Type> = arg_types.iter().map(|a| a.ty.clone()).collect();

                // `joints` is a variadic constructor: any number of Angle/Length/
                // Number values, or a single Vector3 expanded to three. The
                // fixed-arity overload model cannot express it, so it is handled
                // explicitly (mirrors the compile-time evaluator).
                if callee == "joints" {
                    if arg_types.iter().any(|a| a.ty.is_error()) {
                        return TypedExpr {
                            expr: expr.clone(),
                            ty: Type::Error,
                            span: None,
                        };
                    }
                    let dimension = if arg_types.len() == 1 && param_types[0] == Type::Vector3 {
                        Some(3)
                    } else {
                        Some(arg_types.len())
                    };
                    return TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Joints { dimension },
                        span: None,
                    };
                }

                // `offset` is receiver-type-directed: the receiver's type
                // decides the delta shape and component names. Canonicalization
                // is shared with the evaluator (crate::offset).
                if callee == "offset" {
                    let Some(receiver_arg) = args.first() else {
                        self.push_diag("offset requires a receiver argument".to_string());
                        return TypedExpr {
                            expr: expr.clone(),
                            ty: Type::Error,
                            span: None,
                        };
                    };
                    let receiver = self.infer_expr(&receiver_arg.value);
                    if receiver.ty.is_error() {
                        return TypedExpr {
                            expr: expr.clone(),
                            ty: Type::Error,
                            span: None,
                        };
                    }
                    let canonical = match crate::offset::canonicalize_delta(&receiver.ty, &args[1..])
                    {
                        Ok(canonical) => canonical,
                        Err(message) => {
                            self.push_diag(message);
                            return TypedExpr {
                                expr: expr.clone(),
                                ty: Type::Error,
                                span: None,
                            };
                        }
                    };
                    let mut ok = true;
                    match canonical.shape {
                        crate::offset::DeltaShape::Vector3 => {
                            let delta_ty = self.infer_expr(&canonical.args[0].value).ty;
                            if !delta_ty.is_error() && delta_ty != Type::Vector3 {
                                self.push_diag(format!(
                                    "offset delta expected Vector3, got {:?}",
                                    delta_ty
                                ));
                                ok = false;
                            }
                        }
                        crate::offset::DeltaShape::Scalars => {
                            for arg in &canonical.args {
                                let delta_ty = self.infer_expr(&arg.value).ty;
                                if !delta_ty.is_error() && !is_offset_scalar(&delta_ty) {
                                    self.push_diag(format!(
                                        "offset joint delta expected Angle, Length, or number, got {:?}",
                                        delta_ty
                                    ));
                                    ok = false;
                                }
                            }
                        }
                    }
                    let ty = if ok { receiver.ty } else { Type::Error };
                    return TypedExpr {
                        expr: expr.clone(),
                        ty,
                        span: None,
                    };
                }

                if let Some(symbols) = self.symbol_table.lookup(callee) {
                    // If an argument already failed to type-check, the call
                    // itself is not the root cause: suppress the overload error.
                    if param_types.iter().any(Type::is_error) {
                        return TypedExpr {
                            expr: expr.clone(),
                            ty: Type::Error,
                            span: None,
                        };
                    }

                    // Overload matching
                    let mut matched_return = None;
                    for sym in symbols {
                        if let Type::Function(ref ft) = sym.ty
                            && ft.params == param_types {
                                matched_return = Some(*ft.return_type.clone());
                                break;
                            }
                    }

                    if let Some(ret_ty) = matched_return {
                        TypedExpr {
                            expr: expr.clone(),
                            ty: ret_ty,
                            span: None,
                        }
                    } else {
                        self.push_diag(format!(
                            "No matching overload for call '{}' with argument types {:?}",
                            callee, param_types
                        ));
                        TypedExpr {
                            expr: expr.clone(),
                            ty: Type::Error,
                            span: None,
                        }
                    }
                } else {
                    self.push_diag(format!("Unknown function '{}'", callee));
                    TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Error,
                        span: None,
                    }
                }
            }
            Expr::MemberAccess { object, member } => {
                // `object` is a bare identifier in v1. Resolve its type first;
                // the member is then looked up in that type's schema. This also
                // means a namespace like `module.channel` no longer maps to a
                // value: if `module` is not a declared value, the receiver
                // inference reports it (no dead ChannelAccess fallback).
                let receiver = self.infer_expr(&Expr::Identifier(object.clone()));
                if receiver.ty.is_error() {
                    return TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Error,
                        span: None,
                    };
                }
                match receiver.ty.member_schema() {
                    Some(schema) => match schema.field_ty(member) {
                        Some(member_ty) => TypedExpr {
                            expr: expr.clone(),
                            ty: member_ty.clone(),
                            span: None,
                        },
                        None => {
                            let available: Vec<&str> = schema.names().collect();
                            self.push_diag(format!(
                                "Unknown member '{}' on type {:?}; available: {}",
                                member,
                                receiver.ty,
                                available.join(", ")
                            ));
                            TypedExpr {
                                expr: expr.clone(),
                                ty: Type::Error,
                                span: None,
                            }
                        }
                    },
                    None => {
                        self.push_diag(format!(
                            "Type {:?} has no member '{}'",
                            receiver.ty, member
                        ));
                        TypedExpr {
                            expr: expr.clone(),
                            ty: Type::Error,
                            span: None,
                        }
                    }
                }
            }
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
                if !typed_cond.ty.is_error() && typed_cond.ty != Type::Bool {
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
                        if !typed_val.ty.is_error() && expected_ty != typed_val.ty {
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
                if !typed_target.ty.is_error() && !typed_target.ty.is_target() {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("movej expected a target (Position, Pose, or Joints), got {:?}", typed_target.ty),
                        span: None,
                    });
                }
            }
            Statement::MoveL { target } => {
                self.check_purity_for_statement("movel");
                let typed_target = self.infer_expr(target);
                if !typed_target.ty.is_error() && !typed_target.ty.is_spatial_target() {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("movel expected a spatial target (Position or Pose), got {:?}", typed_target.ty),
                        span: None,
                    });
                }
            }
            Statement::Wait(dur_expr) => {
                self.check_purity_for_statement("wait");
                let typed_dur = self.infer_expr(dur_expr);
                if !typed_dur.ty.is_error() && typed_dur.ty != Type::Duration {
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
                if let Some(ref ret_ty) = self.current_fn_return_type
                    && *ret_ty != Type::Unit && typed_expr.ty == Type::Unit {
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
            _ => {}
        }
    }
}

fn is_offset_scalar(ty: &Type) -> bool {
    matches!(ty, Type::Angle | Type::Length | Type::Float | Type::Int)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scope::ScopeKind;
    use thalos_lang::ast::BinaryOp;

    fn table_with_locals() -> SymbolTable {
        let mut table = SymbolTable::new();
        table
            .declare(Symbol::new("ptt", SymbolKind::Target, Type::Position, None))
            .unwrap();
        table
            .declare(Symbol::new("offset1", SymbolKind::Variable, Type::Vector3, None))
            .unwrap();
        table.push_scope(ScopeKind::Function);
        table
    }

    #[test]
    fn unknown_identifier_does_not_cascade_into_operand_errors() {
        let mut table = table_with_locals();
        let mut checker = TypeChecker::new(&mut table);
        checker.current_fn_name = Some("main".to_string());
        checker.current_fn_return_type = Some(Type::Unit);

        // movel(ptt2 - offset1) with `ptt2` undeclared: the only root cause is
        // the unknown identifier, not the subtraction nor the movel argument.
        let stmt = Statement::MoveL {
            target: Expr::Binary {
                left: Box::new(Expr::Identifier("ptt2".to_string())),
                op: BinaryOp::Sub,
                right: Box::new(Expr::Identifier("offset1".to_string())),
            },
        };
        checker.check_statement(&stmt);

        let messages: Vec<&str> = checker
            .diagnostics
            .iter()
            .map(|d| d.message.as_str())
            .collect();
        assert_eq!(
            messages,
            vec!["Unknown identifier 'ptt2'"],
            "cascading diagnostics must be suppressed, got: {messages:?}"
        );
    }

    #[test]
    fn unknown_function_does_not_cascade_into_call_errors() {
        let mut table = table_with_locals();
        let mut checker = TypeChecker::new(&mut table);
        checker.current_fn_name = Some("main".to_string());
        checker.current_fn_return_type = Some(Type::Unit);

        // movel(mystery_position()) where the function is unknown.
        let stmt = Statement::MoveL {
            target: Expr::Call {
                callee: "mystery_position".to_string(),
                args: vec![],
            },
        };
        checker.check_statement(&stmt);

        let messages: Vec<&str> = checker
            .diagnostics
            .iter()
            .map(|d| d.message.as_str())
            .collect();
        assert_eq!(
            messages,
            vec!["Unknown function 'mystery_position'"],
            "cascading diagnostics must be suppressed, got: {messages:?}"
        );
    }

    fn table_with_members() -> SymbolTable {
        let mut table = SymbolTable::new();
        table
            .declare(Symbol::new("ptt", SymbolKind::Target, Type::Position, None))
            .unwrap();
        table
            .declare(Symbol::new(
                "jtt",
                SymbolKind::Target,
                Type::Joints { dimension: Some(6) },
                None,
            ))
            .unwrap();
        table
            .declare(Symbol::new("len", SymbolKind::Target, Type::Length, None))
            .unwrap();
        table
    }

    fn member_ty(table: &mut SymbolTable, object: &str, member: &str) -> Type {
        let checker = &mut TypeChecker::new(table);
        checker
            .infer_expr_silent(&Expr::MemberAccess {
                object: object.to_string(),
                member: member.to_string(),
            })
            .ty
    }

    fn member_diagnostics(table: &mut SymbolTable, object: &str, member: &str) -> Vec<String> {
        let mut checker = TypeChecker::new(table);
        checker.infer_expr(&Expr::MemberAccess {
            object: object.to_string(),
            member: member.to_string(),
        });
        checker
            .diagnostics
            .into_iter()
            .map(|d| d.message)
            .collect()
    }

    #[test]
    fn member_access_types_flow_from_the_schema() {
        let mut table = table_with_members();
        assert_eq!(member_ty(&mut table, "ptt", "x"), Type::Length);
        assert_eq!(member_ty(&mut table, "ptt", "z"), Type::Length);
        assert_eq!(member_ty(&mut table, "jtt", "j2"), Type::Angle);
        assert_eq!(member_ty(&mut table, "jtt", "j6"), Type::Angle);
    }

    #[test]
    fn unknown_member_on_position_is_rejected_from_the_schema() {
        let mut table = table_with_members();
        let diagnostics = member_diagnostics(&mut table, "ptt", "j2");
        assert_eq!(diagnostics.len(), 1);
        assert!(
            diagnostics[0].contains("Unknown member 'j2' on type Position"),
            "got: {diagnostics:?}"
        );
        assert!(diagnostics[0].contains("x, y, z"));
    }

    #[test]
    fn unknown_member_on_joints_lists_joint_members() {
        let mut table = table_with_members();
        let diagnostics = member_diagnostics(&mut table, "jtt", "x");
        assert_eq!(diagnostics.len(), 1);
        assert!(diagnostics[0].contains("Unknown member 'x'"), "got: {diagnostics:?}");
        assert!(diagnostics[0].contains("j1"));
    }

    #[test]
    fn out_of_range_joint_member_is_rejected() {
        let mut table = table_with_members();
        let diagnostics = member_diagnostics(&mut table, "jtt", "j7");
        assert_eq!(diagnostics.len(), 1);
        assert!(diagnostics[0].contains("Unknown member 'j7'"), "got: {diagnostics:?}");
    }

    #[test]
    fn member_on_type_without_schema_is_rejected() {
        let mut table = table_with_members();
        let diagnostics = member_diagnostics(&mut table, "len", "x");
        assert_eq!(
            diagnostics,
            vec!["Type Length has no member 'x'".to_string()]
        );
    }

    #[test]
    fn namespace_style_access_reports_the_unknown_receiver() {
        let mut table = table_with_members();
        let diagnostics = member_diagnostics(&mut table, "module", "channel");
        assert_eq!(diagnostics, vec!["Unknown identifier 'module'".to_string()]);
    }
}
