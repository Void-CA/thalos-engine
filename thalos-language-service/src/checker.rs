use serde::{Deserialize, Serialize};
use thalos_lang::ast::{Arg, Expr, Statement, UnaryOp};
use thalos_lang::span::Span;
use crate::operators::BinaryOpRule;
use crate::registry::{registry, ConstructorShape, ConstructorSpec, TypeRegistry};
use crate::reactive::SignalSchema;
use crate::scope::SymbolTable;
use crate::symbols::{Symbol, SymbolKind};
use crate::types::{resolve_type, Dimension, Type};

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
    /// Declared signals: `module.channel` reads are typed against this.
    signals: Option<&'a SignalSchema>,
    /// `use` aliases: a local name maps to the real signal module.
    signal_aliases: Option<&'a std::collections::HashMap<String, String>>,
    /// Declared process names. `start <process>` is only valid for one of these
    /// inside `main`.
    processes: Option<&'a std::collections::HashSet<String>>,
    /// Constructor/member registry: the single source of constructor semantics.
    registry: &'static TypeRegistry,
    silent: bool,
}

impl<'a> TypeChecker<'a> {
    pub fn new(symbol_table: &'a mut SymbolTable) -> Self {
        Self {
            symbol_table,
            diagnostics: Vec::new(),
            current_fn_name: None,
            current_fn_return_type: None,
            signals: None,
            signal_aliases: None,
            processes: None,
            registry: registry(),
            silent: false,
        }
    }

    /// Attach a signal schema so `module.channel` reads type-check as signals.
    pub fn with_signals(mut self, signals: &'a SignalSchema) -> Self {
        self.signals = Some(signals);
        self
    }

    /// Attach `use` aliases so an aliased module resolves to the real one.
    pub fn with_signal_aliases(
        mut self,
        aliases: &'a std::collections::HashMap<String, String>,
    ) -> Self {
        self.signal_aliases = Some(aliases);
        self
    }

    /// Attach the declared process names so `start <process>` can be validated.
    pub fn with_processes(mut self, processes: &'a std::collections::HashSet<String>) -> Self {
        self.processes = Some(processes);
        self
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
                        | Type::Speed
                        | Type::AngularSpeed
                        | Type::Vector3
                );
                if !negatable {
                    self.push_diag(format!(
                        "Cannot negate value of type {}",
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
            Expr::Vector3(components) => {
                // A `Vector3` is a geometric displacement: each component must be
                // a `Length`. Bare `Number` components are rejected (no implicit
                // Number -> dimension coercion).
                let elems = [
                    self.infer_expr(&components[0]),
                    self.infer_expr(&components[1]),
                    self.infer_expr(&components[2]),
                ];
                if elems.iter().any(|e| e.ty.is_error()) {
                    return TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Error,
                        span: None,
                    };
                }
                let mut ok = true;
                for elem in &elems {
                    if elem.ty != Type::Length {
                        self.push_diag(format!(
                            "vector component expected Length, found {}",
                            elem.ty
                        ));
                        ok = false;
                    }
                }
                TypedExpr {
                    expr: expr.clone(),
                    ty: if ok { Type::Vector3 } else { Type::Error },
                    span: None,
                }
            }
            Expr::Call { callee, args } => {
                let arg_types: Vec<TypedExpr> =
                    args.iter().map(|a| self.infer_expr(&a.value)).collect();
                let param_types: Vec<Type> = arg_types.iter().map(|a| a.ty.clone()).collect();

                // Constructors are declared in the registry. Fixed-arity
                // constructors, variadic `joints` and receiver-directed
                // `offset` share one dispatch point.
                if let Some(overloads) = self.registry.constructor_overloads(callee) {
                    return self.check_constructor_call(expr, args, &arg_types, overloads);
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

                    // Overload matching, with numeric widening Int -> Float
                    // (a `Float` parameter accepts an `Int` argument). This is
                    // representation promotion, never dimensional coercion.
                    let mut matched_return = None;
                    for sym in symbols {
                        if let Type::Function(ref ft) = sym.ty
                            && ft.params.len() == param_types.len()
                            && ft
                                .params
                                .iter()
                                .zip(&param_types)
                                .all(|(param, actual)| param.accepts(actual))
                        {
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
                            "No matching overload for call '{}' with argument types {}",
                            callee,
                            param_types.iter().map(ToString::to_string).collect::<Vec<_>>().join(", ")
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
                // A declared signal module: `module.channel` is a signal read,
                // typed by the schema. An alias resolves to the real module.
                // Only an identifier can be a module; a chained receiver is a
                // value member.
                if let Expr::Identifier(module_name) = object.as_ref() {
                    let module = self
                        .signal_aliases
                        .and_then(|aliases| aliases.get(module_name))
                        .map(String::as_str)
                        .unwrap_or(module_name.as_str());
                    if let Some(signals) = self.signals
                        && signals.knows_module(module)
                    {
                        return match signals.type_of(module, member) {
                            Some(ty) => TypedExpr {
                                expr: expr.clone(),
                                ty: ty.as_type(),
                                span: None,
                            },
                            None => {
                                self.push_diag(format!(
                                    "Module '{}' has no signal '{}'",
                                    module, member
                                ));
                                TypedExpr {
                                    expr: expr.clone(),
                                    ty: Type::Error,
                                    span: None,
                                }
                            }
                        };
                    }
                }
                // Resolve the receiver expression, then look the member up in
                // its type's schema. Chained receivers (`Position(...).x`) use
                // the same path.
                let receiver = self.infer_expr(object);
                if receiver.ty.is_error() {
                    return TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Error,
                        span: None,
                    };
                }
                if matches!(
                    receiver.ty,
                    Type::Joints {
                        dimension: Dimension::Unknown
                    }
                ) {
                    self.push_diag(format!(
                        "Joints dimension is unknown; annotate as Joints<N> to access '{}'",
                        member
                    ));
                    return TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Error,
                        span: None,
                    };
                }
                match self.registry.member_spec(&receiver.ty) {
                    Some(schema) => match schema.field_ty(member) {
                        Some(member_ty) => TypedExpr {
                            expr: expr.clone(),
                            ty: member_ty.clone(),
                            span: None,
                        },
                        None => {
                            let available: Vec<&str> = schema.names().collect();
                            self.push_diag(format!(
                                "Unknown member '{}' on type {}; available: {}",
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
                            "Type {} has no member '{}'",
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
            // Member calls are desugared by the binder (`offset`) or become
            // associated constructors (`Quaternion.euler`). Reaching this arm
            // means normalization already reported the problem; poison the type
            // without duplicating the diagnostic.
            Expr::MemberCall { .. } => TypedExpr {
                expr: expr.clone(),
                ty: Type::Error,
                span: None,
            },
        }
    }

    fn constructor_error(&mut self, expr: &Expr, message: String) -> TypedExpr {
        self.push_diag(message);
        TypedExpr {
            expr: expr.clone(),
            ty: Type::Error,
            span: None,
        }
    }

    /// Check a call to a registry constructor. The registry owns the signature,
    /// arity and shape; this method only applies them to the inferred arguments.
    fn check_constructor_call(
        &mut self,
        expr: &Expr,
        args: &[Arg],
        arg_types: &[TypedExpr],
        overloads: &[ConstructorSpec],
    ) -> TypedExpr {
        let callee = overloads[0].name;
        if arg_types.iter().any(|arg| arg.ty.is_error()) {
            return TypedExpr {
                expr: expr.clone(),
                ty: Type::Error,
                span: None,
            };
        }

        match &overloads[0].shape {
            ConstructorShape::Fixed => {
                let param_types: Vec<Type> = arg_types.iter().map(|arg| arg.ty.clone()).collect();
                let matched = overloads.iter().find(|spec| {
                    spec.params.len() == param_types.len()
                        && spec
                            .params
                            .iter()
                            .zip(&param_types)
                            .all(|(param, actual)| param.accepts(actual))
                });
                match matched {
                    Some(spec) => TypedExpr {
                        expr: expr.clone(),
                        ty: spec.returns.clone(),
                        span: None,
                    },
                    None => self.constructor_error(
                        expr,
                        format!(
                            "No matching overload for constructor '{}' with argument types {}",
                            callee,
                            param_types
                                .iter()
                                .map(ToString::to_string)
                                .collect::<Vec<_>>()
                                .join(", ")
                        ),
                    ),
                }
            }
            ConstructorShape::Variadic { param } => {
                if arg_types.is_empty() {
                    return self.constructor_error(
                        expr,
                        format!("{} requires at least one {}", callee, param),
                    );
                }
                let mut ok = true;
                for arg in arg_types {
                    if !param.accepts(&arg.ty) {
                        self.push_diag(format!(
                            "{} component expected {}, found {}",
                            callee, param, arg.ty
                        ));
                        ok = false;
                    }
                }
                TypedExpr {
                    expr: expr.clone(),
                    ty: if ok {
                        Type::Joints {
                            dimension: crate::types::Dimension::Known(arg_types.len()),
                        }
                    } else {
                        Type::Error
                    },
                    span: None,
                }
            }
            ConstructorShape::ReceiverMethod => {
                let Some(receiver) = arg_types.first() else {
                    return self
                        .constructor_error(expr, format!("{} requires a receiver argument", callee));
                };
                if receiver.ty.is_error() {
                    return TypedExpr {
                        expr: expr.clone(),
                        ty: Type::Error,
                        span: None,
                    };
                }
                let canonical =
                    match crate::offset::canonicalize_delta(&receiver.ty, &args[1..]) {
                        Ok(canonical) => canonical,
                        Err(message) => return self.constructor_error(expr, message),
                    };
                let mut ok = true;
                match canonical.shape {
                    crate::offset::DeltaShape::Vector3 => {
                        let delta_ty = self.infer_expr(&canonical.args[0].value).ty;
                        if !delta_ty.is_error() && delta_ty != Type::Vector3 {
                            self.push_diag(format!(
                                "offset delta expected Vector3, got {}",
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
                                    "offset joint delta expected Angle, found {}",
                                    delta_ty
                                ));
                                ok = false;
                            }
                        }
                    }
                }
                TypedExpr {
                    expr: expr.clone(),
                    ty: if ok { receiver.ty.clone() } else { Type::Error },
                    span: None,
                }
            }
        }
    }

    pub fn check_statement(&mut self, stmt: &Statement) {
        match stmt {
            Statement::If { condition, then_branch, else_branch } => {
                let typed_cond = self.infer_expr(condition);
                if !typed_cond.ty.is_error() && typed_cond.ty != Type::Bool {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("if condition expected Bool, got {}", typed_cond.ty),
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
                let declared_ty = match type_ann {
                    Some(ann) => match resolve_type(ann) {
                        Ok(expected_ty) => {
                            // Numeric widening (Int -> Float) applies to
                            // annotations; physical dimensions require an exact
                            // match.
                            if !typed_val.ty.is_error() && !expected_ty.accepts(&typed_val.ty) {
                                self.diagnostics.push(SemanticDiagnostic {
                                    message: format!(
                                        "Type mismatch in let binding '{}': expected {}, got {}",
                                        name, expected_ty, typed_val.ty
                                    ),
                                    span: Some(ann.span()),
                                });
                            }
                            expected_ty
                        }
                        Err(error) => {
                            self.diagnostics.push(SemanticDiagnostic {
                                message: error.message,
                                span: error.span,
                            });
                            Type::Error
                        }
                    },
                    None => typed_val.ty.clone(),
                };
                // The annotation is a checked assertion; the binding keeps the
                // inferred type so the symbol and the value never diverge (e.g.
                // `let n: Float = 1` binds an `Int`).
                let symbol_ty = if declared_ty.is_error() {
                    Type::Error
                } else {
                    typed_val.ty.clone()
                };
                let _ = self.symbol_table.declare(Symbol::new(
                    name.clone(),
                    SymbolKind::Variable,
                    symbol_ty,
                    None,
                ));
            }
            Statement::MoveJ { target } => {
                let typed_target = self.infer_expr(target);
                if !typed_target.ty.is_error() && !typed_target.ty.is_target() {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("movej expected a target (Position, Pose, or Joints), got {}", typed_target.ty),
                        span: None,
                    });
                }
            }
            Statement::MoveL { target } => {
                let typed_target = self.infer_expr(target);
                if !typed_target.ty.is_error() && !typed_target.ty.is_spatial_target() {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("movel expected a spatial target (Position or Pose), got {}", typed_target.ty),
                        span: None,
                    });
                }
            }
            Statement::MoveC { via, target } => {
                let typed_via = self.infer_expr(via);
                let typed_target = self.infer_expr(target);
                if typed_via.ty.is_error() || typed_target.ty.is_error() {
                    return;
                }
                if !typed_via.ty.is_spatial_target() || !typed_target.ty.is_spatial_target() {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!(
                            "movec expected Position or Pose for both via and target, got {} and {}",
                            typed_via.ty, typed_target.ty
                        ),
                        span: None,
                    });
                } else if typed_via.ty != typed_target.ty {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!(
                            "movec via and target must have the same type, got {} and {}",
                            typed_via.ty, typed_target.ty
                        ),
                        span: None,
                    });
                }
            }
            Statement::Wait(dur_expr) => {
                let typed_dur = self.infer_expr(dur_expr);
                if !typed_dur.ty.is_error() && typed_dur.ty != Type::Duration {
                    self.diagnostics.push(SemanticDiagnostic {
                        message: format!("wait expected Duration, got {}", typed_dur.ty),
                        span: None,
                    });
                }
            }
            Statement::SetOutput { .. } => {
            }
            Statement::Assign { name, .. } => {
                self.push_diag(format!(
                    "Assignment to '{}' is only valid inside a process",
                    name
                ));
            }
            Statement::Request { name, .. } => {
                self.push_diag(format!(
                    "request '{}' is only valid inside a process",
                    name
                ));
            }
            Statement::Start { process } => {
                if self.current_fn_name.as_deref() != Some("main") {
                    self.push_diag(format!(
                        "start '{}' is only valid inside main",
                        process
                    ));
                } else if let Some(processes) = self.processes
                    && !processes.contains(process)
                {
                    self.push_diag(format!("Unknown process '{}'", process));
                }
            }
            Statement::Expr(expr) => {
                // Expression statements are checked for their own errors; the
                // purity of a non-Unit function is decided by effect analysis.
                let _ = self.infer_expr(expr);
            }
        }
    }
}

/// `offset` joint deltas address a revolute joint, so they must be an `Angle`.
/// Prismatic joints (`Length`) are a future joint-kind profile.
fn is_offset_scalar(ty: &Type) -> bool {
    matches!(ty, Type::Angle)
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
                Type::Joints { dimension: crate::types::Dimension::Known(6) },
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
                object: Box::new(Expr::Identifier(object.to_string())),
                member: member.to_string(),
            })
            .ty
    }

    fn member_diagnostics(table: &mut SymbolTable, object: &str, member: &str) -> Vec<String> {
        let mut checker = TypeChecker::new(table);
        checker.infer_expr(&Expr::MemberAccess {
            object: Box::new(Expr::Identifier(object.to_string())),
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
