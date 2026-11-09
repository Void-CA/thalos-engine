use std::collections::HashMap;
use thalos_lang::ast::{ConstDecl, Expr as AstExpr, Item, Program, Statement as AstStatement, TargetDecl, UseTree};
use crate::builtins::register_builtins;
use crate::checker::TypeChecker;
use crate::evaluator::{CompileTimeValue, EvalResult, Evaluator, Position};
use crate::model::{
    JointConfiguration, MotionKind, MotionTarget, Provenance, ResolvedTarget, SemanticArg,
    SemanticExpr, SemanticFunction, SemanticMotion, SemanticProgram, SemanticStatement,
};
use crate::reactive::{ReactiveProgram, SignalSchema};
use crate::scope::SymbolTable;
use crate::symbols::{Symbol, SymbolKind};
use crate::types::Type;

pub struct SemanticCompiler;

impl SemanticCompiler {
    /// Compile WITHOUT a station signal schema.
    ///
    /// `expect` declarations are accepted syntactically and IGNORED here: the
    /// planning path must not depend on the signal interface. Expectations are
    /// validated by [`SemanticCompiler::compile_with_signals`] or
    /// [`SemanticCompiler::compile_expectations`].
    pub fn compile(ast: &Program) -> Result<SemanticProgram, Vec<String>> {
        Self::compile_impl(ast, &SignalSchema::new(), false)
    }

    /// Compile with a declared signal schema: a `module.channel` read becomes a
    /// first-class [`SemanticExpr::ChannelAccess`], and `expect` declarations
    /// are validated and resolved into `SignalCondition`s.
    pub fn compile_with_signals(
        ast: &Program,
        signals: &SignalSchema,
    ) -> Result<SemanticProgram, Vec<String>> {
        Self::compile_impl(ast, signals, true)
    }

    /// Resolve ONLY the declared expectations, validated against the station
    /// schema. Used before a session exists; never involved in planning.
    pub fn compile_expectations(
        ast: &Program,
        signals: &SignalSchema,
    ) -> Result<Vec<thalos_core::execution::condition::SignalCondition>, Vec<String>> {
        let (expectations, errors) = resolve_expectations(ast, signals);
        if errors.is_empty() { Ok(expectations) } else { Err(errors) }
    }

    fn compile_impl(
        ast: &Program,
        signals: &SignalSchema,
        validate_expectations: bool,
    ) -> Result<SemanticProgram, Vec<String>> {
        // Resolve named arguments into canonical positional form once, up front.
        // Every stage below (evaluator, checker, lowering, resolver) only ever
        // sees positional arguments and never re-implements named-arg semantics.
        let (ast, bind_errors) = crate::binder::normalize_program(ast);
        if !bind_errors.is_empty() {
            return Err(bind_errors);
        }

        // `use` aliases: a local name maps to the real signal module. Shared by
        // the checker (signal typing) and the rewrite pass (ChannelAccess).
        let signal_aliases = build_signal_aliases(&ast);

        let PreparedSymbols {
            mut table,
            target_values,
            const_names,
            resolved_targets,
            mut errors,
        } = prepare_symbols(&ast);

        // 3. Type check AST functions and statements in isolated parameter scope
        let mut checker = TypeChecker::new(&mut table)
            .with_signals(signals)
            .with_signal_aliases(&signal_aliases);
        for item in &ast.items {
            if let Item::Function(f) = item {
                let expected_ret = f
                    .return_type
                    .as_deref()
                    .and_then(Type::from_name)
                    .unwrap_or(Type::Unit);

                checker.current_fn_name = Some(f.name.clone());
                checker.current_fn_return_type = Some(expected_ret.clone());

                checker.symbol_table.push_scope(crate::scope::ScopeKind::Function);
                for param in &f.params {
                    let p_ty = param
                        .type_ann
                        .as_deref()
                        .and_then(Type::from_name)
                        .unwrap_or(Type::Position);

                    let _ = checker.symbol_table.declare(Symbol::new(
                        param.name.clone(),
                        SymbolKind::Parameter,
                        p_ty,
                        None,
                    ));
                }
                for stmt in &f.body {
                    checker.check_statement(stmt);
                }
                if let Some(ref tail) = f.tail_expr {
                    let typed_tail = checker.infer_expr(tail);
                    if !typed_tail.ty.is_error()
                        && expected_ret != Type::Unit
                        && !expected_ret.accepts_numeric_widening(&typed_tail.ty)
                    {
                        checker.diagnostics.push(crate::checker::SemanticDiagnostic {
                            message: format!(
                                "Function '{}' return type mismatch: expected {:?}, got {:?}",
                                f.name, expected_ret, typed_tail.ty
                            ),
                            span: None,
                        });
                    }
                }
                checker.symbol_table.pop_scope();
                checker.current_fn_name = None;
                checker.current_fn_return_type = None;
            }
        }

        if !checker.diagnostics.is_empty() {
            for diag in checker.diagnostics {
                errors.push(diag.message);
            }
        }

        // Declared expectations: validated only when a schema is in play.
        let expectations = if validate_expectations {
            let (resolved, expectation_errors) = resolve_expectations(&ast, signals);
            errors.extend(expectation_errors);
            resolved
        } else {
            Vec::new()
        };

        if !errors.is_empty() {
            return Err(errors);
        }

        // 4. Lower functions to SemanticFunction and SemanticStatement
        let evaluator = Evaluator::with_target_values(&table, &target_values);
        let mut semantic_functions = Vec::new();
        for item in &ast.items {
            if let Item::Function(f) = item {
                let mut body = Vec::new();
                let param_names: Vec<String> = f.params.iter().map(|p| p.name.clone()).collect();
                let mut local_names = Vec::new();

                for stmt in &f.body {
                    body.push(lower_statement(stmt, &evaluator, &param_names, &mut local_names, &const_names));
                }

                let tail_expr = f
                    .tail_expr
                    .as_ref()
                    .map(|e| lower_expr(e, &evaluator, &param_names, &local_names, &const_names));

                let return_type = f
                    .return_type
                    .as_deref()
                    .and_then(Type::from_name)
                    .unwrap_or(Type::Unit);

                semantic_functions.push(SemanticFunction {
                    name: f.name.clone(),
                    params: param_names,
                    return_type,
                    body,
                    tail_expr,
                    provenance: Provenance::new(Some(f.name.clone()), None),
                });
            }
        }

        let mut program = SemanticProgram {
            targets: resolved_targets,
            functions: semantic_functions,
            entry_point: "main".to_string(),
            expectations,
        };
        rewrite_signal_access(&mut program, signals, &signal_aliases);
        Ok(program)
    }

    /// Compile straight to a control-flow-preserving [`ReactiveProgram`].
    pub fn compile_reactive(
        ast: &Program,
        signals: &SignalSchema,
    ) -> Result<ReactiveProgram, Vec<String>> {
        let semantic = Self::compile_with_signals(ast, signals)?;
        crate::reactive::lower_reactive(&semantic, signals)
    }
}

/// Symbols resolved from the top-level declarations of a program.
///
/// Shared by the semantic compiler (which lowers to a `SemanticProgram`) and by
/// the tooling intelligence pass (which projects types for the editor), so both
/// agree on how consts/targets/functions are declared and typed.
pub(crate) struct PreparedSymbols {
    pub table: SymbolTable,
    pub target_values: HashMap<String, CompileTimeValue>,
    pub const_names: std::collections::HashSet<String>,
    pub resolved_targets: Vec<ResolvedTarget>,
    pub errors: Vec<String>,
}

/// Resolve consts, targets and user function signatures into a symbol table.
///
/// This is steps 1 and 2 of the compilation pipeline, extracted so type tooling
/// can reuse them without duplicating the declaration/typing rules.
pub(crate) fn prepare_symbols(ast: &Program) -> PreparedSymbols {
    let mut table = SymbolTable::new();
    register_builtins(&mut table);

    let mut resolved_targets = Vec::new();
    let mut target_values = HashMap::new();
    let mut const_names = std::collections::HashSet::new();
    let mut errors = Vec::new();

    // 1. Resolve and evaluate consts and targets sequentially
    for item in &ast.items {
        match item {
            Item::Const(ConstDecl { name, value, .. }) => {
                let eval_result = {
                    let evaluator = Evaluator::with_target_values(&table, &target_values);
                    evaluator.eval_expr(value)
                };

                match eval_result {
                    EvalResult::Value(val) => {
                        const_names.insert(name.clone());
                        target_values.insert(name.clone(), val.clone());
                        let _ = table.declare(Symbol::new(
                            name.clone(),
                            SymbolKind::Const,
                            val.get_type(),
                            None,
                        ));
                    }
                    EvalResult::Error(diag) => {
                        errors.push(format!("const '{}': {}", name, diag.message));
                    }
                    _ => {
                        errors.push(format!("const '{}' must evaluate to a compile-time constant", name));
                    }
                }
            }
            Item::Target(TargetDecl { name, pose, .. }) => {
                let eval_result = {
                    let evaluator = Evaluator::with_target_values(&table, &target_values);
                    evaluator.eval_expr(pose)
                };

                match eval_result {
                    EvalResult::Value(CompileTimeValue::Position(p)) => {
                        let target_val = MotionTarget::Position(p.clone());
                        target_values.insert(name.clone(), CompileTimeValue::Position(p));
                        let _ = table.declare(Symbol::new(
                            name.clone(),
                            SymbolKind::Target,
                            Type::Position,
                            None,
                        ));
                        resolved_targets.push(ResolvedTarget {
                            name: name.clone(),
                            value: target_val,
                            provenance: Provenance::new(Some(name.clone()), None),
                        });
                    }
                    EvalResult::Value(CompileTimeValue::Pose(p)) => {
                        let target_val = MotionTarget::Pose(p.clone());
                        target_values.insert(name.clone(), CompileTimeValue::Pose(p));
                        let _ = table.declare(Symbol::new(
                            name.clone(),
                            SymbolKind::Target,
                            Type::Pose,
                            None,
                        ));
                        resolved_targets.push(ResolvedTarget {
                            name: name.clone(),
                            value: target_val,
                            provenance: Provenance::new(Some(name.clone()), None),
                        });
                    }
                    EvalResult::Value(CompileTimeValue::Joints(j)) => {
                        let target_val = MotionTarget::Joints(JointConfiguration::new(j.clone()));
                        target_values.insert(name.clone(), CompileTimeValue::Joints(j.clone()));
                        let _ = table.declare(Symbol::new(
                            name.clone(),
                            SymbolKind::Target,
                            Type::Joints {
                                dimension: Some(j.len()),
                            },
                            None,
                        ));
                        resolved_targets.push(ResolvedTarget {
                            name: name.clone(),
                            value: target_val,
                            provenance: Provenance::new(Some(name.clone()), None),
                        });
                    }
                    EvalResult::Error(diag) => {
                        errors.push(format!("Target '{}': {}", name, diag.message));
                    }
                    _ => {
                        errors.push(format!("Target '{}' could not be evaluated to a constant target", name));
                    }
                }
            }
            _ => {}
        }
    }

    // 2. Declare all user functions in SymbolTable
    for item in &ast.items {
        if let Item::Function(f) = item {
            let ret_ty = f
                .return_type
                .as_deref()
                .and_then(Type::from_name)
                .unwrap_or(Type::Unit);

            let param_types: Vec<Type> = f
                .params
                .iter()
                .map(|p| {
                    p.type_ann
                        .as_deref()
                        .and_then(Type::from_name)
                        .unwrap_or(Type::Position)
                })
                .collect();

            let param_names: Vec<String> =
                f.params.iter().map(|p| p.name.clone()).collect();

            let _ = table.declare(Symbol::new(
                f.name.clone(),
                SymbolKind::Function,
                Type::Function(crate::types::FunctionType::with_names(
                    param_types,
                    param_names,
                    ret_ty,
                )),
                None,
            ));
        }
    }

    PreparedSymbols {
        table,
        target_values,
        const_names,
        resolved_targets,
        errors,
    }
}

fn lower_statement(
    stmt: &AstStatement,
    evaluator: &Evaluator,
    params: &[String],
    locals: &mut Vec<String>,
    consts: &std::collections::HashSet<String>,
) -> SemanticStatement {
    match stmt {
        AstStatement::Let { name, value, .. } => {
            locals.push(name.clone());
            let sem_expr = lower_expr(value, evaluator, params, locals, consts);
            SemanticStatement::Let {
                name: name.clone(),
                value: sem_expr,
                provenance: Provenance::new(Some(name.clone()), None),
            }
        }
        AstStatement::MoveJ { target } => {
            let source_name = match target {
                AstExpr::Identifier(id) => Some(id.clone()),
                _ => None,
            };
            let sem_expr = lower_expr(target, evaluator, params, locals, consts);
            SemanticStatement::Motion(SemanticMotion {
                kind: MotionKind::MoveJ,
                target: sem_expr,
                provenance: Provenance::new(source_name, None),
            })
        }
        AstStatement::MoveL { target } => {
            let source_name = match target {
                AstExpr::Identifier(id) => Some(id.clone()),
                _ => None,
            };
            let sem_expr = lower_expr(target, evaluator, params, locals, consts);
            SemanticStatement::Motion(SemanticMotion {
                kind: MotionKind::MoveL,
                target: sem_expr,
                provenance: Provenance::new(source_name, None),
            })
        }
        AstStatement::Wait(dur_expr) => {
            let sem_expr = lower_expr(dur_expr, evaluator, params, locals, consts);
            SemanticStatement::Wait {
                duration: sem_expr,
                provenance: Provenance::new(None, None),
            }
        }
        AstStatement::MoveC { via, target } => {
            let source_name = match target {
                AstExpr::Identifier(id) => Some(id.clone()),
                _ => None,
            };
            let sem_target = lower_expr(target, evaluator, params, locals, consts);
            let via_target = match evaluator.eval_expr(via) {
                EvalResult::Value(CompileTimeValue::Position(p)) => MotionTarget::Position(p),
                EvalResult::Value(CompileTimeValue::Pose(p)) => MotionTarget::Pose(p),
                EvalResult::Value(CompileTimeValue::Joints(vals)) => {
                    MotionTarget::Joints(JointConfiguration::new(vals))
                }
                _ => MotionTarget::Position(Position { point: thalos_math::Vector3::zero() }),
            };
            SemanticStatement::Motion(SemanticMotion {
                kind: MotionKind::MoveC { via: via_target },
                target: sem_target,
                provenance: Provenance::new(source_name, None),
            })
        }
        AstStatement::SetOutput { output, value } => {
            let val_bool = match evaluator.eval_expr(value) {
                EvalResult::Value(CompileTimeValue::Bool(b)) => b,
                _ => false,
            };
            SemanticStatement::SetOutput {
                name: output.clone(),
                value: val_bool,
                provenance: Provenance::new(Some(output.clone()), None),
            }
        }
        AstStatement::If { condition, then_branch, else_branch } => {
            let sem_cond = lower_expr(condition, evaluator, params, locals, consts);
            let sem_then = then_branch
                .iter()
                .map(|s| lower_statement(s, evaluator, params, locals, consts))
                .collect();
            let sem_else = else_branch.as_ref().map(|stmts| {
                stmts
                    .iter()
                    .map(|s| lower_statement(s, evaluator, params, locals, consts))
                    .collect()
            });

            SemanticStatement::If {
                condition: sem_cond,
                then_branch: sem_then,
                else_branch: sem_else,
                provenance: Provenance::new(None, None),
            }
        }
        // Reactive statements are parsed (task 1) but their semantics are not
        // lowered here yet; they lower to an inert marker so the run-once
        // pipeline keeps compiling.
        AstStatement::Request { name, args, .. } => {
            let sem_args = args
                .iter()
                .map(|arg| lower_expr(&arg.value, evaluator, params, locals, consts))
                .collect();
            SemanticStatement::Call {
                function: format!("request::{name}"),
                args: sem_args,
                provenance: Provenance::new(Some(name.clone()), None),
            }
        }
        AstStatement::Start { process } => SemanticStatement::Call {
            function: format!("start::{process}"),
            args: Vec::new(),
            provenance: Provenance::new(Some(process.clone()), None),
        },
        AstStatement::Assign { name, value } => SemanticStatement::Call {
            function: format!("assign::{name}"),
            args: vec![lower_expr(value, evaluator, params, locals, consts)],
            provenance: Provenance::new(Some(name.clone()), None),
        },
        AstStatement::Expr(AstExpr::Call { callee, args }) => {
            match callee.as_str() {
                "set_output" => {
                    let name = match args.first().map(|a| evaluator.eval_expr(&a.value)) {
                        Some(EvalResult::Value(CompileTimeValue::String(s))) => s,
                        _ => "output".to_string(),
                    };
                    let value = match args.get(1).map(|a| evaluator.eval_expr(&a.value)) {
                        Some(EvalResult::Value(CompileTimeValue::Bool(b))) => b,
                        _ => false,
                    };
                    SemanticStatement::SetOutput {
                        name: name.clone(),
                        value,
                        provenance: Provenance::new(Some(name), None),
                    }
                }
                "movej" => {
                    let target_expr = args.first().map(|a| a.value.clone()).unwrap_or(AstExpr::Identifier("default".into()));
                    let source_name = match &target_expr {
                        AstExpr::Identifier(id) => Some(id.clone()),
                        _ => None,
                    };
                    let sem_expr = lower_expr(&target_expr, evaluator, params, locals, consts);
                    SemanticStatement::Motion(SemanticMotion {
                        kind: MotionKind::MoveJ,
                        target: sem_expr,
                        provenance: Provenance::new(source_name, None),
                    })
                }
                "movel" => {
                    let target_expr = args.first().map(|a| a.value.clone()).unwrap_or(AstExpr::Identifier("default".into()));
                    let source_name = match &target_expr {
                        AstExpr::Identifier(id) => Some(id.clone()),
                        _ => None,
                    };
                    let sem_expr = lower_expr(&target_expr, evaluator, params, locals, consts);
                    SemanticStatement::Motion(SemanticMotion {
                        kind: MotionKind::MoveL,
                        target: sem_expr,
                        provenance: Provenance::new(source_name, None),
                    })
                }
                "wait" => {
                    let dur_expr = args.first().map(|a| a.value.clone()).unwrap_or(AstExpr::Number(0.0));
                    let sem_expr = lower_expr(&dur_expr, evaluator, params, locals, consts);
                    SemanticStatement::Wait {
                        duration: sem_expr,
                        provenance: Provenance::new(None, None),
                    }
                }
                _ => {
                    let sem_args = args.iter().map(|a| lower_expr(&a.value, evaluator, params, locals, consts)).collect();
                    SemanticStatement::Call {
                        function: callee.clone(),
                        args: sem_args,
                        provenance: Provenance::new(Some(callee.clone()), None),
                    }
                }
            }
        }
        AstStatement::Expr(e) => {
            let sem_expr = lower_expr(e, evaluator, params, locals, consts);
            SemanticStatement::Expr(sem_expr)
        }
    }
}

fn lower_expr(
    expr: &AstExpr,
    evaluator: &Evaluator,
    params: &[String],
    locals: &[String],
    consts: &std::collections::HashSet<String>,
) -> SemanticExpr {
    match expr {
        AstExpr::MemberAccess { object, member } => match evaluator.eval_expr(expr) {
            EvalResult::Value(val) => SemanticExpr::Constant(val),
            _ => SemanticExpr::MemberAccess {
                object: Box::new(lower_expr(
                    &AstExpr::Identifier(object.clone()),
                    evaluator,
                    params,
                    locals,
                    consts,
                )),
                member: member.clone(),
            },
        },
        _ => match evaluator.eval_expr(expr) {
            EvalResult::Value(val) => SemanticExpr::Constant(val),
            _ => match expr {
                AstExpr::Identifier(id) => {
                    if params.contains(id) {
                        SemanticExpr::ParameterRef(id.clone())
                    } else if locals.contains(id) {
                        SemanticExpr::LocalRef(id.clone())
                    } else if consts.contains(id) {
                        SemanticExpr::ConstRef(id.clone())
                    } else {
                        SemanticExpr::TargetRef(id.clone())
                    }
                }
                AstExpr::Binary { left, op, right } => SemanticExpr::Binary {
                    left: Box::new(lower_expr(left, evaluator, params, locals, consts)),
                    op: *op,
                    right: Box::new(lower_expr(right, evaluator, params, locals, consts)),
                },
                AstExpr::Unary { op, operand } => SemanticExpr::Unary {
                    op: *op,
                    operand: Box::new(lower_expr(operand, evaluator, params, locals, consts)),
                },
                AstExpr::Call { callee, args } if callee == "offset" => {
                    // `offset` binding is receiver-type-directed and may only be
                    // resolvable at runtime (`p.offset(x = side)`), so its named
                    // delta components must survive lowering instead of being
                    // flattened positionally.
                    let mut iter = args.iter();
                    match iter.next() {
                        Some(receiver) => SemanticExpr::Offset {
                            receiver: Box::new(lower_expr(
                                &receiver.value,
                                evaluator,
                                params,
                                locals,
                                consts,
                            )),
                            deltas: iter
                                .map(|arg| SemanticArg {
                                    name: arg.name.clone(),
                                    value: lower_expr(
                                        &arg.value,
                                        evaluator,
                                        params,
                                        locals,
                                        consts,
                                    ),
                                })
                                .collect(),
                        },
                        None => SemanticExpr::Call {
                            function: callee.clone(),
                            args: Vec::new(),
                        },
                    }
                }
                AstExpr::Call { callee, args } => SemanticExpr::Call {
                    function: callee.clone(),
                    args: args
                        .iter()
                        .map(|a| lower_expr(&a.value, evaluator, params, locals, consts))
                        .collect(),
                },
                AstExpr::MemberCall { object, method, args } => SemanticExpr::MemberCall {
                    object: Box::new(lower_expr(
                        &AstExpr::Identifier(object.clone()),
                        evaluator,
                        params,
                        locals,
                        consts,
                    )),
                    member: method.clone(),
                    args: args
                        .iter()
                        .map(|a| lower_expr(&a.value, evaluator, params, locals, consts))
                        .collect(),
                },
                _ => SemanticExpr::ParameterRef(format!("{:?}", expr)),
            },
        },
    }
}

/// Rewrite `module.channel` member accesses into first-class
/// [`SemanticExpr::ChannelAccess`] reads, for channels declared in the schema.
/// Build the `use` alias map (local name → real signal module).
fn build_signal_aliases(program: &Program) -> std::collections::HashMap<String, String> {
    let mut aliases = std::collections::HashMap::new();
    for item in &program.items {
        if let Item::Use(u) = item {
            collect_alias_map(&u.tree, &mut aliases);
        }
    }
    aliases
}

fn collect_alias_map(tree: &UseTree, out: &mut std::collections::HashMap<String, String>) {
    if tree.is_group() {
        for child in &tree.group {
            collect_alias_map(child, out);
        }
        return;
    }
    if let Some(node) = tree.prefix.last() {
        let local = tree.alias.clone().unwrap_or_else(|| node.clone());
        out.insert(local, node.clone());
    }
}

/// Rewrite `module.channel` member accesses into first-class
/// [`SemanticExpr::ChannelAccess`] reads, for channels declared in the schema.
/// `aliases` resolves a local `use` name to the real module.
fn rewrite_signal_access(
    program: &mut SemanticProgram,
    signals: &SignalSchema,
    aliases: &std::collections::HashMap<String, String>,
) {
    for func in &mut program.functions {
        rewrite_statements(&mut func.body, signals, aliases);
        if let Some(tail) = func.tail_expr.as_mut() {
            rewrite_expr(tail, signals, aliases);
        }
    }
}

fn rewrite_statements(
    statements: &mut [SemanticStatement],
    signals: &SignalSchema,
    aliases: &std::collections::HashMap<String, String>,
) {
    for statement in statements {
        match statement {
            SemanticStatement::Let { value, .. } => rewrite_expr(value, signals, aliases),
            SemanticStatement::Motion(SemanticMotion { target, .. }) => {
                rewrite_expr(target, signals, aliases)
            }
            SemanticStatement::Wait { duration, .. } => rewrite_expr(duration, signals, aliases),
            SemanticStatement::If {
                condition,
                then_branch,
                else_branch,
                ..
            } => {
                rewrite_expr(condition, signals, aliases);
                rewrite_statements(then_branch, signals, aliases);
                if let Some(else_body) = else_branch {
                    rewrite_statements(else_body, signals, aliases);
                }
            }
            SemanticStatement::Call { args, .. } => {
                for arg in args {
                    rewrite_expr(arg, signals, aliases);
                }
            }
            SemanticStatement::Expr(expr) => rewrite_expr(expr, signals, aliases),
            SemanticStatement::SetOutput { .. } => {}
        }
    }
}

fn rewrite_expr(
    expr: &mut SemanticExpr,
    signals: &SignalSchema,
    aliases: &std::collections::HashMap<String, String>,
) {
    match expr {
        SemanticExpr::MemberAccess { object, member } => {
            rewrite_expr(object, signals, aliases);
            if let SemanticExpr::TargetRef(name) = object.as_ref() {
                let module = aliases.get(name).map(String::as_str).unwrap_or(name.as_str());
                if signals.type_of(module, member).is_some() {
                    *expr = SemanticExpr::ChannelAccess {
                        module: module.to_string(),
                        channel: member.clone(),
                    };
                }
            }
        }
        SemanticExpr::Binary { left, right, .. } => {
            rewrite_expr(left, signals, aliases);
            rewrite_expr(right, signals, aliases);
        }
        SemanticExpr::Unary { operand, .. } => rewrite_expr(operand, signals, aliases),
        SemanticExpr::Offset { receiver, deltas } => {
            rewrite_expr(receiver, signals, aliases);
            for delta in deltas {
                rewrite_expr(&mut delta.value, signals, aliases);
            }
        }
        SemanticExpr::Call { args, .. } => {
            for arg in args {
                rewrite_expr(arg, signals, aliases);
            }
        }
        SemanticExpr::MemberCall { object, args, .. } => {
            rewrite_expr(object, signals, aliases);
            for arg in args {
                rewrite_expr(arg, signals, aliases);
            }
        }
        SemanticExpr::Constant(_)
        | SemanticExpr::ConstRef(_)
        | SemanticExpr::LocalRef(_)
        | SemanticExpr::ParameterRef(_)
        | SemanticExpr::TargetRef(_)
        | SemanticExpr::ChannelAccess { .. } => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reactive::{ReactiveExpr, ReactiveStatement, SignalType};
    use thalos_lang::parse_source;

    #[test]
    fn undeclared_identifier_reports_a_single_root_cause() {
        let source = r#"
target jtt = joints(20deg, 30deg, 0deg, 0deg, 0deg, 0deg)
target ptt = position([2.152m, 0.783m, 1.882m])

fn main() {
    movej(jtt)
    movel(ptt)

    let offset1 = [1m, 0m, 0m]
    movel(ptt2 - offset1)
}
"#;
        let program = parse_source(source).expect("source must parse as syntactically valid");
        let errors = SemanticCompiler::compile(&program).expect_err("compile must fail");

        assert_eq!(
            errors,
            vec!["Unknown identifier 'ptt2'".to_string()],
            "only the root cause must be reported, got: {errors:?}"
        );
    }

    #[test]
    fn channel_access_is_produced_typed_and_preserved() {
        let source = r#"
fn main() {
    if sensors.temp > 50.0 {
        set_output("gripper", true);
    }
}
"#;
        let program = parse_source(source).expect("source must parse");
        let schema = SignalSchema::new().with("sensors", "temp", SignalType::Float);
        let reactive =
            SemanticCompiler::compile_reactive(&program, &schema).expect("compile_reactive");

        assert_eq!(reactive.statements.len(), 1, "one preserved If");
        match &reactive.statements[0] {
            ReactiveStatement::If { condition, .. } => match condition {
                ReactiveExpr::Binary { left, .. } => match left.as_ref() {
                    ReactiveExpr::Signal { module, channel, ty } => {
                        assert_eq!(module, "sensors");
                        assert_eq!(channel, "temp");
                        assert_eq!(*ty, SignalType::Float);
                    }
                    other => panic!("expected a signal read, got {other:?}"),
                },
                other => panic!("expected a binary condition, got {other:?}"),
            },
            other => panic!("expected a preserved If, got {other:?}"),
        }
    }
}


/// Resolve `expect` declarations against the station signal schema.
///
/// Rules:
/// - the comparison is numeric: `Float` and `Int` channels are accepted;
///   `Bool` channels are rejected (a numeric comparison could never be true).
/// - the runtime key is the CHANNEL SYMBOL (the observation bundle is keyed by
///   `signal_key = signal.symbol`), so a symbol declared by more than one
///   module is ambiguous and rejected rather than silently collapsed.
/// - two expectations on the same `module.channel` are a conflict.
fn resolve_expectations(
    ast: &Program,
    signals: &SignalSchema,
) -> (
    Vec<thalos_core::execution::condition::SignalCondition>,
    Vec<String>,
) {
    use thalos_core::execution::condition::{ComparisonOp, SignalCondition};

    let mut expectations = Vec::new();
    let mut errors = Vec::new();
    let mut seen: std::collections::HashSet<(String, String)> = std::collections::HashSet::new();

    for item in &ast.items {
        let Item::Expect(decl) = item else {
            continue;
        };
        if !seen.insert((decl.module.clone(), decl.channel.clone())) {
            errors.push(format!(
                "duplicate expectation on '{}.{}'",
                decl.module, decl.channel
            ));
            continue;
        }

        match signals.type_of(&decl.module, &decl.channel) {
            Some(crate::reactive::SignalType::Float | crate::reactive::SignalType::Int) => {
                let modules_with_channel = signals
                    .bindings()
                    .iter()
                    .filter(|b| b.channel == decl.channel)
                    .map(|b| b.module.as_str())
                    .collect::<std::collections::BTreeSet<_>>();
                if modules_with_channel.len() > 1 {
                    errors.push(format!(
                        "ambiguous expectation: channel '{}' is declared by modules {:?}",
                        decl.channel, modules_with_channel
                    ));
                    continue;
                }
                let op = match decl.op {
                    thalos_lang::ast::item::CompareOp::Eq => ComparisonOp::Eq,
                    thalos_lang::ast::item::CompareOp::Ne => ComparisonOp::Ne,
                    thalos_lang::ast::item::CompareOp::Gt => ComparisonOp::Gt,
                    thalos_lang::ast::item::CompareOp::Ge => ComparisonOp::Ge,
                    thalos_lang::ast::item::CompareOp::Lt => ComparisonOp::Lt,
                    thalos_lang::ast::item::CompareOp::Le => ComparisonOp::Le,
                };
                expectations.push(SignalCondition::new(decl.channel.clone(), op, decl.value));
            }
            Some(crate::reactive::SignalType::Bool) => errors.push(format!(
                "expectation on '{}.{}' is not numeric (Bool channels cannot be compared)",
                decl.module, decl.channel
            )),
            None => errors.push(format!(
                "unknown signal '{}.{}' in expectation",
                decl.module, decl.channel
            )),
        }
    }
    (expectations, errors)
}

#[cfg(test)]
mod expect_tests {
    use super::*;
    use crate::reactive::{SignalSchema, SignalType};
    use thalos_core::execution::condition::{ComparisonOp, SignalCondition};
    use thalos_lang::parse_source;

    fn schema() -> SignalSchema {
        SignalSchema::new()
            .with("oven", "temperature", SignalType::Float)
            .with("conveyor", "position", SignalType::Int)
            .with("oven", "door_open", SignalType::Bool)
    }

    #[test]
    fn expectations_resolve_against_the_station_schema() {
        let program = parse_source("expect oven.temperature <= 75.0\nfn main() { }").unwrap();
        let compiled = SemanticCompiler::compile_with_signals(&program, &schema()).unwrap();

        // The runtime key is the channel SYMBOL (the observation bundle key),
        // not the dotted module path.
        assert_eq!(
            compiled.expectations,
            vec![SignalCondition::new("temperature", ComparisonOp::Le, 75.0)]
        );
    }

    #[test]
    fn the_planning_path_ignores_expectations() {
        let program = parse_source("expect oven.temperature <= 75.0\nfn main() { }").unwrap();
        let compiled = SemanticCompiler::compile(&program).expect("planning path must accept");
        assert!(compiled.expectations.is_empty());
    }

    #[test]
    fn unknown_channel_is_rejected() {
        let program = parse_source("expect oven.nonexistent > 1.0\nfn main() { }").unwrap();
        let errors = SemanticCompiler::compile_with_signals(&program, &schema()).unwrap_err();
        assert!(errors.iter().any(|e| e.contains("unknown signal")), "{errors:?}");
    }

    #[test]
    fn boolean_channel_is_rejected_as_not_numeric() {
        let program = parse_source("expect oven.door_open > 0.0\nfn main() { }").unwrap();
        let errors = SemanticCompiler::compile_with_signals(&program, &schema()).unwrap_err();
        assert!(errors.iter().any(|e| e.contains("not numeric")), "{errors:?}");
    }

    #[test]
    fn duplicate_expectation_is_rejected() {
        let program = parse_source(
            "expect oven.temperature <= 75.0\nexpect oven.temperature <= 80.0\nfn main() { }",
        )
        .unwrap();
        let errors = SemanticCompiler::compile_with_signals(&program, &schema()).unwrap_err();
        assert!(errors.iter().any(|e| e.contains("duplicate expectation")), "{errors:?}");
    }

    #[test]
    fn ambiguous_channel_symbol_is_rejected() {
        // Two modules declaring the same channel symbol: the runtime key would
        // collapse them, so the expectation is ambiguous.
        let ambiguous = SignalSchema::new()
            .with("a", "temperature", SignalType::Float)
            .with("b", "temperature", SignalType::Float);
        let program = parse_source("expect a.temperature <= 75.0\nfn main() { }").unwrap();
        let errors = SemanticCompiler::compile_with_signals(&program, &ambiguous).unwrap_err();
        assert!(errors.iter().any(|e| e.contains("ambiguous")), "{errors:?}");
    }

    #[test]
    fn the_dedicated_api_resolves_only_expectations() {
        let program =
            parse_source("expect conveyor.position >= 10\nfn main() { }").unwrap();
        let expectations = SemanticCompiler::compile_expectations(&program, &schema()).unwrap();
        assert_eq!(
            expectations,
            vec![SignalCondition::new("position", ComparisonOp::Ge, 10.0)]
        );
    }

    #[test]
    fn reactive_processes_reject_expectations() {
        let program = parse_source(
            "expect oven.temperature <= 75.0\nprocess p { cycle { } }\nfn main() { start p }",
        )
        .unwrap();
        let errors = crate::reactive_process::compile_process(&program, &schema()).unwrap_err();
        assert!(
            errors.iter().any(|e| e.contains("only valid for plan-driven")),
            "{errors:?}"
        );
    }
}
