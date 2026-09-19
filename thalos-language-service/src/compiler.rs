use std::collections::HashMap;
use thalos_lang::ast::{ConstDecl, Expr as AstExpr, Item, Program, Statement as AstStatement, TargetDecl};
use crate::builtins::register_builtins;
use crate::checker::TypeChecker;
use crate::evaluator::{CompileTimeValue, EvalResult, Evaluator, Position};
use crate::model::{
    JointConfiguration, MotionKind, MotionTarget, Provenance, ResolvedTarget, SemanticExpr,
    SemanticFunction, SemanticMotion, SemanticProgram, SemanticStatement,
};
use crate::scope::SymbolTable;
use crate::symbols::{Symbol, SymbolKind};
use crate::types::Type;

pub struct SemanticCompiler;

impl SemanticCompiler {
    pub fn compile(ast: &Program) -> Result<SemanticProgram, Vec<String>> {
        // Resolve named arguments into canonical positional form once, up front.
        // Every stage below (evaluator, checker, lowering, resolver) only ever
        // sees positional arguments and never re-implements named-arg semantics.
        let (ast, bind_errors) = crate::binder::normalize_program(ast);
        if !bind_errors.is_empty() {
            return Err(bind_errors);
        }

        let PreparedSymbols {
            mut table,
            target_values,
            const_names,
            resolved_targets,
            mut errors,
        } = prepare_symbols(&ast);

        // 3. Type check AST functions and statements in isolated parameter scope
        let mut checker = TypeChecker::new(&mut table);
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
                    if !typed_tail.ty.is_error() && expected_ret != Type::Unit && expected_ret != typed_tail.ty {
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

        Ok(SemanticProgram {
            targets: resolved_targets,
            functions: semantic_functions,
            entry_point: "main".to_string(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use thalos_lang::parse_source;

    #[test]
    fn undeclared_identifier_reports_a_single_root_cause() {
        let source = r#"
target jtt = joints(20deg, 30deg, 0deg, 0deg, 0deg, 0deg)
target ptt = position([2.152, 0.783, 1.882])

fn main() {
    movej(jtt)
    movel(ptt)

    let offset1 = [1, 0, 0]
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
