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
                            target_values.insert(name.clone(), val);
                            let _ = table.declare(Symbol::new(
                                name.clone(),
                                SymbolKind::Const,
                                Type::Position,
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

                let _ = table.declare(Symbol::new(
                    f.name.clone(),
                    SymbolKind::Function,
                    Type::Function(crate::types::FunctionType {
                        params: param_types,
                        return_type: Box::new(ret_ty),
                    }),
                    None,
                ));
            }
        }

        // 3. Type check AST functions and statements in isolated parameter scope
        let mut checker = TypeChecker::new(&mut table);
        for item in &ast.items {
            if let Item::Function(f) = item {
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
                    let expected_ret = f
                        .return_type
                        .as_deref()
                        .and_then(Type::from_name)
                        .unwrap_or(Type::Unit);

                    if expected_ret != Type::Unit && expected_ret != typed_tail.ty {
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
                    match stmt {
                        AstStatement::Let { name, value, .. } => {
                            local_names.push(name.clone());
                            let sem_expr = lower_expr(value, &evaluator, &param_names, &local_names, &const_names);
                            body.push(SemanticStatement::Let {
                                name: name.clone(),
                                value: sem_expr,
                                provenance: Provenance::new(Some(name.clone()), None),
                            });
                        }
                        AstStatement::MoveJ { target } => {
                            let source_name = match target {
                                AstExpr::Identifier(id) => Some(id.clone()),
                                _ => None,
                            };
                            let sem_expr = lower_expr(target, &evaluator, &param_names, &local_names, &const_names);
                            body.push(SemanticStatement::Motion(SemanticMotion {
                                kind: MotionKind::MoveJ,
                                target: sem_expr,
                                provenance: Provenance::new(source_name, None),
                            }));
                        }
                        AstStatement::MoveL { target } => {
                            let source_name = match target {
                                AstExpr::Identifier(id) => Some(id.clone()),
                                _ => None,
                            };
                            let sem_expr = lower_expr(target, &evaluator, &param_names, &local_names, &const_names);
                            body.push(SemanticStatement::Motion(SemanticMotion {
                                kind: MotionKind::MoveL,
                                target: sem_expr,
                                provenance: Provenance::new(source_name, None),
                            }));
                        }
                        AstStatement::Wait(dur_expr) => {
                            let sem_expr = lower_expr(dur_expr, &evaluator, &param_names, &local_names, &const_names);
                            body.push(SemanticStatement::Wait {
                                duration: sem_expr,
                                provenance: Provenance::new(None, None),
                            });
                        }
                        AstStatement::MoveC { via, target } => {
                            let source_name = match target {
                                AstExpr::Identifier(id) => Some(id.clone()),
                                _ => None,
                            };
                            let sem_target = lower_expr(target, &evaluator, &param_names, &local_names, &const_names);
                            let via_target = match evaluator.eval_expr(via) {
                                EvalResult::Value(CompileTimeValue::Position(p)) => MotionTarget::Position(p),
                                EvalResult::Value(CompileTimeValue::Pose(p)) => MotionTarget::Pose(p),
                                _ => MotionTarget::Position(Position { point: thalos_math::Vector3::zero() }),
                            };
                            body.push(SemanticStatement::Motion(SemanticMotion {
                                kind: MotionKind::MoveC { via: via_target },
                                target: sem_target,
                                provenance: Provenance::new(source_name, None),
                            }));
                        }
                        AstStatement::SetOutput { output, value } => {
                            let val_bool = match evaluator.eval_expr(value) {
                                EvalResult::Value(CompileTimeValue::Bool(b)) => b,
                                _ => false,
                            };
                            body.push(SemanticStatement::SetOutput {
                                name: output.clone(),
                                value: val_bool,
                                provenance: Provenance::new(Some(output.clone()), None),
                            });
                        }
                        AstStatement::Expr(AstExpr::Call { callee, args }) => {
                            let sem_args = args.iter().map(|a| lower_expr(a, &evaluator, &param_names, &local_names, &const_names)).collect();
                            body.push(SemanticStatement::Call {
                                function: callee.clone(),
                                args: sem_args,
                                provenance: Provenance::new(Some(callee.clone()), None),
                            });
                        }
                        AstStatement::Expr(e) => {
                            let sem_expr = lower_expr(e, &evaluator, &param_names, &local_names, &const_names);
                            body.push(SemanticStatement::Expr(sem_expr));
                        }
                    }
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

fn lower_expr(
    expr: &AstExpr,
    evaluator: &Evaluator,
    params: &[String],
    locals: &[String],
    consts: &std::collections::HashSet<String>,
) -> SemanticExpr {
    match evaluator.eval_expr(expr) {
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
            AstExpr::Call { callee, args } => SemanticExpr::Call {
                function: callee.clone(),
                args: args
                    .iter()
                    .map(|a| lower_expr(a, evaluator, params, locals, consts))
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
                    .map(|a| lower_expr(a, evaluator, params, locals, consts))
                    .collect(),
            },
            _ => SemanticExpr::ParameterRef(format!("{:?}", expr)),
        },
    }
}
