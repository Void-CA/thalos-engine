use std::collections::HashMap;
use thalos_lang::ast::{Expr as AstExpr, Item, Program, Statement as AstStatement, TargetDecl};
use crate::builtins::register_builtins;
use crate::checker::TypeChecker;
use crate::evaluator::{CompileTimeValue, EvalResult, Evaluator};
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
        let mut errors = Vec::new();

        // 1. Resolve and evaluate targets sequentially so dependent targets can reference earlier ones
        for item in &ast.items {
            if let Item::Target(TargetDecl { name, pose, .. }) = item {
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
        }

        // 2. Declare all user functions in SymbolTable
        for item in &ast.items {
            if let Item::Function(f) = item {
                let _ = table.declare(Symbol::new(
                    f.name.clone(),
                    SymbolKind::Function,
                    Type::Function(crate::types::FunctionType {
                        params: vec![Type::Position; f.params.len()],
                        return_type: Box::new(Type::Unit),
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
                    let _ = checker.symbol_table.declare(Symbol::new(
                        param.name.clone(),
                        SymbolKind::Variable,
                        Type::Position,
                        None,
                    ));
                }
                for stmt in &f.body {
                    checker.check_statement(stmt);
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

        // 3. Lower functions to SemanticFunction and SemanticStatement
        let evaluator = Evaluator::with_target_values(&table, &target_values);
        let mut semantic_functions = Vec::new();
        for item in &ast.items {
            if let Item::Function(f) = item {
                let mut body = Vec::new();
                let param_names: Vec<String> = f.params.iter().map(|p| p.name.clone()).collect();

                for stmt in &f.body {
                    match stmt {
                        AstStatement::MoveJ { target } => {
                            let source_name = match target {
                                AstExpr::Identifier(id) => Some(id.clone()),
                                _ => None,
                            };
                            let sem_expr = lower_expr(target, &evaluator, &param_names);
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
                            let sem_expr = lower_expr(target, &evaluator, &param_names);
                            body.push(SemanticStatement::Motion(SemanticMotion {
                                kind: MotionKind::MoveL,
                                target: sem_expr,
                                provenance: Provenance::new(source_name, None),
                            }));
                        }
                        AstStatement::Wait(dur_expr) => {
                            let sem_expr = lower_expr(dur_expr, &evaluator, &param_names);
                            body.push(SemanticStatement::Wait {
                                duration: sem_expr,
                                provenance: Provenance::new(None, None),
                            });
                        }
                        AstStatement::Expr(AstExpr::Call { callee, args }) => {
                            let sem_args = args.iter().map(|a| lower_expr(a, &evaluator, &param_names)).collect();
                            body.push(SemanticStatement::Call {
                                function: callee.clone(),
                                args: sem_args,
                                provenance: Provenance::new(Some(callee.clone()), None),
                            });
                        }
                        _ => {}
                    }
                }

                semantic_functions.push(SemanticFunction {
                    name: f.name.clone(),
                    params: param_names,
                    body,
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

fn lower_expr(expr: &AstExpr, evaluator: &Evaluator, params: &[String]) -> SemanticExpr {
    match evaluator.eval_expr(expr) {
        EvalResult::Value(val) => SemanticExpr::Constant(val),
        _ => match expr {
            AstExpr::Identifier(id) => {
                if params.contains(id) {
                    SemanticExpr::ParameterRef(id.clone())
                } else {
                    SemanticExpr::TargetRef(id.clone())
                }
            }
            AstExpr::Binary { left, op, right } => SemanticExpr::Binary {
                left: Box::new(lower_expr(left, evaluator, params)),
                op: *op,
                right: Box::new(lower_expr(right, evaluator, params)),
            },
            AstExpr::Call { callee, args } => SemanticExpr::Call {
                function: callee.clone(),
                args: args.iter().map(|a| lower_expr(a, evaluator, params)).collect(),
            },
            AstExpr::MemberCall { object, method, args } => SemanticExpr::MemberCall {
                object: Box::new(lower_expr(&AstExpr::Identifier(object.clone()), evaluator, params)),
                member: method.clone(),
                args: args.iter().map(|a| lower_expr(a, evaluator, params)).collect(),
            },
            _ => SemanticExpr::ParameterRef(format!("{:?}", expr)),
        },
    }
}
