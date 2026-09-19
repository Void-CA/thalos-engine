use std::collections::HashMap;
use thalos_lang::ast::{BinaryOp, UnaryOp};
use crate::evaluator::{CompileTimeValue, Position};
use crate::model::{
    CallSite, JointConfiguration, MotionTarget, ResolvedMotion, ResolvedProgram,
    ResolvedStatement, SemanticExpr, SemanticFunction, SemanticMotion, SemanticProgram,
    SemanticStatement,
};

pub struct SemanticResolver;

impl SemanticResolver {
    pub fn resolve(program: &SemanticProgram) -> Result<ResolvedProgram, Vec<String>> {
        let mut target_map: HashMap<String, MotionTarget> = HashMap::new();
        for target in &program.targets {
            target_map.insert(target.name.clone(), target.value.clone());
        }

        let mut fn_map: HashMap<String, &SemanticFunction> = HashMap::new();
        for func in &program.functions {
            fn_map.insert(func.name.clone(), func);
        }

        let entry_fn = match fn_map.get(&program.entry_point) {
            Some(f) => f,
            None => return Err(vec![format!("Entry point function '{}' not found", program.entry_point)]),
        };

        let mut resolved_statements = Vec::new();
        let mut errors = Vec::new();
        let mut call_stack = Vec::new();
        let env = HashMap::new();

        Self::resolve_function(
            entry_fn,
            &env,
            &target_map,
            &fn_map,
            &mut call_stack,
            &mut resolved_statements,
            &mut errors,
        );

        if !errors.is_empty() {
            Err(errors)
        } else {
            Ok(ResolvedProgram {
                statements: resolved_statements,
            })
        }
    }

    fn resolve_function(
        func: &SemanticFunction,
        env: &HashMap<String, CompileTimeValue>,
        targets: &HashMap<String, MotionTarget>,
        functions: &HashMap<String, &SemanticFunction>,
        call_stack: &mut Vec<CallSite>,
        out: &mut Vec<ResolvedStatement>,
        errors: &mut Vec<String>,
    ) {
        call_stack.push(CallSite {
            function: func.name.clone(),
            span: func.provenance.span,
        });

        let mut local_env = env.clone();

        for stmt in &func.body {
            Self::resolve_statement(
                stmt,
                &mut local_env,
                targets,
                functions,
                call_stack,
                out,
                errors,
            );
        }

        call_stack.pop();
    }

    fn resolve_statement(
        stmt: &SemanticStatement,
        local_env: &mut HashMap<String, CompileTimeValue>,
        targets: &HashMap<String, MotionTarget>,
        functions: &HashMap<String, &SemanticFunction>,
        call_stack: &mut Vec<CallSite>,
        out: &mut Vec<ResolvedStatement>,
        errors: &mut Vec<String>,
    ) {
        match stmt {
            SemanticStatement::Let { name, value, .. } => {
                match Self::eval_value(value, local_env, targets, functions) {
                    Ok(val) => {
                        local_env.insert(name.clone(), val);
                    }
                    Err(e) => errors.push(e),
                }
            }
            SemanticStatement::Motion(SemanticMotion { kind, target, provenance }) => {
                match Self::eval_expr(target, local_env, targets, functions) {
                    Ok(target_val) => {
                        let mut merged_prov = provenance.clone();
                        merged_prov.call_stack = call_stack.clone();
                        out.push(ResolvedStatement::Motion(ResolvedMotion {
                            kind: kind.clone(),
                            target: target_val,
                            provenance: merged_prov,
                        }));
                    }
                    Err(err) => errors.push(err),
                }
            }
            SemanticStatement::Wait { duration, provenance } => {
                // `wait` accepts a Duration expression; evaluate it with the same
                // shared value algebra used everywhere (fixes `wait(d1 + d2)`).
                match Self::eval_value(duration, local_env, targets, functions) {
                    Ok(CompileTimeValue::Duration(secs)) => {
                        let mut merged_prov = provenance.clone();
                        merged_prov.call_stack = call_stack.clone();
                        out.push(ResolvedStatement::Wait {
                            seconds: secs,
                            provenance: merged_prov,
                        });
                    }
                    Ok(other) => errors.push(format!(
                        "wait expected Duration, got {:?}",
                        other.get_type()
                    )),
                    Err(e) => errors.push(e),
                }
            }
            SemanticStatement::SetOutput { name, value, provenance } => {
                let mut merged_prov = provenance.clone();
                merged_prov.call_stack = call_stack.clone();
                out.push(ResolvedStatement::SetOutput {
                    name: name.clone(),
                    value: *value,
                    provenance: merged_prov,
                });
            }
            SemanticStatement::If { condition, then_branch, else_branch, .. } => {
                let cond_val = Self::eval_value(condition, local_env, targets, functions);
                match cond_val {
                    Ok(CompileTimeValue::Bool(true)) => {
                        for s in then_branch {
                            Self::resolve_statement(s, local_env, targets, functions, call_stack, out, errors);
                        }
                    }
                    Ok(CompileTimeValue::Bool(false)) => {
                        if let Some(else_stmts) = else_branch {
                            for s in else_stmts {
                                Self::resolve_statement(s, local_env, targets, functions, call_stack, out, errors);
                            }
                        }
                    }
                    _ => {
                        for s in then_branch {
                            Self::resolve_statement(s, local_env, targets, functions, call_stack, out, errors);
                        }
                        if let Some(else_stmts) = else_branch {
                            for s in else_stmts {
                                Self::resolve_statement(s, local_env, targets, functions, call_stack, out, errors);
                            }
                        }
                    }
                }
            }
            SemanticStatement::Call { function, args, provenance: _ } => {
                if let Some(target_fn) = functions.get(function) {
                    let mut arg_vals = Vec::new();
                    for arg in args {
                        match Self::eval_value(arg, local_env, targets, functions) {
                            Ok(val) => arg_vals.push(val),
                            Err(e) => errors.push(e),
                        }
                    }

                    if target_fn.params.len() != arg_vals.len() {
                        errors.push(format!(
                            "Function '{}' expected {} arguments, got {}",
                            function,
                            target_fn.params.len(),
                            arg_vals.len()
                        ));
                        return;
                    }

                    let mut new_env = HashMap::new();
                    for (param, val) in target_fn.params.iter().zip(arg_vals) {
                        new_env.insert(param.clone(), val);
                    }

                    Self::resolve_function(
                        target_fn,
                        &new_env,
                        targets,
                        functions,
                        call_stack,
                        out,
                        errors,
                    );
                } else {
                    errors.push(format!("Unresolved function call '{}'", function));
                }
            }
            SemanticStatement::Expr(_) => {}
        }
    }

    fn eval_value(
        expr: &SemanticExpr,
        env: &HashMap<String, CompileTimeValue>,
        targets: &HashMap<String, MotionTarget>,
        functions: &HashMap<String, &SemanticFunction>,
    ) -> Result<CompileTimeValue, String> {
        match expr {
            SemanticExpr::Constant(val) => Ok(val.clone()),
            SemanticExpr::ParameterRef(p) | SemanticExpr::LocalRef(p) | SemanticExpr::ConstRef(p) => {
                if let Some(val) = env.get(p) {
                    Ok(val.clone())
                } else {
                    Err(format!("Unbound variable/parameter '{}'", p))
                }
            }
            SemanticExpr::TargetRef(t) => {
                if let Some(target_val) = targets.get(t) {
                    match target_val {
                        MotionTarget::Position(p) => Ok(CompileTimeValue::Position(p.clone())),
                        MotionTarget::Pose(p) => Ok(CompileTimeValue::Pose(p.clone())),
                        MotionTarget::Joints(j) => Ok(CompileTimeValue::Joints(j.values.clone())),
                    }
                } else {
                    Err(format!("Unresolved target reference '{}'", t))
                }
            }
            SemanticExpr::Binary { left, op, right } => {
                let lhs = Self::eval_value(left, env, targets, functions)?;
                let rhs = Self::eval_value(right, env, targets, functions)?;
                // Single shared value algebra (also used by the evaluator).
                crate::algebra::binary_value(*op, &lhs, &rhs)
            }
            // Reuse the evaluator's single value-level negation so compile-time
            // folding and deferred resolution agree by construction.
            SemanticExpr::Unary {
                op: UnaryOp::Neg,
                operand,
            } => {
                let value = Self::eval_value(operand, env, targets, functions)?;
                crate::evaluator::negate_value(&value)
                    .ok_or_else(|| format!("Cannot negate value of type {:?}", value.get_type()))
            }
            // Receiver-type-directed `offset`: the receiver's value type decides
            // how the named delta components bind. Shares `apply_resolved_delta`
            // with the compile-time path so the two cannot drift.
            SemanticExpr::Offset { receiver, deltas } => {
                let receiver_value = Self::eval_value(receiver, env, targets, functions)?;
                let mut resolved = Vec::with_capacity(deltas.len());
                for delta in deltas {
                    resolved.push((
                        delta.name.clone(),
                        Self::eval_value(&delta.value, env, targets, functions)?,
                    ));
                }
                crate::offset::apply_resolved_delta(&receiver_value, &resolved)
            }
            SemanticExpr::Call { function, args } => {
                if let Some(target_fn) = functions.get(function) {
                    let mut arg_vals = Vec::new();
                    for arg in args {
                        arg_vals.push(Self::eval_value(arg, env, targets, functions)?);
                    }
                    if let Some(ref tail) = target_fn.tail_expr {
                        let mut call_env = HashMap::new();
                        for (p, val) in target_fn.params.iter().zip(arg_vals) {
                            call_env.insert(p.clone(), val);
                        }
                        Self::eval_value(tail, &call_env, targets, functions)
                    } else {
                        Err(format!("Function '{}' has no return value", function))
                    }
                } else {
                    Err(format!("Unresolved function call '{}'", function))
                }
            }
            SemanticExpr::MemberAccess { object, member } => {
                let value = Self::eval_value(object, env, targets, functions)?;
                crate::evaluator::read_member(&value, member)
                    .ok_or_else(|| format!("Unknown member '{}'", member))
            }
            _ => Err("Complex expression resolution not supported yet".to_string()),
        }
    }

    fn eval_scalar(expr: &SemanticExpr, env: &HashMap<String, CompileTimeValue>) -> Result<f64, String> {
        match expr {
            SemanticExpr::Constant(CompileTimeValue::Duration(d)) => Ok(*d),
            SemanticExpr::Constant(CompileTimeValue::Float(f)) => Ok(*f),
            SemanticExpr::Constant(CompileTimeValue::Int(i)) => Ok(*i as f64),
            SemanticExpr::ParameterRef(p) | SemanticExpr::LocalRef(p) | SemanticExpr::ConstRef(p) => {
                if let Some(val) = env.get(p) {
                    match val {
                        CompileTimeValue::Duration(d) => Ok(*d),
                        CompileTimeValue::Float(f) => Ok(*f),
                        CompileTimeValue::Int(i) => Ok(*i as f64),
                        _ => Err(format!("Variable/parameter '{}' is not a scalar number", p)),
                    }
                } else {
                    Err(format!("Unbound variable/parameter '{}'", p))
                }
            }
            _ => Err("Could not resolve scalar expression".to_string()),
        }
    }

    fn eval_expr(
        expr: &SemanticExpr,
        env: &HashMap<String, CompileTimeValue>,
        targets: &HashMap<String, MotionTarget>,
        functions: &HashMap<String, &SemanticFunction>,
    ) -> Result<MotionTarget, String> {
        let val = Self::eval_value(expr, env, targets, functions)?;
        match val {
            CompileTimeValue::Position(p) => Ok(MotionTarget::Position(p)),
            CompileTimeValue::Pose(p) => Ok(MotionTarget::Pose(p)),
            CompileTimeValue::Joints(j) => Ok(MotionTarget::Joints(JointConfiguration::new(j))),
            _ => Err("Expression does not resolve to a motion target".to_string()),
        }
    }
}
