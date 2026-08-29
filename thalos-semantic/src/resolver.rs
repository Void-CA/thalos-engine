use std::collections::HashMap;
use thalos_lang::ast::BinaryOp;
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
            span: func.provenance.span.clone(),
        });

        let mut local_env = env.clone();

        for stmt in &func.body {
            match stmt {
                SemanticStatement::Let { name, value, .. } => {
                    match Self::eval_value(value, &local_env, targets, functions) {
                        Ok(val) => {
                            local_env.insert(name.clone(), val);
                        }
                        Err(e) => errors.push(e),
                    }
                }
                SemanticStatement::Motion(SemanticMotion { kind, target, provenance }) => {
                    match Self::eval_expr(target, &local_env, targets, functions) {
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
                    if let Ok(secs) = Self::eval_scalar(duration, &local_env) {
                        let mut merged_prov = provenance.clone();
                        merged_prov.call_stack = call_stack.clone();
                        out.push(ResolvedStatement::Wait {
                            seconds: secs,
                            provenance: merged_prov,
                        });
                    } else {
                        errors.push("Could not evaluate wait duration".to_string());
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
                SemanticStatement::Call { function, args, provenance: _ } => {
                    if let Some(target_fn) = functions.get(function) {
                        let mut arg_vals = Vec::new();
                        for arg in args {
                            match Self::eval_value(arg, &local_env, targets, functions) {
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
                            continue;
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

        call_stack.pop();
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
                match (lhs, op, rhs) {
                    (CompileTimeValue::Position(p), BinaryOp::Add, CompileTimeValue::Vector3(v)) => {
                        Ok(CompileTimeValue::Position(Position { point: p.point + v }))
                    }
                    (CompileTimeValue::Position(p), BinaryOp::Sub, CompileTimeValue::Vector3(v)) => {
                        Ok(CompileTimeValue::Position(Position { point: p.point - v }))
                    }
                    (CompileTimeValue::Vector3(v1), BinaryOp::Add, CompileTimeValue::Vector3(v2)) => {
                        Ok(CompileTimeValue::Vector3(v1 + v2))
                    }
                    (CompileTimeValue::Vector3(v1), BinaryOp::Sub, CompileTimeValue::Vector3(v2)) => {
                        Ok(CompileTimeValue::Vector3(v1 - v2))
                    }
                    _ => Err("Unsupported binary operation in evaluation".to_string()),
                }
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
